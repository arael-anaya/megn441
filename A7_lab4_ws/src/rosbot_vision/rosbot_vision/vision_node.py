import json
import math
import os
import threading
import time
from collections import deque

import cv2
import numpy as np
import rclpy
import rclpy.time
import tf2_ros
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from rosbot_msgs.msg import BlockDetection, TargetPose, ForwardPose
from rosbot_msgs.srv import DetectBlocks, GetAngles

from rosbot_vision import cameraModel
from rosbot_vision.findBlock import HSV_RANGES, maskPixels
from rosbot_vision.locateBlock import locate_block
from rosbot_vision.depthUtils import depth_to_meters, quat_to_matrix, fit_table_plane, level_rotation

# rosbot_arm already depends on rosbot_vision, so this package cannot declare the reverse dependency in package.xml.
# Both are always built in this workspace; only the kinematics and the look pose are borrowed.
from rosbot_arm import inverseKinematics
from rosbot_arm.armConfig import LOOK_XYZ, BLOCK_SIZE

DEPTH_SCALE = 0.001  # 16UC1 depth images are in mm; 32FC1 depth is already in meters
MAX_PAIR_GAP = 0.1   # color and depth frames further apart in time (s) than this are not paired
# Yaw comes from the block's top-face edges, which are only a few pixels long far away. Beyond this distance from the
# camera (m) the reported yaw is a guess and a warning is logged; re-detect closer.
YAW_MAX_RANGE = 0.55

# The camera housing frame the driver hangs its optical frames off; the arm model ends here
CAMERA_LINK_FRAME = 'depth_cam_link'

# The image is only trusted this long (s) after a commanded arm move was due to finish
ARM_SETTLE = 0.7
ARM_WAIT_MAX = 10.0
# A floor measured from depth is only used if it is this close to where the arm model puts the floor. Further off
# and depth has found something else flat (the robot's deck, a box) or is reading garbage at its minimum range.
PLANE_MAX_HEIGHT_ERROR = 0.04

# When a requested block is not seen, the arm backs up and rises for a wider, higher view of the floor. The gripper
# keeps its pitch and roll; the tool only moves up and toward the base (same heading), to the highest point the arm
# can reach like that. The arm stays there afterwards.
RAISE_MIN_RADIUS = 0.08  # smallest tool radius (m) from the joint 1 axis to retreat to
RAISE_MARGIN = 0.01      # stay this far (m) inside the reach limit, so the arm is never asked to fully straighten
RAISE_DURATION = 1.5

MAX_CAPTURES = 30  # detect_blocks calls kept in capture_dir


def stampSec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def hsvFromParam(values):
    """Flat [h, s, v, h, s, v, ...] list (lower, upper pairs) -> [((h, s, v), (h, s, v)), ...]."""
    values = [int(v) for v in values]
    if len(values) == 0 or len(values) % 6 != 0:
        raise ValueError(f'HSV range list must be 6 numbers per range, got {len(values)}')
    return [(tuple(values[i:i + 3]), tuple(values[i + 3:i + 6])) for i in range(0, len(values), 6)]


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.declare_parameter('color_topic', '/depth_cam/color/image_raw')
        self.declare_parameter('depth_topic', '/depth_cam/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/depth_cam/color/camera_info')

        # The rest are scene tuning, set in config/vision_params.yaml
        # Smallest color blob considered, in pixels. Real size is checked in meters, so this only drops speckles.
        self.declare_parameter('min_area', 60)
        # Anything colored whose top is higher than this above the table (the gripper, the robot) is not a block
        self.declare_parameter('max_block_height', 0.06)
        # Reject color blobs that depth shows lying flat on the table. False keeps them on color alone, for when depth
        # misses the blocks but there is nothing else block-colored on the table.
        self.declare_parameter('require_raised', False)
        # Each request looks at this many new frames and reports the median of the colors seen in at least 2 of them
        self.declare_parameter('num_frames', 5)
        self.declare_parameter('frame_timeout', 3.0)
        # Half the block edge; the block size itself is armConfig.BLOCK_SIZE
        self.declare_parameter('block_half_height', BLOCK_SIZE / 2)
        # Back the arm up for a higher view when a requested block is not seen from where the arm is
        self.declare_parameter('raise_when_missing', True)
        # Every detect_blocks call saves its color, depth and annotated image plus the numbers it used here, so a miss
        # can be looked at afterwards (rosbot_vision replay_capture). Empty string turns it off.
        self.declare_parameter('capture_dir', '~/.ros/vision_captures')
        for color, ranges in HSV_RANGES.items():
            self.declare_parameter(f'hsv_{color}', [int(v) for pair in ranges for bound in pair for v in bound])

        colorTopic = self.get_parameter('color_topic').value
        depthTopic = self.get_parameter('depth_topic').value
        cameraInfoTopic = self.get_parameter('camera_info_topic').value

        self.minArea = self.get_parameter('min_area').value
        self.maxBlockHeight = self.get_parameter('max_block_height').value
        self.requireRaised = self.get_parameter('require_raised').value
        self.numFrames = max(1, int(self.get_parameter('num_frames').value))
        self.minDetections = min(2, self.numFrames)
        self.frameTimeout = self.get_parameter('frame_timeout').value
        self.blockHalfHeight = self.get_parameter('block_half_height').value
        self.raiseWhenMissing = self.get_parameter('raise_when_missing').value
        self.captureDir = os.path.expanduser(self.get_parameter('capture_dir').value or '')
        self.captureCount = 0
        self.hsvRanges = {color: hsvFromParam(self.get_parameter(f'hsv_{color}').value) for color in HSV_RANGES}

        # TF is only used for the camera driver's fixed lens position inside the camera housing. The arm's joints are
        # NOT taken from TF: the robot's own bringup publishes conflicting joint states (see cameraModel).
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self, spin_thread=True)
        self.colorFrame = None
        self.cameraExtrinsic = None
        self.extrinsicWarned = False

        self.bridge = CvBridge()

        # Recent frames as (arrival time, header stamp, image). The service runs on its own thread and waits for
        # frames that arrive after the request, so it never uses an image taken while the arm was still moving.
        self.lock = threading.Lock()
        self.colorFrames = deque(maxlen=30)
        self.depthFrames = deque(maxlen=30)
        self.cameraInfo = None
        # Last pulse every arm servo was sent to, and when that move is due to finish
        self.armPulses = {}
        self.armMoveEnd = 0.0
        self.ikResult = None

        imageGroup = MutuallyExclusiveCallbackGroup()
        serviceGroup = MutuallyExclusiveCallbackGroup()
        # Own thread so arm commands and IK answers still arrive while a service call waits for the arm
        stateGroup = MutuallyExclusiveCallbackGroup()

        # Best-effort QoS matches camera drivers that publish sensor data, and still connects to reliable ones
        self.colorSubscription = self.create_subscription(
            Image, colorTopic, self.colorCallback, qos_profile_sensor_data, callback_group=imageGroup)
        self.depthSubscription = self.create_subscription(
            Image, depthTopic, self.depthCallback, qos_profile_sensor_data, callback_group=imageGroup)
        self.cameraInfoSubscription = self.create_subscription(
            CameraInfo, cameraInfoTopic, self.cameraInfoCallback, qos_profile_sensor_data, callback_group=imageGroup)

        self.detectBlocksSrv = self.create_service(
            DetectBlocks, 'detect_blocks', self.detectBlocksCallback, callback_group=serviceGroup)
        self.raiseViewSrv = self.create_service(
            Trigger, 'raise_view', self.raiseViewCallback, callback_group=serviceGroup)

        # Everything that moves the arm goes through ik_node -> forward_topic -> arm_control
        self.forwardPoseSubscription = self.create_subscription(
            ForwardPose, 'forward_topic', self.forwardPoseCallback, 10, callback_group=stateGroup)
        self.ikResultSubscription = self.create_subscription(
            Bool, 'ik_result', self.ikResultCallback, 10, callback_group=stateGroup)
        self.targetPosePublisher = self.create_publisher(TargetPose, 'inverse_topic', 10)
        # Only for when this node started after the arm was already put in place
        self.getAnglesClient = self.create_client(GetAngles, 'get_angles', callback_group=stateGroup)

        # Color image with what was found drawn on it; view with rqt_image_view
        self.debugPublisher = self.create_publisher(Image, '~/debug_image', 1)

        self.get_logger().info(
            f'Subscribed to color={colorTopic}, depth={depthTopic}, camera_info={cameraInfoTopic}')

    def colorCallback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self.lock:
            self.colorFrames.append((time.monotonic(), stampSec(msg.header.stamp), image))
            self.colorFrame = msg.header.frame_id

    def depthCallback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        with self.lock:
            self.depthFrames.append((time.monotonic(), stampSec(msg.header.stamp), image))

    def cameraInfoCallback(self, msg):
        if self.cameraInfo is None:
            self.get_logger().info(
                f'Got camera_info: {msg.width}x{msg.height}, fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}, '
                f'cx={msg.k[2]:.1f}, cy={msg.k[5]:.1f}')
        self.cameraInfo = msg

    def forwardPoseCallback(self, msg):
        with self.lock:
            for jointId, pulse in zip(msg.ids, msg.pulses):
                if jointId in cameraModel.ARM_JOINTS:
                    self.armPulses[jointId] = cameraModel.clamp_pulse(jointId, pulse)
            self.armMoveEnd = time.monotonic() + max(0.0, float(msg.duration))

    def ikResultCallback(self, msg):
        self.ikResult = msg.data

    def readServos(self, timeout=1.5):
        """{joint id: pulse} for joints 1-5 read from the servos by arm_control, or None."""
        if not self.getAnglesClient.service_is_ready():
            return None
        future = self.getAnglesClient.call_async(GetAngles.Request())
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            time.sleep(0.02)
        if not future.done():
            future.cancel()
            return None
        result = future.result()
        if result is None:
            return None
        pulses = {int(i): int(p) for i, p in zip(result.ids, result.pulses)}
        if all(i in pulses for i in cameraModel.ARM_JOINTS):
            return {i: pulses[i] for i in cameraModel.ARM_JOINTS}
        return None

    def armAngles(self):
        """(servo angles 1-5 in degrees, where they came from), once the arm has stopped moving.

        The angles are the ones the arm was commanded to. Reading them back from the servos is only the fallback:
        the robot's own controller node shares the serial port and reads fail often.
        """
        end = time.monotonic() + ARM_WAIT_MAX
        while time.monotonic() < end:
            with self.lock:
                wait = self.armMoveEnd + ARM_SETTLE - time.monotonic()
            if wait <= 0:
                break
            time.sleep(min(wait, 0.1))

        with self.lock:
            pulses = dict(self.armPulses)
        if len(pulses) == len(cameraModel.ARM_JOINTS):
            return cameraModel.pulses_to_angles(pulses), 'commanded pose'

        for _ in range(3):
            pulses = self.readServos()
            if pulses is not None:
                return cameraModel.pulses_to_angles(pulses), 'servo readings'

        self.get_logger().warn(
            f'No arm command seen since this node started and the servos could not be read; assuming the arm is at '
            f'the look pose {LOOK_XYZ}. Positions are wrong if it is anywhere else.')
        pulses = inverseKinematics.solvePulses(*LOOK_XYZ, 0.0, -90.0)
        return cameraModel.pulses_to_angles(dict(zip(cameraModel.ARM_JOINTS, pulses))), 'assumed look pose'

    def extrinsic(self):
        """The camera driver's optical frame inside the camera housing as (rotation, translation), or None.

        A fixed transform the driver publishes once, so nothing the arm does can make it stale.
        """
        if self.cameraExtrinsic is not None:
            return self.cameraExtrinsic
        end = time.monotonic() + (0.0 if self.extrinsicWarned else 2.0)
        problem = None
        while True:
            try:
                tf = self.tfBuffer.lookup_transform(CAMERA_LINK_FRAME, self.colorFrame, rclpy.time.Time())
                q, t = tf.transform.rotation, tf.transform.translation
                rotation = quat_to_matrix(q.x, q.y, q.z, q.w)
                translation = np.array([t.x, t.y, t.z])
                if cameraModel.plausible_extrinsic(rotation, translation):
                    self.cameraExtrinsic = (rotation, translation)
                    self.get_logger().info(
                        f'Lens position in the camera housing ({CAMERA_LINK_FRAME} <- {self.colorFrame}): '
                        f'({1000 * t.x:.1f}, {1000 * t.y:.1f}, {1000 * t.z:.1f}) mm')
                    return self.cameraExtrinsic
                problem = (f'TF gives ({t.x:.3f}, {t.y:.3f}, {t.z:.3f}) m with an unexpected rotation, '
                           'which is not a lens inside its housing')
                break
            except tf2_ros.TransformException as e:
                problem = str(e)
            if time.monotonic() >= end:
                break
            time.sleep(0.05)
        if not self.extrinsicWarned:
            self.extrinsicWarned = True
            self.get_logger().warn(
                f'No usable {CAMERA_LINK_FRAME} <- {self.colorFrame} transform from the camera driver ({problem}). '
                'Taking the lens to be at the housing origin; positions may be off sideways by 1-2 cm until the '
                'calibration absorbs it.')
        return None

    def cameraPose(self):
        """Camera optical frame -> arm frame as (rotation, translation), the servo angles used and their source."""
        angles, source = self.armAngles()
        rotation, translation = cameraModel.optical_pose(angles, self.extrinsic())
        return (rotation, translation), angles, source

    def raiseView(self):
        """Move the tool to the highest point reachable with the same pitch and roll, along the same heading
        (so only radius and z change). Returns (ok, message)."""
        angles, _ = self.armAngles()
        x, y, z, roll, pitch = (float(v) for v in inverseKinematics.solveFK(angles))
        # Servo quantisation (0.24 deg per pulse) leaves the pitch a hair off; snap it to a whole degree
        pitch = float(round(pitch))
        radius = math.hypot(x, y)
        heading = math.atan2(y, x)

        best = inverseKinematics.highestReach(
            heading, min(RAISE_MIN_RADIUS, radius), radius, roll, pitch, margin=RAISE_MARGIN)
        if best is None or best[2] < z + 0.005:
            return False, f'Tool at ({x:.3f}, {y:.3f}, {z:.3f}) is already as high as it can get.'

        self.ikResult = None
        self.targetPosePublisher.publish(TargetPose(
            x=float(best[0]), y=float(best[1]), z=float(best[2]), roll=roll, pitch=pitch,
            duration=RAISE_DURATION))
        self.get_logger().info(
            f'Raising the camera: tool ({x:.3f}, {y:.3f}, {z:.3f}) -> ({best[0]:.3f}, {best[1]:.3f}, {best[2]:.3f}), '
            f'pitch {pitch:.0f}, roll {roll:.1f} kept.')

        end = time.monotonic() + 2.0
        while self.ikResult is None and time.monotonic() < end:
            time.sleep(0.02)
        if self.ikResult is None:
            return False, 'ik_node did not answer. Is it running?'
        if not self.ikResult:
            return False, f'ik_node rejected ({best[0]:.3f}, {best[1]:.3f}, {best[2]:.3f}).'
        # ik_node's servo command for this move arrives on forward_topic; the next look waits for it to finish
        time.sleep(0.3)
        return True, f'Raising to ({best[0]:.3f}, {best[1]:.3f}, {best[2]:.3f}).'

    def raiseViewCallback(self, request, response):
        response.success, response.message = self.raiseView()
        return response

    def collectFrames(self, since):
        """Up to num_frames (color, depth) pairs whose color frame arrived after `since`, oldest first. Depth is
        None for a color frame without a matching depth frame."""
        end = time.monotonic() + self.frameTimeout
        while time.monotonic() < end:
            with self.lock:
                fresh = sum(1 for f in self.colorFrames if f[0] >= since)
            if fresh >= self.numFrames:
                break
            time.sleep(0.02)

        with self.lock:
            colors = [f for f in self.colorFrames if f[0] >= since][-self.numFrames:]
            depths = list(self.depthFrames)
            if not colors:
                self.get_logger().warn(
                    f'No new color frame in {self.frameTimeout:.1f} s; using the last one received.')
                colors = list(self.colorFrames)[-1:]
        if len(colors) < self.numFrames:
            self.get_logger().warn(f'Only {len(colors)} of {self.numFrames} new frames arrived in time.')

        pairs = []
        problem = 'no depth frames are arriving'
        for arrival, colorStamp, bgr in colors:
            depth = None
            if depths:
                _, depthStamp, candidate = min(depths, key=lambda d: abs(d[1] - colorStamp))
                if abs(depthStamp - colorStamp) > MAX_PAIR_GAP:
                    # Some driver settings stamp color and depth from different clocks; go by arrival instead
                    depthArrival, _, candidate = min(depths, key=lambda d: abs(d[0] - arrival))
                    if abs(depthArrival - arrival) > MAX_PAIR_GAP:
                        candidate = None
                        problem = 'the depth frames are from a different time than the color frames'
                if candidate is not None and candidate.shape[:2] != bgr.shape[:2]:
                    problem = (f'depth is {candidate.shape[:2]} and color is {bgr.shape[:2]}; depth must be '
                               'registered to color (depth_registration:=true)')
                    candidate = None
                depth = candidate
            pairs.append((bgr, depth))
        if pairs and all(depth is None for _, depth in pairs):
            self.get_logger().warn(
                f'No usable depth ({problem}). Locating blocks from color and the arm model alone.')
        return pairs

    def levelledPose(self, depth, intrinsics, pose):
        """Pose with the model's camera tilt and height replaced by the ones measured from the floor seen in depth,
        the floor plane (normal, height) in camera coordinates, and where that plane came from. Once per request.

        The arm sags a few millimetres under its own weight, which tilts the camera; the floor in depth shows by how
        much. The search is limited to planes near the model's floor so walls and furniture in view are not mistaken
        for it. Without a believable fit the model's floor is used.
        """
        rotation, translation = pose
        modelPlane = cameraModel.floor_plane(rotation, translation)
        if depth is None:
            return pose, modelPlane, 'arm model (no depth)'
        fx, fy, cx, cy = intrinsics
        fit = fit_table_plane(depth_to_meters(depth, DEPTH_SCALE), fx, fy, cx, cy, 1.0,
                              expectedNormal=-rotation[2, :])
        if fit is None:
            self.get_logger().warn('Could not find the floor in the depth image; using the arm model for it.')
            return pose, modelPlane, 'arm model (no floor in depth)'
        height, tilt, inliers, normal = fit
        modelTilt = math.degrees(math.asin(max(-1.0, min(1.0, -rotation[2, 2]))))
        modelHeight = translation[2]
        if abs(height - modelHeight) > PLANE_MAX_HEIGHT_ERROR:
            self.get_logger().warn(
                f'Depth sees a flat surface {height:.3f} m from the camera, but the arm model puts the floor at '
                f'{modelHeight:.3f} m. Not the floor (or depth is wrong this close); using the arm model for it.')
            return pose, modelPlane, 'arm model (depth plane rejected)'
        self.get_logger().info(
            f'Floor from depth: camera {height:.3f} m up, {tilt:.1f} deg down ({100 * inliers:.0f}% of depth); '
            f'arm model: {modelHeight:.3f} m, {modelTilt:.1f} deg.')
        pose = (level_rotation(rotation, normal), np.array([translation[0], translation[1], height]))
        return pose, (normal, height), 'depth'

    def detectFrame(self, bgr, depth, pose, plane, intrinsics):
        """Every color found in one frame as {color: detection dict}, why the others were not, and the debug image."""
        rotation, translation = pose
        depthM = np.zeros(bgr.shape[:2]) if depth is None else depth_to_meters(depth, DEPTH_SCALE)

        found = {}
        missed = {}
        debug = bgr.copy()
        for color in HSV_RANGES:
            reasons = []
            det = locate_block(bgr, depthM, color, intrinsics, plane, rotation, translation,
                               2 * self.blockHalfHeight, self.minArea, self.maxBlockHeight,
                               self.hsvRanges[color], reasons, self.requireRaised)
            if det is None:
                missed[color] = (f'{maskPixels(bgr, color, self.hsvRanges[color])} px match the color; '
                                 f'{"; ".join(reasons)}')
                continue
            found[color] = det
            u, v = det['pixel']
            cv2.circle(debug, (int(u), int(v)), 6, (255, 255, 255), 2)
            cv2.putText(debug, f'{color} ({det["x"]:.3f}, {det["y"]:.3f}) {det["yaw_deg"]:.0f}deg',
                        (int(u) + 8, int(v) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        return found, missed, debug

    def saveCapture(self, bgr, depth, debug, meta):
        """Keep what this request looked at, so a miss can be replayed off the robot. Never fails the request."""
        if not self.captureDir:
            return
        try:
            os.makedirs(self.captureDir, exist_ok=True)
            self.captureCount += 1
            stem = os.path.join(self.captureDir, f'{time.strftime("%Y%m%d_%H%M%S")}_{self.captureCount % 1000:03d}')
            cv2.imwrite(stem + '_color.png', bgr)
            cv2.imwrite(stem + '_debug.png', debug)
            if depth is not None:
                mm = np.clip(depth_to_meters(depth, DEPTH_SCALE) * 1000.0, 0, 65535).astype(np.uint16)
                cv2.imwrite(stem + '_depth.png', mm)
            with open(stem + '_meta.json', 'w') as f:
                json.dump(meta, f, indent=1)

            metas = sorted(name for name in os.listdir(self.captureDir) if name.endswith('_meta.json'))
            for old in metas[:-MAX_CAPTURES]:
                for suffix in ('_color.png', '_debug.png', '_depth.png', '_meta.json'):
                    path = os.path.join(self.captureDir, old[:-len('_meta.json')] + suffix)
                    if os.path.exists(path):
                        os.remove(path)
        except Exception as e:  # noqa: B902 - a full disk must not stop a pick
            self.get_logger().warn(f'Could not save the capture to {self.captureDir}: {e}', throttle_duration_sec=30.0)

    def detectBlocksCallback(self, request, response):
        if self.cameraInfo is None or self.colorFrame is None:
            self.get_logger().warn('detect_blocks called before a color image and camera_info were received. '
                                   'Is the camera running with camera_name:=depth_cam?')
            return response

        self.detectOnce(request, response)
        found = {b.color for b in response.blocks}
        # An empty request means every color, and not all of them need to be on the table: only retreat if none show
        missing = set(request.colors) - found if request.colors else (set() if found else set(HSV_RANGES))
        if not missing or not self.raiseWhenMissing:
            return response

        self.get_logger().warn(f'Not seen: {sorted(missing)}. Backing up for a higher view of the floor.')
        ok, message = self.raiseView()
        self.get_logger().info(message)
        if not ok:
            return response
        higher = DetectBlocks.Response()
        self.detectOnce(request, higher)
        if len(higher.blocks) >= len(response.blocks):
            response.blocks = higher.blocks
        return response

    def detectOnce(self, request, response):
        # Waits for the arm to stop, so every frame collected after this is from the pose the model describes
        pose, angles, source = self.cameraPose()
        modelPose = pose

        pairs = self.collectFrames(time.monotonic())
        if not pairs:
            self.get_logger().warn('No color frames to look at. Is the camera running?')
            return response

        k = self.cameraInfo.k
        intrinsics = (k[0], k[4], k[2], k[5])

        # One floor fit per request: the camera does not move while the frames are collected
        depths = [depth for _, depth in pairs if depth is not None]
        pose, plane, planeSource = self.levelledPose(depths[-1] if depths else None, intrinsics, pose)
        self.get_logger().info(
            f'Camera at ({pose[1][0]:.3f}, {pose[1][1]:.3f}, {pose[1][2]:.3f}) m in the arm frame from the '
            f'{source}; floor from {planeSource}.')
        # Blocks raised off the floor (a stack): look for them on a plane that high instead
        if request.height > 0:
            plane = (plane[0], plane[1] - float(request.height))
            self.get_logger().info(f'Looking for blocks {1000 * request.height:.0f} mm above the floor.')

        perColor = {color: [] for color in HSV_RANGES}
        firstMissed = {}
        debug = None
        for i, (bgr, depth) in enumerate(pairs):
            found, missed, debug = self.detectFrame(bgr, depth, pose, plane, intrinsics)
            if i == 0:
                firstMissed = missed
            for color, det in found.items():
                perColor[color].append(det)
        self.debugPublisher.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))

        need = min(self.minDetections, len(pairs))
        # An empty request means every color
        wanted = set(request.colors) or set(perColor)
        reported = {}
        for color, dets in perColor.items():
            if color not in wanted:
                continue
            if not dets:
                self.get_logger().info(f'{color}: not found ({firstMissed.get(color, "seen in no frame")})')
                continue
            if len(dets) < need:
                self.get_logger().warn(
                    f'{color}: seen in {len(dets)} of {len(pairs)} frames (need {need}); not reporting it.')
                continue

            x = float(np.median([d['x'] for d in dets]))
            y = float(np.median([d['y'] for d in dets]))
            # Blocks lie flat on the floor (z = 0 in the arm frame), so the centre height is known, not measured
            z = self.blockHalfHeight
            # Yaw repeats every 90 deg, so average on a circle of 4 * yaw
            turns = np.radians([4 * d['yaw_deg'] for d in dets])
            yawDeg = math.degrees(math.atan2(np.sin(turns).mean(), np.cos(turns).mean())) / 4

            distance = math.hypot(x - pose[1][0], y - pose[1][1])
            if distance > YAW_MAX_RANGE:
                self.get_logger().warn(
                    f'{color}: {distance:.2f} m from the camera; yaw is unreliable beyond '
                    f'{YAW_MAX_RANGE:.2f} m, re-detect from closer before aligning the gripper.')

            det = BlockDetection()
            det.color = color
            det.x = x
            det.y = y
            det.z = float(z)
            det.yaw = math.radians(yawDeg)
            response.blocks.append(det)

            spread = 1000 * max(np.ptp([d['x'] for d in dets]), np.ptp([d['y'] for d in dets]))
            sizes = dets[-1]['size']
            reported[color] = {'x': x, 'y': y, 'yaw_deg': yawDeg, 'frames': len(dets), 'spread_mm': float(spread),
                               'pixel': [float(v) for v in dets[-1]['pixel']]}
            self.get_logger().info(
                f'{color}: arm=({x:.3f}, {y:.3f}) m yaw={yawDeg:.1f} deg, seen in {len(dets)}/{len(pairs)} frames, '
                f'spread {spread:.0f} mm, footprint {1000 * sizes[0]:.0f}x{1000 * sizes[1]:.0f} mm, '
                f'pixel={dets[-1]["pixel"][0]:.0f},{dets[-1]["pixel"][1]:.0f}')

        self.saveCapture(pairs[-1][0], pairs[-1][1], debug, {
            'intrinsics': [float(v) for v in intrinsics],
            'servo_angles_deg': [float(v) for v in angles],
            'pose_source': source,
            'plane_source': planeSource,
            'model_rotation': np.asarray(modelPose[0]).tolist(),
            'model_translation': np.asarray(modelPose[1]).tolist(),
            'rotation': np.asarray(pose[0]).tolist(),
            'translation': np.asarray(pose[1]).tolist(),
            'plane_normal': np.asarray(plane[0]).tolist(),
            'plane_distance': float(plane[1]),
            'block_height': 2 * self.blockHalfHeight,
            'min_area': self.minArea,
            'max_block_height': self.maxBlockHeight,
            'require_raised': bool(self.requireRaised),
            'hsv': {color: [list(bound) for pair in ranges for bound in pair]
                    for color, ranges in self.hsvRanges.items()},
            'requested': list(request.colors),
            'reported': reported,
            'not_found': firstMissed,
        })
        return response


def main():
    rclpy.init()
    vision_node = VisionNode()
    # Images keep arriving on one thread while detect_blocks waits for fresh ones on another
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(vision_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    vision_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
