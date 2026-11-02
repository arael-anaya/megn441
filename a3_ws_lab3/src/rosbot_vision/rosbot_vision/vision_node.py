import math
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

from rosbot_msgs.msg import BlockDetection
from rosbot_msgs.srv import DetectBlocks

from rosbot_vision.findBlock import HSV_RANGES, maskPixels
from rosbot_vision.locateBlock import locate_block
from rosbot_vision.depthUtils import (depth_to_meters, quat_to_matrix, rpy_to_matrix, fit_table_plane,
                                      level_rotation)


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

        # Used for 16UC1 depth images; 32FC1 depth is already in meters
        self.declare_parameter('depth_scale', 0.001)
        # Smallest color blob considered, in pixels. Real size is checked in meters, so this only drops speckles.
        self.declare_parameter('min_area', 150)
        # Anything colored whose top is higher than this above the table (the gripper, the robot) is not a block
        self.declare_parameter('max_block_height', 0.06)
        # Reject color blobs that depth shows lying flat on the table. False keeps them on color alone, for when depth
        # misses the blocks but there is nothing else block-colored on the table.
        self.declare_parameter('require_raised', True)

        # Each request looks at this many new frames and keeps a color seen in at least min_detections of them
        # (0 = more than half). Results are the median over those frames.
        self.declare_parameter('num_frames', 5)
        self.declare_parameter('min_detections', 0)
        self.declare_parameter('frame_timeout', 3.0)
        # Color and depth frames further apart in time than this are not paired
        self.declare_parameter('max_pair_gap', 0.1)

        self.declare_parameter('cam_translation', [0.0, 0.0, 0.0])
        self.declare_parameter('cam_rpy_deg', [0.0, 0.0, 0.0])
        self.declare_parameter('block_half_height', 0.0127)
        # Blocks lie flat on the table, so their height is known. Report z = table_z + block_half_height instead of
        # the measured z. Set False to use the measured z.
        self.declare_parameter('fixed_table_z', True)
        self.declare_parameter('table_z', 0.0)
        # Raises the reported block z without moving the camera; increase if the gripper hits the table
        self.declare_parameter('grasp_z_offset', 0.015)
        # Replace the TF camera tilt and height with the ones measured from the table in the depth image
        self.declare_parameter('level_from_table', True)
        # Added to every block's x / y in the arm frame. Leave at 0; only for a constant error measured after the
        # camera pose is confirmed right (see the depth-plane check in the log)
        self.declare_parameter('xy_offset', [0.0, 0.0])
        # Direction (deg, arm frame) the camera sits in when detecting, i.e. joint 1's angle at the look pose. The
        # camera lies in the arm's vertical plane, so TF must put it on this bearing; if it does not (a bad joint 1
        # reading), every block is rotated about the base by the difference. NaN = trust TF.
        self.declare_parameter('expected_camera_heading_deg', float('nan'))

        # Look the camera pose up in TF instead of using the fixed cam_translation / cam_rpy_deg
        self.declare_parameter('use_tf', True)
        # Frame the arm / IK coordinates are measured in: ground level below the base, shifted to the joint 1 axis
        self.declare_parameter('target_frame', 'base_footprint')
        self.declare_parameter('arm_origin_xyz', [0.0251328, 0.0, 0.0])

        for color, ranges in HSV_RANGES.items():
            self.declare_parameter(f'hsv_{color}', [int(v) for pair in ranges for bound in pair for v in bound])

        colorTopic = self.get_parameter('color_topic').value
        depthTopic = self.get_parameter('depth_topic').value
        cameraInfoTopic = self.get_parameter('camera_info_topic').value

        self.depthScale = self.get_parameter('depth_scale').value
        self.minArea = self.get_parameter('min_area').value
        self.maxBlockHeight = self.get_parameter('max_block_height').value
        self.requireRaised = self.get_parameter('require_raised').value
        self.numFrames = max(1, int(self.get_parameter('num_frames').value))
        minDetections = int(self.get_parameter('min_detections').value)
        self.minDetections = minDetections if minDetections > 0 else self.numFrames // 2 + 1
        self.frameTimeout = self.get_parameter('frame_timeout').value
        self.maxPairGap = self.get_parameter('max_pair_gap').value
        self.camTranslation = self.get_parameter('cam_translation').value
        self.camRpyDeg = self.get_parameter('cam_rpy_deg').value
        self.blockHalfHeight = self.get_parameter('block_half_height').value
        self.fixedTableZ = self.get_parameter('fixed_table_z').value
        self.tableZ = self.get_parameter('table_z').value
        self.graspZOffset = self.get_parameter('grasp_z_offset').value
        self.levelFromTable = self.get_parameter('level_from_table').value
        self.xyOffset = [float(v) for v in self.get_parameter('xy_offset').value]
        self.useTf = self.get_parameter('use_tf').value
        self.targetFrame = self.get_parameter('target_frame').value
        self.armOrigin = np.asarray(self.get_parameter('arm_origin_xyz').value, dtype=float)
        self.hsvRanges = {color: hsvFromParam(self.get_parameter(f'hsv_{color}').value) for color in HSV_RANGES}

        self.tfBuffer = tf2_ros.Buffer()
        # Own thread: with the image and service groups each holding an executor thread, /tf callbacks starve
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self, spin_thread=True)
        self.colorFrame = None

        self.bridge = CvBridge()

        # Recent frames as (arrival time, header stamp, image). The service runs on its own thread and waits for
        # frames that arrive after the request, so it never uses an image taken while the arm was still moving.
        self.lock = threading.Lock()
        self.colorFrames = deque(maxlen=30)
        self.depthFrames = deque(maxlen=30)
        self.cameraInfo = None

        imageGroup = MutuallyExclusiveCallbackGroup()
        serviceGroup = MutuallyExclusiveCallbackGroup()

        # Best-effort QoS matches camera drivers that publish sensor data, and still connects to reliable ones
        self.colorSubscription = self.create_subscription(
            Image, colorTopic, self.colorCallback, qos_profile_sensor_data, callback_group=imageGroup)
        self.depthSubscription = self.create_subscription(
            Image, depthTopic, self.depthCallback, qos_profile_sensor_data, callback_group=imageGroup)
        self.cameraInfoSubscription = self.create_subscription(
            CameraInfo, cameraInfoTopic, self.cameraInfoCallback, qos_profile_sensor_data, callback_group=imageGroup)

        self.detectBlocksSrv = self.create_service(
            DetectBlocks, 'detect_blocks', self.detectBlocksCallback, callback_group=serviceGroup)

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

    def collectFrames(self, since):
        """Up to num_frames (color, depth) pairs whose color frame arrived after `since`, oldest first."""
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
        for _, colorStamp, bgr in colors:
            if not depths:
                break
            _, depthStamp, depth = min(depths, key=lambda d: abs(d[1] - colorStamp))
            if abs(depthStamp - colorStamp) > self.maxPairGap:
                continue
            if depth.shape[:2] != bgr.shape[:2]:
                self.get_logger().warn(
                    f'Depth {depth.shape[:2]} and color {bgr.shape[:2]} sizes differ; '
                    'depth must be registered to color (depth_registration:=true).')
                continue
            pairs.append((bgr, depth))
        return pairs

    def settledTransform(self, timeout=6.0, maxAge=0.6, holdTime=0.5, tolerance=0.002):
        """Latest TF once it is fresh (recently stamped) and has not moved for holdTime seconds.

        The latest transform can be seconds old right after the arm moves (servo reads are 5 Hz and the TF
        subscription shares threads with image callbacks). Using it puts every block a whole arm move away.
        """
        end = time.monotonic() + timeout
        last = None
        stableSince = None
        error = None
        while time.monotonic() < end:
            try:
                tf = self.tfBuffer.lookup_transform(self.targetFrame, self.colorFrame, rclpy.time.Time())
            except tf2_ros.TransformException as e:
                error = e
                time.sleep(0.05)
                continue
            age = self.get_clock().now().nanoseconds * 1e-9 - stampSec(tf.header.stamp)
            t = tf.transform.translation
            pos = np.array([t.x, t.y, t.z])
            if last is not None and np.linalg.norm(pos - last) < tolerance and age < maxAge:
                if stableSince is None:
                    stableSince = time.monotonic()
                if time.monotonic() - stableSince >= holdTime:
                    return tf
            else:
                stableSince = None
            last = pos
            time.sleep(0.05)
        if last is None:
            self.get_logger().error(
                f'No TF {self.targetFrame} <- {self.colorFrame}: {error}. '
                'Is robot_state_publisher / arm_state_publisher running?')
            return None
        self.get_logger().warn('Camera TF never settled; the arm may still be moving. Not detecting.')
        return None

    def cameraPose(self):
        """Camera optical frame -> arm frame as (rotation, translation), or None."""
        if not self.useTf:
            return rpy_to_matrix(self.camRpyDeg), np.asarray(self.camTranslation, dtype=float)
        tf = self.settledTransform()
        if tf is None:
            return None
        q = tf.transform.rotation
        t = tf.transform.translation
        rotation = quat_to_matrix(q.x, q.y, q.z, q.w)
        translation = np.array([t.x, t.y, t.z]) - self.armOrigin
        expected = float(self.get_parameter('expected_camera_heading_deg').value)
        if not math.isnan(expected):
            bearing = math.degrees(math.atan2(translation[1], translation[0]))
            delta = (expected - bearing + 180.0) % 360.0 - 180.0
            if abs(delta) > 5.0:
                self.get_logger().error(
                    f'TF puts the camera at {bearing:.0f} deg around the base but the arm is at the look pose '
                    f'({expected:.0f} deg): joint 1 in TF is off by {-delta:.0f} deg (check arm_state_publisher / '
                    f'joint 1 servo reads). Rotating the camera pose by {delta:.0f} deg so blocks are not swung round.')
                c, s_ = math.cos(math.radians(delta)), math.sin(math.radians(delta))
                rz = np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]])
                rotation = rz @ rotation
                translation = rz @ translation
        self.get_logger().info(
            f'TF {self.targetFrame} <- {self.colorFrame}: camera at '
            f'({translation[0]:.3f}, {translation[1]:.3f}, {translation[2]:.3f}) m, '
            f'optical z axis (viewing direction) in arm frame = '
            f'({rotation[0, 2]:.2f}, {rotation[1, 2]:.2f}, {rotation[2, 2]:.2f}) [looking straight down = (0, 0, -1)]')
        return rotation, translation

    def checkAgainstTf(self, fit, rotation, translation):
        """Compare the camera pose from TF with what the depth image says about the table."""
        height, tilt, fraction, _ = fit
        tfTilt = math.degrees(math.asin(max(-1.0, min(1.0, -rotation[2, 2]))))
        tfHeight = translation[2] - self.tableZ
        message = (f'Depth-plane check ({fraction * 100:.0f}% of points on the plane): '
                   f'camera is {height:.3f} m from the table, optical axis {tilt:.0f} deg below horizontal. '
                   f'TF says {tfHeight:.3f} m, {tfTilt:.0f} deg.')
        if abs(tilt - tfTilt) > 8.0 or abs(height - tfHeight) > 0.03:
            # TF and the IK share one kinematic model, so a camera pose this far off means the model does not match
            # the arm (look pose, joint4_sign, servo zeros). Positions will be off by centimetres; no offset fixes it.
            self.get_logger().error(
                message + ' These disagree: the arm model in TF does not match the real arm, so block positions '
                'will be off. Check joint4_sign in arm_state_publisher and that the gripper points straight down '
                'at the look pose.')
        else:
            self.get_logger().info(message)

    def detectFrame(self, bgr, depth, pose, intrinsics, logPlane):
        """Every color found in one frame as {color: detection dict}, plus the debug image."""
        rotation, translation = pose
        fx, fy, cx, cy = intrinsics
        depthM = depth_to_meters(depth, self.depthScale)

        plane = None
        fit = fit_table_plane(depthM, fx, fy, cx, cy, 1.0)
        if fit is None:
            self.get_logger().warn('Could not find the table in the depth image; positions will be less accurate.')
        else:
            height, _, _, normal = fit
            plane = (normal, height)
            if logPlane:
                self.checkAgainstTf(fit, rotation, translation)
            if self.levelFromTable:
                rotation = level_rotation(rotation, normal)
                translation = np.array([translation[0], translation[1], self.tableZ + height])

        found = {}
        debug = bgr.copy()
        for color in HSV_RANGES:
            reasons = []
            det = locate_block(bgr, depthM, color, intrinsics, plane, rotation, translation,
                               2 * self.blockHalfHeight, self.minArea, self.maxBlockHeight,
                               self.hsvRanges[color], reasons, self.requireRaised)
            if det is None:
                if logPlane:
                    self.get_logger().info(
                        f'{color}: not found ({maskPixels(bgr, color, self.hsvRanges[color])} px match the color; '
                        f'{"; ".join(reasons)})')
                continue
            found[color] = det
            u, v = det['pixel']
            cv2.circle(debug, (int(u), int(v)), 6, (255, 255, 255), 2)
            cv2.putText(debug, f'{color} ({det["x"]:.3f}, {det["y"]:.3f})', (int(u) + 8, int(v) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        return found, debug

    def detectBlocksCallback(self, request, response):
        requested = time.monotonic()
        if self.cameraInfo is None or self.colorFrame is None:
            self.get_logger().warn('detect_blocks called before color, depth and camera_info were received.')
            return response

        pose = self.cameraPose()
        if pose is None:
            return response

        pairs = self.collectFrames(requested)
        if not pairs:
            self.get_logger().warn('No color / depth frame pairs to look at.')
            return response

        k = self.cameraInfo.k
        intrinsics = (k[0], k[4], k[2], k[5])

        perColor = {color: [] for color in HSV_RANGES}
        for i, (bgr, depth) in enumerate(pairs):
            found, debug = self.detectFrame(bgr, depth, pose, intrinsics, logPlane=(i == 0))
            for color, det in found.items():
                perColor[color].append(det)
        self.debugPublisher.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))

        need = min(self.minDetections, len(pairs))
        # An empty request means every color
        wanted = set(request.colors) or set(perColor)
        for color, dets in perColor.items():
            if color not in wanted:
                continue
            if len(dets) < need:
                self.get_logger().warn(
                    f'{color}: seen in {len(dets)} of {len(pairs)} frames (need {need}); not reporting it.')
                continue

            x = float(np.median([d['x'] for d in dets])) + self.xyOffset[0]
            y = float(np.median([d['y'] for d in dets])) + self.xyOffset[1]
            measuredZ = float(np.median([d['z'] for d in dets]))
            z = (self.tableZ + self.blockHalfHeight if self.fixedTableZ else measuredZ) + self.graspZOffset
            # Yaw repeats every 90 deg, so average on a circle of 4 * yaw
            angles = np.radians([4 * d['yaw_deg'] for d in dets])
            yawDeg = math.degrees(math.atan2(np.sin(angles).mean(), np.cos(angles).mean())) / 4

            det = BlockDetection()
            det.color = color
            det.x = x
            det.y = y
            det.z = float(z)
            det.yaw = math.radians(yawDeg)
            response.blocks.append(det)

            spread = 1000 * max(np.ptp([d['x'] for d in dets]), np.ptp([d['y'] for d in dets]))
            sizes = dets[-1]['size']
            self.get_logger().info(
                f'{color}: arm=({x:.3f}, {y:.3f}, {z:.3f}) m (measured centre z={measuredZ:.3f}) '
                f'yaw={yawDeg:.1f} deg, seen in {len(dets)}/{len(pairs)} frames, spread {spread:.0f} mm, '
                f'top face {1000 * sizes[0]:.0f}x{1000 * sizes[1]:.0f} mm, pixel={dets[-1]["pixel"][0]:.0f},'
                f'{dets[-1]["pixel"][1]:.0f}')

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
