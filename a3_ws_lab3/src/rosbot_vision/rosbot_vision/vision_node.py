import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo

from rosbot_msgs.msg import BlockDetection
from rosbot_msgs.srv import DetectBlocks

from rosbot_vision.findBlock import find_block, HSV_RANGES, maskPixels

import math

import numpy as np
import rclpy.time
import tf2_ros

from rosbot_vision.depthUtils import median_depth, deproject, camera_to_arm, quat_to_matrix, fit_table_plane, level_rotation

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.declare_parameter('color_topic', '/depth_cam/color/image_raw')
        self.declare_parameter('depth_topic', '/depth_cam/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/depth_cam/color/camera_info')

        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('depth_patch_half', 3)
        self.declare_parameter('min_area', 500)

        self.declare_parameter('cam_translation', [0.0, 0.0, 0.0])
        self.declare_parameter('cam_rpy_deg', [0.0, 0.0, 0.0])
        self.declare_parameter('block_half_height', 0.0)
        # Blocks lie flat on the table, so their height is known. The TF camera height is unreliable (it put blocks
        # ~0.6 m up), so report z = table_z + block_half_height instead of the measured z. Set False to use the TF z.
        self.declare_parameter('fixed_table_z', True)
        self.declare_parameter('table_z', 0.0)
        # Raises the reported block z without moving the camera; increase if the gripper hits the table
        self.declare_parameter('grasp_z_offset', 0.015)
        # Replace the TF camera tilt and height with the ones measured from the table in the depth image
        self.declare_parameter('level_from_table', True)
        # Added to every block's x / y in the arm frame; tune if the gripper still lands short or past the blocks
        self.declare_parameter('xy_offset', [0.5, 0.0])

        # Look the camera pose up in TF instead of using the fixed cam_translation / cam_rpy_deg
        self.declare_parameter('use_tf', True)
        # Frame the arm / IK coordinates are measured in: ground level below the base, shifted to the joint 1 axis
        self.declare_parameter('target_frame', 'base_footprint')
        self.declare_parameter('arm_origin_xyz', [0.0251328, 0.0, 0.0])

        colorTopic = self.get_parameter('color_topic').value
        depthTopic = self.get_parameter('depth_topic').value
        cameraInfoTopic = self.get_parameter('camera_info_topic').value

        self.depthScale = self.get_parameter('depth_scale').value
        self.patchHalf = self.get_parameter('depth_patch_half').value
        self.minArea = self.get_parameter('min_area').value
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

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.colorFrame = None

        self.bridge = CvBridge()

        self.colorImage = None     
        self.depthImage = None      
        self.cameraInfo = None
        self.colorStamp = None
        self.depthStamp = None

        # Best-effort QoS matches camera drivers that publish sensor data, and still connects to reliable ones
        self.colorSubscription = self.create_subscription(Image, colorTopic, self.colorCallback, qos_profile_sensor_data)
        self.depthSubscription = self.create_subscription(Image, depthTopic, self.depthCallback, qos_profile_sensor_data)
        self.cameraInfoSubscription = self.create_subscription(CameraInfo, cameraInfoTopic, self.cameraInfoCallback, qos_profile_sensor_data)

        self.detectBlocksSrv = self.create_service(DetectBlocks, 'detect_blocks', self.detectBlocksCallback)

        self.get_logger().info(
            f'Subscribed to color={colorTopic}, depth={depthTopic}, camera_info={cameraInfoTopic}')

    def colorCallback(self, msg):
        self.colorImage = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.colorStamp = msg.header.stamp
        self.colorFrame = msg.header.frame_id

    def depthCallback(self, msg):
        self.depthImage = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.depthStamp = msg.header.stamp

    def cameraInfoCallback(self, msg):
        if self.cameraInfo is None:
            self.get_logger().info(
                f'Got camera_info: {msg.width}x{msg.height}, fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}, '
                f'cx={msg.k[2]:.1f}, cy={msg.k[5]:.1f}')
        self.cameraInfo = msg

    def haveAllInputs(self):
        return (self.colorImage is not None and
                self.depthImage is not None and
                self.cameraInfo is not None)

    def checkAgainstDepthPlane(self, depth, rotation, translation, fx, fy, cx0, cy0):
        """Compare the camera pose from TF with what the depth image says about the table."""
        fit = fit_table_plane(depth, fx, fy, cx0, cy0, self.depthScale)
        if fit is None:
            return None
        height, tilt, fraction, _ = fit
        axisZ = rotation[2, 2]
        tfTilt = math.degrees(math.asin(max(-1.0, min(1.0, -axisZ))))
        self.get_logger().info(
            f'Depth-plane check ({fraction * 100:.0f}% of points on the plane): '
            f'camera is {height:.3f} m from the table, optical axis {tilt:.0f} deg from table-parallel. '
            f'TF says camera z={translation[2]:.3f} m, axis {tfTilt:.0f} deg below horizontal. '
            f'These should match (table at z=0) if the arm pose in TF is right.')
        return fit

    def detectBlocksCallback(self, request, response):
        if not self.haveAllInputs():
            self.get_logger().warn('detect_blocks called before color, depth and camera_info were received.')
            return response
        
        bgr = self.colorImage.copy()
        depth = self.depthImage.copy()

        if depth.shape[:2] != bgr.shape[:2]:
            self.get_logger().warn(
                f'Depth {depth.shape[:2]} and color {bgr.shape[:2]} sizes differ; '
                'depth must be registered to color.')
            return response

        rotation, translation = None, None
        if self.useTf:
            try:
                tf = self.tfBuffer.lookup_transform(
                    self.targetFrame, self.colorFrame, rclpy.time.Time())
            except tf2_ros.TransformException as e:
                self.get_logger().error(
                    f'No TF {self.targetFrame} <- {self.colorFrame}: {e}. '
                    'Is robot_state_publisher / arm_state_publisher running?')
                return response
            q = tf.transform.rotation
            t = tf.transform.translation
            rotation = quat_to_matrix(q.x, q.y, q.z, q.w)
            translation = np.array([t.x, t.y, t.z]) - self.armOrigin
            self.get_logger().info(
                f'TF {self.targetFrame} <- {self.colorFrame}: camera at '
                f'({translation[0]:.3f}, {translation[1]:.3f}, {translation[2]:.3f}) m, '
                f'optical z axis (viewing direction) in arm frame = '
                f'({rotation[0, 2]:.2f}, {rotation[1, 2]:.2f}, {rotation[2, 2]:.2f}) [looking straight down = (0, 0, -1)]')

        fx = self.cameraInfo.k[0]
        cx0 = self.cameraInfo.k[2]
        fy = self.cameraInfo.k[4]
        cy0 = self.cameraInfo.k[5]


        self.get_logger().info(
            f'detect_blocks: color {bgr.shape}, depth {depth.shape} {depth.dtype}, '
            f'depth range {depth.min()}..{depth.max()}')

        if rotation is not None:
            fit = self.checkAgainstDepthPlane(depth, rotation, translation, fx, fy, cx0, cy0)
            if self.levelFromTable and fit is not None:
                height, _, _, normal = fit
                rotation = level_rotation(rotation, normal)
                translation = np.array([translation[0], translation[1], self.tableZ + height])
                self.get_logger().info(
                    f'Leveled camera from the table: z={translation[2]:.3f} m, optical z axis in arm frame = '
                    f'({rotation[0, 2]:.2f}, {rotation[1, 2]:.2f}, {rotation[2, 2]:.2f})')

        for color in HSV_RANGES:
            result = find_block(bgr, color, self.minArea)
            if result is None:
                self.get_logger().warn(
                    f'{color}: no blob >= {self.minArea} px ({maskPixels(bgr, color)} matching px). '
                    'Check the arm is at the look pose and the HSV ranges in findBlock.py.')
                continue

            u, v, yawDeg = result

            raw = median_depth(depth, u, v, self.patchHalf)

            if raw is None:
                self.get_logger().warn(f'{color}: no valid depth near pixel ({u:.0f},{v:.0f}); skipping.')
                continue

            z = raw * self.depthScale
            camXyz = deproject(u, v, z, fx, fy, cx0, cy0)
            if rotation is not None:
                x, y, z = rotation @ np.asarray(camXyz, dtype=float) + translation
                # Rotate the block's in-image edge direction into the arm frame so yaw is relative to the arm axes
                imgAngle = math.radians(yawDeg)
                edge = rotation @ np.array([math.cos(imgAngle), math.sin(imgAngle), 0.0])
                yawDeg = math.degrees(math.atan2(edge[1], edge[0]))
                yawDeg = ((yawDeg + 45.0) % 90.0) - 45.0
            else:
                x, y, z = camera_to_arm(camXyz, self.camTranslation, self.camRpyDeg)
            x += self.xyOffset[0]
            y += self.xyOffset[1]
            measuredZ = z - self.blockHalfHeight
            z = (self.tableZ + self.blockHalfHeight if self.fixedTableZ else measuredZ) + self.graspZOffset

            det = BlockDetection()

            det.color = color
            det.x = float(x)
            det.y = float(y)
            det.z = float(z)
            det.yaw = math.radians(yawDeg)

            response.blocks.append(det)

            self.get_logger().info(
                f'{color}: pixel=({u:.1f},{v:.1f}) cam=({camXyz[0]:.3f},{camXyz[1]:.3f},{camXyz[2]:.3f}) arm=({x:.3f},{y:.3f},{z:.3f}) m (measured z={measuredZ:.3f}) yaw={yawDeg:.1f} deg')


        return response


def main():
    rclpy.init()
    vision_node = VisionNode()
    rclpy.spin(vision_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
