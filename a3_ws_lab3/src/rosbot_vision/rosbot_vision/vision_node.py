import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo

from rosbot_msgs.msg import BlockDetection
from rosbot_msgs.srv import DetectBlocks

from rosbot_vision.findBlock import find_block, HSV_RANGES

import math

from rosbot_vision.depthUtils import median_depth, deproject, camera_to_arm

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

        colorTopic = self.get_parameter('color_topic').value
        depthTopic = self.get_parameter('depth_topic').value
        cameraInfoTopic = self.get_parameter('camera_info_topic').value

        self.depthScale = self.get_parameter('depth_scale').value
        self.patchHalf = self.get_parameter('depth_patch_half').value
        self.minArea = self.get_parameter('min_area').value
        self.camTranslation = self.get_parameter('cam_translation').value
        self.camRpyDeg = self.get_parameter('cam_rpy_deg').value
        self.blockHalfHeight = self.get_parameter('block_half_height').value

        self.bridge = CvBridge()

        self.colorImage = None     
        self.depthImage = None      
        self.cameraInfo = None
        self.colorStamp = None
        self.depthStamp = None

        self.colorSubscription = self.create_subscription(Image, colorTopic, self.colorCallback, 10)
        self.depthSubscription = self.create_subscription(Image, depthTopic, self.depthCallback, 10)
        self.cameraInfoSubscription = self.create_subscription(CameraInfo, cameraInfoTopic, self.cameraInfoCallback, 10)

        self.detectBlocksSrv = self.create_service(DetectBlocks, 'detect_blocks', self.detectBlocksCallback)

        self.get_logger().info(
            f'Subscribed to color={colorTopic}, depth={depthTopic}, camera_info={cameraInfoTopic}')

    def colorCallback(self, msg):
        self.colorImage = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.colorStamp = msg.header.stamp

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

        fx = self.cameraInfo.k[0]
        cx0 = self.cameraInfo.k[2]
        fy = self.cameraInfo.k[4]
        cy0 = self.cameraInfo.k[5]


        for color in HSV_RANGES:
            result = find_block(bgr, color, self.minArea)
            if result is None:
                continue

            u, v, yawDeg = result

            raw = median_depth(depth, u, v, self.patchHalf)

            if raw is None:
                self.get_logger().warn(f'{color}: no valid depth near pixel ({u:.0f},{v:.0f}); skipping.')
                continue

            z = raw * self.depthScale
            camXyz = deproject(u, v, z, fx, fy, cx0, cy0)
            x, y, z = camera_to_arm(camXyz, self.camTranslation, self.camRpyDeg)
            z -= self.blockHalfHeight

            det = BlockDetection()

            det.color = color
            det.x = float(x)
            det.y = float(y)
            det.z = float(z)
            det.yaw = math.radians(yawDeg)

            response.blocks.append(det)

            self.get_logger().info(
                f'{color}: pixel=({u:.1f},{v:.1f}) cam=({camXyz[0]:.3f},{camXyz[1]:.3f},{camXyz[2]:.3f}) arm=({x:.3f},{y:.3f},{z:.3f}) m yaw={yawDeg:.1f} deg')


        return response


def main():
    rclpy.init()
    vision_node = VisionNode()
    rclpy.spin(vision_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
