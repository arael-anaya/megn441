import rclpy
from rclpy.node import Node

from rosbot_msgs.msg import BlockDetection 
from sensor_msgs.msg.Image import Color , Depth, CameraInfo
from rosbot_msgs.srv import DetectBlocks

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')


        self.colorSubscription = self.create_subscription(Color , "color_topic" , self.colorCallback , 10)
        self.depthSubscription = self.create_subscription(Depth , "depth_topic" , self.depthCallback , 10)
        self.cameraInfoSubscription = self.create_subscription(CameraInfo , "cameraInfo_topic" , self.cameraInfoCallback , 10)
    
        self.detectBlocksSrv = self.create_service(DetectBlocks , 'detect_blocks' , self.detectBlocksCallback)


    def colorCallback(self, msg):
            return

    def depthCallback(self, msg):
        return

    def cameraInfoCallback(self, msg):
        return
    
    def detectBlocksCallback(self, request, response):
        return


def main():
    rclpy.init()
    vision_node = VisionNode()
    rclpy.spin(vision_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
