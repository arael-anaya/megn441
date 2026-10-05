import rclpy
from rclpy.node import Node

from rosbot_msgs.msg import TargetPose , ForwardPose , GripperControl
from rosbot_msgs.srv import GetAngles, DetectBlocks




class StackerNode(Node):
    def __init__(self):
        super().__init__('stacker_node')


        # self.targetPoseSubscription = self.create_subscription(TargetPose , "inverse_topic" , self.targetPoseCallback , 10)
        
        self.detectBlocksSrv = self.create_client(DetectBlocks , 'detect_blocks')
        
        self.targetPosePublisher = self.create_publisher(TargetPose, "inverse_topic" , 10)
        self.gripperControlPublisher = self.create_publisher(GripperControl, "gripper_topic" , 10)
        self.getAngleSrv = self.create_client(GetAngles , 'get_angles')
            


def main():
    rclpy.init()
    stacker_node = StackerNode()
    rclpy.spin(stacker_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
