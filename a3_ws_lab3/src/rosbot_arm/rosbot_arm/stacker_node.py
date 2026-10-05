   import rclpy
from rclpy.node import Node

from rosbot_msgs.msg import TargetPose , ForwardPose , GripperControl
from rosbot_msgs.srv import GetAngles, DetectBlocks




class StackerNode(Node):
    def __init__(self):
        super().__init__('stacker_node')

        OPENPULSE = 1000
        CLOSEPULSE = 0
        GRIP_DURATION = 1.0

        HOVER_Z = .1
        GRASP_Z = .1
        BLOCK_HEIGHT = .1
        STACK_X = 1
        STACK_Y = 1
        STACK_BASE_Z = .1

        PITCH = 10
        ROLL = 10

        MOVE_TIME = 2
        SETTLE_TIME = .3

        self.state = 'WAIT_SERVICES'
        self.blocks = []
        self.currentBlock = None
        self.stackLevel = 0
        self.nextActionTime = 0.0
        self.timer = self.create_timer(0.1,self.tick)


        self.detectBlocksSrv = self.create_client(DetectBlocks , 'detect_blocks')
        
        self.targetPosePublisher = self.create_publisher(TargetPose, "inverse_topic" , 10)
        self.gripperControlPublisher = self.create_publisher(GripperControl, "gripper_topic" , 10)
        self.getAngleSrv = self.create_client(GetAngles , 'get_angles')


        def moveTo(x,y,z,duration):
            msg = TargetPose()

            msg.x = x
            msg.y = y
            msg.z = z
            msg.duration = duration

            return msg

        def openGripper():
            msg = GripperControl()
            msg.pulse = OPENPULSE
            msg.duration = GRIP_DURATION

            msg.gripperControlPublisher.publish(msg)

        def closeGripper():
                msg = GripperControl()
                msg.pulse = CLOSEPULSE
                msg.duration = GRIP_DURATION
    
                msg.gripperControlPublisher.publish(msg)

        def now():
            return float(self.get_clock().now().nanoseconds / 1e9)

        def sendPose(x, y, z, duration):
            msg = TargetPose()
            msg.x = x
            msg.y = y
            msg.z = z
            msg.roll = ROLL
            msg.pitch = PITCH
            msg.duration = duration

            self.targetPosePublisher(msg)
            self.nextActionTime = now() + duration + SETTLE_TIME

        def sendGripper(pulse):
            msg = GripperControl()
            msg.pulse
            msg.duration = GRIP_DURATION
            self.gripperControlPublisher(msg)
            self.nextActionTime = now() + GRIP_DURATION + SETTLE_TIME

        def setState(name):
            self.state = name
            self.get_logger().info(f'State updated:{name}')
            

            


def main():
    rclpy.init()
    stacker_node = StackerNode()
    rclpy.spin(stacker_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
