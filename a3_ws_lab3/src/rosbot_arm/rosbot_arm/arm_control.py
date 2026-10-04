import rclpy
from rclpy.node import Node
from ros_robot_controller.ros_robot_controller_sdk import Board
from rosbot_msgs.msg import ForwardPose , GripperControl
from rosbot_msgs.srv import GetAngles


class ArmControl(Node):
    def __init__(self):
        super().__init__('arm_control')

        self.board = Board()
        self.board.enable_reception()

        self.forwardControlSubscription = self.create_subscription(ForwardPose , 'forward_topic' , self.forwardPoseCallback, 10)
        self.gripperControlSubscription = self.create_subscription(GripperControl , 'gripper_topic' , self.gripperControlCallback , 10)

        self.getAngleSrv = self.create_service(GetAngles , 'get_angles' , self.getAnglesCallback)

        self.jointIds = [1, 2, 3, 4, 5, 10]

    def forwardPoseCallback(self, msg):

        if len(msg.ids) != len(msg.pulses):
            self.get_logger().warn('ForwardPose ids and pulses differ in length. Ignoring.')
            return

        limits = {2: (125, 775), 10: (0, 650)}
        targets = [(i, min(max(p, limits[i][0]), limits[i][1]) if i in limits else p)
                    for i, p in zip(msg.ids, msg.pulses)]

        self.board.bus_servo_set_position(msg.duration, targets)


    def gripperControlCallback(self, msg):
        if msg.pulse > 650: msg.pulse = 650

        self.board.bus_servo_set_position(msg.duration, [(10, msg.pulse)])

    def getAnglesCallback(self, request, response):

        for jointId in self.jointIds:
            currPulse =  self.board.bus_servo_read_position(jointId)
            
            if currPulse is not None:
                currPulse = currPulse[0]
                if currPulse < 0:
                    currPulse = 0
                response.ids.append(jointId)
                response.pulses.append(currPulse)

        return response



def main():
    rclpy.init()
    node = ArmControl()
    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__=="__main__":
    main()
