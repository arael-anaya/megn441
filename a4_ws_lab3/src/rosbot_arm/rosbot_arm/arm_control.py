import rclpy
from rclpy.node import Node
from ros_robot_controller.ros_robot_controller_sdk import Board


class ArmControl(Node):
    def __init__(self):
        super().__init__('arm_control')
        # TODO: Subscribe to a forward control topic
        # TODO: Subscribe to a gripper control topic
    
    board = Board()

    # TODO: Write an arm_control node to receive joint angles
    # and send messages to the Board.

    # See arm_test for example of board functionality.


def main():
    rclpy.init()
    node = ArmControl()
    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__=="__main__":
    main()
