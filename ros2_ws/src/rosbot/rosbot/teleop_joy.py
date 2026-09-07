import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist , Point
from sensor_msgs.msg import Joy

import numpy as np


class teleopJoy(Node):
    def __init__(self):
        self.MAX_SPEED = 5.0
        self.MAX_TURN = 5.0

        super().__init__("teleop_joy")
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel' , 10)
        self.poseSubscription = self.create_subscription(
            Joy, 'ros_robot_controller/joy' , self.joyCallback, 10)


    def joyCallback(self,msg):
        axes = msg.axes
        leftStickY = axes[1]
        rightStickX = axes[2]
        buttons = msg.buttons

        speedCMD = Twist()

        speedCMD.linear.x = leftStickY * self.MAX_SPEED
        speedCMD.angular.z = rightStickX * self.MAX_TURN

        self.publisher_.publish(speedCMD)
        



def main(args=None):
    rclpy.init(args=args)
    node = teleopJoy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()