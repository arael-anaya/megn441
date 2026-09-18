import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np

class DemoPub(Node):
    def __init__(self):
        super().__init__('demo_pub')
        self.publisher = self.create_publisher(
            Twist, 'turtle1/cmd_vel', 10)
        timer_period = 1.0 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        vel = 1.0
        omega = 0.5

        msg.linear.x = vel
        msg.angular.z = omega
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: v={v}, omega={w}'.format(v=vel, w=omega))

def main():
    rclpy.init()
    demo_pub = DemoPub()
    rclpy.spin(demo_pub)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
