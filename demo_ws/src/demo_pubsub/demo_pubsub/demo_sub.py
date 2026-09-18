import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DemoSub(Node):
    def __init__(self):
        super().__init__('demo_sub')
        self.subscriber = self.create_subscription(
            Twist, 'turtle1/cmd_vel', self.listener_callback, 10)
        
    def listener_callback(self, msg):
        self.get_logger().info(
            'The velocity was {} and the angular velocity was {}!'.format(
                msg.linear.x, msg.angular.z))

def main():
    rclpy.init()
    demo_sub = DemoSub()
    rclpy.spin(demo_sub)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
