#!/usr/bin/env python3

from ros_robot_controller.ros_robot_controller_sdk import Board
import rclpy
import sys
import time

def reliable_spin(node, pub, msg, max_time=2.0):
    timeout = time.time() + max_time
    while pub.get_subscription_count() < 1 and time.time() < timeout:
        rclpy.spin_once(node, timeout_sec=0.05)

    pub.publish(msg)
    node.get_logger().info(f"Publishing xyz={xyz}, roll={roll}, pitch={pitch}")
    for _ in range(5):
        rclpy.spin_once(node, timeout_sec=0.05)




targets = [ (1, 500),
            (2, 750),
            (3, 40),
            (4, 350),
            (5, 500),
            (10, 350)]

duration = 1.0

def direct():
    board = Board()
    board.enable_reception()
    board.bus_servo_set_position(duration, targets)
    for i in [1, 2, 3, 4, 5, 10]:
        print(i, ":\t", board.bus_servo_read_position(i))



from rosbot_msgs.msg import ForwardPose

def forward():
    rclpy.init()
    node = rclpy.create_node('arm_test')

    forward_pub = node.create_publisher(ForwardPose, 'forward_topic', 10)
    msg = ForwardPose()

    for target in targets:
        msg.ids.append(target[0])
        msg.pulses.append(target[1])

    msg.duration = duration

    reliable_spin(node, forward_pub, msg)


from rosbot_msgs.msg import TargetPose

xyz = [0.18, 0.02, 0.32]
roll = 0.0
pitch = -90.0

def inverse():
    rclpy.init()
    node = rclpy.create_node('arm_test')
    inverse_pub = node.create_publisher(TargetPose, 'inverse_topic', 10)
    msg = TargetPose()


    msg.x = xyz[0]
    msg.y = xyz[1]
    msg.z = xyz[2]

    msg.roll = roll
    msg.pitch = pitch

    msg.duration = duration



    reliable_spin(node, inverse_pub, msg)



def main(mode=None):
    if mode is None:
        mode = sys.argv[1] if len(sys.argv) > 1 else 'direct'
    if mode == 'direct':
        direct()
    elif mode=='forward':
        forward()
    elif mode=='inverse':
        inverse()
    else:
        print('Mode options are direct, forward, or inverse')

if __name__ == '__main__':
    main()
