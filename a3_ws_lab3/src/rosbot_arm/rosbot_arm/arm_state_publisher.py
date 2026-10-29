import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from rosbot_msgs.srv import GetAngles


def pulseToDeg(pulse):
    return (pulse - 500) * 120.0 / 500.0


class ArmStatePublisher(Node):
    """Reads the real servo positions and publishes them as URDF joint states,
    so robot_state_publisher can provide the base -> camera transform.

    The servos read 0 with the arm straight up, which is also where the URDF joints 2-4 are zero (the IK output uses
    the same convention), so:
        urdf2 = -j2,  urdf3 = -j3,  urdf4 = joint4_sign * j4
    and joints 1 and 5 have a -z axis, so urdf1 = -j1, urdf5 = -j5.

    joint4_sign was -1 at first, but the camera then sat 2 cm above the table looking backwards while the depth image
    showed the table 0.18 m away. +1 matches the depth image (camera ~0.11 m up, looking forward and down).
    """

    def __init__(self):
        super().__init__('arm_state_publisher')

        self.declare_parameter('rate', 5.0)
        self.declare_parameter('joint4_sign', -1.0)
        self.joint4Sign = self.get_parameter('joint4_sign').value

        self.jointStatePublisher = self.create_publisher(JointState, 'joint_states', 10)
        self.getAnglesClient = self.create_client(GetAngles, 'get_angles')

        # Last known pulses so one failed servo read does not drop a joint
        self.pulses = {1: 500, 2: 500, 3: 500, 4: 500, 5: 500}
        self.haveReading = False
        self.requestPending = False

        self.timer = self.create_timer(1.0 / self.get_parameter('rate').value, self.update)

    def update(self):
        if self.haveReading:
            self.publish()

        if self.requestPending or not self.getAnglesClient.service_is_ready():
            return
        self.requestPending = True
        future = self.getAnglesClient.call_async(GetAngles.Request())
        future.add_done_callback(self.anglesCallback)

    def anglesCallback(self, future):
        self.requestPending = False
        result = future.result()
        if result is None:
            return
        got = 0
        for jointId, pulse in zip(result.ids, result.pulses):
            if jointId in self.pulses:
                self.pulses[jointId] = pulse
                got += 1
        if got == len(self.pulses):
            self.haveReading = True

    def publish(self):
        j = {i: math.radians(pulseToDeg(p)) for i, p in self.pulses.items()}

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        msg.position = [-j[1], -j[2], -j[3], self.joint4Sign * j[4], -j[5]]
        self.jointStatePublisher.publish(msg)

        self.get_logger().info(
            f'servo pulses {self.pulses} -> urdf joints (deg) '
            f'{[round(math.degrees(v), 1) for v in msg.position]}', throttle_duration_sec=2.0)


def main():
    rclpy.init()
    node = ArmStatePublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
