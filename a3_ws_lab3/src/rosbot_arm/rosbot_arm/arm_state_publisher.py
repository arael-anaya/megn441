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

    URDF joints 2-4 all turn about +y, and the IK gives servos 2-4 the same sense, so joint4_sign must be -1 like the
    others: running the URDF chain on ik_node's output then lands the tool point on the IK target (test_ikMatchesUrdf).
    With +1, TF puts the camera somewhere the arm never is, and every block position is off by centimetres.
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
        self.readTimes = {i: 0.0 for i in self.pulses}
        self.haveReading = False
        self.requestPending = False
        # A joint whose read failed (checksum error) keeps its old pulse; publishing it with a fresh stamp puts
        # the camera where the arm was, not where it is. Stop publishing until every joint has been read again.
        self.declare_parameter('max_reading_age', 0.7)
        self.maxReadingAge = self.get_parameter('max_reading_age').value

        self.timer = self.create_timer(1.0 / self.get_parameter('rate').value, self.update)

    def update(self):
        if self.haveReading:
            now = self.get_clock().now().nanoseconds * 1e-9
            stale = [i for i, t in self.readTimes.items() if now - t > self.maxReadingAge]
            if stale:
                self.get_logger().warn(
                    f'No recent servo reading for joint(s) {stale}; not publishing joint states.',
                    throttle_duration_sec=2.0)
            else:
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
        now = self.get_clock().now().nanoseconds * 1e-9
        for jointId, pulse in zip(result.ids, result.pulses):
            if jointId in self.pulses:
                self.pulses[jointId] = pulse
                self.readTimes[jointId] = now
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
