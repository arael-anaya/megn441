import rclpy
from rclpy.node import Node

from rosbot_msgs.msg import TargetPose, GripperControl
from rosbot_msgs.srv import DetectBlocks


class PickNode(Node):
    """Move to the detection pose, look for a block, pick it up and lift it."""

    def __init__(self):
        super().__init__('pick_node')

        self.declare_parameter('open_pulse', 600)    # gripper limit is 650 in arm_control
        self.declare_parameter('close_pulse', 300)   # tune so it squeezes the block
        self.declare_parameter('grip_duration', 1.0)

        self.declare_parameter('look_xyz', [0.18, 0.0, 0.22])
        self.declare_parameter('pitch', -90.0)
        self.declare_parameter('roll', 0.0)

        self.declare_parameter('hover_offset', 0.08)   # above block center, meters
        self.declare_parameter('grasp_offset', 0.0)    # added to block z for the grasp
        self.declare_parameter('lift_offset', 0.10)

        self.declare_parameter('move_time', 2.0)
        self.declare_parameter('settle_time', 0.5)
        self.declare_parameter('detect_retries', 15)

        p = lambda name: self.get_parameter(name).value
        self.openPulse = p('open_pulse')
        self.closePulse = p('close_pulse')
        self.gripDuration = p('grip_duration')
        self.lookXyz = p('look_xyz')
        self.pitch = p('pitch')
        self.roll = p('roll')
        self.hoverOffset = p('hover_offset')
        self.graspOffset = p('grasp_offset')
        self.liftOffset = p('lift_offset')
        self.moveTime = p('move_time')
        self.settleTime = p('settle_time')
        self.detectRetries = p('detect_retries')

        self.targetPosePublisher = self.create_publisher(TargetPose, 'inverse_topic', 10)
        self.gripperPublisher = self.create_publisher(GripperControl, 'gripper_topic', 10)
        self.detectBlocksSrv = self.create_client(DetectBlocks, 'detect_blocks')

        self.state = 'WAIT_SERVICES'
        self.nextActionTime = 0.0
        self.block = None
        self.future = None
        self.attempts = 0

        self.timer = self.create_timer(0.1, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def setState(self, name):
        self.state = name
        self.get_logger().info(f'State: {name}')

    def sendPose(self, x, y, z):
        msg = TargetPose()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        msg.roll = float(self.roll)
        msg.pitch = float(self.pitch)
        msg.duration = float(self.moveTime)
        self.targetPosePublisher.publish(msg)
        self.nextActionTime = self.now() + self.moveTime + self.settleTime

    def sendGripper(self, pulse):
        msg = GripperControl()
        msg.pulse = int(pulse)
        msg.duration = float(self.gripDuration)
        self.gripperPublisher.publish(msg)
        self.nextActionTime = self.now() + self.gripDuration + self.settleTime

    def tick(self):
        if self.now() < self.nextActionTime:
            return

        if self.state == 'WAIT_SERVICES':
            if not self.detectBlocksSrv.service_is_ready():
                self.get_logger().info('Waiting for detect_blocks...', throttle_duration_sec=2.0)
                return
            self.sendGripper(self.openPulse)
            self.setState('LOOK')

        elif self.state == 'LOOK':
            self.sendPose(*self.lookXyz)
            self.setState('DETECT')

        elif self.state == 'DETECT':
            if self.future is None:
                self.future = self.detectBlocksSrv.call_async(DetectBlocks.Request())
                return
            if not self.future.done():
                return
            blocks = self.future.result().blocks
            self.future = None
            if not blocks:
                self.attempts += 1
                if self.attempts >= self.detectRetries:
                    self.get_logger().error('No block found. Giving up.')
                    self.setState('FAILED')
                else:
                    self.get_logger().warn('No block found, retrying.')
                    self.nextActionTime = self.now() + 1.0
                return
            self.block = blocks[0]
            self.get_logger().info(
                f'Picking {self.block.color} block at '
                f'({self.block.x:.3f}, {self.block.y:.3f}, {self.block.z:.3f})')
            self.setState('HOVER')

        elif self.state == 'HOVER':
            b = self.block
            self.sendPose(b.x, b.y, b.z + self.hoverOffset)
            self.setState('DESCEND')

        elif self.state == 'DESCEND':
            b = self.block
            self.sendPose(b.x, b.y, b.z + self.graspOffset)
            self.setState('GRASP')

        elif self.state == 'GRASP':
            self.sendGripper(self.closePulse)
            self.setState('LIFT')

        elif self.state == 'LIFT':
            b = self.block
            self.sendPose(b.x, b.y, b.z + self.liftOffset)
            self.setState('DONE')

        elif self.state == 'DONE':
            self.get_logger().info('Pick sequence finished.')
            self.timer.cancel()

        elif self.state == 'FAILED':
            self.timer.cancel()


def main():
    rclpy.init()
    node = PickNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
