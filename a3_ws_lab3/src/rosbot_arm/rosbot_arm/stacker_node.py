import math
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from rosbot_msgs.msg import TargetPose, GripperControl
from rosbot_msgs.srv import DetectBlocks

# Bottom of the stack first
STACK_ORDER = ['red', 'green', 'blue']


class UnreachableError(Exception):
    """The IK node refused a target pose."""


class StackerNode(Node):
    def __init__(self):
        super().__init__('stacker_node')


        self.declare_parameter('stack_x', 0.0)
        self.declare_parameter('stack_y', -0.20)

        # Ground height of the stack in the arm frame. NaN = use the average height of the detected blocks.
        self.declare_parameter('stack_z', float('nan'))
        self.declare_parameter('block_size', 0.025)

        self.declare_parameter('hover_height', 0.06)

        self.declare_parameter('gripper_open', 300)
        self.declare_parameter('gripper_closed', 600)

        self.declare_parameter('move_duration', 2.0)
        self.declare_parameter('settle_time', 0.5)

        self.declare_parameter('roll_sign', 1.0)

        self.stackX = self.get_parameter('stack_x').value
        self.stackY = self.get_parameter('stack_y').value
        self.stackZ = self.get_parameter('stack_z').value
        self.blockSize = self.get_parameter('block_size').value
        self.hover = self.get_parameter('hover_height').value
        self.gripperOpen = int(self.get_parameter('gripper_open').value)
        self.gripperClosed = int(self.get_parameter('gripper_closed').value)
        self.moveDuration = self.get_parameter('move_duration').value
        self.settleTime = self.get_parameter('settle_time').value
        self.rollSign = self.get_parameter('roll_sign').value

        self.detectBlocksSrv = self.create_client(DetectBlocks, 'detect_blocks')

        self.targetPosePublisher = self.create_publisher(TargetPose, 'inverse_topic', 10)
        self.gripperControlPublisher = self.create_publisher(GripperControl, 'gripper_topic', 10)

        self.ikResult = None
        self.ikResultSubscription = self.create_subscription(Bool, 'ik_result', self.ikResultCallback, 10)

    def ikResultCallback(self, msg):
        self.ikResult = msg.data

    def wait(self, seconds):
        """Sleep while still letting ROS process callbacks."""
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def waitForSubscribers(self, timeout=10.0):
        """A publish is lost if nobody is listening yet, so wait for ik_node and arm_control."""
        end = time.time() + timeout
        while rclpy.ok() and time.time() < end:
            if (self.targetPosePublisher.get_subscription_count() > 0 and
                    self.gripperControlPublisher.get_subscription_count() > 0):
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def detectBlocks(self):
        """Ask the vision node for blocks. Returns {color: BlockDetection}."""
        while not self.detectBlocksSrv.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                return {}
            self.get_logger().info('Waiting for detect_blocks service...')

        for attempt in range(10):
            future = self.detectBlocksSrv.call_async(DetectBlocks.Request())
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            blocks = {b.color: b for b in result.blocks} if result is not None else {}
            if blocks:
                return blocks
            self.get_logger().warn(f'No blocks detected (attempt {attempt + 1}/10), retrying...')
            self.wait(1.0)
        return {}

    def moveGripper(self, pulse):
        msg = GripperControl()
        msg.pulse = int(pulse)
        msg.duration = 0.8
        self.gripperControlPublisher.publish(msg)
        self.wait(msg.duration + self.settleTime)

    def moveArm(self, x, y, z, rollDeg):
        """Send the gripper tip to (x, y, z) pointing straight down, and wait until it gets there."""
        msg = TargetPose()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        msg.roll = float(rollDeg)
        msg.pitch = -90.0
        msg.duration = float(self.moveDuration)

        self.ikResult = None
        self.targetPosePublisher.publish(msg)

        # The IK node answers right away; give it a moment, then stop if it rejected the pose
        end = time.time() + 2.0
        while rclpy.ok() and self.ikResult is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.ikResult is False:
            raise UnreachableError(f'IK rejected target ({x:.3f}, {y:.3f}, {z:.3f})')

        self.wait(self.moveDuration + self.settleTime)

    def rollFor(self, x, y, blockYawRad):
        """Joint 5 spins relative to the arm's current heading (set by joint 1), so subtract the heading.
        A square block looks the same every 90 degrees, so wrap into [-45, 45)."""
        heading = math.degrees(math.atan2(y, x))
        roll = self.rollSign * (math.degrees(blockYawRad) - heading)
        return ((roll + 45.0) % 90.0) - 45.0



    def pickAndPlace(self, block, level, groundZ):
        bx, by, bz = block.x, block.y, block.z
        pickRoll = self.rollFor(bx, by, block.yaw)

        # Block centres sit half a block above the ground; stack on a fixed ground height, not this block's noisy z
        placeZ = groundZ + level * self.blockSize
        placeRoll = 0.0

        self.get_logger().info(
            f'Picking {block.color} at ({bx:.3f}, {by:.3f}, {bz:.3f}), '
            f'placing at ({self.stackX:.3f}, {self.stackY:.3f}, {placeZ:.3f})')

        self.moveGripper(self.gripperOpen)
        self.moveArm(bx, by, bz + self.hover, pickRoll)    
        self.moveArm(bx, by, bz, pickRoll)                 
        self.moveGripper(self.gripperClosed)               
        self.moveArm(bx, by, bz + self.hover, pickRoll)    
        self.moveArm(self.stackX, self.stackY, placeZ + self.hover, placeRoll) 
        self.moveArm(self.stackX, self.stackY, placeZ, placeRoll)              
        self.moveGripper(self.gripperOpen)               
        self.moveArm(self.stackX, self.stackY, placeZ + self.hover, placeRoll)  # back up

    def run(self):
        if not self.waitForSubscribers():
            self.get_logger().error('ik_node / arm_control are not running (nobody on inverse_topic or gripper_topic).')
            return

        blocks = self.detectBlocks()
        if not blocks:
            self.get_logger().error('No blocks found. Nothing to stack.')
            return

        groundZ = self.stackZ
        if math.isnan(groundZ):
            groundZ = sum(b.z for b in blocks.values()) / len(blocks)

        level = 0
        for color in STACK_ORDER:
            if color not in blocks:
                self.get_logger().warn(f'{color} block not detected, skipping it.')
                continue
            try:
                self.pickAndPlace(blocks[color], level, groundZ)
            except UnreachableError as e:
                self.get_logger().error(f'{e}. Stopping so the arm does not grab or drop blocks in the wrong place.')
                break
            level += 1

        self.get_logger().info(f'Done. Stacked {level} block(s).')


def main():
    rclpy.init()
    stacker_node = StackerNode()
    try:
        stacker_node.run()
    except KeyboardInterrupt:
        pass
    stacker_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
