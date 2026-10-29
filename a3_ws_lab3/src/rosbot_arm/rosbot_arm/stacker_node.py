import math
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from rosbot_msgs.msg import TargetPose, GripperControl, ForwardPose
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
        self.declare_parameter('block_size', 0.0254)

        self.declare_parameter('hover_height', 0.06)
        # How far (m) to lower the release point of every block; increase if blocks are still dropped too high.
        self.declare_parameter('place_lower', 0.0)

        self.declare_parameter('gripper_open', 300)
        self.declare_parameter('gripper_closed', 600)

        self.declare_parameter('move_duration', 2.0)
        self.declare_parameter('settle_time', 0.5)

        self.declare_parameter('roll_sign', 1.0)

        # Camera is on the arm: it must be at this pose (matches cam_translation in vision_params.yaml) to see the blocks
        # (0.10, 0, 0.25) at pitch -90 is outside joint 4's range; this one is reachable
        # Joint-space look pose for joints 1..5, as pulses (0 deg = straight up, pulse 500). Upper arm 66 deg above
        # horizontal = servo 2 at -24 deg (400); servo 3 = -90 deg (125); servo 4 = -66 deg (225) keeps the gripper
        # pointing down (elevation 66 - 90 - 66 = -90).
        self.declare_parameter('look_pulses', [500, 400, 125, 225, 500])
        self.declare_parameter('look_duration', 2.5)
        self.declare_parameter('go_to_look_pose', True)
        # Go back to the look pose and find each block again right before picking it, in case it was bumped
        self.declare_parameter('redetect_each_pick', True)
        self.declare_parameter('detect_attempts', 10)

        self.stackX = self.get_parameter('stack_x').value
        self.stackY = self.get_parameter('stack_y').value
        self.stackZ = self.get_parameter('stack_z').value
        self.blockSize = self.get_parameter('block_size').value
        self.hover = self.get_parameter('hover_height').value
        self.placeLower = self.get_parameter('place_lower').value
        self.gripperOpen = int(self.get_parameter('gripper_open').value)
        self.gripperClosed = int(self.get_parameter('gripper_closed').value)
        self.moveDuration = self.get_parameter('move_duration').value
        self.settleTime = self.get_parameter('settle_time').value
        self.rollSign = self.get_parameter('roll_sign').value
        self.lookPulses = [int(p) for p in self.get_parameter('look_pulses').value]
        self.lookDuration = float(self.get_parameter('look_duration').value)
        self.goToLookPose = self.get_parameter('go_to_look_pose').value
        self.redetectEachPick = self.get_parameter('redetect_each_pick').value
        self.detectAttempts = max(1, int(self.get_parameter('detect_attempts').value))

        self.detectBlocksSrv = self.create_client(DetectBlocks, 'detect_blocks')

        self.targetPosePublisher = self.create_publisher(TargetPose, 'inverse_topic', 10)
        self.gripperControlPublisher = self.create_publisher(GripperControl, 'gripper_topic', 10)
        self.forwardPosePublisher = self.create_publisher(ForwardPose, 'forward_topic', 10)

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

    def detectBlocks(self, wanted):
        """Ask the vision node for blocks until every color in `wanted` is seen or the attempts run out.
        Returns {color: BlockDetection}, possibly missing some colors."""
        while not self.detectBlocksSrv.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                return {}
            self.get_logger().info('Waiting for detect_blocks service...')

        blocks = {}
        for attempt in range(self.detectAttempts):
            future = self.detectBlocksSrv.call_async(DetectBlocks.Request())
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            if result is not None:
                blocks.update({b.color: b for b in result.blocks if b.color in wanted})
            missing = [c for c in wanted if c not in blocks]
            if not missing:
                return blocks
            self.get_logger().warn(
                f'Not seen yet: {missing} (attempt {attempt + 1}/{self.detectAttempts}), retrying...')
            self.wait(0.5)
        return blocks

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

    def moveToLookPose(self):
        msg = ForwardPose()
        msg.ids = [1, 2, 3, 4, 5]
        msg.pulses = self.lookPulses
        msg.duration = self.lookDuration
        self.forwardPosePublisher.publish(msg)
        self.wait(self.lookDuration + self.settleTime)

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
        placeZ = groundZ + level * self.blockSize - self.placeLower
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

        if self.goToLookPose:
            self.moveGripper(self.gripperOpen)
            self.moveToLookPose()

        blocks = self.detectBlocks(STACK_ORDER)
        if not blocks:
            self.get_logger().error('No blocks found. Nothing to stack.')
            return

        groundZ = self.stackZ
        if math.isnan(groundZ):
            groundZ = sum(b.z for b in blocks.values()) / len(blocks)

        level = 0
        for i, color in enumerate(STACK_ORDER):
            if self.redetectEachPick and i > 0 and self.goToLookPose:
                self.moveToLookPose()
                fresh = self.detectBlocks([color])
                if color in fresh:
                    blocks[color] = fresh[color]
                elif color in blocks:
                    self.get_logger().warn(f'{color} not seen again; using where it was first seen.')
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
