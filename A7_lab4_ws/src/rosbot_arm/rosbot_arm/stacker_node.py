import math
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from rosbot_msgs.msg import TargetPose, GripperControl
from rosbot_msgs.srv import DetectBlocks

from rosbot_arm import calibration

# Bottom of the stack first
STACK_ORDER = ['red', 'green', 'blue']


class UnreachableError(Exception):
    """The IK node refused a target pose."""


class StackerNode(Node):
    def __init__(self, nodeName='stacker_node', applyCalibration=True):
        super().__init__(nodeName)

        self.declare_parameter('stack_x', 0.0)
        self.declare_parameter('stack_y', -0.20)

        # Tool point height (m) the gripper closes at, for every pick and for calibrate_node. Blocks always sit on the
        # floor, so this is fixed rather than taken from the camera. Negative values are fine: lower it if the fingers
        # close above the block, raise it if they hit the floor.
        self.declare_parameter('grasp_z', -0.01)
        # Release height of the bottom block of the stack. NaN = grasp_z, the height the block was held at.
        self.declare_parameter('stack_z', float('nan'))
        self.declare_parameter('block_size', 0.0254)

        self.declare_parameter('hover_height', 0.06)
        # How far (m) to lower the release point of every block; increase if blocks are still dropped too high.
        self.declare_parameter('place_lower', 0.01)
        # How far (m) to pull each placement toward the arm base, per stack level above the first. Blocks land further
        # out as the stack gets taller; raise this if they still land too far forward, lower it (or go negative) if short.
        self.declare_parameter('place_pullback_per_level', 0.01)

        self.declare_parameter('gripper_open', 300)
        self.declare_parameter('gripper_closed', 600)

        self.declare_parameter('move_duration', 1.0)
        self.declare_parameter('settle_time', 0.5)

        self.declare_parameter('roll_sign', 1.0)
        self.declare_parameter('roll_gain', 1.0)  # >1 turns joint 5 further than the computed angle
        self.declare_parameter('heading_sign', -1.0)  # -1 if off-centre blocks are missed: joint 1 turns the gripper the other way

        # Look pose as an IK target (tool point, gripper pointing straight down), so it goes through the same
        # kinematic model as the picks. With the URDF camera mount, (0.20, 0, 0.14) puts the camera at about
        # (0.25, 0, 0.25) looking straight down, 0.22 m above the block tops: as low as the depth camera's ~0.2 m
        # minimum range allows. Lower look_z below ~0.13 and depth drops out on the blocks.
        self.declare_parameter('look_xyz', [0.20, 0.0, 0.14])
        self.declare_parameter('look_duration', 1.25)
        # Extra wait at the look pose so arm_state_publisher (5 Hz servo reads) and TF catch up with the real arm
        self.declare_parameter('look_settle', 1.0)
        self.declare_parameter('go_to_look_pose', True)
        # Go back to the look pose and find each block again right before picking it, in case it was bumped
        self.declare_parameter('redetect_each_pick', True)
        self.declare_parameter('detect_attempts', 10)
        # Camera -> arm correction measured by calibrate_node; applied to every detection when the file exists
        self.declare_parameter('calibration_file', '~/.ros/block_calibration.json')

        self.stackX = self.get_parameter('stack_x').value
        self.stackY = self.get_parameter('stack_y').value
        self.graspZ = float(self.get_parameter('grasp_z').value)
        self.get_logger().info(f'grasp_z = {self.graspZ:.3f} m')
        self.stackZ = self.get_parameter('stack_z').value
        self.blockSize = self.get_parameter('block_size').value
        self.hover = self.get_parameter('hover_height').value
        self.placeLower = self.get_parameter('place_lower').value
        self.placePullback = float(self.get_parameter('place_pullback_per_level').value)
        self.gripperOpen = int(self.get_parameter('gripper_open').value)
        self.gripperClosed = int(self.get_parameter('gripper_closed').value)
        self.moveDuration = self.get_parameter('move_duration').value
        self.settleTime = self.get_parameter('settle_time').value
        self.rollSign = self.get_parameter('roll_sign').value
        self.rollGain = self.get_parameter('roll_gain').value
        self.headingSign = self.get_parameter('heading_sign').value
        self.lookXyz = [float(v) for v in self.get_parameter('look_xyz').value]
        self.lookDuration = float(self.get_parameter('look_duration').value)
        self.lookSettle = float(self.get_parameter('look_settle').value)
        self.goToLookPose = self.get_parameter('go_to_look_pose').value
        self.redetectEachPick = self.get_parameter('redetect_each_pick').value
        self.detectAttempts = max(1, int(self.get_parameter('detect_attempts').value))
        self.calibrationFile = self.get_parameter('calibration_file').value
        self.calibration = None
        if applyCalibration:
            self.loadCalibration()

        self.detectBlocksSrv = self.create_client(DetectBlocks, 'detect_blocks')

        self.targetPosePublisher = self.create_publisher(TargetPose, 'inverse_topic', 10)
        self.gripperControlPublisher = self.create_publisher(GripperControl, 'gripper_topic', 10)

        self.ikResult = None
        self.ikResultSubscription = self.create_subscription(Bool, 'ik_result', self.ikResultCallback, 10)

    def ikResultCallback(self, msg):
        self.ikResult = msg.data

    def loadCalibration(self):
        loaded = calibration.load(self.calibrationFile)
        if loaded is None:
            self.get_logger().warn(
                f'No calibration at {self.calibrationFile}; using raw camera positions. '
                'Run: ros2 launch rosbot_arm stack_launch.py mode:=calibrate')
            return
        A, b, lookXyz = loaded
        if any(abs(a - c) > 1e-3 for a, c in zip(lookXyz, self.lookXyz)):
            self.get_logger().error(
                f'Calibration was measured from look pose {lookXyz}, not {self.lookXyz}; not using it. '
                'Recalibrate after changing the look pose.')
            return
        self.calibration = (A, b)
        self.get_logger().info(f'Using calibration {self.calibrationFile}: {calibration.describe(A, b)}')

    def correct(self, block):
        """Move a detection from where the camera sees it to where the arm has to go for it."""
        if self.calibration is None:
            return block
        rawX, rawY = block.x, block.y
        block.x, block.y, block.yaw = calibration.apply(*self.calibration, block.x, block.y, block.yaw)
        self.get_logger().info(
            f'{block.color}: camera ({rawX:.3f}, {rawY:.3f}) -> arm ({block.x:.3f}, {block.y:.3f})')
        return block

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
            future = self.detectBlocksSrv.call_async(DetectBlocks.Request(colors=list(wanted)))
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
        msg.duration = 0.4
        self.gripperControlPublisher.publish(msg)
        self.wait(msg.duration + self.settleTime)

    def moveArm(self, x, y, z, rollDeg, duration=None):
        """Send the gripper tip to (x, y, z) pointing straight down, and wait until it gets there."""
        duration = self.moveDuration if duration is None else duration
        msg = TargetPose()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        msg.roll = float(rollDeg)
        msg.pitch = -90.0
        msg.duration = float(duration)

        self.ikResult = None
        self.get_logger().info(f'Arm -> ({x:.3f}, {y:.3f}, {z:.3f}) roll {rollDeg:.0f}')
        self.targetPosePublisher.publish(msg)

        # The IK node answers right away; give it a moment, then stop if it rejected the pose
        end = time.time() + 2.0
        while rclpy.ok() and self.ikResult is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.ikResult is False:
            raise UnreachableError(f'IK rejected target ({x:.3f}, {y:.3f}, {z:.3f})')

        self.wait(duration + self.settleTime)

    def moveToLookPose(self):
        x, y, z = self.lookXyz
        self.moveArm(x, y, z, 0.0, self.lookDuration)
        self.wait(self.lookSettle)

    def rollFor(self, x, y, blockYawRad):
        """Joint 5 spins relative to the arm's current heading (set by joint 1), so subtract the heading.
        A square block looks the same every 90 degrees, so wrap into [-45, 45)."""
        heading = math.degrees(math.atan2(y, x))
        roll = self.rollSign * (math.degrees(blockYawRad) - self.headingSign * heading)
        return self.rollGain * (((roll + 45.0) % 90.0) - 45.0)

    def pickAndPlace(self, block, level, groundZ):
        # The detection is already in the IK frame: go straight to it
        bx, by, bz = block.x, block.y, self.graspZ
        pickRoll = self.rollFor(bx, by, block.yaw)

        # Block centres sit half a block above the ground; stack on a fixed ground height, not this block's noisy z
        placeZ = groundZ + level * self.blockSize - self.placeLower
        if level >= 1:
            placeZ += 0.02  # extra 2 cm of clearance for the second block and everything above it
        placeRoll = 0.0

        # Pull higher levels back toward the base (radially) to cancel the forward drift
        stackR = math.hypot(self.stackX, self.stackY)
        pull = level * self.placePullback
        scale = (stackR - pull) / stackR if stackR > 1e-6 else 1.0
        placeX, placeY = self.stackX * scale, self.stackY * scale

        self.get_logger().info(
            f'Picking {block.color} at ({bx:.3f}, {by:.3f}, {bz:.3f}), '
            f'placing at ({placeX:.3f}, {placeY:.3f}, {placeZ:.3f})')
        # Joint 5 pulse 500 = roll 0 (gripper horizontal); 500 / 120 pulses per degree, same as ik_node
        self.get_logger().info(
            f'{block.color} block yaw {math.degrees(block.yaw):.1f} deg -> roll {pickRoll:.1f} deg '
            f'(joint 5 pulse {500 + pickRoll * 500 / 120:.0f})')

        self.moveGripper(self.gripperOpen)
        self.moveArm(bx, by, bz + self.hover, pickRoll)
        self.moveArm(bx, by, bz, pickRoll)
        self.moveGripper(self.gripperClosed)               
        self.moveArm(bx, by, bz + self.hover, pickRoll)    
        self.moveArm(placeX, placeY, placeZ + self.hover, placeRoll)
        self.moveArm(placeX, placeY, placeZ, placeRoll)
        self.moveGripper(self.gripperOpen)
        self.moveArm(placeX, placeY, placeZ + self.hover, placeRoll)  # back up

    def run(self):
        if not self.waitForSubscribers():
            self.get_logger().error('ik_node / arm_control are not running (nobody on inverse_topic or gripper_topic).')
            return

        if self.goToLookPose:
            self.moveGripper(self.gripperOpen)
            self.moveToLookPose()

        blocks = {c: self.correct(b) for c, b in self.detectBlocks(STACK_ORDER).items()}
        if not blocks:
            self.get_logger().error('No blocks found. Nothing to stack.')
            return

        groundZ = self.graspZ if math.isnan(self.stackZ) else self.stackZ

        level = 0
        for i, color in enumerate(STACK_ORDER):
            if self.redetectEachPick and i > 0 and self.goToLookPose:
                self.moveToLookPose()
                fresh = self.detectBlocks([color])
                if color in fresh:
                    new = self.correct(fresh[color])
                    old = blocks.get(color)
                    if old is not None and math.hypot(new.x - old.x, new.y - old.y) > 0.05:
                        # Blocks do not teleport; a jump this big is a bad camera pose, not a moved block
                        self.get_logger().warn(
                            f'{color} appears to have moved {100 * math.hypot(new.x - old.x, new.y - old.y):.0f} cm '
                            f'since the first look; ignoring the new reading and using the first.')
                    else:
                        blocks[color] = new
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
