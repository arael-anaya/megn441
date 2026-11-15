import math
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from rosbot_msgs.msg import TargetPose, GripperControl
from rosbot_msgs.srv import DetectBlocks

from rosbot_arm import calibration
from rosbot_arm.armConfig import (BLOCK_SIZE, CAMERA_AHEAD, GRIPPER_OPEN, GRIPPER_CLOSED, LOOK_XYZ, GRASP_Z,
                                  REACH_ERROR, towardBase, gripRoll)

STACK_ORDER = ['red', 'green', 'blue']
STACK_XY = (0.0, -0.20)
HOVER = 0.06
TRAVEL_Z = 0.14

STACK_LEAN = 0.01
RELEASE_GAP = 0.01
PLACE_LOWER = 0.008

MOVE_TIME = 1.0
SETTLE_TIME = 0.5
LOOK_SETTLE = 1.0
DETECT_ATTEMPTS = 10


class UnreachableError(Exception):
    """The IK node refused a target pose."""


class StackerNode(Node):
    def __init__(self, nodeName='stacker_node', applyCalibration=True):
        super().__init__(nodeName)

        self.declare_parameter('look_xyz', LOOK_XYZ)
        self.declare_parameter('look_duration', 1.25)
        self.declare_parameter('grasp_z', GRASP_Z)

        self.lookXyz = [float(v) for v in self.get_parameter('look_xyz').value]
        self.lookDuration = float(self.get_parameter('look_duration').value)
        self.graspZ = float(self.get_parameter('grasp_z').value)

        self.correctDetections = applyCalibration
        self.calibration = None
        if applyCalibration:
            self.calibration, status = calibration.loadFor(self.lookXyz)
            (self.get_logger().info if self.calibration else self.get_logger().warn)(status)

        self.detectBlocksSrv = self.create_client(DetectBlocks, 'detect_blocks')
        self.stackXy = STACK_XY

        self.targetPosePublisher = self.create_publisher(TargetPose, 'inverse_topic', 10)
        self.gripperControlPublisher = self.create_publisher(GripperControl, 'gripper_topic', 10)

        self.ikResult = None
        self.ikResultSubscription = self.create_subscription(Bool, 'ik_result', self.ikResultCallback, 10)

    def ikResultCallback(self, msg):
        self.ikResult = msg.data

    def correct(self, block):
        """Turn a detection (where the camera sees the block) into the IK target that puts the gripper on it."""
        if not self.correctDetections:
            return block
        rawX, rawY = block.x, block.y
        block.x, block.y, block.yaw = calibration.pickTarget(self.calibration, block.x, block.y, block.yaw)
        self.get_logger().info(
            f'{block.color}: camera ({rawX:.3f}, {rawY:.3f}) -> IK target ({block.x:.3f}, {block.y:.3f})')
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
        self.get_logger().error('ik_node / arm_control are not running (nobody on inverse_topic or gripper_topic).')
        return False

    def detectBlocks(self, wanted, height=0.0, correct=True):
        """Ask the vision node for blocks until every color in `wanted` is seen or the attempts run out.
        `height` is how far above the floor their undersides are (a stack). Returns {color: BlockDetection} in arm
        coordinates, possibly missing some colors; `correct` turns them into IK targets for picking."""
        while not self.detectBlocksSrv.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                return {}
            self.get_logger().info('Waiting for detect_blocks service...')

        blocks = {}
        for attempt in range(DETECT_ATTEMPTS):
            future = self.detectBlocksSrv.call_async(DetectBlocks.Request(colors=list(wanted), height=float(height)))
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            if result is not None:
                blocks.update({b.color: b for b in result.blocks if b.color in wanted})
            missing = [c for c in wanted if c not in blocks]
            if not missing:
                break
            self.get_logger().warn(
                f'Not seen yet: {missing} (attempt {attempt + 1}/{DETECT_ATTEMPTS}), retrying...')
            self.wait(0.5)
        return {color: self.correct(b) if correct else b for color, b in blocks.items()}

    def moveGripper(self, pulse):
        msg = GripperControl()
        msg.pulse = int(pulse)
        msg.duration = 0.4
        self.gripperControlPublisher.publish(msg)
        self.wait(msg.duration + SETTLE_TIME)

    def sendPose(self, x, y, z, rollDeg, duration):
        """Command the tool to exactly (x, y, z), pointing straight down, and wait until it gets there."""
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

        end = time.time() + 2.0
        while rclpy.ok() and self.ikResult is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.ikResult is False:
            raise UnreachableError(f'IK rejected target ({x:.3f}, {y:.3f}, {z:.3f})')

        self.wait(duration + SETTLE_TIME)

    def moveArm(self, x, y, z, rollDeg):
        """Put the gripper at the real-world point (x, y, z): the commanded pose is pulled in by the arm's reach
        error. For places that are given in real coordinates (the stack)."""
        self.sendPose(*towardBase(x, y, REACH_ERROR), z, rollDeg, MOVE_TIME)

    def moveRaw(self, x, y, z, rollDeg):
        """Send the IK target (x, y, z) as is. For detections, which correct() already turned into IK targets."""
        self.sendPose(x, y, z, rollDeg, MOVE_TIME)

    def moveToLookPose(self):
        self.sendPose(*self.lookXyz, 0.0, self.lookDuration)
        self.wait(LOOK_SETTLE)

    def touchDown(self, x, y, z, rollDeg, atBottom, move=None):
        """Hover over (x, y), go down to z, do `atBottom`, come back up. `move` is moveArm (default) or moveRaw."""
        move = move or self.moveArm
        move(x, y, z + HOVER, rollDeg)
        move(x, y, z, rollDeg)
        atBottom()
        move(x, y, z + HOVER, rollDeg)

    def lookAtStack(self, color, level):
        """Right after placing: where the stack really is, (x, y), from the block just put on top (`level` blocks
        were under it), seen from above with the gripper empty. Blocks drift as they are placed; the next one goes
        where this one ended up. Keeps the old position if it cannot be seen."""
        x, y = towardBase(*STACK_XY, CAMERA_AHEAD)
        self.sendPose(x, y, self.lookXyz[2] + (level + 1) * BLOCK_SIZE, 0.0, self.lookDuration)
        self.wait(LOOK_SETTLE)
        top = self.detectBlocks([color], height=level * BLOCK_SIZE, correct=False).get(color)
        if top is None or math.hypot(top.x - STACK_XY[0], top.y - STACK_XY[1]) > 0.05:
            self.get_logger().warn(f'The {color} block on the stack was not seen near {STACK_XY}; keeping the old '
                                   f'stack position {self.stackXy}.')
            return
        self.stackXy = (top.x, top.y)
        self.get_logger().info(
            f'Stack seen at ({top.x:.3f}, {top.y:.3f}), {1000 * (top.x - STACK_XY[0]):+.0f}, '
            f'{1000 * (top.y - STACK_XY[1]):+.0f} mm from where it should be.')

    def pickAndPlace(self, block, level):
        pickRoll = gripRoll(block.x, block.y, block.yaw)
        placeX, placeY = towardBase(*self.stackXy, level * STACK_LEAN)
        placeZ = self.graspZ + level * BLOCK_SIZE + (RELEASE_GAP if level else -RELEASE_GAP) - PLACE_LOWER
        travelZ = max(TRAVEL_Z, placeZ + HOVER)

        self.get_logger().info(
            f'Picking {block.color} at ({block.x:.3f}, {block.y:.3f}, {self.graspZ:.3f}) roll {pickRoll:.1f} deg '
            f'(block yaw {math.degrees(block.yaw):.1f}), placing at ({placeX:.3f}, {placeY:.3f}, {placeZ:.3f})')

        self.moveGripper(GRIPPER_OPEN)
        self.touchDown(block.x, block.y, self.graspZ, pickRoll, lambda: self.moveGripper(GRIPPER_CLOSED),
                       move=self.moveRaw)
        self.moveRaw(block.x, block.y, travelZ, pickRoll)
        self.moveArm(placeX, placeY, travelZ, 0.0)
        self.touchDown(placeX, placeY, placeZ, 0.0, lambda: self.moveGripper(GRIPPER_OPEN))
        self.lookAtStack(block.color, level)

    def lookAgain(self, color, first):
        """Find one block again right before picking it, in case it was bumped. Returns its detection or None."""
        self.moveToLookPose()
        new = self.detectBlocks([color]).get(color)
        if new is None:
            if first is not None:
                self.get_logger().warn(f'{color} not seen again; using where it was first seen.')
            return first
        if first is not None and math.hypot(new.x - first.x, new.y - first.y) > 0.05:
            self.get_logger().warn(
                f'{color} appears to have moved {100 * math.hypot(new.x - first.x, new.y - first.y):.0f} cm '
                f'since the first look; ignoring the new reading and using the first.')
            return first
        return new

    def run(self):
        if not self.waitForSubscribers():
            return

        self.moveGripper(GRIPPER_OPEN)
        self.moveToLookPose()
        blocks = self.detectBlocks(STACK_ORDER)
        if not blocks:
            self.get_logger().error('No blocks found. Nothing to stack.')
            return

        level = 0
        for color in STACK_ORDER:
            block = self.lookAgain(color, blocks.get(color))
            if block is None:
                self.get_logger().warn(f'{color} block not detected, skipping it.')
                continue
            try:
                self.pickAndPlace(block, level)
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
