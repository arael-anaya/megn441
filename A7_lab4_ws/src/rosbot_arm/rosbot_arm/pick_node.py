"""Look at the floor, find a block, pick it up. The one place that procedure lives.

    ros2 launch rosbot_arm pick_launch.py        # picks once, then exits
    ros2 run rosbot_arm pick_node --ros-args -p serve:=true   # stays up; call the pick_block service (nav_manager does)

Uses the look pose from armConfig, the vision node's detect_blocks, the saved calibration and ik_node's Pick action
(which turns the gripper to the block and fails if it closes on nothing).
"""
import math
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from rosbot_msgs.action import Pick
from rosbot_msgs.msg import TargetPose
from rosbot_msgs.srv import DetectBlocks, FindTarget
from std_srvs.srv import Trigger

from rosbot_arm import calibration, inverseKinematics
from rosbot_arm.armConfig import LOOK_XYZ, LOOK_PITCH, GRASP_Z, gripRoll
from rosbot_arm.ik_node import HOVER, GRASP_PITCH

LOOK_MOVE_TIME = 2.0
LOOK_SETTLE = 1.0
DETECT_ATTEMPTS = 10
PICK_ATTEMPTS = 4


class PickNode(Node):
    def __init__(self):
        super().__init__('pick_node')

        self.calibration, status = calibration.loadFor(LOOK_XYZ)
        (self.get_logger().info if self.calibration else self.get_logger().warn)(status)

        self.declare_parameter('serve', False)
        self.serve = self.get_parameter('serve').value
        # Block colors to pick. One color is requested from vision by name (it backs the arm up for a wider view if
        # that color is missing). Several are requested as "any" and filtered here, so one missing color does not.
        self.declare_parameter('colors', ['red'])
        self.colors = list(self.get_parameter('colors').value)

        clients = ReentrantCallbackGroup()
        self.targetPosePublisher = self.create_publisher(TargetPose, 'inverse_topic', 10)
        self.detectBlocksSrv = self.create_client(DetectBlocks, 'detect_blocks', callback_group=clients)
        self.pickClient = ActionClient(self, Pick, 'pick', callback_group=clients)
        if self.serve:
            # One group for both services: the arm and the camera are used by one request at a time
            serving = MutuallyExclusiveCallbackGroup()
            self.create_service(Trigger, 'pick_block', self.pickBlockCallback, callback_group=serving)
            self.create_service(FindTarget, 'find_target', self.findTargetCallback, callback_group=serving)

    def pickBlockCallback(self, request, response):
        response.success, response.message = self.run()
        return response

    def findTargetCallback(self, request, response):
        if request.kind != 'block':
            response.message = f"Unknown kind '{request.kind}' (only 'block' so far)."
            return response
        if not self.waitForArm():
            response.message = 'Shut down while waiting for ik_node / vision_node.'
            return response
        if not request.keep_arm:
            self.moveToLook()
        block = self.detectBlock(1 if request.keep_arm else DETECT_ATTEMPTS, request.no_raise)
        if block is None:
            response.message = 'No block found.'
            return response
        response.found = True
        response.x, response.y, response.yaw = float(block.x), float(block.y), float(block.yaw)
        response.reachable = self.canPick(block)
        response.message = (f'Block at arm ({block.x:.3f}, {block.y:.3f}); '
                            f'{"reachable" if response.reachable else "out of reach"}.')
        return response

    def canPick(self, block):
        """True if ik_node would accept both poses of a pick of this block from where the arm is."""
        x, y, yaw = calibration.pickTarget(self.calibration, block.x, block.y, block.yaw)
        roll = gripRoll(x, y, yaw)
        return all(inverseKinematics.solveWithTilt(x, y, z, roll, GRASP_PITCH) is not None
                   for z in (GRASP_Z + HOVER, GRASP_Z))

    def wait(self, seconds):
        time.sleep(seconds)

    def waitFor(self, future):
        """Block until a future is done (the executor in main() delivers the reply); None if shut down first."""
        while rclpy.ok() and not future.done():
            time.sleep(0.02)
        return future.result() if future.done() else None

    def waitForArm(self):
        while rclpy.ok():
            if (self.targetPosePublisher.get_subscription_count() > 0 and self.detectBlocksSrv.service_is_ready()
                    and self.pickClient.server_is_ready()):
                return True
            self.get_logger().info('Waiting for ik_node (inverse_topic, pick action) and vision_node (detect_blocks)...',
                                   throttle_duration_sec=2.0)
            self.wait(0.2)
        return False

    def moveToLook(self):
        """Put the arm in the look pose and wait for it to settle."""
        self.targetPosePublisher.publish(TargetPose(
            x=LOOK_XYZ[0], y=LOOK_XYZ[1], z=LOOK_XYZ[2], roll=0.0, pitch=LOOK_PITCH, duration=LOOK_MOVE_TIME))
        self.wait(LOOK_MOVE_TIME + LOOK_SETTLE)

    def lookForBlock(self):
        """Move to the look pose and return the block nearest the middle of the view, or None."""
        self.moveToLook()
        return self.detectBlock()

    def detectBlock(self, attempts=DETECT_ATTEMPTS, noRaise=False):
        """The block nearest the middle of the view from where the arm is now, or None. Does not move the arm
        (unless vision backs it up for a wider view, which noRaise prevents)."""
        for attempt in range(attempts):
            result = self.waitFor(self.detectBlocksSrv.call_async(
                DetectBlocks.Request(colors=self.colors if len(self.colors) == 1 else [], no_raise=noRaise)))
            blocks = [b for b in result.blocks if b.color in self.colors] if result is not None else []
            if blocks:
                return min(blocks, key=lambda b: math.hypot(b.x - (LOOK_XYZ[0] + 0.05), b.y - LOOK_XYZ[1]))
            if attempt + 1 < attempts:
                self.get_logger().warn(f'No block seen (attempt {attempt + 1}/{attempts}), retrying.')
                self.wait(1.0)
        return None

    def pick(self, block):
        x, y, yaw = calibration.pickTarget(self.calibration, block.x, block.y, block.yaw)
        self.get_logger().info(
            f'Picking the {block.color} block: camera ({block.x:.3f}, {block.y:.3f}) -> IK target ({x:.3f}, {y:.3f}), '
            f'block yaw {math.degrees(yaw):.0f} deg')
        goal = Pick.Goal()
        goal.block = Point(x=float(x), y=float(y), z=float(GRASP_Z))
        goal.yaw = float(yaw)

        future = self.pickClient.send_goal_async(
            goal, feedback_callback=lambda m: self.get_logger().info(f' arm stage: {m.feedback.stage}'))
        handle = self.waitFor(future)
        if handle is None or not handle.accepted:
            self.get_logger().error('ik_node rejected the pick goal (arm busy?).')
            return False
        resultFuture = handle.get_result_async()
        wrapped = self.waitFor(resultFuture)
        if wrapped is None:
            return False
        if wrapped.status == GoalStatus.STATUS_SUCCEEDED and wrapped.result.success:
            return True
        self.get_logger().warn(f'Pick failed: {wrapped.result.message}')
        return False

    def run(self):
        """Look, find a red block and pick it, retrying. Returns (success, message)."""
        if not self.waitForArm():
            return False, 'Shut down while waiting for ik_node / vision_node.'
        for attempt in range(PICK_ATTEMPTS):
            block = self.lookForBlock()
            if block is None:
                return False, 'No block found.'
            if self.pick(block):
                return True, 'Block picked up and stowed.'
            self.get_logger().warn(f'Trying again ({attempt + 2}/{PICK_ATTEMPTS})...' if attempt + 1 < PICK_ATTEMPTS
                                   else 'Out of attempts.')
        return False, 'Out of pick attempts.'


def main():
    rclpy.init()
    node = PickNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        if node.serve:
            executor.spin()
        else:
            threading.Thread(target=executor.spin, daemon=True).start()
            ok, message = node.run()
            (node.get_logger().info if ok else node.get_logger().error)(message)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
