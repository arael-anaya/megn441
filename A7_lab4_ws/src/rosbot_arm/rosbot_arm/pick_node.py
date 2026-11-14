"""Look at the floor, find a block, pick it up. The arm-only version of what nav_manager does before it drives.

    ros2 launch rosbot_arm pick_launch.py

Uses the same pieces as the mission: the look pose from armConfig, the vision node's detect_blocks, the saved
calibration and ik_node's Pick action (which turns the gripper to the block and fails if it closes on nothing).
"""
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from rosbot_msgs.action import Pick
from rosbot_msgs.msg import TargetPose
from rosbot_msgs.srv import DetectBlocks

from rosbot_arm import calibration
from rosbot_arm.armConfig import LOOK_XYZ, GRASP_Z

LOOK_MOVE_TIME = 2.0
LOOK_SETTLE = 1.0
DETECT_ATTEMPTS = 10
PICK_ATTEMPTS = 4


class PickNode(Node):
    def __init__(self):
        super().__init__('pick_node')

        self.calibration, status = calibration.loadFor(LOOK_XYZ)
        (self.get_logger().info if self.calibration else self.get_logger().warn)(status)

        self.targetPosePublisher = self.create_publisher(TargetPose, 'inverse_topic', 10)
        self.detectBlocksSrv = self.create_client(DetectBlocks, 'detect_blocks')
        self.pickClient = ActionClient(self, Pick, 'pick')

    def wait(self, seconds):
        """Sleep while still letting ROS process callbacks."""
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def waitForArm(self):
        while rclpy.ok():
            if (self.targetPosePublisher.get_subscription_count() > 0 and self.detectBlocksSrv.service_is_ready()
                    and self.pickClient.server_is_ready()):
                return True
            self.get_logger().info('Waiting for ik_node (inverse_topic, pick action) and vision_node (detect_blocks)...',
                                   throttle_duration_sec=2.0)
            self.wait(0.2)
        return False

    def lookForBlock(self):
        """Move to the look pose and return the block nearest the middle of the view, or None."""
        self.targetPosePublisher.publish(TargetPose(
            x=LOOK_XYZ[0], y=LOOK_XYZ[1], z=LOOK_XYZ[2], roll=0.0, pitch=-90.0, duration=LOOK_MOVE_TIME))
        self.wait(LOOK_MOVE_TIME + LOOK_SETTLE)

        for attempt in range(DETECT_ATTEMPTS):
            future = self.detectBlocksSrv.call_async(DetectBlocks.Request())
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            if result is not None and result.blocks:
                # The camera sits about 5 cm ahead of the tool point; blocks under it are located best
                return min(result.blocks, key=lambda b: math.hypot(b.x - (LOOK_XYZ[0] + 0.05), b.y - LOOK_XYZ[1]))
            self.get_logger().warn(f'No block seen (attempt {attempt + 1}/{DETECT_ATTEMPTS}), retrying.')
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
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('ik_node rejected the pick goal (arm busy?).')
            return False
        resultFuture = handle.get_result_async()
        rclpy.spin_until_future_complete(self, resultFuture)
        wrapped = resultFuture.result()
        if wrapped.status == GoalStatus.STATUS_SUCCEEDED and wrapped.result.success:
            return True
        self.get_logger().warn(f'Pick failed: {wrapped.result.message}')
        return False

    def run(self):
        if not self.waitForArm():
            return
        for attempt in range(PICK_ATTEMPTS):
            block = self.lookForBlock()
            if block is None:
                self.get_logger().error('No block found. Giving up.')
                return
            if self.pick(block):
                self.get_logger().info('Block picked up and stowed.')
                return
            self.get_logger().warn(f'Trying again ({attempt + 2}/{PICK_ATTEMPTS})...' if attempt + 1 < PICK_ATTEMPTS
                                   else 'Out of attempts.')


def main():
    rclpy.init()
    node = PickNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
