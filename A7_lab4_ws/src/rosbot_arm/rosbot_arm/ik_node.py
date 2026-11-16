import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool

from rosbot_msgs.action import Pick, Place
from rosbot_msgs.msg import TargetPose, ForwardPose, GripperControl
from rosbot_msgs.srv import GetAngles

from rosbot_arm import calibration, inverseKinematics
from rosbot_arm.armConfig import GRIPPER_ID, GRIPPER_EMPTY_MARGIN, STOW_XYZ, STOW_PITCH, gripRoll

JOINT_IDS = [1, 2, 3, 4, 5]
GRASP_PITCH = -90.0
HOVER = 0.05
MOVE_TIME = 2.0
GRIP_TIME = 1.0
SETTLE_TIME = 0.5


class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')

        self.targetPoseSubscription = self.create_subscription(TargetPose, 'inverse_topic', self.targetPoseCallback, 10)
        self.forwardPosePublisher = self.create_publisher(ForwardPose, 'forward_topic', 10)
        self.ikResultPublisher = self.create_publisher(Bool, 'ik_result', 10)
        self.gripperPublisher = self.create_publisher(GripperControl, 'gripper_topic', 10)

        self.busy = False
        self.cbGroup = ReentrantCallbackGroup()
        self.getAnglesClient = self.create_client(GetAngles, 'get_angles', callback_group=self.cbGroup)
        self.pickServer = ActionServer(
            self, Pick, 'pick',
            execute_callback=self.executePick,
            goal_callback=self.goalCallback,
            cancel_callback=self.cancelCallback,
            callback_group=self.cbGroup)
        self.placeServer = ActionServer(
            self, Place, 'place',
            execute_callback=self.executePlace,
            goal_callback=self.goalCallback,
            cancel_callback=self.cancelCallback,
            callback_group=self.cbGroup)

    def sendPose(self, pose):
        """Send the servos to a TargetPose. Returns False, sending nothing, if it cannot be reached."""
        solution = inverseKinematics.solveWithTilt(pose.x, pose.y, pose.z, pose.roll, pose.pitch)
        if solution is not None:
            pulses, pitch = solution
            if pitch != pose.pitch:
                self.get_logger().info(f'Target needed pitch {pitch} instead of {pose.pitch}.')
            self.forwardPosePublisher.publish(ForwardPose(ids=JOINT_IDS, pulses=pulses, duration=pose.duration))
            return True
        self.get_logger().warn(
            f'Target ({pose.x:.3f}, {pose.y:.3f}, {pose.z:.3f}) pitch={pose.pitch} '
            'is unreachable or outside joint limits. Not sending.')
        return False

    def targetPoseCallback(self, msg):
        self.ikResultPublisher.publish(Bool(data=self.sendPose(msg)))

    def moveTo(self, x, y, z, pitch=GRASP_PITCH, roll=0.0):
        if not self.sendPose(TargetPose(x=float(x), y=float(y), z=float(z), roll=float(roll), pitch=float(pitch),
                                        duration=MOVE_TIME)):
            return False
        time.sleep(MOVE_TIME + SETTLE_TIME)
        return True

    def gripperPulse(self):
        """Where the gripper servo really is (pulse), or None if it cannot be read."""
        for _ in range(3):
            if not self.getAnglesClient.service_is_ready():
                return None
            future = self.getAnglesClient.call_async(GetAngles.Request())
            end = time.time() + 1.5
            while not future.done() and time.time() < end:
                time.sleep(0.02)
            if not future.done():
                future.cancel()
                continue
            result = future.result()
            if result is None:
                continue
            pulses = dict(zip(result.ids, result.pulses))
            if GRIPPER_ID in pulses:
                return int(pulses[GRIPPER_ID])
        return None

    def holdingBlock(self):
        """False if the gripper closed all the way, i.e. there is nothing between the fingers."""
        pulse = self.gripperPulse()
        if pulse is None:
            self.get_logger().warn('Could not read the gripper servo; assuming it is holding the block.')
            return True
        closed = calibration.robotSettings()['gripper_closed']
        holding = pulse <= closed - GRIPPER_EMPTY_MARGIN
        self.get_logger().info(
            f'Gripper stopped at pulse {pulse} (sent to {closed}): '
            f'{"holding a block" if holding else "EMPTY, the grasp missed"}.')
        return holding

    def moveGripper(self, pulse):
        self.gripperPublisher.publish(GripperControl(pulse=int(pulse), duration=GRIP_TIME))
        time.sleep(GRIP_TIME + SETTLE_TIME)
        return True

    def stow(self):
        return self.moveTo(*STOW_XYZ, STOW_PITCH)

    def goalCallback(self, goalRequest):
        if self.busy:
            self.get_logger().warn('Arm is busy, rejecting goal.')
            return GoalResponse.REJECT
        self.busy = True
        return GoalResponse.ACCEPT

    def cancelCallback(self, goalHandle):
        return CancelResponse.ACCEPT

    def runSteps(self, goalHandle, action, steps):
        """Run (stage name, function) pairs in order, reporting each stage as feedback. Stops at the first failure."""
        result = action.Result()
        feedback = action.Feedback()
        try:
            for stage, step in steps:
                if goalHandle.is_cancel_requested:
                    goalHandle.canceled()
                    result.success = False
                    result.message = f'Cancelled before: {stage}'
                    return result
                feedback.stage = stage
                goalHandle.publish_feedback(feedback)
                if not step():
                    goalHandle.abort()
                    result.success = False
                    result.message = f'Failed during: {stage}'
                    return result

            goalHandle.succeed()
            result.success = True
            result.message = 'Done'
            return result
        finally:
            self.busy = False

    def executePick(self, goalHandle):
        b = goalHandle.request.block
        roll = gripRoll(b.x, b.y, goalHandle.request.yaw)
        self.get_logger().info(
            f'Pick at IK target ({b.x:.3f}, {b.y:.3f}, {b.z:.3f}), gripper roll {roll:.1f} deg.')
        robot = calibration.robotSettings()
        return self.runSteps(goalHandle, Pick, [
            ('opening gripper', lambda: self.moveGripper(robot['gripper_open'])),
            ('hovering over block', lambda: self.moveTo(b.x, b.y, b.z + HOVER, roll=roll)),
            ('descending', lambda: self.moveTo(b.x, b.y, b.z, roll=roll)),
            ('grasping', lambda: self.moveGripper(robot['gripper_closed'])),
            ('lifting', lambda: self.moveTo(b.x, b.y, b.z + HOVER, roll=roll)),
            ('checking the grasp', self.holdingBlock),
            ('stowing', self.stow),
        ])

    def executePlace(self, goalHandle):
        t = goalHandle.request.block
        openPulse = calibration.robotSettings()['gripper_open']
        return self.runSteps(goalHandle, Place, [
            ('moving over target', lambda: self.moveTo(t.x, t.y, t.z + HOVER)),
            ('lowering', lambda: self.moveTo(t.x, t.y, t.z)),
            ('releasing', lambda: self.moveGripper(openPulse)),
            ('retreating', lambda: self.moveTo(t.x, t.y, t.z + HOVER)),
            ('stowing', self.stow),
        ])


def main():
    rclpy.init()
    ik_node = IKNode()
    executor = MultiThreadedExecutor()
    executor.add_node(ik_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    ik_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
