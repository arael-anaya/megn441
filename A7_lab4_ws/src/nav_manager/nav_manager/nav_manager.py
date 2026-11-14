#!/usr/bin/env python3

import math
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Point, PointStamped, PoseStamped, PoseWithCovarianceStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav2_msgs.action import NavigateToPose
from rosbot_msgs.action import Pick, Place
from rosbot_msgs.msg import TargetPose
from rosbot_msgs.srv import DetectBlocks
from rosbot_arm import calibration
from rosbot_arm.armConfig import STOW_XYZ, STOW_PITCH, LOOK_XYZ
from rclpy.time import Time
import tf2_ros

LOOK_MOVE_TIME = 2.0      # seconds the arm takes to reach the look pose
LOOK_SETTLE = 1.0         # extra wait so the arm has stopped swaying; the vision node waits for the move itself
DETECT_RETRY_DELAY = 1.5
DETECT_MAX_ATTEMPTS = 10
PICK_MAX_ATTEMPTS = 4     # look + grab tries before giving up (ik_node fails a pick whose gripper closed on nothing)


# An enum allows you to use words instead of arbitrary
# numbers to represent your states
class State(Enum):
    INIT = 1
    NAV_TO_BLOCK = 2
    PICK = 3
    NAV_TO_GOAL = 4
    PLACE = 5
    DONE = 6
    FAILED = 7
    WAIT_POSES = 8
    IDLE = 9          # after placing: wait for clicked points
    NAV_IDLE = 10     # driving to a point clicked while idle
    LOOK = 11         # arm moves to the block-capture pose
    DETECT = 12       # ask the vision node where the red block is
    # You can add other states to represent other behaviors

class NavManager(Node):
    def __init__(self):
        super().__init__('nav_manager')
        self.state = State.INIT # Initialize internal variable to represent state

        # The timer calls the timer_callback function 10 times a second.
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.declare_parameter('block_nav_pose', [1.0, 0.0, 0.0])
        self.declare_parameter('block_position', [0.2, 0.0, -0.01])
        self.declare_parameter('goal_position', [0.0, 0.0, 0.0])
        self.declare_parameter('place_position', [0.2, 0.0, -0.01])
        self.declare_parameter('wait_for_clicks', True)

        self.declare_parameter('end_at_start_yaw', True)
        self.end_at_start_yaw = self.get_parameter('end_at_start_yaw').value
        self.start_yaw = None

        self.declare_parameter('face_travel_direction', True)

        self.declare_parameter('pick_in_place', False)
        self.pick_in_place = self.get_parameter('pick_in_place').value

        self.declare_parameter('final_pose', [0.0, 0.0, 0.0])
        self.declare_parameter('finalPoseOne', False)
        self.final_pose = list(self.get_parameter('final_pose').value)
        self.finalPoseOne = self.get_parameter('finalPoseOne').value

        self.declare_parameter('start_pose', [0.0, 0.0, 0.0])
        self.declare_parameter('startPoseOne', False)
        self.start_pose = list(self.get_parameter('start_pose').value)
        self.startPoseOne = self.get_parameter('startPoseOne').value
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)

        self.final_pose_pub = self.create_publisher(
            PoseStamped, 'final_pose_marker',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.publish_final_pose(self.final_pose)
        self.add_on_set_parameters_callback(self.on_params)

        self.block_nav_pose = self.get_parameter('block_nav_pose').value
        self.block_position = self.get_parameter('block_position').value
        self.goal_nav_pose = self.get_parameter('goal_position').value
        self.place_position = self.get_parameter('place_position').value

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.pick_client = ActionClient(self, Pick, 'pick')
        self.place_client = ActionClient(self, Place, 'place')
        self.stow_pub = self.create_publisher(TargetPose, 'inverse_topic' , 10)
        self.stow_sent_at = None

        # Track the in-flight goal so it can be cancelled on shutdown, and keep
        # direct velocity publishers as a backstop to force the robot to stop.
        self.active_handle = None
        self.cmd_vel_pubs = [self.create_publisher(Twist, 'cmd_vel', 10),
                             self.create_publisher(Twist, 'cmd_vel_nav', 10)]

        # Camera -> IK target correction measured by `ros2 launch rosbot_arm stack_launch.py mode:=calibrate`
        self.calib, status = calibration.loadFor(LOOK_XYZ)
        (self.get_logger().info if self.calib else self.get_logger().warn)(status)
        self.preset_block_position = list(self.block_position)
        self.block_yaw = 0.0
        self.pick_attempts = 0
        self.ik_ok = None
        self.create_subscription(Bool, 'ik_result', lambda m: setattr(self, 'ik_ok', m.data), 10)
        self.detect_client = self.create_client(DetectBlocks, 'detect_blocks')
        self.look_sent_at = None
        self.detect_future = None
        self.detect_attempts = 0

        self.wait_for_clicks_enabled = self.get_parameter('wait_for_clicks').value
        self.block_click = None
        self.goal_click = None
        self.idle_click = None
        self.idle_nav_pose = None
        if self.finalPoseOne:
            self.goal_click = (float(self.final_pose[0]), float(self.final_pose[1]))
            self.get_logger().info(f'finalPoseOne: using hardcoded final pose {self.final_pose}')
        self.create_subscription(PointStamped, 'clicked_point', self.clicked_point_callback, 10)

        self.face_travel = self.get_parameter('face_travel_direction').value
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Keep track of whether bot is currently busy
        self.BUSY = False
        self.nav_retries = 0
        self.retry_after = 0.0

        # Can use this finished variable to end the node once it is done
        self.finished = False
        self.shutting_down = False

    # The timer callback handles all logic of what to do when
    def timer_callback(self):
        # Set state to BUSY while something is happening
        if self.BUSY:
            # If currently BUSY, do nothing
            return

        if self.get_clock().now().nanoseconds / 1e9 < self.retry_after:
            return

        if (self.end_at_start_yaw and self.start_yaw is None
                and self.state in (State.INIT, State.WAIT_POSES, State.PICK)):
            yaw = self.robot_yaw()
            if yaw is not None:
                self.start_yaw = yaw
                self.get_logger().info(f'Start yaw (map): {math.degrees(yaw):.0f} deg')
                self.publish_final_pose(self.final_pose)

        if self.state is State.INIT:
            if self.startPoseOne and self.robot_xy() is None:
                self.set_start_pose()  # stay in INIT until AMCL accepts it and publishes map->base_link
                return
            self.init_stow()
          
        elif self.state is State.WAIT_POSES:
            self.wait_for_clicks()

        elif self.state is State.NAV_TO_BLOCK:
            self.send_goal(self.nav_client, self.make_nav_goal(self.block_nav_pose),
                     State.LOOK, 'navigate to block')

        elif self.state is State.LOOK:
            now = self.get_clock().now().nanoseconds / 1e9
            if self.look_sent_at is None:
                if self.stow_pub.get_subscription_count() == 0:
                    return
                self.stow_pub.publish(TargetPose(x=LOOK_XYZ[0], y=LOOK_XYZ[1], z=LOOK_XYZ[2], roll=0.0,
                                                 pitch=-90.0, duration=LOOK_MOVE_TIME))
                self.ik_ok = None
                self.look_sent_at = now
                self.get_logger().info(f'Moving arm to block-capture pose {LOOK_XYZ}...')
            elif self.ik_ok is False:
                self.get_logger().error(f'ik_node rejected look pose {LOOK_XYZ}; using preset block_position')
                self.look_sent_at = None
                self.use_preset_block()
                self.state = State.PICK
            elif now - self.look_sent_at > LOOK_MOVE_TIME + LOOK_SETTLE:
                self.look_sent_at = None  # so a retry moves back to the look pose
                self.detect_attempts = 0
                self.state = State.DETECT

        elif self.state is State.DETECT:
            if self.detect_future is None:
                if not self.detect_client.service_is_ready():
                    self.get_logger().info('Waiting for detect_blocks...', throttle_duration_sec=2.0)
                    return
                self.detect_future = self.detect_client.call_async(DetectBlocks.Request(colors=['red']))
            elif self.detect_future.done():
                result = self.detect_future.result()
                blocks = result.blocks if result is not None else []
                self.detect_future = None
                if blocks:
                    # Where the camera sees the block -> the IK target that puts the gripper on it
                    x, y, yaw = calibration.pickTarget(self.calib, blocks[0].x, blocks[0].y, blocks[0].yaw)
                    self.block_position = [x, y, self.preset_block_position[2]]
                    self.block_yaw = yaw
                    self.get_logger().info(
                        f'Red block: camera ({blocks[0].x:.3f}, {blocks[0].y:.3f}) -> pick ({x:.3f}, {y:.3f}), '
                        f'block yaw {math.degrees(yaw):.0f} deg')
                    self.state = State.PICK
                else:
                    self.detect_attempts += 1
                    self.get_logger().warn(
                        f'Red block not found (attempt {self.detect_attempts}/{DETECT_MAX_ATTEMPTS})')
                    if self.detect_attempts >= DETECT_MAX_ATTEMPTS:
                        self.get_logger().warn('Giving up on vision; using the preset block_position')
                        self.use_preset_block()
                        self.state = State.PICK
                    else:
                        self.retry_after = self.get_clock().now().nanoseconds / 1e9 + DETECT_RETRY_DELAY

        elif self.state is State.PICK:
            goal = Pick.Goal()
            goal.block = self.make_point(self.block_position)
            goal.yaw = float(self.block_yaw)
            after_pick = State.WAIT_POSES if self.pick_in_place else State.NAV_TO_GOAL
            self.send_goal(self.pick_client, goal, after_pick, 'pick',
                            feedback_cb=self.arm_feedback)

        elif self.state is State.NAV_TO_GOAL:
            if self.end_at_start_yaw:
                if self.start_yaw is not None:
                    self.goal_nav_pose = [self.goal_nav_pose[0], self.goal_nav_pose[1], self.start_yaw]
                else:
                    self.get_logger().warn('Start yaw unknown (no map->base_link TF); using planned yaw',
                                           throttle_duration_sec=5.0)
            self.send_goal(self.nav_client, self.make_nav_goal(self.goal_nav_pose),
                           State.PLACE, 'navigate to goal')

        elif self.state is State.PLACE:
            goal = Place.Goal()
            goal.block = self.make_point(self.place_position) #using the block as a makeshift place target pose since the data shared is the same
            self.send_goal(self.place_client, goal, State.IDLE, 'place', feedback_cb=self.arm_feedback)

        elif self.state is State.IDLE:
            if self.idle_click is None:
                self.get_logger().info('Idle. Click a point in RViz to drive there...',
                                       throttle_duration_sec=5.0)
                return
            x, y = self.idle_click
            self.idle_click = None
            yaw = None
            if self.end_at_start_yaw and self.start_yaw is not None:
                yaw = self.start_yaw
            else:
                start = self.robot_xy()
                if start is not None:
                    yaw = math.atan2(y - start[1], x - start[0])
            if yaw is None:
                yaw = 0.0
            self.idle_nav_pose = [x, y, yaw]
            self.get_logger().info(f'Driving to clicked point ({x:.2f}, {y:.2f}, yaw {math.degrees(yaw):.0f} deg)')
            self.state = State.NAV_IDLE

        elif self.state is State.NAV_IDLE:
            self.send_goal(self.nav_client, self.make_nav_goal(self.idle_nav_pose),
                           State.IDLE, 'navigate to clicked point')

        elif self.state is State.DONE:
            self.get_logger().info("Complete.")
            self.finished = True
        
        elif self.state is State.FAILED:
            self.get_logger().error('Failed.')
            self.finished = True

    def use_preset_block(self):
        self.block_position = list(self.preset_block_position)
        self.block_yaw = 0.0

    def set_start_pose(self):
        x, y, yaw = (float(v) for v in self.start_pose)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.02
        msg.pose.covariance[35] = 0.01
        self.initial_pose_pub.publish(msg)
        self.get_logger().info(f'Set start pose {self.start_pose}', throttle_duration_sec=1.0)

    def init_stow(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.stow_sent_at is None:
            if self.stow_pub.get_subscription_count() == 0:
                self.get_logger().info('Waiting for ik_node...' , throttle_duration_sec=2.0)
                return
            self.stow_pub.publish(TargetPose(x=STOW_XYZ[0], y=STOW_XYZ[1], z=STOW_XYZ[2], roll=0.0,
                                             pitch=STOW_PITCH, duration=2.0))
            self.stow_sent_at = now
            self.get_logger().info('Stowing arm...')
        elif now - self.stow_sent_at > 3.0:
            if self.pick_in_place:
                self.state = State.LOOK
            else:
                self.state = State.WAIT_POSES if self.wait_for_clicks_enabled else State.NAV_TO_BLOCK

    def clicked_point_callback(self, msg):
        accepted = (State.INIT, State.WAIT_POSES, State.LOOK, State.DETECT, State.PICK) if self.pick_in_place \
            else (State.INIT, State.WAIT_POSES)
        if self.state not in accepted + (State.IDLE,):
            return
        if msg.header.frame_id not in ('map', ''):
            self.get_logger().warn(
                f"Click is in frame '{msg.header.frame_id}', not 'map'. "
                "Set RViz Fixed Frame to 'map'. Ignoring.")
            return
        if self.state is State.IDLE:
            self.idle_click = (msg.point.x, msg.point.y)
            return
        if self.pick_in_place:
            if self.goal_click is None:
                self.goal_click = (msg.point.x, msg.point.y)
                self.get_logger().info(f'DROP spot set: ({msg.point.x:.2f}, {msg.point.y:.2f})')
        elif self.block_click is None:
            self.block_click = (msg.point.x, msg.point.y)
            self.get_logger().info(f'PICK spot set: ({msg.point.x:.2f}, {msg.point.y:.2f})')
        elif self.goal_click is None:
            self.goal_click = (msg.point.x, msg.point.y)
            self.get_logger().info(f'PLACE spot set: ({msg.point.x:.2f}, {msg.point.y:.2f})')

    def wait_for_clicks(self):
        if self.pick_in_place:
            if self.goal_click is None:
                self.get_logger().info('Click the DROP spot with Publish Point in RViz...', throttle_duration_sec=3.0)
                return
            goal_yaw = self.goal_nav_pose[2]
            if self.face_travel:
                start = self.robot_xy()
                if start is None:
                    self.get_logger().info(
                        'Waiting for map->base_link TF (set the 2D Pose Estimate in RViz)...',
                        throttle_duration_sec=3.0)
                    return
                goal_yaw = math.atan2(self.goal_click[1] - start[1], self.goal_click[0] - start[0])
            self.goal_nav_pose = [self.goal_click[0], self.goal_click[1], goal_yaw]
            if self.finalPoseOne:
                self.goal_nav_pose = list(self.final_pose)
                goal_yaw = self.final_pose[2]
            self.get_logger().info(
                f'Drop pose: ({self.goal_nav_pose[0]:.2f}, {self.goal_nav_pose[1]:.2f}, '
                f'yaw {math.degrees(goal_yaw):.0f} deg)')
            self.state = State.NAV_TO_GOAL
            return

        if self.block_click is None:
            self.get_logger().info('Click the PICK spot with Publish Point in RViz...', throttle_duration_sec=3.0)
        elif self.goal_click is None:
            self.get_logger().info('Click the PLACE spot with Publish Point in RViz...', throttle_duration_sec=3.0)
        else:
            block_yaw = self.block_nav_pose[2]
            goal_yaw = self.goal_nav_pose[2]
            if self.face_travel:
                start = self.robot_xy()
                if start is None:
                    self.get_logger().info(
                        'Waiting for map->base_link TF (set the 2D Pose Estimate in RViz)...',
                        throttle_duration_sec=3.0)
                    return
                block_yaw = math.atan2(self.block_click[1] - start[1], self.block_click[0] - start[0])
                goal_yaw = math.atan2(self.goal_click[1] - self.block_click[1],
                                      self.goal_click[0] - self.block_click[0])
            self.block_nav_pose = [self.block_click[0], self.block_click[1], block_yaw]
            self.goal_nav_pose = [self.goal_click[0], self.goal_click[1], goal_yaw]
            if self.finalPoseOne:
                self.goal_nav_pose = list(self.final_pose)
                goal_yaw = self.final_pose[2]
            self.get_logger().info(
                f'Pick pose: ({self.block_nav_pose[0]:.2f}, {self.block_nav_pose[1]:.2f}, yaw {math.degrees(block_yaw):.0f} deg); '
                f'place pose: ({self.goal_nav_pose[0]:.2f}, {self.goal_nav_pose[1]:.2f}, yaw {math.degrees(goal_yaw):.0f} deg)')
            self.state = State.NAV_TO_BLOCK

    def publish_final_pose(self, pose):
        x, y, yaw = (float(v) for v in pose)
        if self.end_at_start_yaw and self.start_yaw is not None:
            yaw = self.start_yaw
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.final_pose_pub.publish(msg)

    def on_params(self, params):
        for p in params:
            if p.name == 'final_pose' and len(p.value) == 3:
                self.final_pose = [float(v) for v in p.value]
                self.publish_final_pose(self.final_pose)
                if self.finalPoseOne:
                    self.goal_click = (self.final_pose[0], self.final_pose[1])
                    if self.state in (State.INIT, State.WAIT_POSES, State.NAV_TO_BLOCK, State.PICK):
                        self.goal_nav_pose = list(self.final_pose)
            elif p.name == 'finalPoseOne':
                self.finalPoseOne = bool(p.value)
        return SetParametersResult(successful=True)

    def robot_yaw(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', Time())
        except tf2_ros.TransformException:
            return None
        q = t.transform.rotation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', Time())
        except tf2_ros.TransformException:
            return None
        return (t.transform.translation.x, t.transform.translation.y)

    def send_goal(self, client, goal, next_state, label, feedback_cb=None):
        if not client.server_is_ready():
            self.get_logger().info(f'Waiting for server: {label}', throttle_duration_sec=2.0)
            return
        self.BUSY = True
        self.get_logger().info(f'Sending goal: {label}')
        future = client.send_goal_async(goal,feedback_callback=feedback_cb)
        future.add_done_callback(lambda f: self.goal_response(f, next_state, label))

    def goal_response(self, future, next_state, label):
        handle = future.result()
        if self.shutting_down:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            if label.startswith('navigate') and self.nav_retries < 15:
                self.nav_retries += 1
                self.retry_after = self.get_clock().now().nanoseconds / 1e9 + 2.0
                self.get_logger().warn(
                    f'{label}: goal rejected (Nav2 not active yet?), retry {self.nav_retries}/15')
                self.BUSY = False
                return
            self.get_logger().error(f'{label}: goal rejected')
            self.state = State.IDLE if label == 'navigate to clicked point' else State.FAILED
            self.BUSY = False
            return

        self.active_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self.goal_result(f, next_state, label))

    def goal_result(self, future, next_state, label):
        wrapped = future.result()
        self.active_handle = None
        if self.shutting_down:
            return
        if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'{label}: succeeded')
            self.state = next_state
        else:
            msg = getattr(wrapped.result, 'message', '')
            self.get_logger().error(f'{label}: failed (status {wrapped.status}) {msg}')
            if label == 'pick' and self.pick_attempts + 1 < PICK_MAX_ATTEMPTS:
                # The gripper came up empty (or the target was out of reach): look at the floor again and retry
                self.pick_attempts += 1
                self.get_logger().warn(
                    f'Looking for the block again (pick attempt {self.pick_attempts + 1}/{PICK_MAX_ATTEMPTS})')
                self.look_sent_at = None
                self.state = State.LOOK
            else:
                # A failed idle drive just returns to idle so another point can be clicked
                self.state = State.IDLE if label == 'navigate to clicked point' else State.FAILED

        self.BUSY = False

    def arm_feedback(self, feedback_msg):
        self.get_logger().info(f' arm stage: {feedback_msg.feedback.stage}')


    def make_nav_goal(self, pose):
        x, y, yaw = pose
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw/2.0)
        goal.pose.pose.orientation.w = math.cos(yaw/2.0)
        return goal

    def make_point(self, xyz):
        return Point(x=float(xyz[0]), y = float(xyz[1]), z=float(xyz[2]))
     
    def stop_robot(self):
        """Cancel any in-flight goal and command zero velocity so the robot halts."""
        self.shutting_down = True
        self.BUSY = True  # stop the timer from sending new goals
        handle = self.active_handle
        cancel_future = handle.cancel_goal_async() if handle is not None else None
        stop = Twist()
        deadline = time.time() + 2.0
        # Keep publishing zeros while the cancel propagates, so the controller /
        # velocity smoother can't leave the last forward command latched.
        while time.time() < deadline:
            for pub in self.cmd_vel_pubs:
                pub.publish(stop)
            try:
                rclpy.spin_once(self, timeout_sec=0.05)
            except Exception:
                break
            if cancel_future is None and time.time() > deadline - 1.5:
                break
            if cancel_future is not None and cancel_future.done() and time.time() > deadline - 1.5:
                break
        for _ in range(5):
            for pub in self.cmd_vel_pubs:
                pub.publish(stop)


def main():
    # Keep the context alive on Ctrl-C so we can still publish stop commands.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = NavManager()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().warn('Interrupted: cancelling goals and stopping robot')
    finally:
        try:
            node.stop_robot()
        except Exception as e:
            print(f'stop_robot failed: {e}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
