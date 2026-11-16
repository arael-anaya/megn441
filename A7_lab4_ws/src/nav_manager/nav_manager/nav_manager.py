#!/usr/bin/env python3

import math
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist, Point, PointStamped, PoseStamped, PoseWithCovarianceStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav2_msgs.action import NavigateToPose
from rosbot_msgs.action import Place
from rosbot_msgs.msg import TargetPose
from rosbot_msgs.srv import FindTarget
from rosbot_arm.armConfig import STOW_XYZ, STOW_PITCH, LOOK_XYZ, LOOK_PITCH, ARM_BASE_OFFSET, ARM_STANDOFF
from nav_manager.geometry import standoff_pose
from rclpy.time import Time
import tf2_ros


class State(Enum):
    INIT = 1
    NAV_TO_BLOCK = 2
    PICK = 3
    NAV_TO_GOAL = 4
    PLACE = 5
    DONE = 6
    FAILED = 7
    WAIT_POSES = 8
    IDLE = 9
    NAV_IDLE = 10
    NAV_TO_REACH = 11  # driving to where the arm can reach a block it could not from where it was
    NEXT_POSE = 12     # one block is done: take the next search pose, if any
    NAV_TO_SEARCH = 13  # driving to a search pose to look for the next block
    NAV_CLOSER = 14     # a pick failed: creeping slightly closer before retrying

class NavManager(Node):
    STOW_MAX_SENDS = 4

    def __init__(self):
        super().__init__('nav_manager')
        self.state = State.INIT

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.declare_parameter('block_nav_pose', [1.0, 0.0, 0.0])
        self.declare_parameter('goal_position', [0.0, 0.0, 0.0])
        self.declare_parameter('place_position', [0.2, 0.0, -0.01])
        self.declare_parameter('wait_for_clicks', True)

        self.declare_parameter('end_at_start_yaw', True)
        self.end_at_start_yaw = self.get_parameter('end_at_start_yaw').value
        self.start_yaw = None

        self.declare_parameter('face_travel_direction', True)

        self.declare_parameter('pick_in_place', False)
        self.pick_in_place = self.get_parameter('pick_in_place').value

        # Look for the block before picking and, if the arm cannot reach it from here, drive to where it can
        self.declare_parameter('drive_to_block', False)
        self.drive_to_block = self.get_parameter('drive_to_block').value
        self.find_client = self.create_client(FindTarget, 'find_target')  # pick_node serve:=true
        self.find_future = None
        self.reach_checked = False
        self.reach_pose = None

        # If a pick fails, creep forward pick_retry_step m along the robot's heading and try again, up to
        # pick_retries times per block (0 disables).
        self.declare_parameter('pick_retries', 2)
        self.declare_parameter('pick_retry_step', 0.1)
        self.pick_retries = int(self.get_parameter('pick_retries').value)
        self.retry_step = float(self.get_parameter('pick_retry_step').value)
        self.pick_attempts = 0
        self.closer_pose = None

        # More blocks: after the first pick-and-place, visit each of these [x, y, yaw] map poses (flat list, 3 numbers
        # each), pick a block there and bring it to the same drop-off. Empty (the default) = one block, as before.
        # Declared by type, not by an empty-list default: rclpy types an empty list as a byte array, which clashes
        # with the float list the launch file passes.
        self.declare_parameter('search_poses', Parameter.Type.DOUBLE_ARRAY)
        flat = [float(v) for v in (self.get_parameter('search_poses').value or [])]
        if len(flat) % 3 != 0:
            self.get_logger().error(f'search_poses needs 3 numbers per pose, got {len(flat)}; ignoring it.')
            flat = []
        self.search_poses = [flat[i:i + 3] for i in range(0, len(flat), 3)]
        self.pose_idx = 0
        self.searching = False  # True once the first block is done and the search poses are being visited

        # While driving to a search pose with the arm in the look pose, keep asking pick_node whether a block is in
        # view. When one is, stop and drive to where the arm can reach it (a coarse sighting: the robot is moving, so
        # PICK looks again from the new spot). A block within dropoff_exclusion m of the drop-off is the one already
        # placed there and is ignored.
        self.declare_parameter('scan_while_driving', False)
        self.declare_parameter('dropoff_exclusion', 1.0)
        self.scan_while_driving = self.get_parameter('scan_while_driving').value
        self.dropoff_exclusion = self.get_parameter('dropoff_exclusion').value
        self.scan_future = None
        self.scan_robot = None
        self.scan_reach_pose = None
        self.scan_timer = self.create_timer(0.5, self.scan_callback)

        self.declare_parameter('final_pose', [0.0, 0.0, 0.0])
        self.declare_parameter('finalPoseOne', False)
        self.final_pose = list(self.get_parameter('final_pose').value)
        self.finalPoseOne = self.get_parameter('finalPoseOne').value

        self.declare_parameter('start_pose', [0.0, 0.0, 0.0])
        self.declare_parameter('startPoseOne', False)
        self.start_pose = list(self.get_parameter('start_pose').value)
        self.startPoseOne = self.get_parameter('startPoseOne').value
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.start_pose_sent_at = None
        self.start_pose_sends = 0
        self.amcl_pose_seen = False
        self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self.amcl_pose_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))

        self.final_pose_pub = self.create_publisher(
            PoseStamped, 'final_pose_marker',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.publish_final_pose(self.final_pose)
        self.add_on_set_parameters_callback(self.on_params)

        self.block_nav_pose = self.get_parameter('block_nav_pose').value
        self.goal_nav_pose = self.get_parameter('goal_position').value
        self.place_position = self.get_parameter('place_position').value

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.pick_client = self.create_client(Trigger, 'pick_block')
        self.pick_future = None
        self.place_client = ActionClient(self, Place, 'place')
        self.stow_pub = self.create_publisher(TargetPose, 'inverse_topic' , 10)
        self.stow_sent_at = None
        self.stow_sends = 0
        self.ik_ok = None
        self.create_subscription(Bool, 'ik_result', lambda m: setattr(self, 'ik_ok', m.data), 10)

        self.active_handle = None
        self.cmd_vel_pubs = [self.create_publisher(Twist, 'cmd_vel', 10),
                             self.create_publisher(Twist, 'cmd_vel_nav', 10)]

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

        self.BUSY = False
        self.nav_retries = 0
        self.retry_after = 0.0

        self.finished = False
        self.shutting_down = False

    def timer_callback(self):
        if self.BUSY:
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
                self.set_start_pose()
                if self.start_pose_sends >= 3 and not self.amcl_pose_seen:
                    self.get_logger().warn(
                        'No map->base_link yet. Is /scan publishing and is odom->base_link up?',
                        throttle_duration_sec=5.0)
                return
            self.init_stow()
          
        elif self.state is State.WAIT_POSES:
            self.wait_for_clicks()

        elif self.state is State.NAV_TO_BLOCK:
            self.send_goal(self.nav_client, self.make_nav_goal(self.block_nav_pose),
                     State.PICK, 'navigate to block')

        elif self.state is State.PICK:
            if self.drive_to_block and not self.reach_checked:
                self.check_reach()
                return
            if self.pick_future is None:
                if not self.pick_client.service_is_ready():
                    self.get_logger().info('Waiting for pick_block (pick_node serve:=true)...', throttle_duration_sec=2.0)
                    return
                self.get_logger().info('Picking the block...')
                self.pick_future = self.pick_client.call_async(Trigger.Request())
            elif self.pick_future.done():
                result = self.pick_future.result()
                self.pick_future = None
                if result is not None and result.success:
                    self.get_logger().info(f'pick: {result.message}')
                    self.reach_checked = False
                    self.pick_attempts = 0
                    self.state = State.WAIT_POSES if self.pick_in_place else State.NAV_TO_GOAL
                else:
                    self.get_logger().error(f'pick failed: {result.message if result is not None else "no reply"}')
                    robot = self.robot_pose()
                    if self.pick_attempts < self.pick_retries and robot is not None:
                        self.pick_attempts += 1
                        rx, ry, ryaw = robot
                        self.closer_pose = [rx + self.retry_step * math.cos(ryaw),
                                            ry + self.retry_step * math.sin(ryaw), ryaw]
                        self.get_logger().warn(
                            f'Pick failed: creeping {self.retry_step:.2f} m closer and retrying '
                            f'({self.pick_attempts}/{self.pick_retries})')
                        self.reach_checked = True  # do not let the reach check drive us back to the old standoff
                        self.nav_retries = 0
                        self.state = State.NAV_CLOSER
                    else:
                        self.state = self.failure_state('pick')

        elif self.state is State.NAV_CLOSER:
            self.send_goal(self.nav_client, self.make_nav_goal(self.closer_pose),
                           State.PICK, 'navigate closer')

        elif self.state is State.NAV_TO_REACH:
            self.send_goal(self.nav_client, self.make_nav_goal(self.reach_pose),
                           State.PICK, 'navigate to reach block')

        elif self.state is State.NEXT_POSE:
            self.reach_checked = False
            self.pick_attempts = 0
            self.nav_retries = 0
            if self.pose_idx >= len(self.search_poses):
                self.get_logger().info('All search poses done.')
                self.state = State.IDLE
                return
            self.searching = True
            if self.scan_while_driving:  # drive with the camera looking at the floor
                self.stow_pub.publish(TargetPose(x=LOOK_XYZ[0], y=LOOK_XYZ[1], z=LOOK_XYZ[2], roll=0.0,
                                                 pitch=LOOK_PITCH, duration=2.0))
            x, y, yaw = self.search_poses[self.pose_idx]
            self.pose_idx += 1
            self.get_logger().info(
                f'Search pose {self.pose_idx}/{len(self.search_poses)}: ({x:.2f}, {y:.2f}, yaw {math.degrees(yaw):.0f} deg)')
            self.state = State.NAV_TO_SEARCH

        elif self.state is State.NAV_TO_SEARCH:
            self.send_goal(self.nav_client, self.make_nav_goal(self.search_poses[self.pose_idx - 1]),
                           State.PICK, 'navigate to search pose')

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
            goal.block = self.make_point(self.place_position)
            self.send_goal(self.place_client, goal, State.NEXT_POSE if self.search_poses else State.IDLE, 'place',
                           feedback_cb=self.arm_feedback)

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

    def check_reach(self):
        """Ask pick_node to look for the block and say whether the arm can reach it from here. If not, plan a drive
        to where it can (NAV_TO_REACH, then PICK as usual). Called each tick from PICK until it has an answer."""
        if self.find_future is None:
            if not self.find_client.service_is_ready():
                self.get_logger().info('Waiting for find_target (pick_node serve:=true)...', throttle_duration_sec=2.0)
                return
            self.get_logger().info('Looking for the block to check it is in reach...')
            self.find_future = self.find_client.call_async(FindTarget.Request(kind='block'))
            return
        if not self.find_future.done():
            return
        result = self.find_future.result()
        self.find_future = None
        self.reach_checked = True
        if result is None or not result.found:
            self.get_logger().error(f'find block failed: {result.message if result is not None else "no reply"}')
            self.state = self.failure_state('find')
            return
        self.get_logger().info(f'find: {result.message}')
        if result.reachable:
            return  # the next tick picks
        robot = self.robot_pose()
        if robot is None:
            self.get_logger().warn('Block is out of reach but there is no map->base_link TF to plan a drive; '
                                   'trying to pick from here.')
            return
        self.reach_pose = standoff_pose(robot, (result.x, result.y), ARM_STANDOFF, ARM_BASE_OFFSET)
        self.get_logger().info(
            f'Driving to ({self.reach_pose[0]:.2f}, {self.reach_pose[1]:.2f}, '
            f'yaw {math.degrees(self.reach_pose[2]):.0f} deg) to reach it')
        self.state = State.NAV_TO_REACH

    def scan_callback(self):
        """Runs alongside the timer_callback (which is idle while a drive is in progress)."""
        if not self.scan_while_driving:
            return
        if self.state is not State.NAV_TO_SEARCH or self.active_handle is None or self.scan_reach_pose is not None:
            self.scan_future = None  # a late answer from an earlier drive is of no use
            return
        if self.scan_future is None:
            self.scan_robot = self.robot_pose()  # where the robot was when it looked
            if self.scan_robot is None or not self.find_client.service_is_ready():
                return
            self.scan_future = self.find_client.call_async(FindTarget.Request(kind='block', keep_arm=True, no_raise=True))
            return
        if not self.scan_future.done():
            return
        result = self.scan_future.result()
        self.scan_future = None
        if result is None or not result.found:
            return
        rx, ry, ryaw = self.scan_robot
        ax, ay = result.x + ARM_BASE_OFFSET[0], result.y + ARM_BASE_OFFSET[1]
        bx = rx + math.cos(ryaw) * ax - math.sin(ryaw) * ay
        by = ry + math.sin(ryaw) * ax + math.cos(ryaw) * ay
        if math.hypot(bx - self.goal_nav_pose[0], by - self.goal_nav_pose[1]) < self.dropoff_exclusion:
            return  # the block already in the drop-off
        self.get_logger().info(f'Block spotted while driving at map ({bx:.2f}, {by:.2f}); stopping to approach it.')
        self.scan_reach_pose = standoff_pose(self.scan_robot, (result.x, result.y), ARM_STANDOFF, ARM_BASE_OFFSET)
        self.active_handle.cancel_goal_async()

    def failure_state(self, label):
        """Where to go when `label` failed. A failed idle drive goes back to idle. While visiting search poses, a
        failure before a block is in the gripper (drive, find or pick) skips to the next pose; anything else, such
        as the drop-off with a block in hand, stops the mission as before."""
        if label == 'navigate to clicked point':
            return State.IDLE
        if self.searching and label in ('navigate to search pose', 'navigate to reach block', 'navigate closer',
                                       'find', 'pick'):
            self.get_logger().warn(f'{label} failed: skipping to the next search pose.')
            return State.NEXT_POSE
        return State.FAILED

    def robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', Time())
        except tf2_ros.TransformException:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (t.transform.translation.x, t.transform.translation.y, yaw)

    def amcl_pose_callback(self, msg):
        if self.start_pose_sends > 0:
            self.amcl_pose_seen = True

    def set_start_pose(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.initial_pose_pub.get_subscription_count() == 0:
            self.get_logger().info('Waiting for AMCL to subscribe to initialpose...', throttle_duration_sec=2.0)
            return
        if self.start_pose_sent_at is not None and now - self.start_pose_sent_at < 3.0:
            return
        self.start_pose_sent_at = now
        self.start_pose_sends += 1
        x, y, yaw = (float(v) for v in self.start_pose)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.02
        msg.pose.covariance[35] = 0.01
        self.initial_pose_pub.publish(msg)
        self.get_logger().info(f'Set start pose {self.start_pose} (send #{self.start_pose_sends}); '
                               'waiting for map->base_link TF')

    def init_stow(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.stow_sent_at is None or (now - self.stow_sent_at > 1.5 and self.stow_sends < self.STOW_MAX_SENDS
                                         and (self.ik_ok is not True or self.stow_sends < 2)):
            if self.stow_pub.get_subscription_count() == 0:
                self.get_logger().info('Waiting for ik_node...' , throttle_duration_sec=2.0)
                return
            self.ik_ok = None
            self.stow_pub.publish(TargetPose(x=STOW_XYZ[0], y=STOW_XYZ[1], z=STOW_XYZ[2], roll=0.0,
                                             pitch=STOW_PITCH, duration=2.0))
            self.stow_sent_at = now
            self.stow_sends += 1
            self.get_logger().info(f'Stowing arm (send #{self.stow_sends})...')
        elif now - self.stow_sent_at > 3.0:
            if self.ik_ok is False:
                self.get_logger().error('ik_node rejected the stow pose (unreachable?); continuing anyway.')
            if self.pick_in_place:
                self.state = State.PICK
            else:
                self.state = State.WAIT_POSES if self.wait_for_clicks_enabled else State.NAV_TO_BLOCK

    def clicked_point_callback(self, msg):
        accepted = (State.INIT, State.WAIT_POSES, State.PICK) if self.pick_in_place \
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
            self.state = self.failure_state(label)
            self.BUSY = False
            return

        self.active_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self.goal_result(f, next_state, label))

    def goal_result(self, future, next_state, label):
        wrapped = future.result()
        self.active_handle = None
        if self.shutting_down:
            return
        if self.scan_reach_pose is not None and label == 'navigate to search pose':
            self.reach_pose, self.scan_reach_pose = self.scan_reach_pose, None
            self.reach_checked = False
            self.state = State.NAV_TO_REACH
            self.BUSY = False
            return
        if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'{label}: succeeded')
            self.state = next_state
        else:
            msg = getattr(wrapped.result, 'message', '')
            self.get_logger().error(f'{label}: failed (status {wrapped.status}) {msg}')
            self.state = self.failure_state(label)

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
        self.BUSY = True
        handle = self.active_handle
        cancel_future = handle.cancel_goal_async() if handle is not None else None
        stop = Twist()
        deadline = time.time() + 2.0
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
