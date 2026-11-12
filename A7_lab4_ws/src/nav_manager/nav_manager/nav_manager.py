#!/usr/bin/env python3

import math
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PointStamped
from nav2_msgs.action import NavigateToPose
from rosbot_msgs.action import Pick, Place
from rosbot_msgs.msg import TargetPose
from rclpy.time import Time
import tf2_ros

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
    WAIT_POSES = 8   # waiting for the pick / place spots to be clicked in RViz
    # You can add other states to represent other behaviors

class NavManager(Node):
    def __init__(self):
        super().__init__('nav_manager')
        self.state = State.INIT # Initialize internal variable to represent state

        # The timer calls the timer_callback function 10 times a second.
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.declare_parameter('block_nav_pose', [1.0, 0.0, 0.0])    # where to stand to pick
        self.declare_parameter('block_position', [0.2, 0.0, -0.01])  # block relative to the arm
        self.declare_parameter('goal_position', [0.0, 0.0, 0.0])     # where to stand to place
        self.declare_parameter('place_position', [0.2, 0.0, -0.01])  # drop point relative to the arm
        self.declare_parameter('stow_xyz', [0.1, 0.0, 0.24])         # same as ik_node
        self.declare_parameter('stow_pitch', -60.0)  # same as ik_node
        # If true, the x/y of block_nav_pose and goal_position are replaced by two RViz
        # "Publish Point" clicks (pick spot first, place spot second). Yaw stays as given.
        self.declare_parameter('wait_for_clicks', True)
        # If true, the arrival yaw at each clicked spot is the direction of travel (robot -> pick spot,
        # then pick spot -> place spot) instead of the yaw in block_nav_pose / goal_position. The hybrid
        # planner honors the goal heading, so a heading that disagrees with the travel direction
        # makes it plan loops.
        self.declare_parameter('face_travel_direction', True)

        self.block_nav_pose = self.get_parameter('block_nav_pose').value
        self.block_position = self.get_parameter('block_position').value
        self.goal_nav_pose = self.get_parameter('goal_position').value
        self.place_position = self.get_parameter('place_position').value
        self.stow_xyz = self.get_parameter('stow_xyz').value
        self.stow_pitch = self.get_parameter('stow_pitch').value

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.pick_client = ActionClient(self, Pick, 'pick')
        self.place_client = ActionClient(self, Place, 'place')
        self.stow_pub = self.create_publisher(TargetPose, 'inverse_topic' , 10)
        self.stow_sent_at = None

        self.wait_for_clicks_enabled = self.get_parameter('wait_for_clicks').value
        self.block_click = None    # (x, y) of the first click
        self.goal_click = None     # (x, y) of the second click
        self.create_subscription(PointStamped, 'clicked_point', self.clicked_point_callback, 10)

        self.face_travel = self.get_parameter('face_travel_direction').value
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Keep track of whether bot is currently busy
        self.BUSY = False

        # Can use this finished variable to end the node once it is done
        self.finished = False

    # The timer callback handles all logic of what to do when
    def timer_callback(self):
        # Set state to BUSY while something is happening
        if self.BUSY:
            # If currently BUSY, do nothing
            return

        if self.state is State.INIT:
            self.init_stow()
          
        elif self.state is State.WAIT_POSES:
            self.wait_for_clicks()

        elif self.state is State.NAV_TO_BLOCK:
            self.send_goal(self.nav_client, self.make_nav_goal(self.block_nav_pose),
                     State.PICK, 'navigate to block')

        elif self.state is State.PICK:
            goal = Pick.Goal()
            goal.block = self.make_point(self.block_position)
            self.send_goal(self.pick_client, goal, State.NAV_TO_GOAL, 'pick',
                            feedback_cb=self.arm_feedback)

        elif self.state is State.NAV_TO_GOAL:
            self.send_goal(self.nav_client, self.make_nav_goal(self.goal_nav_pose),
                           State.PLACE, 'navigate to goal')

        elif self.state is State.PLACE:
            goal = Place.Goal()
            goal.block = self.make_point(self.place_position) #using the block as a makeshift place target pose since the data shared is the same
            self.send_goal(self.place_client, goal, State.DONE, 'place', feedback_cb=self.arm_feedback)
        
        elif self.state is State.DONE:
            self.get_logger().info("Complete.")
            self.finished = True
        
        elif self.state is State.FAILED:
            self.get_logger().error('Failed.')
            self.finished = True

    def init_stow(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.stow_sent_at is None:
            if self.stow_pub.get_subscription_count() == 0:
                self.get_logger().info('Waiting for ik_node...' , throttle_duration_sec=2.0)
                return
            msg = TargetPose(x=float(self.stow_xyz[0]), y=float(self.stow_xyz[1]),
                                     z=float(self.stow_xyz[2]), roll=0.0,
                                     pitch=float(self.stow_pitch), duration=2.0)
            self.stow_pub.publish(msg)
            self.stow_sent_at = now
            self.get_logger().info('Stowing arm...')
        elif now - self.stow_sent_at > 3.0:
            self.state = State.WAIT_POSES if self.wait_for_clicks_enabled else State.NAV_TO_BLOCK

    def clicked_point_callback(self, msg):
        # Accept clicks as soon as the node is up so an early click isn't lost
        if self.state not in (State.INIT, State.WAIT_POSES):
            return
        if msg.header.frame_id not in ('map', ''):
            self.get_logger().warn(
                f"Click is in frame '{msg.header.frame_id}', not 'map'. "
                "Set RViz Fixed Frame to 'map'. Ignoring.")
            return
        if self.block_click is None:
            self.block_click = (msg.point.x, msg.point.y)
            self.get_logger().info(f'PICK spot set: ({msg.point.x:.2f}, {msg.point.y:.2f})')
        elif self.goal_click is None:
            self.goal_click = (msg.point.x, msg.point.y)
            self.get_logger().info(f'PLACE spot set: ({msg.point.x:.2f}, {msg.point.y:.2f})')

    def wait_for_clicks(self):
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
            self.get_logger().info(
                f'Pick pose: ({self.block_nav_pose[0]:.2f}, {self.block_nav_pose[1]:.2f}, yaw {math.degrees(block_yaw):.0f} deg); '
                f'place pose: ({self.goal_nav_pose[0]:.2f}, {self.goal_nav_pose[1]:.2f}, yaw {math.degrees(goal_yaw):.0f} deg)')
            self.state = State.NAV_TO_BLOCK

    def robot_xy(self):
        """Current robot position in the map frame, or None if localization / TF isn't up yet."""
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
        if not handle.accepted:
            self.get_logger().error(f'{label}: goal rejected')
            self.state = State.FAILED
            self.BUSY = False
            return

        handle.get_result_async().add_done_callback(lambda f: self.goal_result(f, next_state, label))

    def goal_result(self, future, next_state, label):
        wrapped = future.result()
        if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'{label}: succeeded')
            self.state = next_state
        else:
            msg = getattr(wrapped.result, 'message', '')
            self.get_logger().error(f'{label}: failed( status {wrapped.status}){msg}')
            self.state = State.FAILED

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
     
def main():
    rclpy.init()
    node = NavManager()
    while rclpy.ok() and not node.finished:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()