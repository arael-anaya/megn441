#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from nav2_msgs.action import NavigationToPose
from rosbot_msgs.action import Pick, Place
from rosbot_msgs.msg import TargetPose

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
        self.declare_parameter('stow_pitch', -45.0)

        self.block_nav_pose = self.get_parameter('block_nav_pose').value
        self.block_position = self.get_parameter('block_position').value
        self.goal_nav_pose = self.get_parameter('goal_position').value
        self.place_position = self.get_parameter('place_position').value
        self.stow_xyz = self.get_parameter('stow_xyz').value
        self.stow_pitch = self.get_parameter('stow_pitch').value

        self.nav_client = ActionClient(self, NavigationToPose, 'navigate_to_pose')
        self.pick_client = ActionClient(self, Pick, 'pick')
        self.place_client = ActionClient(self, Place, 'place')
        self.stow_pub = self.create_publisher(TargetPose, 'inverse_topic' , 10)
        self.stow_sent_at = None

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
          
        elif self.state is State.NAV_TO_BLOCK:
            self.send_goal(self.nav_client, self.make_nav_goal(self.block_nav_pose),
                     State.PICK, 'navigate to block')

        elif self.state is State.PICK:
            # Mark as finished and quit code
            goal = Pick.Goal()
            goal.block = self.make_point(self.block_position)
            self.send_goal(self.pick_client, goal, State.NAV_TO_GOAL, 'pick',
                            feedback_cb=self.arm_feedback)

        elif self.state is State.NAV_TO_GOAL:
            self.send_goal(self.nav_client, self.make_nav_goal(self.goal_nav_pose),
                           State.PLACE, 'navigate to goal')

        elif self.state is State.PLACE:
            goal = Place.goal()
            goal.block = self.make_point(self.place_position) #using the block as a makeshift place target pose since the data shared is the same
            self.send_goal(self.place_client, goal, State.Done, 'place', feedback_cb=self.arm_feedback)
        
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
            self.state = State.NAV_TO_BLOCK

    def send_goal(self, client, goal, next_state, label, feedback_cb=None):
        if not client.server_is_ready():
            self.get_logger().info(f'Waiting for server: {label}', throttle_duration_sec=2.0)
            return
        self.BUSY = True
        self.get_logger().info(f'Sending goal: {label}')
        future = client.send_goal_async(goal,feedback_callback=feedback_cb)
        future.add_done_callback(lambda f: self.goal_reponse(f, next_state, label))

    def goal_response(self,future, next_state, label):
        handle = future.resut()
        if not handle.accepted:
            self.get_logger().error(f'{label}: goal rejected')
            self.state = State.FAILED
            self.BUY = False
            return

        handle.get_result_async().add_done_callback(lambda f: self.goal_response(f, next_state, label))

    def goal_result(self, future, next_state, label):
        wrapped = future.result()
        if wrapped.statues == GoalStatus.STATUS_SUCCEEDED:
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
        goal = NavigationToPose.goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = math.sin(yaw/2.0)
        goal.pose.pose.position.w = math.cos(yaw/2.0)

    def make_pont(self, xyz):
        return Point(x=float(xyz[0]), y = float(xyz[1]), z=float(xyz[2]))
     
def main():
    rclpy.init()
    node = NavManager()
    while rclpy.ok() and not node.finished:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()