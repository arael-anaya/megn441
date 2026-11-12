"""Everything for the pick-and-place mission in one command.

    ros2 launch nav_manager mission_launch.py

Starts: sensors + Nav2 + RViz, arm_control + ik_node (the /pick and /place action servers),
and nav_manager.

In RViz:
  1. "2D Pose Estimate"  -> tell AMCL where the robot is (the start pose).
  2. "Publish Point"     -> click the PICK spot (where the robot should stand to pick).
  3. "Publish Point"     -> click the PLACE spot (where the robot should stand to place).
nav_manager then runs the whole mission by itself.
"""
import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ---------------- hardcoded mission settings ----------------
# Yaw (radians, map frame) the robot faces when it arrives. x/y come from the RViz clicks.
PICK_YAW = 0.0
PLACE_YAW = 3.14

# Where the block is / where to drop it, relative to the arm (arm IK frame, meters).
BLOCK_POSITION = [0.2, 0.0, -0.01]
PLACE_POSITION = [0.2, 0.0, -0.01]

# Stow pose. Keep in sync with ik_node.py.
STOW_XYZ = [0.1, 0.0, 0.24]
STOW_PITCH = -60.0

# Gripper pulses. Check on your arm which one opens and which closes.
GRIPPER_OPEN = 300
GRIPPER_CLOSED = 600
# -------------------------------------------------------------


def generate_launch_description():
    nav_pkg_path = get_package_share_directory('rosbot_nav')
    
    start_nav_arg = DeclareLaunchArgument(
        'start_nav', default_value='true',
        description='Start sensors + Nav2 + RViz (set false if already running)')
    start_arm_arg = DeclareLaunchArgument(
        'start_arm', default_value='true',
        description='Start arm_control + ik_node (set false if already running)')

    nav_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg_path, 'launch', 'sensors_nav2_rviz_launch.py')),
        condition=IfCondition(LaunchConfiguration('start_nav')))

    # arm_control + ik_node (ik_node hosts the /pick and /place action servers).
    # Not full_arm_launch.py: that also starts the camera and keyboard teleop, which we don't need.
    arm_control = Node(
        package='rosbot_arm', executable='arm_control', name='arm_control', output='screen',
        condition=IfCondition(LaunchConfiguration('start_arm')))

    ik_node = Node(
        package='rosbot_arm', executable='ik_node', name='ik_node', output='screen',
        condition=IfCondition(LaunchConfiguration('start_arm')),
        parameters=[{
            'stow_xyz': STOW_XYZ,
            'stow_pitch': STOW_PITCH,
            'gripper_open': GRIPPER_OPEN,
            'gripper_closed': GRIPPER_CLOSED,
        }])

    nav_manager = Node(
        package='nav_manager',
        executable='nav_manager',
        name='nav_manager',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'wait_for_clicks': True,
            # Arrive facing the direction of travel. PICK_YAW / PLACE_YAW are only used if this is False.
            'face_travel_direction': True,
            # [x, y, yaw]; x and y are overwritten by the RViz clicks
            'block_nav_pose': [0.0, 0.0, PICK_YAW],
            'goal_position': [0.0, 0.0, PLACE_YAW],
            'block_position': BLOCK_POSITION,
            'place_position': PLACE_POSITION,
            'stow_xyz': STOW_XYZ,
            'stow_pitch': STOW_PITCH,
        }])

    return launch.LaunchDescription([
        start_nav_arg,
        start_arm_arg,
        nav_stack,
        arm_control,
        ik_node,
        nav_manager,
    ])


if __name__ == '__main__':
    ld = generate_launch_description()

    ls = launch.LaunchService()
    ls.include_launch_description(ld)
    ls.run()
