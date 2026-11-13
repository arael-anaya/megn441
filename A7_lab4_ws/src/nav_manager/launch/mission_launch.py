import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PICK_IN_PLACE = True
START_POSE = [-1.3965, 0.1878, -0.8135]  # measured from /amcl_pose after an RViz 2D Pose Estimate
START_POSE_ONE = True
PICK_YAW = 0.0
PLACE_YAW = 3.14

BLOCK_POSITION = [0.2, 0.0, 0.00]
PLACE_POSITION = [0.0, -0.2, 0.00]

STOW_XYZ = [0.1, 0.0, 0.24]
STOW_PITCH = -60.0


FINAL_POSE = [5.45, -8.6, -0.869077]
FINAL_POSE_ONE = True
END_AT_START_YAW = True
GRIPPER_OPEN = 300
GRIPPER_CLOSED = 600



def generate_launch_description():
    nav_pkg_path = get_package_share_directory('rosbot_nav')
    
    start_nav_arg = DeclareLaunchArgument(
        'start_nav', default_value='true',
        description='Start sensors + Nav2 + RViz (set false if already running)')
    start_arm_arg = DeclareLaunchArgument(
        'start_arm', default_value='true',
        description='Start arm_control + ik_node (set false if already running)')

    pick_in_place_arg = DeclareLaunchArgument(
        'pick_in_place', default_value=str(PICK_IN_PLACE).lower(),
        description='Pick at the start spot, then drive to one drop point')
    final_pose_one_arg = DeclareLaunchArgument(
        'finalPoseOne', default_value=str(FINAL_POSE_ONE).lower(),
        description='Drive to the hardcoded FINAL_POSE instead of waiting for a drop click')

    start_pose_one_arg = DeclareLaunchArgument(
        'startPoseOne', default_value=str(START_POSE_ONE).lower(),
        description='Set the robot start pose to the hardcoded START_POSE instead of using RViz')

    nav_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg_path, 'launch', 'sensors_nav2_rviz_launch.py')),
        condition=IfCondition(LaunchConfiguration('start_nav')))

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
            'pick_in_place': ParameterValue(LaunchConfiguration('pick_in_place'), value_type=bool),
            'face_travel_direction': True,
            'block_nav_pose': [0.0, 0.0, PICK_YAW],
            'goal_position': [0.0, 0.0, PLACE_YAW],
            'block_position': BLOCK_POSITION,
            'place_position': PLACE_POSITION,
            'stow_xyz': STOW_XYZ,
            'stow_pitch': STOW_PITCH,
            'final_pose': FINAL_POSE,
            'end_at_start_yaw': END_AT_START_YAW,
            'finalPoseOne': ParameterValue(LaunchConfiguration('finalPoseOne'), value_type=bool),
            'start_pose': START_POSE,
            'startPoseOne': ParameterValue(LaunchConfiguration('startPoseOne'), value_type=bool),
        }])

    return launch.LaunchDescription([
        start_nav_arg,
        start_arm_arg,
        pick_in_place_arg,
        final_pose_one_arg,
        start_pose_one_arg,
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
