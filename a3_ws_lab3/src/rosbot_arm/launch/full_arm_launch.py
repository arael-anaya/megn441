"""Entire arm control stack in one command: camera, arm_control, ik_node, teleop_arm.

    ros2 launch rosbot_arm full_arm_launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory

import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    orbbec_pkg_path = get_package_share_directory('orbbec_camera')

    start_camera_arg = DeclareLaunchArgument('start_camera', default_value='true')

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(orbbec_pkg_path, 'launch', 'dabai_dcw.launch.py')),
        launch_arguments={'camera_name': 'depth_cam', 'depth_registration': 'true'}.items(),
        condition=IfCondition(LaunchConfiguration('start_camera')))

    arm_control_node = Node(
        package='rosbot_arm', executable='arm_control', name='arm_control', output='screen')

    ik_node = Node(
        package='rosbot_arm', executable='ik_node', name='ik_node', output='screen')

    teleop_arm_node = Node(
        package='rosbot_arm', executable='teleop_arm', name='teleop_arm', output='screen')

    return launch.LaunchDescription([
        start_camera_arg,
        camera,
        arm_control_node,
        ik_node,
        teleop_arm_node,
    ])
