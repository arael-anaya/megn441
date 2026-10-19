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
    vision_pkg_path = get_package_share_directory('rosbot_vision')
    vision_params = os.path.join(vision_pkg_path, 'config', 'vision_params.yaml')

    start_camera_arg = DeclareLaunchArgument('start_camera', default_value='false')
    start_arm_control_arg = DeclareLaunchArgument('start_arm_control', default_value='false')

    orbbec_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(orbbec_pkg_path, 'launch', 'dabai_dcw.launch.py')),
        launch_arguments={'camera_name': 'depth_cam', 'depth_registration': 'true'}.items(),
        condition=IfCondition(LaunchConfiguration('start_camera')),
    )

    arm_control_node = Node(
        package='rosbot_arm',
        executable='arm_control',
        name='arm_control',
        condition=IfCondition(LaunchConfiguration('start_arm_control')))

    ik_node = Node(
        package='rosbot_arm',
        executable='ik_node',
        name='ik_node')

    vision_node = Node(
        package='rosbot_vision',
        executable='vision_node',
        name='vision_node',
        output='screen',
        parameters=[vision_params])

   
    pick_node = Node(
        package='rosbot_arm',
        executable='pick_node',
        name='pick_node',
        output='screen',
        parameters=[{
            'look_xyz': [0.18, 0.0, 0.22],
            'open_pulse': 600,
            'close_pulse': 300,
            'hover_offset': 0.08,
            'lift_offset': 0.10,
        }])

    return launch.LaunchDescription([
        start_camera_arg,
        start_arm_control_arg,
        orbbec_camera_launch,
        arm_control_node,
        ik_node,
        vision_node,
        pick_node,
    ])


if __name__ == '__main__':
    ld = generate_launch_description()

    ls = launch.LaunchService()
    ls.include_launch_description(ld)
    ls.run()
