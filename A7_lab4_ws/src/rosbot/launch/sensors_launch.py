import os
from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    orbbec_pkg_path = get_package_share_directory('orbbec_camera')
    rosbot_pkg_path = get_package_share_directory('rosbot')
    laser_filters_config = os.path.join(rosbot_pkg_path, 'config/lidar_filters_config_a1.yaml')

    sllidar_node = launch_ros.actions.Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='lidar',
        parameters=[{
            'serial_port': '/dev/ttyCH341USB0',
            'serial_baudrate': 115200,
            'frame_id': 'lidar_frame',
            'angle_compensate': True,
        }],
        remappings=[('scan', 'scan_raw')]
        )


    orbbec_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(orbbec_pkg_path, 'launch', 'dabai_dcw.launch.py')),
        launch_arguments={'camera_name': 'depth_cam', 'depth_registration': 'true', 'align_mode': 'HW'}.items(),
    )


    laser_filter_node = launch_ros.actions.Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            output='screen',
            parameters=[laser_filters_config],
            remappings=[('scan', 'scan_raw'),
                        ('scan_filtered', 'scan')]
        )


    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch the sensors RViz view')

    rviz_node = launch_ros.actions.Node(
        condition = IfCondition(LaunchConfiguration('use_rviz')),
        package = 'rviz2',
        executable = 'rviz2',
        name = 'rviz2',
        output = 'screen',
        arguments = ['-d', os.path.join(rosbot_pkg_path, 'config/sensors.rviz')]

    )

    return launch.LaunchDescription([
        use_rviz_arg,
        sllidar_node,
        orbbec_camera_launch,
        laser_filter_node,
        rviz_node
    ])


if __name__ == '__main__':
    ld = generate_launch_description()

    ls = launch.LaunchService()
    ls.include_launch_description(ld)
    ls.run()
