import os
from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Find the location of the package on your computer

    orbbec_pkg_path = get_package_share_directory('orbbec_camera')
    rosbot_pkg_path = get_package_share_directory('rosbot')
    laser_filters_config = os.path.join(rosbot_pkg_path, 'config/lidar_filters_config_a1.yaml')

    # TODO: Launch your sllidar_node, adding parameters options
    sllidar_node = launch_ros.actions.Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='lidar',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,  
            'frame_id': 'laser',
            'angle_compensate': True,
        }],
        # Don't edit the remapping.
        # It is needed for the lidar filter included below.
        remappings=[('scan', 'scan_raw')]
        )


    # TODO: Launch dabai_dcw.launch.py from orbbec camera
    # This is the same as running
    # `ros2 launch example_pkg example_launch.py`
    orbbec_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(orbbec_pkg_path, 'launch', 'dabai_dcw.launch.py')),
        launch_arguments={'depth_cam': 'depth_cam'}.items(),
    )


    laser_filter_node = launch_ros.actions.Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            output='screen',
            parameters=[laser_filters_config],
            remappings=[('scan', 'scan_raw'),
                        ('scan_filtered', 'scan')]
        )

    # TODO: Launch rviz. 

    rviz_node = launch_ros.actions.Node(
        package = 'rviz2',
        executable = 'rviz2',
        name = 'rviz2',
        output = 'screen',
        arguments = ['-d', os.path.join(rosbot_pkg_path, 'config/sensors.rviz')]

    )

    # Your final returned LaunchDescription includes 
    # a list of all of the nodes, launch files, etc.
    return launch.LaunchDescription([
        sllidar_node,
        orbbec_camera_launch,
        laser_filter_node,
        rviz_node
    ])


if __name__ == '__main__':
    # Create a LaunchDescription object
    ld = generate_launch_description()

    ls = launch.LaunchService()
    ls.include_launch_description(ld)
    ls.run()
