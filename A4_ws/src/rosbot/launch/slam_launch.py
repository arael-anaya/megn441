import os
from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
# ros2 run nav2_map_server map_saver_cli -f ~/basement_map
#  rsync -ruvl A4_ws ubuntu@192.168.149.1:~/
def generate_launch_description():
    rosbot_pkg_path = get_package_share_directory('rosbot')
    slam_params = os.path.join(rosbot_pkg_path, 'config/slam_params.yaml')

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rosbot_pkg_path, 'launch', 'sensors_launch.py')),
    )


    joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rosbot_pkg_path, 'launch', 'joyLaunch.py')),
    )

    laser_tf_node = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_static_tf',
        # Offsets from the Hiwonder JetRover URDF (A4_ws/src/simulations/jetrover_description/urdf):
        #   base_footprint -> base_link   z = 0.05 + 0.06549      (car_acker.urdf.xacro)
        #   base_link      -> lidar_link  x = 0.09000, z = 0.04052 (lidar.urdf.xacro)
        #   lidar_link     -> lidar_frame yaw = pi (A1 mounting)   (lidar_a1.urdf.xacro)
        arguments=['--x', '0.09000', '--y', '0.0', '--z', '0.15601',
                   '--yaw', '3.14159', '--pitch', '0.0', '--roll', '0.0',
                   '--frame-id', 'base_footprint', '--child-frame-id', 'laser'],
    )


    slam_node = launch_ros.actions.Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params],
    )

    return launch.LaunchDescription([
        sensors_launch,
        joy_launch,
        laser_tf_node,
        TimerAction(period=5.0, actions=[slam_node]),
    ])


if __name__ == '__main__':
    ld = generate_launch_description()

    ls = launch.LaunchService()
    ls.include_launch_description(ld)
    ls.run()
