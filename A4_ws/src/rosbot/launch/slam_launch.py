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

    # No manual static_transform_publisher needed: the lidar scan now uses
    # frame_id 'lidar_frame', which robot_state_publisher already publishes
    # relative to base_footprint via the robot's URDF (brought up automatically
    # by bringup.launch.py -> controller.launch.py -> odom_publisher.launch.py
    # -> robot_description.launch.py).

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
        TimerAction(period=5.0, actions=[slam_node]),
    ])


if __name__ == '__main__':
    ld = generate_launch_description()

    ls = launch.LaunchService()
    ls.include_launch_description(ld)
    ls.run()
