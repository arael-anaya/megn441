import os
from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    bringup_pkg_path = get_package_share_directory('bringup')

    teleop_joy_node = launch_ros.actions.Node(
                package='rosbot',
                executable='teleop_joy',
                name='teleop_joy',
                prefix=['xterm -e'],
                output='screen')


    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg_path, 'launch/bringup.launch.py')),
    )

    return launch.LaunchDescription([
        teleop_joy_node,
        bringup_launch,
    ])


if __name__ == '__main__':
    # Create a LaunchDescription object
    ld = generate_launch_description()

    ls = launch.LaunchService()
    ls.include_launch_description(ld)
    ls.run()
