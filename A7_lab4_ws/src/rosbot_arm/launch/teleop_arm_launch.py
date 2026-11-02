import os
from ament_index_python.packages import get_package_share_directory

import launch
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rosbot_arm'), 'launch', 'arm_launch.py')))

    teleop_arm_node = Node(
                package='rosbot_arm',
                executable='teleop_arm',
                name='teleop_arm',
                output='screen')

    return launch.LaunchDescription([
        arm_launch,
        teleop_arm_node,
    ])


if __name__ == '__main__':
    ld = generate_launch_description()

    ls = launch.LaunchService()
    ls.include_launch_description(ld)
    ls.run()
