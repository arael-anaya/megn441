import os
from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Find the location of the package on your computer
    example_pkg_path = get_package_share_directory('example_pkg')


    # Launch an individual node from a package.
    # This is the same as using:
    # `ros2 run example_pkg example_node``
    example_node = launch_ros.actions.Node(
                package='example_pkg',
                executable='example_node',
                name='example')


    # Launch another launch file.
    # This is the same as running
    # `ros2 launch example_pkg example_launch.py`
    example_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(example_pkg_path, 'launch/example_launch.py')),
    )

    # Your final returned LaunchDescription includes 
    # a list of all of the nodes, launch files, etc.
    return launch.LaunchDescription([
        example_node,
        example_launch,
    ])


if __name__ == '__main__':
    # Create a LaunchDescription object
    ld = generate_launch_description()

    ls = launch.LaunchService()
    ls.include_launch_description(ld)
    ls.run()
