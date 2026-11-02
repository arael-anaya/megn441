import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# ros2 launch rosbot_nav sensors_nav2_rviz_launch.py \
#   params_file:=/home/<user>/<your_ws>/src/rosbot_nav/params/nav2_params.yaml

# rsync -ruvl A7_lab4_ws ubuntu@192.168.149.1:~/


def generate_launch_description():
    rosbot_pkg_path = get_package_share_directory('rosbot')
    nav_pkg_path = get_package_share_directory('rosbot_nav')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(nav_pkg_path, 'maps', 'Robo_Lab.yaml'),
        description='Full path to map yaml file to load')

    declare_params_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav_pkg_path, 'params', 'nav2_params.yaml'),
        description='Full path to the nav2 params file')

    declare_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')


    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rosbot_pkg_path, 'launch', 'sensors_launch.py')),
        launch_arguments={'use_rviz': 'false'}.items(),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg_path, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg_path, 'launch', 'rviz_launch.py')),
    )
    # I think we need a delay before we start the other packages so the sensors are properly set up
    delayed_nav2 = TimerAction(period=5.0, actions=[nav2_launch])
    delayed_rviz = TimerAction(period=8.0, actions=[rviz_launch])

    return LaunchDescription([
        declare_map_cmd,
        declare_params_cmd,
        declare_sim_time_cmd,
        sensors_launch,
        delayed_nav2,
        delayed_rviz,
    ])
