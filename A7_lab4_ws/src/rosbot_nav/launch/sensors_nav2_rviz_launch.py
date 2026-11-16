import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration





def generate_launch_description():
    rosbot_pkg_path = get_package_share_directory('rosbot')
    nav_pkg_path = get_package_share_directory('rosbot_nav')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Robo_Lab.yaml is the original map and is never modified. map_updater writes the refined
    # map next to it (resolving symlinks so a --symlink-install build writes into the source
    # tree); once it exists, it is used automatically. Delete it to go back to the original.
    base_map = os.path.join(nav_pkg_path, 'maps', 'Robo_Lab.yaml')
    maps_dir = os.path.dirname(os.path.realpath(base_map))
    updated_map = os.path.join(maps_dir, 'Robo_Lab_updated.yaml')
    default_map = updated_map if os.path.exists(updated_map) else base_map

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to map yaml file to load')
    declare_update_map_cmd = DeclareLaunchArgument(
        'update_map', default_value='true',
        description='Refine the map while driving and save it for later runs')

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
            'log_level': 'info',
        }.items(),
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg_path, 'launch', 'rviz_launch.py')),
    )
    map_updater = Node(
        package='nav_manager', executable='map_updater', name='map_updater', output='screen',
        parameters=[{'map_yaml': map_yaml,
                     'output_yaml': updated_map,
                     'update': ParameterValue(LaunchConfiguration('update_map'), value_type=bool),
                     'persist_time': 4.0}])
    delayed_nav2 = TimerAction(period=5.0, actions=[nav2_launch])
    delayed_rviz = TimerAction(period=8.0, actions=[rviz_launch])

    return LaunchDescription([
        declare_map_cmd,
        declare_update_map_cmd,
        declare_params_cmd,
        declare_sim_time_cmd,
        sensors_launch,
        map_updater,
        delayed_nav2,
        delayed_rviz,
    ])
