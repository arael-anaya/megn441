"""Entire block stacking stack in one command: camera, arm_control, ik_node, vision_node, stacker_node.

    ros2 launch rosbot_arm stack_launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory

import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def makeNodes(context):
    """Build the stacker node from the look pose launch arguments.

    The camera pose comes from TF (URDF + real servo angles), so the look pose can be changed freely.
    """
    pulses = [int(LaunchConfiguration(f'j{i}').perform(context)) for i in range(1, 6)]
    print(f'[stack_launch] look pulses {pulses}')

    # Give the camera and the other nodes a few seconds to come up before the arm starts moving
    stacker_node = TimerAction(
        period=5.0,
        actions=[Node(
            package='rosbot_arm', executable='stacker_node', name='stacker_node',
            output='screen', on_exit=launch.actions.Shutdown(),
            parameters=[{
                'look_pulses': pulses,
                'look_duration': float(LaunchConfiguration('look_duration').perform(context)),
            }])])

    return [stacker_node]


def generate_launch_description():
    orbbec_pkg_path = get_package_share_directory('orbbec_camera')
    urdfPath = os.path.join(get_package_share_directory('jetrover_description'), 'urdf', 'jetrover.xacro')
    visionParams = os.path.join(get_package_share_directory('rosbot_vision'), 'config', 'vision_params.yaml')

    # The URDF reads these with $(env ...); the robot normally has them set already
    os.environ.setdefault('LIDAR_TYPE', 'A1')
    os.environ.setdefault('MACHINE_TYPE', 'JetRover_Acker')

    # Look pose, one pulse per joint (pulse = 500 + angle * 500 / 120). Edit on the command line, e.g.
    #   ros2 launch rosbot_arm stack_launch.py j2:=700 j3:=150 j4:=300
    args = [
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('j1', default_value='500', description='look pose joint 1 pulse'),
        DeclareLaunchArgument('j2', default_value='775', description='look pose joint 2 pulse (125..775)'),
        DeclareLaunchArgument('j3', default_value='125', description='look pose joint 3 pulse'),
        DeclareLaunchArgument('j4', default_value='225', description='look pose joint 4 pulse'),
        DeclareLaunchArgument('j5', default_value='500', description='look pose joint 5 pulse'),
        DeclareLaunchArgument('look_duration', default_value='2.5', description='seconds to reach look pose'),
    ]

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(orbbec_pkg_path, 'launch', 'dabai_dcw.launch.py')),
        launch_arguments={'camera_name': 'depth_cam', 'depth_registration': 'true'}.items(),
        condition=IfCondition(LaunchConfiguration('start_camera')))

    arm_control_node = Node(
        package='rosbot_arm', executable='arm_control', name='arm_control', output='screen')

    ik_node = Node(
        package='rosbot_arm', executable='ik_node', name='ik_node', output='screen')

    # base_footprint -> ... -> link4 -> depth_cam_link from the URDF. The orbbec driver publishes the camera
    # frames below depth_cam_link, so the vision node can look up base_footprint <- camera optical frame.
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': ParameterValue(Command(['xacro ', urdfPath]), value_type=str)}])

    # Real servo angles -> joint states for robot_state_publisher
    arm_state_publisher = Node(
        package='rosbot_arm', executable='arm_state_publisher', name='arm_state_publisher', output='screen')

    vision_node = Node(
        package='rosbot_vision', executable='vision_node', name='vision_node', output='screen',
        parameters=[visionParams])

    return launch.LaunchDescription(args + [
        camera,
        arm_control_node,
        ik_node,
        robot_state_publisher,
        arm_state_publisher,
        vision_node,
        OpaqueFunction(function=makeNodes),
    ])
