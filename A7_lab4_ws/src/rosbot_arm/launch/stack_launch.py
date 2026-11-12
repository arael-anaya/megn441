"""Entire block stacking stack in one command: camera, arm_control, ik_node, vision_node, stacker_node.

    ros2 launch rosbot_arm stack_launch.py
    ros2 launch rosbot_arm stack_launch.py mode:=calibrate   # once, and after changing the look pose
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
    lookXyz = [float(LaunchConfiguration(f'look_{a}').perform(context)) for a in 'xyz']
    print(f'[stack_launch] look pose (IK target, gripper down) {lookXyz}, '
          f'grasp_z {LaunchConfiguration("grasp_z").perform(context)}')

    # mode:=calibrate measures the camera -> arm correction instead of stacking
    executable = 'calibrate_node' if LaunchConfiguration('mode').perform(context) == 'calibrate' else 'stacker_node'

    # Give the camera and the other nodes a few seconds to come up before the arm starts moving
    stacker_node = TimerAction(
        period=5.0,
        actions=[Node(
            package='rosbot_arm', executable=executable, name=executable,
            output='screen', on_exit=launch.actions.Shutdown(),
            parameters=[{
                'look_xyz': lookXyz,
                'grasp_z': float(LaunchConfiguration('grasp_z').perform(context)),
                'look_duration': float(LaunchConfiguration('look_duration').perform(context)),
            }])])

    return [stacker_node]


def makeVision(context):
    """Vision node with the shared vision_params.yaml."""
    visionParams = os.path.join(get_package_share_directory('rosbot_vision'), 'config', 'vision_params.yaml')
    return [Node(
        package='rosbot_vision', executable='vision_node', name='vision_node', output='screen',
        parameters=[visionParams])]


def generate_launch_description():
    orbbec_pkg_path = get_package_share_directory('orbbec_camera')
    urdfPath = os.path.join(get_package_share_directory('jetrover_description'), 'urdf', 'jetrover.xacro')

    # The URDF reads these with $(env ...); the robot normally has them set already
    os.environ.setdefault('LIDAR_TYPE', 'A1')
    os.environ.setdefault('MACHINE_TYPE', 'JetRover_Acker')

    # Look pose as an IK target for the tool point, gripper (and camera) pointing straight down. The camera sits about
    # 5 cm ahead of and 0.11 m above the tool point. Edit on the command line, e.g.
    #   ros2 launch rosbot_arm stack_launch.py look_x:=0.22 look_z:=0.15
    args = [
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('mode', default_value='stack', description='stack, or calibrate to measure the '
                              'camera -> arm correction (saved to ~/.ros/block_calibration.json)'),
        DeclareLaunchArgument('look_x', default_value='0.20', description='look pose tool x (m)'),
        DeclareLaunchArgument('look_y', default_value='0.0', description='look pose tool y (m)'),
        DeclareLaunchArgument('look_z', default_value='0.14', description='look pose tool z (m), >= 0.13 for depth'),
        DeclareLaunchArgument('grasp_z', default_value='-0.01',
                              description='gripper close height (m) for picks and calibration; lower if it closes '
                              'above the block'),
        DeclareLaunchArgument('look_duration', default_value='1.25', description='seconds to reach look pose'),
    ]

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(orbbec_pkg_path, 'launch', 'dabai_dcw.launch.py')),
        # HW alignment puts depth on the same pixels as color (the course block_detector launch uses it too)
        launch_arguments={'camera_name': 'depth_cam', 'depth_registration': 'true', 'align_mode': 'HW'}.items(),
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

    return launch.LaunchDescription(args + [
        camera,
        arm_control_node,
        ik_node,
        robot_state_publisher,
        arm_state_publisher,
        OpaqueFunction(function=makeVision),
        OpaqueFunction(function=makeNodes),
    ])
