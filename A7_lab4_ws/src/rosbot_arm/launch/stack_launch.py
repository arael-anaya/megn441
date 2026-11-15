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
from launch_ros.actions import Node

from rosbot_arm.armConfig import GRASP_Z


def makeNodes(context):
    """Build the stacker node from the look pose launch arguments.

    The vision node works out the camera pose from the arm pose ik_node commands, so the look pose can be changed
    freely (recalibrate afterwards).
    """
    lookXyz = [float(LaunchConfiguration(f'look_{a}').perform(context)) for a in 'xyz']
    print(f'[stack_launch] look pose (IK target, gripper down) {lookXyz}, '
          f'grasp_z {LaunchConfiguration("grasp_z").perform(context)}')

    executable = 'calibrate_node' if LaunchConfiguration('mode').perform(context) == 'calibrate' else 'stacker_node'

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

    args = [
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('mode', default_value='stack', description='stack, or calibrate to measure the '
                              'camera -> arm correction (saved to ~/.ros/block_calibration.json)'),
        DeclareLaunchArgument('look_x', default_value='0.20', description='look pose tool x (m)'),
        DeclareLaunchArgument('look_y', default_value='0.0', description='look pose tool y (m)'),
        DeclareLaunchArgument('look_z', default_value='0.14', description='look pose tool z (m), >= 0.13 for depth'),
        DeclareLaunchArgument('grasp_z', default_value=str(GRASP_Z),
                              description='gripper close height (m) for picks and calibration; the fingertips must '
                              'stay just clear of the floor (armConfig.GRASP_Z)'),
        DeclareLaunchArgument('look_duration', default_value='1.25', description='seconds to reach look pose'),
    ]

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(orbbec_pkg_path, 'launch', 'dabai_dcw.launch.py')),
        launch_arguments={'camera_name': 'depth_cam', 'depth_registration': 'true', 'align_mode': 'HW'}.items(),
        condition=IfCondition(LaunchConfiguration('start_camera')))

    arm_control_node = Node(
        package='rosbot_arm', executable='arm_control', name='arm_control', output='screen')

    ik_node = Node(
        package='rosbot_arm', executable='ik_node', name='ik_node', output='screen')


    return launch.LaunchDescription(args + [
        camera,
        arm_control_node,
        ik_node,
        OpaqueFunction(function=makeVision),
        OpaqueFunction(function=makeNodes),
    ])
