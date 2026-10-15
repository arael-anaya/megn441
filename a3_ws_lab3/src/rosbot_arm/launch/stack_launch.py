import os
from ament_index_python.packages import get_package_share_directory

import launch
from launch_ros.actions import Node


def generate_launch_description():
    visionParams = os.path.join(get_package_share_directory('rosbot_vision'), 'config', 'vision_params.yaml')

    arm_control_node = Node(
        package='rosbot_arm',
        executable='arm_control',
        name='arm_control')

    ik_node = Node(
        package='rosbot_arm',
        executable='ik_node',
        name='ik_node')

    vision_node = Node(
        package='rosbot_vision',
        executable='vision_node',
        name='vision_node',
        output='screen',
        parameters=[visionParams])

    # Starts the stacking sequence as soon as the other nodes are up
    stacker_node = Node(
        package='rosbot_arm',
        executable='stacker_node',
        name='stacker_node',
        output='screen')

    return launch.LaunchDescription([
        arm_control_node,
        ik_node,
        vision_node,
        stacker_node,
    ])
