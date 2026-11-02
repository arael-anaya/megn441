import launch
import launch_ros.actions

def generate_launch_description():
    teleop_twist_keyboard_node = launch_ros.actions.Node(
                package='teleop_twist_keyboard',
                executable='teleop_twist_keyboard',
                name='teleop_twist_keyboard',
                prefix=['xterm -e'],
                output='screen')

    return launch.LaunchDescription([
        teleop_twist_keyboard_node,
    ])


if __name__ == '__main__':
    # Create a LaunchDescription object
    ld = generate_launch_description()

    ls = launch.LaunchService()
    ls.include_launch_description(ld)
    ls.run()
