import launch
import launch_ros.actions

def generate_launch_description():
    teleop_joy_node = launch_ros.actions.Node(
                package='rosbot',
                executable='teleop_joy',
                name='teleop_joy',
                prefix=['xterm -e'],
                output='screen')

    return launch.LaunchDescription([
        teleop_joy_node,
    ])


if __name__ == '__main__':
    # Create a LaunchDescription object
    ld = generate_launch_description()

    ls = launch.LaunchService()
    ls.include_launch_description(ld)
    ls.run()
