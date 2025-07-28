from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'scanner_node',
            executable = 'scanner',
            output = 'screen'
        ),
        Node(
            package='door_handle_node',
            executable='door_handle',
            output='screen'
        ),
        Node(
            package='emergency_button_node',
            executable='emergency_button',
            output='screen'
        ),
        Node(
            package='stack_light_node',
            executable='stack_light',
            output='screen'
        ),
    ])
