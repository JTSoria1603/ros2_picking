
"""
ROS2 launch file for starting all cell-related nodes in the picking system.

This launch file starts the following nodes:
  - scanner_node: Publishes barcodes.
  - door_handle_node: Simulates a door handle sensor and actuator.
  - emergency_button_node: Simulates an emergency stop button.
  - stack_light_node: Simulates a stack light indicator.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate a LaunchDescription for all cell nodes in the picking system.

    Returns:
        LaunchDescription: Launches scanner, door handle, emergency button, and stack light nodes.
    """
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
