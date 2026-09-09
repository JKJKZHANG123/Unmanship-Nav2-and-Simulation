"""Launch the Qt5 USV monitoring dashboard for real-boat diagnostics.

The dashboard can read ROS parameters and write only the explicitly listed
core parameters through ROS parameter services; it does not start actuators.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usv_monitor',
            executable='usv_monitor_node',
            name='usv_monitor',
            output='screen',
        ),
    ])
