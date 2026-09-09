"""Real-boat geodetic waypoint input and path output."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Start target serial input, geodetic goal conversion, and path output."""
    share = get_package_share_directory('usv_navigation')
    params = os.path.join(share, 'config', 'geodetic_io_real.yaml')
    target_port = LaunchConfiguration('target_port')
    target_baud = LaunchConfiguration('target_baud')
    path_output_port = LaunchConfiguration('path_output_port')
    path_output_baud = LaunchConfiguration('path_output_baud')
    geo_path_topic = LaunchConfiguration('geo_path_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'target_port', default_value='/dev/ttyUSB2',
            description='Serial port carrying target latitude/longitude'),
        DeclareLaunchArgument(
            'target_baud', default_value='115200',
            description='Target waypoint serial baud rate'),
        DeclareLaunchArgument(
            'path_output_port', default_value='',
            description='Optional serial port for JSON geodetic path output'),
        DeclareLaunchArgument(
            'path_output_baud', default_value='115200',
            description='Serial baud for geodetic path output'),
        DeclareLaunchArgument(
            'geo_path_topic', default_value='/plan_geodetic',
            description='WGS84 GeoPath topic published for mission upload'),
        Node(
            package='usv_navigation',
            executable='target_waypoint_serial',
            name='target_waypoint_serial',
            output='screen',
            parameters=[params, {
                'port': target_port,
                'baud': ParameterValue(target_baud, value_type=int),
            }]),
        Node(
            package='usv_navigation',
            executable='target_to_goal',
            name='target_to_goal',
            output='screen',
            parameters=[params]),
        Node(
            package='usv_navigation',
            executable='target_waypoints_to_goals',
            name='target_waypoints_to_goals',
            output='screen',
            parameters=[params]),
        Node(
            package='usv_navigation',
            executable='geodetic_goal_planner',
            name='geodetic_goal_planner',
            output='screen',
            parameters=[params]),
        Node(
            package='usv_navigation',
            executable='geodetic_path_publisher',
            name='geodetic_path_publisher',
            output='screen',
            parameters=[params, {
                'output_port': path_output_port,
                'output_baud': ParameterValue(path_output_baud, value_type=int),
                'geo_path_topic': geo_path_topic,
            }]),
    ])
