"""Standalone dual-antenna RTK MAVLink bridge launch."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('usv_navigation'), 'config',
        'mavlink_rtk_real.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value='/dev/ttyTHS1',
            description='Jetson GPIO UART connected to flight-controller MAVLink'),
        DeclareLaunchArgument(
            'baud', default_value='115200',
            description='MAVLink UART baud rate'),
        DeclareLaunchArgument(
            'request_stream', default_value='true',
            description='Request GPS and heading message intervals'),
        DeclareLaunchArgument(
            'heading_offset_deg', default_value='0.0',
            description='Compass correction; use 180 if baseline points aft'),
        DeclareLaunchArgument(
            'heading_fallback_timeout_s', default_value='2.0',
            description='Allow fallback heading only after RTK yaw is stale'),
        DeclareLaunchArgument(
            'mission_upload_enabled', default_value='false',
            description='Upload /plan_geodetic to the FC mission protocol'),
        DeclareLaunchArgument(
            'mission_path_topic', default_value='/plan_geodetic',
            description='GeoPath topic used for mission upload'),
        DeclareLaunchArgument(
            'mission_frame', default_value='3',
            description=(
                'MAVLink mission frame: 3=GLOBAL_RELATIVE_ALT, '
                '6=GLOBAL_RELATIVE_ALT_INT')),
        DeclareLaunchArgument(
            'mission_altitude_m', default_value='0.0',
            description='Mission altitude in metres (relative if frame 3/6)'),
        DeclareLaunchArgument(
            'mission_acceptance_radius_m', default_value='1.0',
            description='Mission waypoint acceptance radius in metres'),
        Node(
            package='usv_navigation',
            executable='mavlink_rtk_bridge',
            name='mavlink_rtk_bridge',
            output='screen',
            parameters=[config, {
                'port': LaunchConfiguration('port'),
                'baud': ParameterValue(
                    LaunchConfiguration('baud'), value_type=int),
                'request_stream': ParameterValue(
                    LaunchConfiguration('request_stream'), value_type=bool),
                'heading_offset_deg': ParameterValue(
                    LaunchConfiguration('heading_offset_deg'), value_type=float),
                'heading_fallback_timeout_s': ParameterValue(
                    LaunchConfiguration('heading_fallback_timeout_s'),
                    value_type=float),
                'mission_upload_enabled': ParameterValue(
                    LaunchConfiguration('mission_upload_enabled'),
                    value_type=bool),
                'mission_path_topic': LaunchConfiguration('mission_path_topic'),
                'mission_frame': ParameterValue(
                    LaunchConfiguration('mission_frame'), value_type=int),
                'mission_altitude_m': ParameterValue(
                    LaunchConfiguration('mission_altitude_m'), value_type=float),
                'mission_acceptance_radius_m': ParameterValue(
                    LaunchConfiguration('mission_acceptance_radius_m'),
                    value_type=float),
            }]),
    ])
