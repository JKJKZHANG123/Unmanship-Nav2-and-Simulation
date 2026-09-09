# Copyright 2026 jkjkzhang
# SPDX-License-Identifier: MIT

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('usv_cloud_filter'),
            'config', 'cloud_filter_params.yaml']),
        description='Point-cloud filter parameter file')

    obstacle_filter = Node(
        package='usv_cloud_filter',
        executable='obstacle_filter',
        name='obstacle_filter',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([params_arg, obstacle_filter])
