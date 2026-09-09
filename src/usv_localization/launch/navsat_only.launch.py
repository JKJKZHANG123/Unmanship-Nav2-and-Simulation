"""Anchor Point-LIO's local world frame with the real RTK fix.

This launch intentionally does not start an EKF or any actuator node. It keeps
the existing RTK interface (/gps/fix and /gps/vel) and publishes the
navsat_transform output for later global-frame fusion.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('usv_localization')
    navsat_params = os.path.join(pkg_share, 'config', 'navsat_real.yaml')

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use /clock; keep false for the real boat')
    gps_topic = DeclareLaunchArgument(
        'gps_topic', default_value='/gps/fix',
        description='RTK NavSatFix topic')
    imu_topic = DeclareLaunchArgument(
        'imu_topic', default_value='/unilidar/imu',
        description='Unitree L1 IMU topic')
    odometry_topic = DeclareLaunchArgument(
        'odometry_topic', default_value='/aft_mapped_to_init',
        description='Point-LIO odometry topic')
    navsat_imu_topic = DeclareLaunchArgument(
        'navsat_imu_topic', default_value='/gps/heading_imu',
        description='IMU topic consumed by navsat_transform; '
                    'use /gps/heading_imu for RTK yaw anchoring')
    use_rtk_heading = DeclareLaunchArgument(
        'use_rtk_heading', default_value='true',
        description='Start heading_to_imu and use RTK yaw for the GPS anchor')
    heading_topic = DeclareLaunchArgument(
        'heading_topic', default_value='/gps/heading',
        description='ENU yaw topic from the dual-antenna RTK bridge')
    heading_imu_topic = DeclareLaunchArgument(
        'heading_imu_topic', default_value='/gps/heading_imu',
        description='Combined IMU topic used by navsat_transform')
    heading_timeout_s = DeclareLaunchArgument(
        'heading_timeout_s', default_value='2.5',
        description='Stop publishing combined IMU when RTK heading is stale')
    use_odometry_yaw = DeclareLaunchArgument(
        'use_odometry_yaw', default_value='false',
        description='Use Point-LIO odometry yaw instead of IMU yaw when true; '
                    'keep false so navsat anchors utm->camera_init with the '
                    'RTK yaw carried by /gps/heading_imu')

    heading_imu = Node(
        package='usv_navigation',
        executable='heading_to_imu',
        name='heading_to_imu',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rtk_heading')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'heading_topic': LaunchConfiguration('heading_topic'),
            'output_topic': LaunchConfiguration('heading_imu_topic'),
            'require_heading': True,
            'heading_timeout_s': ParameterValue(
                LaunchConfiguration('heading_timeout_s'), value_type=float),
        }],
    )

    # With RTK heading enabled, navsat_transform consumes heading_imu_topic;
    # otherwise it keeps the historical raw IMU path.
    navsat_imu = LaunchConfiguration('navsat_imu_topic')
    navsat = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[
            navsat_params,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'use_odometry_yaw': ParameterValue(
                    LaunchConfiguration('use_odometry_yaw'), value_type=bool),
            },
        ],
        remappings=[
            ('imu', navsat_imu),
            ('gps/fix', LaunchConfiguration('gps_topic')),
            ('odometry/filtered', LaunchConfiguration('odometry_topic')),
        ],
    )

    return LaunchDescription([
        use_sim_time, gps_topic, imu_topic, odometry_topic,
        navsat_imu_topic, use_rtk_heading, heading_topic, heading_imu_topic,
        heading_timeout_s, use_odometry_yaw, heading_imu, navsat,
    ])
