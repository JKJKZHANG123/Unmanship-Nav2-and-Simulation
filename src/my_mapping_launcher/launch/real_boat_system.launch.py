"""
Real-boat launch for Unitree L1 + RTK + Point-LIO + Nav2.

This is deliberately separate from the VRX/Gazebo launch files. It starts no
simulator, bridge, click plugin, or thrust output. Nav2 still computes plans
and publishes /cmd_vel_smoothed for logging or a future actuator adapter.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _static_tf(name, parent, child, x, y, z, roll, pitch, yaw):
    """Create a configurable static sensor transform."""
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        output='screen',
        arguments=[
            '--x', x, '--y', y, '--z', z,
            '--roll', roll, '--pitch', pitch, '--yaw', yaw,
            '--frame-id', parent, '--child-frame-id', child,
        ],
        parameters=[{'use_sim_time': False}],
    )


def generate_launch_description():
    lidar_port = LaunchConfiguration('lidar_port')
    gps_port = LaunchConfiguration('gps_port')
    gps_baud = LaunchConfiguration('gps_baud')
    gps_protocol = LaunchConfiguration('gps_protocol')
    request_stream = LaunchConfiguration('request_stream')
    heading_offset_deg = LaunchConfiguration('heading_offset_deg')
    heading_fallback_timeout_s = LaunchConfiguration(
        'heading_fallback_timeout_s')
    mission_upload_enabled = LaunchConfiguration('mission_upload_enabled')
    mission_path_topic = LaunchConfiguration('mission_path_topic')
    mission_frame = LaunchConfiguration('mission_frame')
    mission_altitude_m = LaunchConfiguration('mission_altitude_m')
    mission_acceptance_radius_m = LaunchConfiguration(
        'mission_acceptance_radius_m')
    enable_goal_gateway = LaunchConfiguration('enable_goal_gateway')
    rviz = LaunchConfiguration('rviz')
    enable_dynamic_tracker = LaunchConfiguration('enable_dynamic_tracker')
    target_port = LaunchConfiguration('target_port')
    target_baud = LaunchConfiguration('target_baud')
    path_output_port = LaunchConfiguration('path_output_port')
    path_output_baud = LaunchConfiguration('path_output_baud')

    lidar_frame = LaunchConfiguration('lidar_frame')
    imu_frame = LaunchConfiguration('imu_frame')
    gps_frame = LaunchConfiguration('gps_frame')
    base_frame = 'base_link'

    # These are intentionally zero defaults: the physical mounting offsets
    # must be measured on the boat. The arguments avoid inventing an extrinsic
    # calibration while still providing the TFs required by Nav2 and filtering.
    lidar_tf = [_static_tf(
        'base_to_unilidar_lidar', base_frame, lidar_frame,
        LaunchConfiguration('lidar_x'), LaunchConfiguration('lidar_y'),
        LaunchConfiguration('lidar_z'), LaunchConfiguration('lidar_roll'),
        LaunchConfiguration('lidar_pitch'), LaunchConfiguration('lidar_yaw'))]
    imu_tf = [_static_tf(
        'base_to_unilidar_imu', base_frame, imu_frame,
        LaunchConfiguration('imu_x'), LaunchConfiguration('imu_y'),
        LaunchConfiguration('imu_z'), LaunchConfiguration('imu_roll'),
        LaunchConfiguration('imu_pitch'), LaunchConfiguration('imu_yaw'))]
    gps_tf = [_static_tf(
        'base_to_gps', base_frame, gps_frame,
        LaunchConfiguration('gps_x'), LaunchConfiguration('gps_y'),
        LaunchConfiguration('gps_z'), LaunchConfiguration('gps_roll'),
        LaunchConfiguration('gps_pitch'), LaunchConfiguration('gps_yaw'))]

    lidar_node = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'port': lidar_port,
            'rotate_yaw_bias': 0.0,
            'range_scale': 0.001,
            'range_bias': 0.0,
            'range_max': 50.0,
            'range_min': 0.0,
            'cloud_frame': lidar_frame,
            'cloud_topic': '/unilidar/cloud',
            'cloud_scan_num': 18,
            'imu_frame': imu_frame,
            'imu_topic': '/unilidar/imu',
        }],
    )

    mapping_share = get_package_share_directory('point_lio')
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            mapping_share, 'launch', 'mapping_unilidar_l1.launch.py')),
        launch_arguments={'rviz': 'false'}.items(),
    )

    gps_share = get_package_share_directory('my_gps_driver')
    mavlink_config = os.path.join(
        get_package_share_directory('usv_navigation'), 'config',
        'mavlink_rtk_real.yaml')
    mavlink_enabled = PythonExpression(["'", gps_protocol, "' == 'mavlink'"])
    mavlink_rtk = Node(
        package='usv_navigation',
        executable='mavlink_rtk_bridge',
        name='mavlink_rtk_bridge',
        output='screen',
        condition=IfCondition(mavlink_enabled),
        parameters=[mavlink_config, {
            'port': gps_port,
            'baud': ParameterValue(gps_baud, value_type=int),
            'request_stream': ParameterValue(request_stream, value_type=bool),
            'heading_offset_deg': ParameterValue(
                heading_offset_deg, value_type=float),
            'heading_fallback_timeout_s': ParameterValue(
                heading_fallback_timeout_s, value_type=float),
            'mission_upload_enabled': ParameterValue(
                mission_upload_enabled, value_type=bool),
            'mission_path_topic': mission_path_topic,
            'mission_frame': ParameterValue(
                mission_frame, value_type=int),
            'mission_altitude_m': ParameterValue(
                mission_altitude_m, value_type=float),
            'mission_acceptance_radius_m': ParameterValue(
                mission_acceptance_radius_m, value_type=float),
        }],
    )

    gps = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            gps_share, 'launch', 'gps_driver.launch.py')),
        launch_arguments={
            'port': gps_port,
            'baud': gps_baud,
            'frame_id': gps_frame,
        }.items(),
        condition=UnlessCondition(mavlink_enabled),
    )

    cloud_filter_share = get_package_share_directory('usv_cloud_filter')
    cloud_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            cloud_filter_share, 'launch', 'cloud_filter.launch.py')),
        launch_arguments={
            'params_file': os.path.join(
                cloud_filter_share, 'config', 'cloud_filter_params_real.yaml'),
        }.items(),
    )

    localization_share = get_package_share_directory('usv_localization')
    navsat = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            localization_share, 'launch', 'navsat_only.launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'gps_topic': '/gps/fix',
            'imu_topic': '/unilidar/imu',
            'navsat_imu_topic': '/gps/heading_imu',
            'use_rtk_heading': 'true',
            'heading_topic': '/gps/heading',
            'heading_timeout_s': '2.5',
            'use_odometry_yaw': 'false',
            'odometry_topic': '/aft_mapped_to_init',
        }.items(),
    )

    navigation_share = get_package_share_directory('usv_navigation')
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            navigation_share, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': PathJoinSubstitution([
                navigation_share, 'config', 'nav2_params_real.yaml']),
            'dynamic_tracker_params_file': os.path.join(
                navigation_share, 'config', 'dynamic_obstacle_tracker_real.yaml'),
            'goal_guard_params_file': os.path.join(
                navigation_share, 'config', 'goal_guard_real.yaml'),
            'enable_dynamic_tracker': enable_dynamic_tracker,
            'enable_goal_gateway': enable_goal_gateway,
        }.items(),
    )

    geodetic_share = get_package_share_directory('usv_navigation')
    geodetic_io = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            geodetic_share, 'launch', 'geodetic_io.launch.py')),
        launch_arguments={
            'target_port': target_port,
            'target_baud': target_baud,
            'path_output_port': path_output_port,
            'path_output_baud': path_output_baud,
            'geo_path_topic': mission_path_topic,
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='real_boat_rviz',
        output='screen',
        arguments=['-d', os.path.join(
            navigation_share, 'config', 'nav_view.rviz')],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(rviz),
    )

    args = [
        DeclareLaunchArgument(
            'lidar_port', default_value='/dev/ttyUSB0',
            description='Unitree L1 serial device'),
        DeclareLaunchArgument(
            'gps_protocol', default_value='mavlink',
            description='GPS transport: mavlink (flight controller) or nmea'),
        DeclareLaunchArgument(
            'gps_port', default_value='/dev/ttyTHS1',
            description='Jetson GPIO UART connected to flight-controller MAVLink'),
        DeclareLaunchArgument(
            'gps_baud', default_value='115200',
            description='MAVLink UART baud rate (use NMEA baud only in nmea mode)'),
        DeclareLaunchArgument(
            'request_stream', default_value='true',
            description='Request GPS/heading MAVLink message intervals'),
        DeclareLaunchArgument(
            'heading_offset_deg', default_value='0.0',
            description='Compass heading correction; use 180 for reversed baseline'),
        DeclareLaunchArgument(
            'heading_fallback_timeout_s', default_value='2.0',
            description='Allow fallback heading only after RTK yaw is stale'),
        DeclareLaunchArgument(
            'mission_upload_enabled', default_value='false',
            description=(
                'Upload changed /plan_geodetic to the flight-controller mission; '
                'does not arm or switch flight mode')),
        DeclareLaunchArgument(
            'mission_path_topic', default_value='/plan_geodetic',
            description='GeoPath topic consumed by the MAVLink mission uploader'),
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
        DeclareLaunchArgument(
            'enable_goal_gateway', default_value='false',
            description=(
                'Start NavigateToPose gateway; keep false for path-only output')),
        DeclareLaunchArgument(
            'target_port', default_value='/dev/ttyUSB2',
            description='Serial device carrying target latitude/longitude'),
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
            'rviz', default_value='true',
            description='Start RViz2'),
        DeclareLaunchArgument(
            'enable_dynamic_tracker', default_value='true',
            description='Enable dynamic obstacle prediction'),
        DeclareLaunchArgument(
            'lidar_frame', default_value='unilidar_lidar',
            description='Unitree L1 point-cloud frame'),
        DeclareLaunchArgument(
            'imu_frame', default_value='unilidar_imu',
            description='Unitree L1 IMU frame'),
        DeclareLaunchArgument(
            'gps_frame', default_value='gps_link',
            description='RTK antenna frame'),
    ]

    # L1 defaults use the calibrated L1 pose in the Point-LIO IMU frame.
    # Override these only if the installed sensor orientation/offset differs.
    for prefix, label in (
            ('lidar', 'L1'), ('imu', 'L1 IMU'), ('gps', 'RTK antenna')):
        default_x = '0.007698' if prefix == 'lidar' else '0.0'
        default_y = '0.014655' if prefix == 'lidar' else '0.0'
        default_z = '-0.00667' if prefix == 'lidar' else '0.0'
        args.extend([
            DeclareLaunchArgument(
                f'{prefix}_x', default_value=default_x,
                description=f'{label} x offset (m)'),
            DeclareLaunchArgument(
                f'{prefix}_y', default_value=default_y,
                description=f'{label} y offset (m)'),
            DeclareLaunchArgument(
                f'{prefix}_z', default_value=default_z,
                description=f'{label} z offset (m)'),
            DeclareLaunchArgument(
                f'{prefix}_roll', default_value='0.0',
                description=f'{label} roll (rad)'),
            DeclareLaunchArgument(
                f'{prefix}_pitch', default_value='0.0',
                description=f'{label} pitch (rad)'),
            DeclareLaunchArgument(
                f'{prefix}_yaw', default_value='0.0',
                description=f'{label} yaw (rad)'),
        ])

    return LaunchDescription([
        *args,
        lidar_node,
        *lidar_tf,
        *imu_tf,
        *gps_tf,
        mavlink_rtk,
        gps,
        TimerAction(period=2.0, actions=[mapping]),
        TimerAction(period=3.0, actions=[cloud_filter]),
        TimerAction(period=4.0, actions=[navsat]),
        TimerAction(period=5.0, actions=[navigation]),
        TimerAction(period=5.5, actions=[geodetic_io]),
        TimerAction(period=7.0, actions=[rviz_node]),
    ])
