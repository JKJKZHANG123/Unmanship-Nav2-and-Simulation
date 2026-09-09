"""Planner-only real-LiDAR experiment: cloud filter + Nav2 planner + RViz."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    cloud_params_file = LaunchConfiguration('cloud_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    enable_click_planner = LaunchConfiguration('enable_click_planner')

    # Launch the filter node directly. Including cloud_filter.launch.py would
    # introduce a second launch argument also named ``params_file`` and could
    # accidentally make planner_server consume the cloud-filter YAML.
    cloud_filter = Node(
        package='usv_cloud_filter',
        executable='obstacle_filter',
        name='obstacle_filter',
        output='screen',
        parameters=[cloud_params_file],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_local_planning',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['planner_server'],
        }],
    )

    # planner_server owns only the global costmap in this planner-only
    # experiment. Convert it to 3D markers so Jetson can visualize occupied
    # cells without enabling RViz's problematic 8-bit Map texture plugin.
    costmap_markers = Node(
        package='usv_navigation',
        executable='costmap_3d_markers',
        name='costmap_3d_markers',
        output='screen',
        parameters=[{
            'costmap_topic': '/global_costmap/costmap',
            'use_world_frame': 'camera_init',
        }],
    )

    click_planner = Node(
        package='usv_navigation',
        executable='local_plan_click',
        name='local_plan_click',
        output='screen',
        condition=IfCondition(enable_click_planner),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_local_planning',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('usv_navigation'),
            'config',
            'nav3d_view.rviz',
        ])],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('usv_navigation'),
                'config',
                'nav2_params_real.yaml',
            ]),
            description='Planner and rolling costmap parameters'),
        DeclareLaunchArgument(
            'cloud_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('usv_cloud_filter'),
                'config',
                'cloud_filter_params_real.yaml',
            ]),
            description='Real Unitree L1 obstacle-filter parameters'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Must remain false for the real-sensor experiment'),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz with Point-LIO cloud and /plan displays'),
        DeclareLaunchArgument(
            'enable_click_planner',
            default_value='true',
            description='Plan to RViz /clicked_point without commanding motion'),
        cloud_filter,
        planner_server,
        lifecycle_manager,
        costmap_markers,
        click_planner,
        rviz_node,
    ])
