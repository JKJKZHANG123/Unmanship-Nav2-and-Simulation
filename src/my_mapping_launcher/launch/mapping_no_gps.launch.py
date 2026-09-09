"""
无 GPS 建图启动文件
仅启动: 激光雷达驱动 → Point-LIO → RViz2
不做 UTM 融合, 不启动 GPS 驱动, 不需要 navsat_transform_node
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ==========================================
    # 1. 获取各包的路径
    # ==========================================
    point_lio_dir = get_package_share_directory('point_lio')
    mapping_launcher_dir = get_package_share_directory('my_mapping_launcher')

    # ==========================================
    # 2. 可选的 RViz 启动参数 (默认开启)
    # ==========================================
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='是否启动 RViz2 可视化')

    # ==========================================
    # 3. 定义各个节点
    # ==========================================

    # 动作 A: 启动激光雷达与 IMU 驱动
    lidar_driver_node = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node',
        output='screen',
        parameters=[{
            'port': '/dev/ttyUSB0',
            'rotate_yaw_bias': 0.0,
            'range_scale': 0.001,
            'range_bias': 0.0,
            'range_max': 50.0,
            'range_min': 0.0,
            'cloud_frame': 'unilidar_lidar',
            'cloud_topic': 'unilidar/cloud',
            'cloud_scan_num': 18,
            'imu_frame': 'unilidar_imu',
            'imu_topic': 'unilidar/imu',
        }]
    )

    # Point-LIO publishes camera_init -> base_link.  The raw L1 cloud still
    # needs fixed sensor frames so the obstacle filter can transform it into
    # base_link for the Nav2 costmap.  base_link represents the L1 IMU body;
    # the LiDAR translation matches mapping.extrinsic_T in unilidar_l1.yaml.
    base_to_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_unilidar_lidar',
        output='screen',
        arguments=[
            '--x', '0.007698', '--y', '0.014655', '--z', '-0.00667',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'base_link', '--child-frame-id', 'unilidar_lidar',
        ],
        parameters=[{'use_sim_time': False}],
    )

    base_to_imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_unilidar_imu',
        output='screen',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'base_link', '--child-frame-id', 'unilidar_imu',
        ],
        parameters=[{'use_sim_time': False}],
    )

    # 动作 B: 启动 Point-LIO (复用它自带的 launch)
    point_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(point_lio_dir, 'launch', 'mapping_unilidar_l1.launch.py')
        ),
        # 覆盖 rviz 参数: Point-LIO 自带的 launch 也会启动 RViz,
        # 我们在这里统一用外层的 rviz 参数控制
        launch_arguments={'rviz': 'false'}.items()
    )

    # 动作 C: RViz2 使用展示配置 (显示 /Laser_map 地图 + 轨迹 + 里程计)
    rviz_config = os.path.join(mapping_launcher_dir, 'rviz', 'demo_point_lio.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # ==========================================
    # 4. 编排启动顺序
    # ==========================================
    return LaunchDescription([
        rviz_arg,

        # 第 0 秒: 立刻启动激光雷达 & IMU 驱动
        lidar_driver_node,
        base_to_lidar_tf,
        base_to_imu_tf,

        # 第 2 秒: 等传感器数据稳定后启动 Point-LIO
        TimerAction(
            period=2.0,
            actions=[point_lio_launch]
        ),

        # 第 4 秒: 等 Point-LIO 开始输出后启动 RViz
        TimerAction(
            period=4.0,
            actions=[rviz_node]
        ),
    ])
