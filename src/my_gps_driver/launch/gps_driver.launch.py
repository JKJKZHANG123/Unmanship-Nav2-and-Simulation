from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port', default_value='/dev/ttyUSB1',
        description='Serial device connected to the RTK/NMEA host')
    baud_arg = DeclareLaunchArgument(
        'baud', default_value='9600',
        description='RTK/NMEA serial baud rate')
    frame_arg = DeclareLaunchArgument(
        'frame_id', default_value='gps_link',
        description='Frame attached to the RTK antenna')

    gps_node = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        output='screen',
        name='gps_driver',
        parameters=[{
            'use_sim_time': False,
            'port': LaunchConfiguration('port'),
            'baud': ParameterValue(LaunchConfiguration('baud'), value_type=int),
            'frame_id': LaunchConfiguration('frame_id'),
        }],
        # Keep the historical interface used by the rest of the workspace.
        arguments=['--ros-args', '--log-level', 'error'],
        remappings=[
            ('/fix', '/gps/fix'),
            ('/vel', '/gps/vel'),
        ]
    )

    return LaunchDescription([port_arg, baud_arg, frame_arg, gps_node])
