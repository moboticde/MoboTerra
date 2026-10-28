from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value=EnvironmentVariable('ROBOT_NAME', default_value=''),
            description='Namespace containing the existing can_rx topic',
        ),
        DeclareLaunchArgument(
            'can_interface',
            default_value='can1',
            description='CANable SocketCAN interface used for battery telemetry',
        ),
        DeclareLaunchArgument(
            'record_on_start',
            default_value='false',
            choices=['true', 'false'],
            description='Start JSON recording when the monitor starts',
        ),
        DeclareLaunchArgument(
            'recording_dir',
            default_value='can_telemetry_records',
            description='Directory for timestamped Vanguard charge JSON files',
        ),
        DeclareLaunchArgument(
            'recording_hz',
            default_value='10.0',
            description='JSON snapshot rate',
        ),
        DeclareLaunchArgument(
            'battery_source_address',
            default_value='0xF3',
            description='J1939 source address of the parallel battery controller',
        ),
        DeclareLaunchArgument(
            'battery_soc_pgn',
            default_value='0xF091',
            description='PGN carrying battery state of charge',
        ),
        DeclareLaunchArgument(
            'battery_electrical_pgn',
            default_value='0xF090',
            description='PGN carrying battery pack voltage and current',
        ),
        GroupAction(actions=[
            PushRosNamespace(LaunchConfiguration('robot_name')),
            PushRosNamespace('battery_can'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('ros2_socketcan'),
                        'launch',
                        'socket_can_receiver.launch.py',
                    ])
                ]),
                launch_arguments={
                    'interface': LaunchConfiguration('can_interface'),
                    'interval_sec': '0.01',
                    'enable_can_fd': 'false',
                    'from_can_bus_topic': 'can_rx',
                }.items(),
            ),
        ]),
        Node(
            package='vanguard_battery_driver',
            executable='vanguard_charge_monitor',
            name='vanguard_charge_monitor',
            namespace=LaunchConfiguration('robot_name'),
            output='screen',
            parameters=[{
                'stale_timeout_sec': 0.75,
                'active_current_threshold_a': 1.0,
                'record_on_start': LaunchConfiguration('record_on_start'),
                'recording_dir': LaunchConfiguration('recording_dir'),
                'recording_hz': LaunchConfiguration('recording_hz'),
                'recording_prefix': 'vanguard_charge',
                'battery_source_address': LaunchConfiguration('battery_source_address'),
                'battery_soc_pgn': LaunchConfiguration('battery_soc_pgn'),
                'battery_electrical_pgn': LaunchConfiguration('battery_electrical_pgn'),
            }],
            remappings=[
                ('can_rx', 'battery_can/can_rx'),
            ],
        ),
    ])
