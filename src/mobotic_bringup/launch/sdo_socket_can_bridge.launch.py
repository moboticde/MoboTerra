"""SDO-only hardware: one paced sender shared by every producer on this bus."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    defaults = {
        'interface': 'can0', 'receiver_interval_sec': '0.1',
        'tx_max_queue_age_sec': '0.05',
        'from_can_bus_topic': 'can_rx', 'to_can_bus_topic': 'can_tx',
        'filters': '0:0,#1FFFFFFF', 'use_bus_time': 'true',
        'warn_on_receive_timeout': 'false',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('ros2_socketcan'), 'launch', 'socket_can_receiver.launch.py',
            ])),
            launch_arguments={
                'interface': LaunchConfiguration('interface'),
                'interval_sec': LaunchConfiguration('receiver_interval_sec'),
                'enable_can_fd': 'false',
                'from_can_bus_topic': LaunchConfiguration('from_can_bus_topic'),
                'filters': LaunchConfiguration('filters'),
                'use_bus_time': LaunchConfiguration('use_bus_time'),
                'warn_on_receive_timeout': LaunchConfiguration('warn_on_receive_timeout'),
            }.items(),
        ),
        Node(
            package='mobotic_driver', executable='paced_socket_can_sender',
            name='socket_can_sender', output='screen',
            parameters=[{
                'interface': LaunchConfiguration('interface'),
                'max_queue_age_sec': ParameterValue(LaunchConfiguration('tx_max_queue_age_sec'), value_type=float),
            }],
            remappings=[('can_tx', LaunchConfiguration('to_can_bus_topic'))],
        ),
    ])
