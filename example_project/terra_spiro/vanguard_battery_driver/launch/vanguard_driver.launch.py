from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments for all parameters
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value=EnvironmentVariable('ROBOT_NAME', default_value=''),
        description='Namespace for the node'
    )
    can_device_arg = DeclareLaunchArgument(
        'can_device',
        description='CAN device name'
    )
    source_address_arg = DeclareLaunchArgument(
        'source_address',
        description='Source address'
    )
    high_voltage_publish_period_arg = DeclareLaunchArgument(
        'high_voltage_publish_period',
        description='High voltage publish period'
    )
    batteries_ids_arg = DeclareLaunchArgument(
        'batteries.ids',
        description='Battery IDs'
    )
    batteries_status_pgns_arg = DeclareLaunchArgument(
        'batteries.status_pgns',
        description='Battery status PGNs'
    )
    batteries_source_addresses_arg = DeclareLaunchArgument(
        'batteries.source_addresses',
        description='Battery source addresses'
    )

    op_consent_duration_arg = DeclareLaunchArgument(
        'op_consent_duration',
        description='Duration of sending op_consent to the battery.'
    )

    mobotic_driver_source_address_arg = DeclareLaunchArgument(
        'mobotic_driver_source_address',
        description='CAN source address of Mobotic driver (needed for heartbeat checking)'
    )

    mobotic_driver_watchdog_timeout_arg = DeclareLaunchArgument(
        'mobotic_driver_watchdog_timeout',
        description='Timeout threshold for Mobotic driver CAN heartbeat watchdog'
    )

    # Define the vanguard_driver node
    vanguard_driver_node = Node(
        package='vanguard_battery_driver',
        executable='vanguard_driver',
        name='vanguard_battery_driver',
        output='screen',
        namespace=LaunchConfiguration('robot_name'),
        respawn=True,
        parameters=[{
            'can_device': LaunchConfiguration('can_device'),
            'source_address': LaunchConfiguration('source_address'),
            'high_voltage_publish_period': LaunchConfiguration('high_voltage_publish_period'),
            'batteries.ids': LaunchConfiguration('batteries.ids'),
            'batteries.status_pgns': LaunchConfiguration('batteries.status_pgns'),
            'batteries.source_addresses': LaunchConfiguration('batteries.source_addresses'),
            'op_consent_duration': LaunchConfiguration('op_consent_duration'),
            'mobotic_driver_watchdog_timeout': LaunchConfiguration('mobotic_driver_watchdog_timeout'),
            'mobotic_driver_source_address': LaunchConfiguration('mobotic_driver_source_address')
        }]
    )

    return LaunchDescription([
        robot_name_arg,
        can_device_arg,
        source_address_arg,
        high_voltage_publish_period_arg,
        batteries_ids_arg,
        batteries_status_pgns_arg,
        batteries_source_addresses_arg,
        op_consent_duration_arg,
        mobotic_driver_source_address_arg,
        mobotic_driver_watchdog_timeout_arg,
        vanguard_driver_node,
    ])
