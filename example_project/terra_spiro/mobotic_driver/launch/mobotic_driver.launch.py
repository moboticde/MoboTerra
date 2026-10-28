from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    # Declare launch arguments for all parameters
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value=EnvironmentVariable('ROBOT_NAME', default_value=''),
        description='Namespace for the node'
    )
    can_node_id_arg = DeclareLaunchArgument(
        'can_node_id',
        description='CAN node ID'
    )
    feedback_request_period_arg = DeclareLaunchArgument(
        'feedback_request_period',
        description='Feedback request period in seconds'
    )
    can_heartbeat_period_arg = DeclareLaunchArgument(
        'can_heartbeat_period',
        description='CAN heartbeat period in seconds'
    )
    watchdog_timeout_arg = DeclareLaunchArgument(
        'watchdog_timeout',
        description='Watchdog timeout in seconds'
    )
    diagnostic_sdo_period_arg = DeclareLaunchArgument(
        'diagnostic_sdo_period',
        default_value='1.0',
        description='Period for operational status and error SDO reads'
    )
    traction_pdo_enabled_arg = DeclareLaunchArgument(
        'traction_pdo_enabled',
        default_value='true',
        description='Use PDOs for traction commands and feedback'
    )
    telemetry_pdo_enabled_arg = DeclareLaunchArgument(
        'telemetry_pdo_enabled',
        default_value='true',
        description='Use PDOs for steering motion, current, temperature, and voltage'
    )
    telemetry_pdo_sync_divider_arg = DeclareLaunchArgument(
        'telemetry_pdo_sync_divider',
        default_value='1',
        description='Emit telemetry every N SYNC frames (1 at a 50 Hz SYNC rate gives 50 Hz)'
    )
    traction_pdo_period_arg = DeclareLaunchArgument(
        'traction_pdo_period',
        default_value='0.02',
        description='CANopen SYNC and traction command period in seconds'
    )
    traction_max_profile_velocity_arg = DeclareLaunchArgument(
        'traction_max_profile_velocity',
        default_value='185429',
        description='CANopen 0x607F maximum profile velocity in controller increments per second'
    )
    drives_names_arg = DeclareLaunchArgument(
        'drives.names',
        description='Names of the drives'
    )
    drives_can_node_ids_arg = DeclareLaunchArgument(
        'drives.can_node_ids',
        description='CAN node IDs for the drives'
    )
    drives_modes_arg = DeclareLaunchArgument(
        'drives.modes',
        description='Operating modes for the drives'
    )
    drives_resolutions_arg = DeclareLaunchArgument(
        'drives.resolutions',
        description='Resolutions for the drives in increments'
    )
    drives_min_steering_velocities_arg = DeclareLaunchArgument(
        'drives.min_steering_velocities',
        description='Minimum steering velocities in rpm'
    )
    drives_max_steering_velocities_arg = DeclareLaunchArgument(
        'drives.max_steering_velocities',
        description='Maximum steering velocities in rpm'
    )
    drives_min_traction_velocities_arg = DeclareLaunchArgument(
        'drives.min_traction_velocities',
        description='Minimum EC traction velocities in controller increments per second'
    )
    drives_max_traction_velocities_arg = DeclareLaunchArgument(
        'drives.max_traction_velocities',
        description='Maximum EC traction velocities in controller increments per second'
    )
    drives_gear_ratios_arg = DeclareLaunchArgument(
        'drives.gear_ratios',
        description='Gear ratios for the drives'
    )
    drives_min_currents_arg = DeclareLaunchArgument(
        'drives.min_currents',
        description='Minimum currents in mA'
    )
    drives_max_currents_arg = DeclareLaunchArgument(
        'drives.max_currents',
        description='Maximum currents in mA'
    )

    # Define the node
    mobotic_driver_node = Node(
        package='mobotic_driver',
        executable='mobotic_driver',
        name='mobotic_driver',
        output='screen',
        namespace=LaunchConfiguration('robot_name'),
        respawn=True,
        parameters=[{
            'can_node_id': LaunchConfiguration('can_node_id'),
            'feedback_request_period': LaunchConfiguration('feedback_request_period'),
            'can_heartbeat_period': LaunchConfiguration('can_heartbeat_period'),
            'watchdog_timeout': LaunchConfiguration('watchdog_timeout'),
            'diagnostic_sdo_period': LaunchConfiguration('diagnostic_sdo_period'),
            'traction_pdo_enabled': LaunchConfiguration('traction_pdo_enabled'),
            'telemetry_pdo_enabled': LaunchConfiguration('telemetry_pdo_enabled'),
            'telemetry_pdo_sync_divider': LaunchConfiguration('telemetry_pdo_sync_divider'),
            'traction_pdo_period': LaunchConfiguration('traction_pdo_period'),
            'traction_max_profile_velocity': LaunchConfiguration('traction_max_profile_velocity'),
            'drives.names': LaunchConfiguration('drives.names'),
            'drives.can_node_ids': LaunchConfiguration('drives.can_node_ids'),
            'drives.modes': LaunchConfiguration('drives.modes'),
            'drives.resolutions': LaunchConfiguration('drives.resolutions'),
            'drives.min_steering_velocities': LaunchConfiguration('drives.min_steering_velocities'),
            'drives.max_steering_velocities': LaunchConfiguration('drives.max_steering_velocities'),
            'drives.min_traction_velocities': LaunchConfiguration('drives.min_traction_velocities'),
            'drives.max_traction_velocities': LaunchConfiguration('drives.max_traction_velocities'),
            'drives.gear_ratios': LaunchConfiguration('drives.gear_ratios'),
            'drives.min_currents': LaunchConfiguration('drives.min_currents'),
            'drives.max_currents': LaunchConfiguration('drives.max_currents'),
        }]
    )

    return LaunchDescription([
        # Arguments
        robot_name_arg,
        can_node_id_arg,
        feedback_request_period_arg,
        can_heartbeat_period_arg,
        watchdog_timeout_arg,
        diagnostic_sdo_period_arg,
        traction_pdo_enabled_arg,
        telemetry_pdo_enabled_arg,
        telemetry_pdo_sync_divider_arg,
        traction_pdo_period_arg,
        traction_max_profile_velocity_arg,
        drives_names_arg,
        drives_can_node_ids_arg,
        drives_modes_arg,
        drives_resolutions_arg,
        drives_min_steering_velocities_arg,
        drives_max_steering_velocities_arg,
        drives_min_traction_velocities_arg,
        drives_max_traction_velocities_arg,
        drives_gear_ratios_arg,
        drives_min_currents_arg,
        drives_max_currents_arg,

        # Nodes
        mobotic_driver_node,
    ])
