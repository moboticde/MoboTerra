from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            'robot_name',
            default_value=EnvironmentVariable('ROBOT_NAME', default_value=''),
            description='Namespace containing can_rx and can_tx',
        ),
        DeclareLaunchArgument(
            'enable_controller',
            default_value='false',
            choices=['true', 'false'],
            description='Commissioning interlock; disabled means no EFF3 CAN transmission',
        ),
        DeclareLaunchArgument(
            'system_capacity_ah',
            default_value='67.5',
            description='Total Ah capacity of the configured one-pack Vanguard Fi3.5 system',
        ),
        DeclareLaunchArgument(
            'maximum_c_rate',
            default_value='0.5',
            description='Maximum requested charge C-rate; values above 0.5 are rejected',
        ),
        DeclareLaunchArgument(
            'absolute_current_limit_a',
            default_value='60.0',
            description='Independent maximum EFF3 requested current in amperes',
        ),
        DeclareLaunchArgument(
            'soc_start_threshold_percent',
            default_value='50.0',
            description='SOC at or below which the charging cycle is enabled',
        ),
        DeclareLaunchArgument(
            'soc_stop_threshold_percent',
            default_value='80.0',
            description='SOC at or above which the charging cycle is disabled',
        ),
        DeclareLaunchArgument(
            'battery_soc_timeout_sec',
            default_value='0.75',
            description='Maximum accepted age of battery SOC data',
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
    ]

    controller = Node(
        package='vanguard_battery_driver',
        executable='vanguard_charge_rate_controller',
        name='vanguard_charge_rate_controller',
        namespace=LaunchConfiguration('robot_name'),
        output='screen',
        parameters=[{
            'enabled': LaunchConfiguration('enable_controller'),
            'system_capacity_ah': LaunchConfiguration('system_capacity_ah'),
            'maximum_c_rate': LaunchConfiguration('maximum_c_rate'),
            'absolute_current_limit_a': LaunchConfiguration('absolute_current_limit_a'),
            'soc_start_threshold_percent': LaunchConfiguration('soc_start_threshold_percent'),
            'soc_stop_threshold_percent': LaunchConfiguration('soc_stop_threshold_percent'),
            'battery_soc_timeout_sec': LaunchConfiguration('battery_soc_timeout_sec'),
            'source_address': ParameterValue('0xD0', value_type=str),
            'battery_source_address': ParameterValue(
                LaunchConfiguration('battery_source_address'), value_type=str
            ),
            'battery_soc_pgn': ParameterValue(
                LaunchConfiguration('battery_soc_pgn'), value_type=str
            ),
        }],
    )

    return LaunchDescription(arguments + [controller])
