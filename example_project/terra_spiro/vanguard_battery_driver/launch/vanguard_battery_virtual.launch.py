from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments for the namespace and parameters
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value=EnvironmentVariable('ROBOT_NAME', default_value=''),
        description='Namespace for the node'
    )
    battery_id_arg = DeclareLaunchArgument(
        'battery_id',
        description='ID of the battery'
    )
    source_address_arg = DeclareLaunchArgument(
        'source_address',
        description='Source address of the battery'
    )
    status_pgn_arg = DeclareLaunchArgument(
        'status_pgn',
        description='Status PGN (Parameter Group Number) of the battery'
    )
    controller_address_arg = DeclareLaunchArgument(
        'controller_address',
        description='Controller address for communication'
    )

    # Define the virtual_vanguard_battery node
    virtual_vanguard_battery_node = Node(
        package='vanguard_battery_driver',
        executable='virtual_vanguard_battery',
        name='virtual_vanguard_battery',
        output='screen',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[{
            'battery_id': LaunchConfiguration('battery_id'),
            'source_address': LaunchConfiguration('source_address'),
            'status_pgn': LaunchConfiguration('status_pgn'),
            'controller_address': LaunchConfiguration('controller_address')
        }]
    )

    return LaunchDescription([
        # Arguments
        robot_name_arg,
        battery_id_arg,
        source_address_arg,
        status_pgn_arg,
        controller_address_arg,

        # Node
        virtual_vanguard_battery_node,
    ])
