from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare the 'port' launch argument with a default value
    port_arg = DeclareLaunchArgument(
        'port',
        description='The TCP port for communication with the SICK FlexiSoft controller'
    )

    # Get the path to the package's share directory
    pkg_share_path = get_package_share_directory('sick_flexisoft_eth_comm')

    # Define the node, using the launch argument for the port parameter
    node = Node(
        package='sick_flexisoft_eth_comm',
        executable='sick_flexisoft_eth_comm_node',
        name='sick_flexisoft_eth_comm',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port')
        }]
    )

    return LaunchDescription([
        port_arg,
        node,
    ])