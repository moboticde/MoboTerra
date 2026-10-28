from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    # Declare the namespace argument
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value=EnvironmentVariable('ROBOT_NAME', default_value='my_robot'),
        description='Namespace for the node'
    )

    # Declare the node's parameters as launch arguments
    can_node_id_arg = DeclareLaunchArgument(
        'can_node_id',
        description='The CAN node ID for the virtual drive'
    )

    # Define the virtual_mobotic_drive node
    virtual_drive_node = Node(
        package='mobotic_driver',
        executable='virtual_mobotic_drive',
        name='virtual_mobotic_drive',
        output='screen',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[{
            'can_node_id': LaunchConfiguration('can_node_id')
        }]
    )

    return LaunchDescription([
        robot_name_arg,
        can_node_id_arg,
        virtual_drive_node,
    ])
