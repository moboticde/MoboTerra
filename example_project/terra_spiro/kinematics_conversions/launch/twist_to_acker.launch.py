from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value=EnvironmentVariable('ROBOT_NAME', default_value=''),
        description='Namespace for the node'
    )

    wheelbase_arg = DeclareLaunchArgument(
        'wheelbase',
        description='Distance between front and rear axles'
    )

    twist_to_acker_node = Node(
        package='kinematics_conversions',
        executable='twist_to_acker',
        name='twist_to_acker_converter',
        output='screen',
        parameters=[{
            'wheelbase': LaunchConfiguration('wheelbase')
        }],
        namespace=LaunchConfiguration('robot_name'),
    )

    return LaunchDescription([
        # Arguments
        robot_name_arg,
        wheelbase_arg,

        # Nodes
        twist_to_acker_node,
    ])
