import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='mobotic',
        description='Namespace for the path following nodes'
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true'
    )

    # Path to your config file
    config_path = os.path.join(
        get_package_share_directory('mobotic_launch'),
        'config',
        'ekf.yaml'
    )

    return LaunchDescription([
        declare_robot_name_arg,
        declare_use_sim_time_arg,
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config_path],
            namespace=robot_name
        )
    ])