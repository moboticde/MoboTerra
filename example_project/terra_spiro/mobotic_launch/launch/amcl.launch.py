import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path to your config file
    config_file = '/home/tibor/devel/mobotic/testing_platform_ws/src/terra_spiro/mobotic_launch/config/amcl.config.yaml' # Update with absolute path using get_package_share_directory in a real package

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[config_file]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[config_file]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}]
        )
    ])