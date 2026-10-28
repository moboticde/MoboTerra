from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("mobotic_localization")

    slam_params = Path(pkg_share, "config", "slam_mapping.yaml")

    declare_args = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("slam_params_file", default_value=str(slam_params)),
        DeclareLaunchArgument(
            "scan_topic", default_value="/mobotic/front_scanner/scan"
        ),
        DeclareLaunchArgument("odom_topic", default_value="/mobotic/odometry"),
        DeclareLaunchArgument("laser_frame", default_value="front_scan"),
        DeclareLaunchArgument("laser_x", default_value="1.050"),
        DeclareLaunchArgument("laser_y", default_value="0.661"),
        DeclareLaunchArgument("laser_z", default_value="0.0"),
        DeclareLaunchArgument("laser_yaw", default_value="0.785398163"),
        DeclareLaunchArgument("laser_pitch", default_value="0.0"),
        DeclareLaunchArgument("laser_roll", default_value="0.0"),
    ]

    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            LaunchConfiguration("slam_params_file"),
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "scan_topic": LaunchConfiguration("scan_topic"),
            },
        ],
    )

    odom_to_tf_node = Node(
        package="mobotic_localization",
        executable="odom_to_tf",
        name="odom_to_tf",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "odom_topic": LaunchConfiguration("odom_topic"),
            }
        ],
    )

    base_to_laser_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_laser_static_tf",
        arguments=[
            "--x", LaunchConfiguration("laser_x"),
            "--y", LaunchConfiguration("laser_y"),
            "--z", LaunchConfiguration("laser_z"),
            "--yaw", LaunchConfiguration("laser_yaw"),
            "--pitch", LaunchConfiguration("laser_pitch"),
            "--roll", LaunchConfiguration("laser_roll"),
            "--frame-id", "base_link",
            "--child-frame-id", LaunchConfiguration("laser_frame"),
        ],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "autostart": True,
                "node_names": ["slam_toolbox"],
                "bond_timeout": 0.0,
            }
        ],
    )

    return LaunchDescription(
        declare_args
        + [odom_to_tf_node, base_to_laser_tf, slam_toolbox_node, lifecycle_manager]
    )
