from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("mobotic_localization")
    slam_params = Path(pkg_share, "config", "slam_calibration.yaml")

    declare_args = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("slam_params_file", default_value=str(slam_params)),
        DeclareLaunchArgument("scan_topic", default_value="/mobotic/front_scanner/scan"),
        DeclareLaunchArgument("odom_topic", default_value="/mobotic/odometry"),
        DeclareLaunchArgument("laser_frame", default_value="front_scan"),
        DeclareLaunchArgument("map_frame", default_value="maps"),
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
        arguments=["--ros-args", "--log-level", "slam_toolbox:=debug"],
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

    calibrator_node = Node(
        package="mobotic_localization",
        executable="laser_tf_calibrator",
        name="laser_tf_calibrator",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "map_frame": LaunchConfiguration("map_frame"),
                "laser_frame": LaunchConfiguration("laser_frame"),
            }
        ],
    )

    return LaunchDescription(
        declare_args + [slam_toolbox_node, lifecycle_manager, calibrator_node]
    )
