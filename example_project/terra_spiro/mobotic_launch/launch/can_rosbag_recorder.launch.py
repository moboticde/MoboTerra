"""Launch combined platform and Vanguard CAN telemetry recording."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_name = LaunchConfiguration("robot_name")

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_name",
            default_value=EnvironmentVariable("ROBOT_NAME", default_value="mobotic"),
            description="Robot namespace",
        ),
        DeclareLaunchArgument(
            "battery_can_interface",
            default_value="can1",
            description="SocketCAN interface used for Vanguard battery telemetry",
        ),
        DeclareLaunchArgument(
            "recording_dir",
            default_value="can_telemetry_records",
            description="Directory for combined telemetry JSON files",
        ),
        DeclareLaunchArgument(
            "recorder_record_on_start",
            default_value="true",
            choices=["true", "false"],
            description="Start the combined JSON recording immediately",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("vanguard_battery_driver"),
                    "launch",
                    "vanguard_charge_monitor.launch.py",
                ])
            ]),
            launch_arguments={
                "robot_name": robot_name,
                "can_interface": LaunchConfiguration("battery_can_interface"),
                # The combined recorder owns the only output file.
                "record_on_start": "false",
            }.items(),
        ),
        Node(
            package="mobotic_launch",
            executable="can_rosbag_recorder.py",
            name="can_rosbag_recorder",
            namespace=robot_name,
            output="screen",
            parameters=[{
                "recording_dir": LaunchConfiguration("recording_dir"),
                "recording_format": "json",
                "record_on_start": LaunchConfiguration("recorder_record_on_start"),
            }],
        ),
    ])
