from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generates the launch description for the vehicle supervisor node."""

    # Declare launch arguments for each parameter
    max_velocity_safety_disabled_arg = DeclareLaunchArgument(
        'max_velocity_safety_disabled',
        description='Maximum velocity when safety is disabled'
    )

    max_velocity_warning_breached_arg = DeclareLaunchArgument(
        'max_velocity_warning_breached',
        description='Maximum velocity when SICK warning zone is breached'
    )

    max_acceleration_arg = DeclareLaunchArgument(
        'max_acceleration',
        description='Maximum acceleration  defined by vehicle construction'
    )

    max_velocity_arg = DeclareLaunchArgument(
        'max_velocity',
        description='Maximum velocity defined by vehicle construction'
    )

    # Define the node and pass the launch arguments as parameters
    return LaunchDescription([
        max_velocity_safety_disabled_arg,
        max_velocity_warning_breached_arg,
        max_acceleration_arg,
        max_velocity_arg,
        Node(
            package='terraspiro_vehicle_supervisor',
            executable='terraspiro_vehicle_supervisor',
            name='vehicle_supervisor',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'max_velocity_safety_disabled': LaunchConfiguration('max_velocity_safety_disabled')},
                {'max_velocity_warning_breached': LaunchConfiguration('max_velocity_warning_breached')},
                {'max_acceleration': LaunchConfiguration('max_acceleration')},
                {'max_velocity': LaunchConfiguration('max_velocity')}
            ]
        )
    ])
