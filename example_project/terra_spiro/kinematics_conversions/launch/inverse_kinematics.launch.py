from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value=EnvironmentVariable('ROBOT_NAME', default_value='my_robot'),
        description='The name of the robot'
    )

    wheelbase_arg = DeclareLaunchArgument('wheelbase',
                                          description='Distance between the front and back wheel axles')
    track_arg = DeclareLaunchArgument('track',
                                      description='Distance between the left and right wheels')
    wheel_radius_arg = DeclareLaunchArgument('wheel_radius')
    max_velocity_steering_arg = DeclareLaunchArgument('max_velocity_steering',
                                                      description='Max velocity - steering (rad/s)')
    max_velocity_traction_arg = DeclareLaunchArgument('max_velocity_traction',
                                                      description='Max velocity - traction (rad/s)')
    max_current_traction_arg = DeclareLaunchArgument('max_current_traction',
                                                     description='Max current - traction (A)')
    wheel_mount_angles_arg = DeclareLaunchArgument('wheel_mount_angles',
                                                   description='Wheel mounting angles (rad)')

    # Create launch configurations for the arguments
    robot_name = LaunchConfiguration('robot_name')
    wheelbase = LaunchConfiguration('wheelbase')
    track = LaunchConfiguration('track')
    wheel_radius = LaunchConfiguration('wheel_radius')
    max_velocity_steering = LaunchConfiguration('max_velocity_steering')
    max_velocity_traction = LaunchConfiguration('max_velocity_traction')
    max_current_traction = LaunchConfiguration('max_current_traction')
    wheel_mount_angles = LaunchConfiguration('wheel_mount_angles')

    # Define the inverse_kinematics_node
    inverse_kinematics_node = Node(
        package='kinematics_conversions',
        executable='inverse_kinematics_node',
        name='kinematics',
        namespace=robot_name,
        parameters=[{
            'wheelbase': wheelbase,
            'track': track,
            'wheel_radius': wheel_radius,
            'max_velocity_steering': max_velocity_steering,
            'max_velocity_traction': max_velocity_traction,
            'max_current_traction': max_current_traction,
            'wheel_mount_angles': wheel_mount_angles,
        }]
    )

    return LaunchDescription([
        robot_name,
        wheelbase_arg,
        track_arg,
        wheel_radius_arg,
        max_velocity_steering_arg,
        max_velocity_traction_arg,
        max_current_traction_arg,
        wheel_mount_angles_arg,
        inverse_kinematics_node,
    ])
