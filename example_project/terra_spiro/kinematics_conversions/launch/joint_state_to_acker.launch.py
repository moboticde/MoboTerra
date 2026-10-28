from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    # Declare launch arguments for all parameters
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value=EnvironmentVariable('ROBOT_NAME', default_value=''),
        description='Namespace for the node'
    )
    steering_joint_names_arg = DeclareLaunchArgument(
        'steering_joint_names',
        description='List of steering joint names'
    )
    traction_joint_names_arg = DeclareLaunchArgument(
        'traction_joint_names',
        description='List of traction joint names'
    )
    wheels_x_arg = DeclareLaunchArgument(
        'wheels_x',
        description='X coordinates of the wheels'
    )
    wheels_y_arg = DeclareLaunchArgument(
        'wheels_y',
        description='Y coordinates of the wheels'
    )
    wheelbase_arg = DeclareLaunchArgument(
        'wheelbase',
        description='The distance between the front and rear axles'
    )
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        description='The radius of the wheels'
    )

    # Node configuration
    js_to_acker_node = Node(
        package='kinematics_conversions',
        executable='joint_states_to_acker.py',
        name='js_to_acker_converter',
        output='screen',
        parameters=[{
            'steering_joint_names': LaunchConfiguration('steering_joint_names'),
            'traction_joint_names': LaunchConfiguration('traction_joint_names'),
            'wheels_x': LaunchConfiguration('wheels_x'),
            'wheels_y': LaunchConfiguration('wheels_y'),
            'wheelbase': LaunchConfiguration('wheelbase'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
        }],
        namespace=LaunchConfiguration('robot_name')
    )

    return LaunchDescription([
        # Arguments
        robot_name_arg,
        steering_joint_names_arg,
        traction_joint_names_arg,
        wheels_x_arg,
        wheels_y_arg,
        wheelbase_arg,
        wheel_radius_arg,

        # Node
        js_to_acker_node
    ])