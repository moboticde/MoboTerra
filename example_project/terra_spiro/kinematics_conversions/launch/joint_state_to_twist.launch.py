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
    wheels_x_arg = DeclareLaunchArgument(
        'wheels_x',
        description='X coordinates of the wheels'
    )
    wheels_y_arg = DeclareLaunchArgument(
        'wheels_y',
        description='Y coordinates of the wheels'
    )
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        description='The radius of the wheels'
    )
    steering_joint_names_arg = DeclareLaunchArgument(
        'steering_joint_names',
        description='List of steering joint names'
    )
    traction_joint_names_arg = DeclareLaunchArgument(
        'traction_joint_names',
        description='List of traction joint names'
    )

    # Define the node and pass the launch arguments as parameters
    js_to_twist_node = Node(
        package='kinematics_conversions',
        executable='joint_state_to_twist',
        name='js_to_twist_converter',
        output='screen',
        parameters=[{'wheel_mount_angles': [3.141592, 0],
                     'wheels_x': [0.51, -0.55],
                     'wheels_y': [-0.44, 0.44],
                     "steering_joint_names": ["front_steering", "rear_steering"],
                     "traction_joint_names": ["front_traction", "rear_traction"],
                     'wheel_radius': 0.1125}],
        namespace=LaunchConfiguration('robot_name')
    )




    return LaunchDescription([
        # Arguments
        robot_name_arg,
        #wheels_x_arg,
        #wheels_y_arg,
        #wheel_radius_arg,
        #steering_joint_names_arg,
        #traction_joint_names_arg,

        # Nodes
        js_to_twist_node
    ])
