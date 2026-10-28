from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value=EnvironmentVariable('ROBOT_NAME', default_value='my_robot'),
        description='The name of the robot'
    )

    enable_all_drives_topic = DeclareLaunchArgument(
        'enable_all_drives_topic',
        description='The remapped topic for enabling all drives'
    )

    # Joystick node parameters as launch arguments
    joy_autorepeat_rate = DeclareLaunchArgument(
        'joy_autorepeat_rate',
        description='The autorepeat rate for the joystick node'
    )
    joy_default_trig_val = DeclareLaunchArgument(
        'joy_default_trig_val',
        description='The default trigger value for the joystick node'
    )

    # Teleoperation parameters as launch arguments
    max_velocity_linear = DeclareLaunchArgument('max_velocity_linear')
    max_velocity_angular = DeclareLaunchArgument('max_velocity_angular')
    max_acceleration_linear = DeclareLaunchArgument('max_acceleration_linear')
    max_acceleration_angular = DeclareLaunchArgument('max_acceleration_angular')
    axis_forward = DeclareLaunchArgument('axis_forward')
    axis_reverse = DeclareLaunchArgument('axis_reverse')
    axis_steer = DeclareLaunchArgument('axis_steer')
    axis_crab = DeclareLaunchArgument('axis_crab')
    button_turbo_1 = DeclareLaunchArgument('button_turbo_1')
    button_turbo_2 = DeclareLaunchArgument('button_turbo_2')
    button_enable = DeclareLaunchArgument('button_enable')
    button_disable = DeclareLaunchArgument('button_disable')
    button_clear = DeclareLaunchArgument('button_clear')
    scale_linear = DeclareLaunchArgument('scale_linear')
    scale_angular = DeclareLaunchArgument('scale_angular')
    commands_timeout_sec = DeclareLaunchArgument('commands_timeout_sec')
    commands_publish_sec = DeclareLaunchArgument('commands_publish_sec')

    # Node for the joystick
    joystick_node = Node(
        package='joy',
        executable='joy_node',
        name='joystick',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[
            {'autorepeat_rate': LaunchConfiguration('joy_autorepeat_rate')},
            {'default_trig_val': LaunchConfiguration('joy_default_trig_val')}
        ]
    )

    # Node for the teleoperation
    teleop_node = Node(
        package='four_wheel_teleop',
        executable='teleop_node',
        name='teleop',
        namespace=LaunchConfiguration('robot_name'),
        remappings=[
            ('enable_all_drives', LaunchConfiguration('enable_all_drives_topic'))
        ],
        parameters=[{
            'max_velocity_linear': LaunchConfiguration('max_velocity_linear'),
            'max_velocity_angular': LaunchConfiguration('max_velocity_angular'),
            'max_acceleration_linear': LaunchConfiguration('max_acceleration_linear'),
            'max_acceleration_angular': LaunchConfiguration('max_acceleration_angular'),
            'axis_forward': LaunchConfiguration('axis_forward'),
            'axis_reverse': LaunchConfiguration('axis_reverse'),
            'axis_steer': LaunchConfiguration('axis_steer'),
            'axis_crab': LaunchConfiguration('axis_crab'),
            'button_turbo_1': LaunchConfiguration('button_turbo_1'),
            'button_turbo_2': LaunchConfiguration('button_turbo_2'),
            'button_enable': LaunchConfiguration('button_enable'),
            'button_disable': LaunchConfiguration('button_disable'),
            'button_clear': LaunchConfiguration('button_clear'),
            'scale_linear': LaunchConfiguration('scale_linear'),
            'scale_angular': LaunchConfiguration('scale_angular'),
            'commands_timeout_sec': LaunchConfiguration('commands_timeout_sec'),
            'commands_publish_sec': LaunchConfiguration('commands_publish_sec'),
        }]
    )

    return LaunchDescription([
        # Arguments
        robot_name,
        enable_all_drives_topic,
        joy_autorepeat_rate,
        joy_default_trig_val,
        max_velocity_linear,
        max_velocity_angular,
        max_acceleration_linear,
        max_acceleration_angular,
        axis_forward,
        axis_reverse,
        axis_steer,
        axis_crab,
        button_turbo_1,
        button_turbo_2,
        button_enable,
        button_disable,
        button_clear,
        scale_linear,
        scale_angular,
        commands_timeout_sec,
        commands_publish_sec,

        # Nodes
        joystick_node,
        teleop_node,
    ])
