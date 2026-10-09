from mobotic_config.launching import common_arguments, load_from_context, deployment_actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def _stack_condition(stack_type, expected):
    return IfCondition(
        PythonExpression(["'", stack_type, "' == '", expected, "'"])
    )


def _hardware_component_condition(stack_type, enabled, *other_enabled):
    expression = ["'", stack_type, "' == 'hardware'"]
    for flag in (enabled, *other_enabled):
        expression.extend([" and '", flag, "' == 'true'"])
    return IfCondition(
        PythonExpression(expression)
    )


def _launch_setup(context):
    cfg = load_from_context(context, {
        'driver': 'driver_parameters_file', 'kinematics': 'kinematics_parameters_file',
        'manual': 'manual_parameters_file', 'safety': 'safety_parameters_file',
        'battery': 'battery_parameters_file', 'supervisor': 'supervisor_parameters_file',
        'odometry': 'odometry_parameters_file', 'scanners': 'scanner_parameters_file',
        'diagnostics': 'diagnostics_parameters_file',
    })
    resolved = deployment_actions(context, cfg)
    robot_name = LaunchConfiguration('robot_name')
    stack_type = LaunchConfiguration('stack_type')
    can_interface = LaunchConfiguration('can_interface')
    battery_can_interface = LaunchConfiguration('battery_can_interface')
    start_manual_interface = LaunchConfiguration('start_manual_interface')
    start_safety = LaunchConfiguration('start_safety')
    start_battery = LaunchConfiguration('start_battery')
    start_socketcan = LaunchConfiguration('start_socketcan')
    start_battery_socketcan = LaunchConfiguration('start_battery_socketcan')
    start_scanners = LaunchConfiguration('start_scanners')
    start_odometry = LaunchConfiguration('start_odometry')

    driver_config = cfg.parameters('driver')
    kinematics_config = cfg.parameters('kinematics')
    manual_config = cfg.parameters('manual')
    safety_config = cfg.parameters('safety')
    battery_config = cfg.parameters('battery')
    supervisor_config = cfg.parameters('supervisor')

    socketcan = GroupAction(
        condition=_hardware_component_condition(stack_type, start_socketcan),
        actions=[
            PushRosNamespace(robot_name),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare('ros2_socketcan'),
                            'launch',
                            'socket_can_bridge.launch.xml',
                        ]
                    )
                ),
                launch_arguments={
                    'interface': can_interface,
                    'from_can_bus_topic': 'can_rx',
                    'to_can_bus_topic': 'can_tx',
                    # Preserve all data traffic and opt in to SocketCAN errors.
                    'filters': '0:0,#1FFFFFFF',
                    'receiver_interval_sec': '0.01',
                    'use_bus_time': 'true',
                    'sender_timeout_sec': '0.05',
                }.items(),
            ),
        ],
    )

    battery_socketcan = GroupAction(
        condition=_hardware_component_condition(
            stack_type, start_battery_socketcan, start_battery
        ),
        actions=[
            PushRosNamespace(robot_name),
            PushRosNamespace('battery_can'),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('ros2_socketcan'),
                        'launch',
                        'socket_can_receiver.launch.py',
                    ])
                ),
                launch_arguments={
                    'interface': battery_can_interface,
                    'from_can_bus_topic': 'can_rx',
                    'enable_can_fd': 'false',
                    'filters': '0:0',
                    'interval_sec': '0.01',
                    'use_bus_time': 'true',
                }.items(),
            ),
        ],
    )

    hardware_driver = Node(
        condition=_stack_condition(stack_type, 'hardware'),
        package='mobotic_driver',
        executable='mobotic_driver',
        name='mobotic_driver',
        namespace=robot_name,
        parameters=[driver_config],
        output='screen',
        respawn=True,
    )
    virtual_driver = Node(
        condition=_stack_condition(stack_type, 'virtual'),
        package='mobotic_driver',
        executable='mobotic_driver',
        name='mobotic_driver',
        namespace=robot_name,
        parameters=[driver_config],
        output='screen',
        respawn=True,
    )

    virtual_drives = [
        Node(
            condition=_stack_condition(stack_type, 'virtual'),
            package='mobotic_driver',
            executable='virtual_mobotic_drive',
            name=f'virtual_{name}',
            namespace=robot_name,
            parameters=[parameters],
            remappings=[('can_rx', 'can_tx'), ('can_tx', 'can_rx')],
            output='screen',
        )
        for name, parameters in cfg.virtual_drives()
    ]

    mock_platform_state = Node(
        condition=_stack_condition(stack_type, 'virtual'),
        package='mobotic_bringup',
        executable='mock_platform_state',
        name='mock_platform_state',
        namespace=robot_name,
        output='screen',
    )

    kinematics = Node(
        package='mobotic_kinematics',
        executable='mobotic_kinematics',
        name='mobotic_kinematics',
        namespace=robot_name,
        parameters=[kinematics_config],
        output='screen',
        respawn=True,
    )
    odometry = Node(
        condition=IfCondition(start_odometry),
        package='mobotic_odometry', executable='mobotic_odometry', name='mobotic_odometry',
        namespace=robot_name, output='screen',
        parameters=[cfg.parameters('odometry'), {
            'publish_tf': ParameterValue(LaunchConfiguration('publish_odom_tf'), value_type=bool),
        }],
        remappings=[('/tf', 'tf')],
        # An automatic restart would silently reset the local pose to its origin.
        respawn=False,
    )
    supervisor = Node(
        package='mobotic_supervisor',
        executable='mobotic_supervisor',
        name='mobotic_supervisor',
        namespace=robot_name,
        parameters=[supervisor_config],
        output='screen',
        respawn=True,
    )

    joystick = Node(
        condition=IfCondition(start_manual_interface),
        package='joy',
        executable='game_controller_node',
        name='joystick',
        namespace=robot_name,
        parameters=[cfg.platform['usb_joystick']],
        output='screen',
    )
    manual_control = Node(
        condition=IfCondition(start_manual_interface),
        package='mobotic_manual_control',
        executable='manual_control',
        name='mobotic_manual_control',
        namespace=robot_name,
        parameters=[manual_config],
        output='screen',
        respawn=True,
    )

    flexisoft_bridge = Node(
        condition=_hardware_component_condition(stack_type, start_safety),
        package='mobotic_safety',
        executable='flexisoft_tcp_bridge',
        name='flexisoft_tcp_bridge',
        namespace=robot_name,
        parameters=[safety_config],
        output='screen',
        respawn=True,
        respawn_delay=1.0,
    )
    safety_monitor = Node(
        condition=_hardware_component_condition(stack_type, start_safety),
        package='mobotic_safety',
        executable='safety_monitor',
        name='safety_monitor',
        namespace=robot_name,
        parameters=[safety_config],
        output='screen',
        respawn=True,
    )
    battery_monitor = Node(
        condition=_hardware_component_condition(stack_type, start_battery),
        package='mobotic_vanguard_battery',
        executable='vanguard_battery_monitor',
        name='vanguard_battery_monitor',
        namespace=robot_name,
        parameters=[battery_config],
        remappings=[('can_rx', 'battery_can/can_rx')],
        output='screen',
        respawn=True,
    )

    scanners = [
        Node(
            condition=_hardware_component_condition(stack_type, start_scanners),
            package='sick_safetyscanners2', executable='sick_safetyscanners2_node',
            name=f'{location}_scanner', namespace=robot_name, output='screen', respawn=True,
            parameters=[cfg.scanner_parameters(location)],
            remappings=[('~/scan', f'scanner/{location}/scan'),
                        ('~/raw_data', f'scanner/{location}/raw_data'),
                        ('~/output_paths', f'scanner/{location}/output_paths'),
                        ('~/extended_laser_scan', f'scanner/{location}/extended_laser_scan'),
                        ('~/diagnostics', 'diagnostics')],
        )
        for location, ip_argument in (('front_left', 'front_scanner_ip'), ('rear_right', 'rear_scanner_ip'))
    ]
    diagnostics = Node(
        package='mobotic_bringup', executable='platform_diagnostics',
        name='platform_diagnostics', namespace=robot_name, output='screen', respawn=True,
        parameters=[cfg.diagnostic_parameters(), {
            'monitor_manual': ParameterValue(start_manual_interface, value_type=bool),
            'monitor_odometry': ParameterValue(start_odometry, value_type=bool),
            'monitor_scanners': ParameterValue(PythonExpression([
                "'", stack_type, "' == 'hardware' and '", start_scanners, "' == 'true'"
            ]), value_type=bool),
        }],
    )

    return resolved + [
        socketcan,
        battery_socketcan,
        hardware_driver,
        virtual_driver,
        *virtual_drives,
        mock_platform_state,
        kinematics,
        odometry,
        joystick,
        manual_control,
        flexisoft_bridge,
        safety_monitor,
        battery_monitor,
        *scanners,
        diagnostics,
        supervisor,
    ]


def generate_launch_description():
    arguments = common_arguments() + [
        DeclareLaunchArgument('start_odometry', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('publish_odom_tf', default_value='auto', choices=['auto', 'true', 'false'],
                              description='Disable if an external state estimator owns odom -> base_link'),
        DeclareLaunchArgument('odometry_parameters_file', default_value=''),
        DeclareLaunchArgument('start_scanners', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('front_scanner_ip', default_value='auto'),
        DeclareLaunchArgument('rear_scanner_ip', default_value='auto'),
        DeclareLaunchArgument('scanner_host_ip', default_value='auto',
                              description='Host NIC address reachable by both scanners; verify on this machine'),
        DeclareLaunchArgument('scanner_parameters_file', default_value=''),
        DeclareLaunchArgument(
            'stack_type',
            default_value='hardware',
            choices=['hardware', 'virtual'],
            description='Select physical hardware or the local virtual CAN drives',
        ),
        DeclareLaunchArgument(
            'can_interface',
            default_value='auto',
            description='Drive SocketCAN interface used in hardware mode',
        ),
        DeclareLaunchArgument(
            'battery_can_interface',
            default_value='auto',
            description='Receive-only battery SocketCAN interface; separate from drives',
        ),
        DeclareLaunchArgument(
            'start_socketcan',
            default_value='true',
            choices=['true', 'false'],
            description='Start the drive ros2_socketcan bridge in hardware mode',
        ),
        DeclareLaunchArgument(
            'start_battery_socketcan',
            default_value='true',
            choices=['true', 'false'],
            description='Start the battery CAN receiver when hardware battery monitoring is enabled',
        ),
        DeclareLaunchArgument(
            'start_manual_interface',
            default_value='true',
            choices=['true', 'false'],
            description='Start game_controller_node and mobotic_manual_control',
        ),
        DeclareLaunchArgument(
            'start_safety',
            default_value='true',
            choices=['true', 'false'],
            description='Start the FlexiSoft monitoring nodes in hardware mode',
        ),
        DeclareLaunchArgument(
            'start_battery',
            default_value='true',
            choices=['true', 'false'],
            description='Start the Vanguard monitor in hardware mode',
        ),
        DeclareLaunchArgument(
            'driver_parameters_file',
            default_value='',
            description='Optional driver policy overlay; hardware comes from platform_config',
        ),
        DeclareLaunchArgument(
            'kinematics_parameters_file',
            default_value='',
            description='Optional kinematics policy overlay',
        ),
        DeclareLaunchArgument(
            'manual_parameters_file',
            default_value='',
            description='Optional manual-control policy overlay',
        ),
        DeclareLaunchArgument(
            'safety_parameters_file',
            default_value='',
            description='Optional safety policy overlay; signal mapping comes from platform_config',
        ),
        DeclareLaunchArgument(
            'battery_parameters_file',
            default_value='',
            description='Optional battery policy overlay; participants come from platform_config',
        ),
        DeclareLaunchArgument(
            'supervisor_parameters_file',
            default_value='',
            description='Optional supervisor policy overlay',
        ),
        DeclareLaunchArgument('diagnostics_parameters_file', default_value=''),
    ]

    return LaunchDescription(arguments + [OpaqueFunction(function=_launch_setup)])
