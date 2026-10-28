from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler, TimerAction, GroupAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                  LaunchConfiguration, PathJoinSubstitution, PythonExpression, EqualsSubstitution)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value=EnvironmentVariable('ROBOT_NAME', default_value=''),
        description='Namespace for the node'
    )

    stack_type_arg = DeclareLaunchArgument(
        'stack_type',
        default_value='real',
        description='Determines the stack type to be used: "real" or "virtual"',
        choices=['real', 'virtual']
    )

    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='Determines the CAN interface which is used for communication'
    )

    stack_type = LaunchConfiguration('stack_type')
    robot_name = LaunchConfiguration('robot_name')
    can_interface = LaunchConfiguration('can_interface')

    socket_can_params = {
        'interface': can_interface,
        'from_can_bus_topic': 'can_rx',
        'to_can_bus_topic': 'can_tx',
        'receiver_interval_sec': '0.01',
        'use_bus_time': 'true',
        'sender_timeout_sec': '0.01'
    }

    ec_traction_velocity_limit = 116053  # EC traction velocity limit in controller increments per second.

    mobotic_driver_params = {
        'can_node_id': '0x0A',
        'feedback_request_period': 0.02,
        'can_heartbeat_period': 0.01,
        'watchdog_timeout': 0.1,
        'drives.names': [
            'front_left_steering', 'rear_left_steering',
            'rear_right_steering', 'front_right_steering',
            'front_left_traction', 'rear_left_traction',
            'rear_right_traction', 'front_right_traction'
        ],
        'drives.can_node_ids': [
            '0x01', '0x07', '0x05', '0x03', '0x02', '0x08', '0x06', '0x04'
        ],
        'drives.modes': [
            'position', 'position', 'position', 'position',
            'velocity', 'velocity', 'velocity', 'velocity'
        ],
        'drives.resolutions': [
            4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096
        ],
        'drives.min_steering_velocities': [
            -5000, -5000, -5000, -5000, -1700, -1700, -1700, -1700
        ],
        'drives.max_steering_velocities': [
            5000, 5000, 5000, 5000, 1700, 1700, 1700, 1700
        ],
        'drives.min_traction_velocities': [
            -5000, -5000, -5000, -5000,
            -ec_traction_velocity_limit, -ec_traction_velocity_limit,
            -ec_traction_velocity_limit, -ec_traction_velocity_limit
        ],
        'drives.max_traction_velocities': [
            5000, 5000, 5000, 5000,
            ec_traction_velocity_limit, ec_traction_velocity_limit,
            ec_traction_velocity_limit, ec_traction_velocity_limit
        ],
        'drives.gear_ratios': [
            121.0, 121.0, 121.0, 121.0, 16.0, 16.0, 16.0, 16.0
        ],
        'drives.min_currents': [
            -4000, -4000, -4000, -4000, -4000, -4000, -4000, -4000
        ],
        'drives.max_currents': [
            4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000
        ]
    }

    vanguard_battery_driver_params = {
        'source_address': '0xD0',
        'high_voltage_publish_period': 0.02,
        'batteries.ids': ["parallel_battery_controller", "battery1", "battery2", "battery3", "battery4"],
        'batteries.status_pgns': ["0xF096", "0xF197", "0xF1A6", "0xF1B5", "0xF1C4"],
        'batteries.soc_pgns': ["0xF091", "0xF192", "0xF1A1", "0xF1B0", "0xF1BF"],
        'batteries.voltage_pgns': ["0xF090", "0xF191", "0xF1A0", "0xF1AF", "0xF1BE"],
        'batteries.source_addresses': ["0xF3", "0xF4", "0xF5", "0xF6", "0x80"],
        'batteries.types': ["controller", "battery", "battery", "battery", "battery"],
        'op_consent_duration': 1.5,
        'mobotic_driver_watchdog_timeout': 0.5,
        'mobotic_driver_source_address': "0x0A"
    }

    joy_params = {
        'joy_autorepeat_rate': 0.0,
        'joy_default_trig_val': True
    }

    teleop_params = {
        'max_velocity_linear': 1.25,
        'max_velocity_angular': 1.51,
        'max_acceleration_linear': 0.5,
        'max_acceleration_angular': 0.52,
        'axis_forward': 5,
        'axis_reverse': 2,
        'axis_steer': 0,
        'axis_crab': 3,
        'button_turbo_1': 4,
        'button_turbo_2': 5,
        'button_enable': 2,
        'button_disable': 3,
        'button_clear': 1,
        'scale_linear': 0.4,
        'scale_angular': 0.4,
        'commands_timeout_sec': 0.25,
        'commands_publish_sec': 0.02
    }

    kinematic_params = {
        'wheelbase': 1.65,
        'track': 0.99,
        'wheel_radius': 0.325,
        'max_velocity_steering': 4.32,
        'max_velocity_traction': 11.12,
        'max_current_traction': 4.0,
        'wheel_mount_angles': [0.0, 3.14152, 0.0, 3.14152],
        'wheels_x': [0.825, 0.825, -0.825, -0.825],
        'wheels_y': [0.495, -0.495, 0.495, -0.495],
        'wheel_mount_angle_reordered': [0.0, 0.0, 3.14152, 3.14152],
        "steering_joint_names": ["front_left_steering", "front_right_steering", "rear_left_steering",
                                 "rear_right_steering"],
        "traction_joint_names": ["front_left_traction", "front_right_traction", "rear_left_traction",
                                 "rear_right_traction"]
    }

    supervisor_params = {
        'max_velocity_safety_disabled': 0.25,
        'max_velocity_warning_breached': 0.30,
        'max_acceleration': teleop_params['max_acceleration_linear'],
        'max_velocity': teleop_params['max_velocity_linear'],
        'wheelbase': kinematic_params['wheelbase'],
        'max_acceleration_angular': teleop_params['max_acceleration_angular'],
        'battery_status_stale_threshold': 0.5,
        'battery_status_check_period': 0.25,
        'batteries.ids': vanguard_battery_driver_params['batteries.ids']
    }

    sick_eth_comm_params = {'port': 9100}

    bag_params = {
        'bag_prefix': 'romb',
        'split_duration': 300,
        'bag_dir': str(Path.home() / '.romb' / 'bags'),
        'use_compression': False
    }

    vanguard_driver_delay = 2.0
    vehicle_supervisor_delay = 0.0
    mobotic_driver_delay = 0.0

    enable_all_drives_topic = 'vehicle_supervisor/enable_drives'

    socket_can_bridge_node = GroupAction(
        actions=[
            PushRosNamespace(robot_name),

            IncludeLaunchDescription(
                AnyLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('ros2_socketcan'),
                        'launch',
                        'socket_can_bridge.launch.xml'
                    ])
                ]),
                # Pass the arguments to the included launch file
                launch_arguments=socket_can_params.items()
            ),
        ]
    )

    # VANGUARD DRIVER Node
    vanguard_battery_driver_node = Node(
        package='vanguard_battery_driver',
        executable='vanguard_driver',
        name='vanguard_battery_driver',
        output='screen',
        namespace=robot_name,
        respawn=True,
        parameters=[vanguard_battery_driver_params]
    )

    # MOBOTIC DRIVER Node
    mobotic_driver_node = Node(
        package='mobotic_driver',
        executable='mobotic_driver',
        name='mobotic_driver',
        output='screen',
        namespace=robot_name,
        respawn=True,
        parameters=[mobotic_driver_params]
    )

    # JOYSTICK Node
    joystick_node = Node(
        package='joy',
        executable='joy_node',
        name='joystick',
        namespace=robot_name,
        parameters=[
            joy_params
        ]
    )

    # TELEOP Node
    teleop_node = Node(
        package='four_wheel_teleop',
        executable='teleop_node',
        name='teleop',
        namespace=robot_name,
        remappings=[
            ('enable_all_drives', enable_all_drives_topic)
        ],
        parameters=[teleop_params]
    )

    # INVERSE KINEMATICS Node
    inverse_kinematics_node = Node(
        package='kinematics_conversions',
        executable='inverse_kinematics_node',
        name='kinematics',
        namespace=robot_name,
        parameters=[kinematic_params],
        remappings=[
            ('cmd_vel', 'cmd_vel_supervised')
        ]
    )

    vehicle_supervisor = Node(
        package='terraspiro_vehicle_supervisor',
        executable='terraspiro_vehicle_supervisor',
        name='vehicle_supervisor',
        output='screen',
        namespace=robot_name,
        emulate_tty=True,
        parameters=[supervisor_params]
    )

    teltonika_gps_node = Node(
        package='teltonika_gps_server',
        executable='teltonika_gps_server',
        name='teltonika_gps_server',
        output='screen',
        namespace=robot_name,
        emulate_tty=True,  # Recommended for seeing print statements in the terminal
        parameters=[
            # Set "tcp" or "udp" here
            {'protocol': 'tcp'},
            # Set the port
            {'port': 10001},
            {'gps_frame_id': 'gps'},
            {'gps_fix_pub_rate': 0.25}

        ]
    )

    # Action to create the directory
    create_dir_cmd = ['mkdir', '-p', bag_params['bag_dir']]
    create_dir_action = ExecuteProcess(
        cmd=create_dir_cmd,
        output='screen'
    )

    # Action to record the bag with compression
    record_bag_cmd = [
        'ros2', 'bag', 'record',
        '-d', str(bag_params['split_duration'])
    ]

    if bag_params['use_compression']:
        record_bag_cmd.extend(['--compression-mode', 'file',
                               '--compression-format', 'zstd', ])

    record_bag_cmd.append('-a')

    record_bag_action = ExecuteProcess(
        cmd=record_bag_cmd,
        output='screen',
        cwd=bag_params['bag_dir']
    )

    record_bag_event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=create_dir_action,
            on_exit=[record_bag_action]
        )
    )

    sick_eth_comm = Node(
        package='sick_flexisoft_eth_comm',
        executable='sick_flexisoft_eth_comm_node',
        name='sick_flexisoft_eth_comm',
        output='screen',
        namespace=robot_name,
        parameters=[sick_eth_comm_params]
    )

    kinematics_converter = Node(
        package='kinematics_conversions',
        executable='joint_state_to_twist',
        name='js_to_twist_converter',
        output='screen',
        parameters=[kinematic_params],
        namespace=robot_name,
    )

    vanguard_driver_handler = TimerAction(
        period=vanguard_driver_delay,
        actions=[vanguard_battery_driver_node]
    )

    vehicle_supervisor_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=vanguard_battery_driver_node,
            on_start=[
                TimerAction(
                    period=vehicle_supervisor_delay,
                    actions=[vehicle_supervisor],
                )
            ]
        )
    )

    mobotic_driver_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=vehicle_supervisor,
            on_start=[
                TimerAction(
                    period=mobotic_driver_delay,
                    actions=[mobotic_driver_node]
                )
            ]
        )
    )

    virtual_drives_nodes = []
    for (can_node_id, name) in zip(mobotic_driver_params['drives.can_node_ids'], mobotic_driver_params['drives.names']):
        virtual_drives_nodes.append(
            Node(
                package='mobotic_driver',
                executable='virtual_mobotic_drive',
                name=f'virtual_mobotic_drive_{name}',
                output='screen',
                namespace=robot_name,
                condition=IfCondition(EqualsSubstitution(stack_type, 'virtual')),
                parameters=[{
                    'can_node_id': can_node_id
                }]
            )
        )

    virtual_battery_nodes = []
    for (id, status_pgn, source_address) in zip(vanguard_battery_driver_params['batteries.ids'],
                                                vanguard_battery_driver_params['batteries.status_pgns'],
                                                vanguard_battery_driver_params['batteries.source_addresses']):
        virtual_battery_nodes.append(Node(
            package='vanguard_battery_driver',
            executable='virtual_vanguard_battery',
            name=f'virtual_vanguard_battery_{id}',
            output='screen',
            namespace=robot_name,
            condition=IfCondition(EqualsSubstitution(stack_type, 'virtual')),
            parameters=[{
                'battery_id': id,
                'source_address': source_address,
                'status_pgn': status_pgn,
                'controller_address': vanguard_battery_driver_params['source_address']
            }]
        ))

    follow_path_params_file = os.path.join(
        get_package_share_directory('follow_path'),
        'config',
        'follow_path_params.yaml'
    )

    # 2. Define the node
    follow_path_node = Node(
        package='follow_path',
        executable='follow_path',
        name='follow_path',
        output='screen',
        parameters=[follow_path_params_file]
    )

    to_launch = [

        # Launch arguments
        robot_name_arg,
        stack_type_arg,
        can_interface_arg,

        # Nodes
        socket_can_bridge_node,
        joystick_node,
        teleop_node,
        inverse_kinematics_node,
        kinematics_converter,
        sick_eth_comm,
        teltonika_gps_node,
        create_dir_action,
        follow_path_node,

        # The event handlers that manage the startup sequence
        record_bag_event_handler,
        vanguard_driver_handler,
        mobotic_driver_handler,
        vehicle_supervisor_handler
    ]

    # Virtual nodes (launched only if stack == 'virtual')
    to_launch.extend(virtual_drives_nodes)
    to_launch.extend(virtual_battery_nodes)

    #
    # == RETURN THE FINAL LAUNCH DESCRIPTION ==
    #
    return LaunchDescription(to_launch)
