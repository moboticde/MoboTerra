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
        'sender_timeout_sec': '0.05'
    }

    ec_traction_velocity_limit = 185429  # EC traction velocity limit in controller increments per second.
    mobotic_driver_params = {
        'can_node_id': '0x0A',
        'feedback_request_period': 0.02,
        'can_heartbeat_period': 2.0,
        'watchdog_timeout': 0.1,
        'traction_max_profile_velocity': 185429,  # CANopen 0x607F, 2.0 m/s.
        'drives.names': [
            'front_steering', 'front_traction', 'rear_steering' , 'rear_traction',
         ],
        'drives.can_node_ids': [
            '0x03', '0x04', '0x07', '0x08'
         ],
        'drives.modes': [
            'position', 'velocity', 'position', 'velocity'
        ],
        'drives.resolutions': [
            4096, 4096, 4096, 4096
        ],
        'drives.min_steering_velocities': [
            -5000, -1700, -5000, -1700
        ],
        'drives.max_steering_velocities': [
            5000, 1700, 5000, 1700
        ],
        'drives.min_traction_velocities': [
            -5000, -ec_traction_velocity_limit, -5000, -ec_traction_velocity_limit
        ],
        'drives.max_traction_velocities': [
            5000, ec_traction_velocity_limit, 5000, ec_traction_velocity_limit
        ],
        'drives.gear_ratios': [
            121.0, 16.0, 121.0, 16.0
        ],
        'drives.min_currents': [
            -4000, -4000, -4000, -4000
        ],
        'drives.max_currents': [
            4000, 4000, 4000, 4000
        ]
    }

    joy_params = {
        'joy_autorepeat_rate': 0.0,
        'joy_default_trig_val': True
    }

    teleop_params = {
        'max_velocity_linear': 0.5,
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
        'scale_linear': 0.5,
        'scale_angular': 0.4,
        'commands_timeout_sec': 0.25,
        'commands_publish_sec': 0.02
    }

    kinematic_params = {
        'wheelbase': 1.02,
        'track': 0.88,
        'wheel_radius': 0.1125,
        'max_velocity_steering': 4.32,
        'max_velocity_traction': 11.12,
        'max_current_traction': 4.0,
        'wheel_mount_angles': [3.22152, 0.08],
        'wheels_x': [0.51, -0.51],
        'wheels_y': [-0.44, 0.44],
        'wheel_mount_angle_reordered': [3.22152, 0.08],
        "steering_joint_names": ["front_steering", "rear_steering"],
        "traction_joint_names": ["front_traction", "rear_traction"]
    }

    bag_params = {
        'bag_prefix': 'romb',
        'split_duration': 300,
        'bag_dir': str(Path.home() / '.romb' / 'bags'),
        'use_compression': False
    }


    follow_path_params = {
        # Controller type: "MPC", "PurePursuit"
        'traction_delay': 0.18,#0.2,#075,
        'steering_delay': 0.2,#0.2
        'creep_distance': 0.2, #0.2
        'creep_velocity': 0.05 ,#12
        'max_linear_velocity': 2.0,
        'max_acceleration': 1.0,#1.0
        'max_deceleration': 1.0, #0.6 #0.75, #0.8
        'max_jerk': 0.6,  #0.6    #1.2
        'safety_deceleration': 1.0,

       # Length of path to precompute before vehicle starts moving
        'early_start_distance': 5.0,

        'max_steering_velocity': 0.8,
        'max_steering_rate': 0.8,

        'map_frame_id': 'world',
        'robot_frame_id': 'base_link',

        'max_steering_angle': 10000000.0,
        'wheel_names': ['front', 'rear'],
        'wheel_x': [0.51, -0.51],      
        'wheel_y': [-0.44, 0.44],
        'wheel_mount_angles': [0.0, 0.0],

        'prediction_horizon': 50,
        'q_y': 50.0,#35.0,
        'q_theta': 6.0,#6.0, 
        'r_delta_min_value': 18.0,#6.0,#3.0,
        'r_delta_min_speed': 0.45,#0.3,
        'r_delta_max_value': 30.0,#15.0,#18.0,
        'r_delta_max_speed': 0.85,#1.0
    }

    planner_parameters = {
        'roadmap_file': os.path.expanduser('~/.romb/curve.roadmap.xml'),
        'distance_to_vertex_threshold': 0.05,
        'distance_to_path_threshold': 0.2,
        'radians_threshold': 0.349,
        'reverse_speed_limit': 2.0,
    }

    enable_all_drives_topic = 'enable_all_drives'

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
        parameters=[kinematic_params]
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

    kinematics_converter = Node(
        package='kinematics_conversions',
        executable='joint_state_to_twist',
        name='js_to_twist_converter',
        output='screen',
        parameters=[kinematic_params],
        namespace=robot_name,
    )

    joint_state_converter_node = Node(
        package='mobotic_launch',
        executable='js_converter.py',
        name='joint_state_converter',
        output='screen',
        parameters=[{
            'wheel_radius': kinematic_params['wheel_radius'],
            'max_steering_velocity': 2.16,
        }],
        namespace=robot_name,
    )

    follow_path_node = Node(
        package='follow_path',
        executable='follow_path',
        name='follow_path',
        output='screen',
        parameters=[follow_path_params],
        namespace=robot_name,
        remappings=[
            ('agv_js', 'joint_states_pf'),
            ('cmd_js', 'joint_setpoints_pf')
        ]
    )

    # 3. Define the Node
    path_planner_node = Node(
        package='roadmap_search',
        executable='path_planning',
        name='path_planner',
        parameters=[planner_parameters],
        namespace=robot_name
    )

    js_to_twist_node = Node(
        package='kinematics_conversions',
        executable='joint_state_to_twist',
        name='js_to_twist_converter',
        output='screen',
        parameters=[kinematic_params],
        namespace=robot_name
    )

    odometry_node = Node(
        package='agv_odometry',
        executable='agv_odometry',
        name='agv_odometry_node',
        output='screen',
        namespace=robot_name
    )


    sick_parameters = {
        "frame_id": "front_scan",
        "sensor_ip": "10.60.20.52",
        "host_ip": "10.60.20.186",
        "interface_ip": "0.0.0.0",
        "host_udp_port": 0,
        "channel": 0,
        "channel_enabled": True,
        "skip": 0,
        "angle_start": 0.0,
        "angle_end": 0.0,
        "time_offset": 0.0,
        "general_system_state": True,
        "derived_settings": True,
        "measurement_data": True,
        "intrusion_data": True,
        "application_io_data": True,
        "use_persistent_config": False,
        "min_intensities": 0.0}

    sick_scanner_node = Node(
        package="sick_safetyscanners2",
        executable="sick_safetyscanners2_node",
        name="sick_safetyscanners2_node",
        output="screen",
        emulate_tty=True,
        parameters=[sick_parameters],
        namespace = [robot_name, '/'  ,'front_scanner']
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
        # joint_state_converter_node,

        # sick_eth_comm,
        # teltonika_gps_node,
        create_dir_action,
        # follow_path_node,
        mobotic_driver_node,
        # path_planner_node,
        # js_to_twist_node,
        odometry_node,
        sick_scanner_node,

        # record_bag_event_handler,

    ]

    # Virtual nodes (launched only if stack == 'virtual')
    # to_launch.extend(virtual_drives_nodes)
    # to_launch.extend(virtual_battery_nodes)

    #
    # == RETURN THE FINAL LAUNCH DESCRIPTION ==
    #
    return LaunchDescription(to_launch)
