import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    EnvironmentVariable,
    PathJoinSubstitution,
    FindExecutable
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

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
        'wheelbase': 1.02,
        'track': 0.88,
        'wheel_radius': 0.1125,
        'max_velocity_steering': 4.32,
        'max_velocity_traction': 11.12,
        'max_current_traction': 4.0,
        'wheel_mount_angles': [3.14159, 0.0],
        'wheels_x': [0.51, -0.55],
        'wheels_y': [-0.44, 0.44],
        'wheel_mount_angle_reordered': [3.14159, 0.0],
        "steering_joint_names": ["front_steering", "rear_steering"],
        "traction_joint_names": ["front_traction", "rear_traction"]
    }

    # follow_path_params = {
    #     # Controller type: "MPC", "PurePursuit"
    #     'traction_delay': 0.0,
    #     'steering_delay': 0.0,
    #     'creep_distance': 0.10,
    #     'creep_velocity': 0.05,
    #     'max_linear_velocity': 1.0,
    #     'max_acceleration': 0.5,
    #     'max_deceleration': 0.5,
    #     'max_jerk': 0.5,
    #     'safety_deceleration': 0.7,
    #
    #     # Length of path to precompute before vehicle starts moving
    #     'early_start_distance': 5.0,
    #
    #     'max_steering_velocity': 0.8,
    #     'max_steering_rate': 0.8,
    #
    #     'map_frame_id': 'maps',
    #     'robot_frame_id': 'mobotic/base_link',
    #
    #     'max_steering_angle': 3.14,
    #     'wheel_names': ['front', 'rear'],
    #     'wheel_x': [0.51, -0.51],
    #     'wheel_y': [0.44, -0.44],
    #     'wheel_mount_angles': [1.5708, 1.5708],
    #
    #     'prediction_horizon': 50,
    #     'q_y': 250,
    #     'q_theta': 20,
    #     'r_delta_min_value': 3.0,
    #     'r_delta_min_speed': 0.15,
    #     'r_delta_max_value': 15.0,
    #     'r_delta_max_speed': 0.5
    # }


    # --- 1. Launch Configurations & Arguments ---
    robot_name = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value=EnvironmentVariable('ROBOT_NAME', default_value='my_robot'),
        description='Name of the robot'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_x_cmd = DeclareLaunchArgument('x', default_value='0.0')
    declare_y_cmd = DeclareLaunchArgument('y', default_value='0.0')
    declare_yaw_cmd = DeclareLaunchArgument('yaw', default_value='0.0')

    # --- 2. Xacro Processing ---
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
            FindPackageShare('mobotic_simulation'),
            'urdf',
            'two-diagonal-wheels.urdf.xacro'
        ]),
        ' robot_name:=', robot_name
    ])

    # --- 3. Gazebo Sim Launch ---
    # In Jazzy, we use ros_gz_sim and pass an SDF world file (empty.sdf is default)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items() # -r starts it unpaused
    )

    # --- 4. ROS <-> Gazebo Clock Bridge ---
    # Essential for use_sim_time=true. It bridges the Gazebo clock to the ROS /clock topic
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock', ['/model/', robot_name, '/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V']],
        output='screen'
    )

    # --- 5. Nodes ---
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
            'frame_prefix': [robot_name, '/']
        }]
    )

    # Spawn entity uses 'create' from ros_gz_sim instead of spawn_entity.py
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', robot_name,
            '-topic', ['/', robot_name,  '/robot_description'],
            '-x', x,
            '-y', y,
            '-z', '0.1', # Adding a slight Z offset is good practice so wheels don't clip the floor
            '-Y', yaw
        ]
    )

    custom_joint_controller_node = Node(
        package='mobotic_simulation',
        executable='joint_controller.py',
        output='screen'
    )

    tf_broadcaster_node = Node(
        package='mobotic_simulation',
        executable='tf_broadcaster.py',
        output='screen',
        remappings=[
            ('pose', ['/model/', robot_name, '/pose'])
        ],
    )

    # --- 6. Controller Spawners ---
    controllers = [
        "front_traction_controller",
        "front_steering_controller",
        "rear_traction_controller",
        "rear_steering_controller",
        "joint_state_broadcaster"
    ]

    controller_spawner_nodes = []
    for controller in controllers:
        controller_spawner_nodes.append(
            Node(
                package='controller_manager',
                executable='spawner',
                output='screen',
                arguments=['-c', ['/', robot_name,  '/controller_manager'], '-p', PathJoinSubstitution([FindPackageShare('mobotic_simulation'),
                                                                                                'config', 'two-wheels-controllers.yaml']), controller]
            )
        )

    # JOYSTICK Node
    joystick_node = Node(
        package='joy',
        executable='joy_node',
        name='joystick',
        parameters=[
            joy_params
        ]
    )

    # TELEOP Node
    teleop_node = Node(
        package='four_wheel_teleop',
        executable='teleop_node',
        name='teleop',
        parameters=[teleop_params]
    )

    # INVERSE KINEMATICS Node
    inverse_kinematics_node = Node(
        package='kinematics_conversions',
        executable='inverse_kinematics_node',
        name='kinematics',
        parameters=[kinematic_params]
    )

    js_to_acker = Node(
        package='kinematics_conversions',
        executable='joint_states_to_acker.py',
        name='js_to_acker',
        parameters=[kinematic_params]
    )


    # follow_path_node = Node(
    #     package='follow_path',
    #     executable='follow_path',
    #     name='follow_path',
    #     output='screen',
    #     parameters=[follow_path_params],
    #     remappings=[
    #         ('agv_js', 'joint_states'),
    #         ('cmd_js', 'joint_setpoints')
    #     ]
    # )

    # Inside your generate_launch_description():
    map_to_odom_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='fake_localization',
        arguments=['0', '0', '0', '0', '0', '0', 'maps', 'odom']
    )



    # --- 7. Group Action (Namespace) ---
    namespaced_group = GroupAction([
        PushRosNamespace(namespace=robot_name),
        robot_state_publisher_node,
        spawn_entity_node,
        *controller_spawner_nodes,
        joystick_node,
        # teleop_node,
        # inverse_kinematics_node,
        # follow_path_node,
        # path_planner_node,
        map_to_odom_pub,
        tf_broadcaster_node,
        # js_to_acker,
        custom_joint_controller_node,
    ])

    # --- 8. Create and Return Launch Description ---
    ld = LaunchDescription()
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_x_cmd)
    ld.add_action(declare_y_cmd)
    ld.add_action(declare_yaw_cmd)

    ld.add_action(gazebo_launch)
    ld.add_action(clock_bridge)
    ld.add_action(namespaced_group)

    return ld