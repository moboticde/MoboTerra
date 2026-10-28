import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    # --- 1. Arguments ---
    # This allows you to pass 'robot_name' from the parent launch file
    robot_name = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='mobotic',
        description='Namespace for the path following nodes'
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false'
    )

    follow_path_params = {
        'traction_delay': 0.08,#0.18
        'steering_delay': 0.05,#0.05

        'creep_distance': 0.1,
        'creep_velocity': 0.05,

        # Unchanged performance requirements
        'max_linear_velocity': 1.7,
        'max_acceleration': 1.0,
        'max_deceleration': 1.0,
        'safety_deceleration': 1.0,

        # Smoother acceleration transitions
        'max_jerk': 0.7,  # previous: 0.6

        'early_start_distance': 5.0,

        'max_steering_velocity': 0.8,
        'max_steering_rate': 0.4,

        'map_frame_id': 'world',
        'robot_frame_id': 'base_link',

        'max_steering_angle': 10000000.0,
        'wheel_names': ['front', 'rear'],
        'wheel_x': [0.51, -0.51],
        'wheel_y': [-0.44, 0.44],
        'wheel_mount_angles': [0.0, 0.0],

        'prediction_horizon': 60,
        'q_y': 35.0,
        'q_theta': 6.0,

        # Stronger penalty on sudden MPC wheel corrections
        'r_delta_min_value': 55.0,  # previous: 28.0
        'r_delta_min_speed': 0.3,   # previous: 0.45
        'r_delta_max_value': 85.0,  # previous: 46.0
        'r_delta_max_speed': 0.7,   # previous: 1.0
    }

    planner_parameters = {
        'roadmap_file': os.path.expanduser('~/.romb/fixed-small-test-circuit.roadmap.xml'),
        'distance_to_vertex_threshold': 0.05,
        'distance_to_path_threshold': 0.2,
        'radians_threshold': 0.349,
        'reverse_speed_limit': 1.7,
    }

    # 3. Define the Node
    path_planner_node = Node(
        package='roadmap_search',
        executable='path_planning',
        name='path_planner',
        parameters=[planner_parameters] # Pass the dictionary directly
    )


    # --- 3. Namespaced Group ---
    # Everything inside this group will be pushed into the /<robot_name> namespace
    namespaced_nodes = GroupAction([
        PushRosNamespace(namespace=robot_name),
        path_planner_node,
        Node(
            package='follow_path',
            executable='follow_path',
            name='follow_path',
            output='screen',
            parameters=[follow_path_params, {'use_sim_time': use_sim_time}],
            #remappings=[
            #    ('agv_js', 'joint_states_pf'),
            #    ('cmd_js', 'joint_setpoints_pf')
            #]
        )
    ])

    return LaunchDescription([
        declare_robot_name_arg,
        declare_use_sim_time_arg,
        namespaced_nodes
    ])
