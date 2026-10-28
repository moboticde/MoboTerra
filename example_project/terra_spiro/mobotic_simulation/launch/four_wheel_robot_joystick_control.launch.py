# Copyright 2024 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch import LaunchContext
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    robot_name = LaunchConfiguration('robot_name')

    declare_robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='terraspiro',
        description='Name of the robot.'
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('mobotic_simulation'),
                 'urdf', 'four_wheel.urdf.xacro']
            ),
            ' ',
            f'robot_name:=terraspiro', ' ', f'use_sim_time:=true'
        ]
    )
    robot_description = {'robot_description': robot_description_content}
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('mobotic_simulation'),
            'config',
            'four-wheels-controllers.yaml',
        ]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=f'terraspiro',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'terraspiro', '-allow_renaming', 'true'],
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='terraspiro',
        output='screen',
        parameters=[robot_description]
    )

    spawn_controllers = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        namespace='terraspiro',
        arguments=['-c', f'/terraspiro/controller_manager', '-p', PathJoinSubstitution([FindPackageShare('mobotic_simulation'),
                                               'config', 'four-wheels-controllers.yaml']),
                   'joint_state_broadcaster', 'front_left_traction_controller', 'front_left_steering_controller',
                   'front_right_traction_controller', 'front_right_steering_controller',
                   'rear_left_traction_controller', 'rear_left_steering_controller',
                   'rear_right_traction_controller', 'rear_right_steering_controller'
                   ]
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Include the kinematics launch file (XML source)
    kinematics_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('four_wheel_kinematics'),
                                  'launch', 'kinematics.launch']),
        ),
        launch_arguments={
            'robot_params_path': PathJoinSubstitution([FindPackageShare('mobotic_simulation'),
                                                 'config', 'robot.yaml'])
        }.items()
    )

    # Include the teleop launch file (XML source)
    teleop_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('four_wheel_teleop'), 'launch', 'joystick_teleop.launch.py'])
        ),
        launch_arguments={
            'teleop_params_path': PathJoinSubstitution([FindPackageShare('mobotic_simulation'),
                                                   'config', 'teleop_f710.yaml'])
        }.items()
    )

    return LaunchDescription([
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [' -r -v 1 empty.sdf'])]),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[spawn_controllers],
            )
        ),
        bridge,
        node_robot_state_publisher,
        gz_spawn_entity,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
        Node(
            package='mobotic_simulation',
            executable='joint_controller.py',
            name='joint_controller',
            output='screen',
            namespace='terraspiro'
        ),
        kinematics_launch,
        teleop_launch,
    ])