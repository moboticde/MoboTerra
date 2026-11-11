"""Launch the unchanged romb model alongside its MoboTerra interface adapter."""
from .romb.runtime import runtime_command


def romb_kinematics_nodes(node_factory, namespace, config):
    runtime_command([])  # Verify the local executable/runtime before starting any nodes.
    modules = config['module_names']
    if (config['steering_joint_names'] != [name + '_steering' for name in modules]
            or config['traction_joint_names'] != [name + '_traction' for name in modules]):
        raise ValueError('Romb kinematics requires <module>_steering/_traction names')
    model_parameters = {
        'wheel_names': modules,
        'wheel_x': config['wheels_x'], 'wheel_y': config['wheels_y'],
        'is_omnidirectional': True,
        'traction_suffix': '_traction', 'steering_suffix': '_steering',
        'speed_epsilon': config['minimum_linear_speed'],
        'enable_direct': True, 'enable_inverse': True,
    }
    adapter_parameters = {key: config[key] for key in (
        'module_names', 'steering_joint_names', 'traction_joint_names',
        'wheel_mount_angles', 'max_steering_velocity', 'max_traction_velocity',
        'minimum_linear_speed', 'cmd_vel_timeout', 'joint_feedback_timeout', 'output_frame_id')}
    return [
        node_factory(package='mobotic_kinematics', executable='romb_kinematics',
            name='romb_kinematics_original', namespace=namespace,
            parameters=[model_parameters], output='screen', respawn=True, respawn_delay=1.0,
            remappings=[('~/inverse_in', 'romb/kinematics/cmd_vel'),
                        ('~/inverse_out', 'romb/kinematics/raw_setpoints'),
                        ('~/direct_in', 'romb/kinematics/joint_states'),
                        ('~/direct_out', 'romb/kinematics/raw_velocity')]),
        node_factory(package='mobotic_kinematics', executable='romb_kinematics_adapter',
            name='mobotic_kinematics', namespace=namespace,
            parameters=[adapter_parameters], output='screen', respawn=True, respawn_delay=1.0),
    ]


def kinematics_nodes(node_factory, namespace, config, backend='romb'):
    """Use the original ROMB solver for every supported launch path."""
    if backend != 'romb':
        raise ValueError('Only the ROMB kinematics backend is supported')
    return romb_kinematics_nodes(node_factory, namespace, config)


def _standalone_setup(context):
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node
    from mobotic_config.launching import load_from_context, namespace_action
    cfg = load_from_context(context, {'kinematics': 'parameters_file'})
    return [namespace_action(context, cfg), *kinematics_nodes(Node,
        LaunchConfiguration('robot_name'), cfg.parameters('kinematics'),
        LaunchConfiguration('kinematics_backend').perform(context))]


def generate_launch_description():
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, OpaqueFunction
    from mobotic_config.launching import common_arguments
    return LaunchDescription(common_arguments() + [
        DeclareLaunchArgument('parameters_file', default_value=''),
        DeclareLaunchArgument('kinematics_backend', default_value='romb', choices=['romb']),
        OpaqueFunction(function=_standalone_setup)])
