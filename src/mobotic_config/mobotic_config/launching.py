"""Shared launch adapters; configuration resolves before any node is started."""
import os
import warnings

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from .configuration import Configuration, ConfigurationError, default_platform_path, validate_platform
from .description import render_description


def value(context, key):
    return LaunchConfiguration(key).perform(context)


def common_arguments():
    return [DeclareLaunchArgument('platform_config', default_value=default_platform_path()),
            DeclareLaunchArgument('robot_name', default_value=os.environ.get('ROBOT_NAME', 'auto'))]


def load_from_context(context, overlay_arguments):
    cfg = Configuration(value(context, 'platform_config'), {
        component: value(context, argument) for component, argument in overlay_arguments.items()})
    return cfg


def namespace_action(context, cfg):
    selected = value(context, 'robot_name')
    return SetLaunchConfiguration('robot_name', cfg.platform['robot_name'] if selected == 'auto' else selected)


def boolean_override(context, argument, default):
    selected = value(context, argument)
    if selected == 'auto':
        return default
    if selected not in ('true', 'false'):
        raise ConfigurationError(f'{argument}: expected auto, true or false')
    return selected == 'true'


def deployment_actions(context, cfg):
    """Resolve transport overrides centrally without configuring the OS."""
    p = cfg.platform
    fields = {'can_interface': (p['can']['drives'], 'interface'),
              'battery_can_interface': (p['can']['battery'], 'interface'),
              'scanner_host_ip': (p['scanners'], 'host_ip')}
    for scanner in p['scanners']['units']:
        fields['front_scanner_ip' if scanner['name'] == 'front_left' else 'rear_scanner_ip'] = (scanner, 'sensor_ip')
    actions = [namespace_action(context, cfg)]
    for argument, (record, field) in fields.items():
        selected = value(context, argument)
        if selected != 'auto':
            record[field] = selected
        actions.append(SetLaunchConfiguration(argument, record[field]))
    validate_platform(p)
    actions.append(SetLaunchConfiguration('publish_odom_tf', str(boolean_override(
        context, 'publish_odom_tf', cfg.parameters('odometry')['publish_tf'])).lower()))
    return actions


_NODES = {
    'driver': ('mobotic_driver', 'mobotic_driver', 'mobotic_driver'),
    'kinematics': ('mobotic_kinematics', 'mobotic_kinematics', 'mobotic_kinematics'),
    'supervisor': ('mobotic_supervisor', 'mobotic_supervisor', 'mobotic_supervisor'),
    'odometry': ('mobotic_odometry', 'mobotic_odometry', 'mobotic_odometry'),
    'manual': ('mobotic_manual_control', 'manual_control', 'manual_control'),
    'battery': ('mobotic_vanguard_battery', 'vanguard_battery_monitor', 'vanguard_battery_monitor'),
}


def _standalone_setup(context, component):
    cfg = load_from_context(context, {component: 'parameters_file'})
    actions = [namespace_action(context, cfg)]
    namespace = LaunchConfiguration('robot_name')
    if component == 'safety':
        return actions + [Node(package='mobotic_safety', executable=executable, name=executable,
                               namespace=namespace, parameters=[cfg.parameters('safety')],
                               output='screen', respawn=True, respawn_delay=1.0)
                          for executable in ('flexisoft_tcp_bridge', 'safety_monitor')]
    parameters = cfg.parameters(component)
    if component == 'driver':
        legacy = {}
        for key in parameters:
            selected = value(context, key)
            if selected != 'auto':
                try:
                    legacy[key] = selected if key == 'can_node_id' else yaml.safe_load(selected)
                except yaml.YAMLError as error:
                    raise ConfigurationError(f'Invalid legacy argument {key}: {error}') from error
        cfg.apply_overrides(component, legacy, legacy=True)
        cfg._validate_policy()
        parameters = cfg.parameters(component)
    if component == 'manual':
        actions.append(Node(package='joy', executable='game_controller_node', name='joystick', namespace=namespace,
                            parameters=[cfg.platform['usb_joystick']], output='screen'))
    if component == 'odometry':
        parameters['publish_tf'] = boolean_override(context, 'publish_tf', parameters['publish_tf'])
    if component == 'battery':
        selected = value(context, 'can_interface')
        interface = cfg.platform['can']['battery']['interface'] if selected == 'auto' else selected
        cfg.platform['can']['battery']['interface'] = interface
        validate_platform(cfg.platform)
        actions.append(GroupAction(condition=IfCondition(LaunchConfiguration('start_socketcan')), actions=[
            PushRosNamespace(namespace), PushRosNamespace('battery_can'),
            IncludeLaunchDescription(AnyLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('ros2_socketcan'), 'launch', 'socket_can_receiver.launch.py'])),
                launch_arguments={'interface': interface, 'from_can_bus_topic': 'can_rx',
                                  'enable_can_fd': 'false', 'filters': '0:0', 'interval_sec': '0.01',
                                  'use_bus_time': 'true'}.items())]))
    package, executable, name = _NODES[component]
    remappings = [('can_rx', 'battery_can/can_rx')] if component == 'battery' else [('/tf', 'tf')] if component == 'odometry' else []
    actions.append(Node(package=package, executable=executable, name=name, namespace=namespace,
                        parameters=[parameters], output='screen', remappings=remappings,
                        respawn=component in ('driver', 'supervisor', 'battery', 'kinematics')))
    return actions


def standalone_launch(component):
    arguments = common_arguments() + [DeclareLaunchArgument('parameters_file', default_value='',
                                                           description='Optional node-policy override YAML; shared hardware comes from platform_config')]
    if component == 'driver':
        # Retain the old launch arguments as adapters, not a second set of defaults.
        for key in Configuration().parameters('driver'):
            arguments.append(DeclareLaunchArgument(key, default_value='auto',
                                                   description='Deprecated override; prefer platform_config / parameters_file'))
    if component == 'odometry':
        arguments.append(DeclareLaunchArgument('publish_tf', default_value='auto', choices=['auto', 'true', 'false']))
    if component == 'battery':
        arguments.extend([DeclareLaunchArgument('can_interface', default_value='auto'),
                          DeclareLaunchArgument('start_socketcan', default_value='true', choices=['true', 'false'])])
    return LaunchDescription(arguments + [OpaqueFunction(function=_standalone_setup, args=[component])])


def _virtual_setup(context):
    cfg = Configuration(value(context, 'platform_config'))
    module = value(context, 'module_name')
    role = value(context, 'drive_type')
    record = next((m for m in cfg.platform['modules'] if m['name'] == module), None)
    if record is None:
        raise ConfigurationError(f'Unknown virtual module {module}')
    joint = record[role]['joint']
    parameters = next(params for name, params in cfg.virtual_drives() if name == joint)
    for key in ('can_node_id', 'encoder_resolution', 'gear_ratio'):
        selected = value(context, key)
        if selected != 'auto':
            warnings.warn(f'{key} is deprecated; set the virtual drive in platform_config', UserWarning)
            expected = parameters[key]
            candidate = selected if key == 'can_node_id' else float(selected)
            if key == 'can_node_id':
                if int(candidate, 0) != int(expected, 0):
                    raise ConfigurationError(f'{key}: conflicts with selected platform module')
            elif candidate != expected:
                raise ConfigurationError(f'{key}: conflicts with selected platform module')
    return [namespace_action(context, cfg), Node(package='mobotic_driver', executable='virtual_mobotic_drive',
            name='virtual_mobotic_drive', namespace=LaunchConfiguration('robot_name'), parameters=[parameters],
            remappings=[('can_rx', 'can_tx'), ('can_tx', 'can_rx')], output='screen')]


def virtual_launch():
    arguments = common_arguments() + [DeclareLaunchArgument('module_name', default_value='front_left'),
        DeclareLaunchArgument('drive_type', default_value='steering', choices=['steering', 'traction'])]
    for key in ('can_node_id', 'encoder_resolution', 'gear_ratio'):
        arguments.append(DeclareLaunchArgument(key, default_value='auto'))
    return LaunchDescription(arguments + [OpaqueFunction(function=_virtual_setup)])


def _description_setup(context):
    cfg = Configuration(value(context, 'platform_config'))
    return [namespace_action(context, cfg), Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', namespace=LaunchConfiguration('robot_name'),
        parameters=[{'robot_description': ParameterValue(render_description(cfg), value_type=str)}],
        output='screen', remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')])]


def description_launch():
    return LaunchDescription(common_arguments() + [OpaqueFunction(function=_description_setup)])
