"""Exercise actual resolved launch adapters without starting ROS/hardware."""
import ast
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'mobotic_config'))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'mobotic_config/test'))
from launch_harness import factory, resolve

SOURCE = Path(__file__).parents[1] / 'launch/moboterra.launch.py'


def resolved_factory(source):
    def make():
        from launch_harness import Entity
        actions, _ = resolve(factory(source)())
        return Entity('LaunchDescription', actions)
    return make

namespace = {'generate_launch_description': resolved_factory(SOURCE)}


class SocketCanLaunchTest(unittest.TestCase):
    def setUp(self):
        description = namespace['generate_launch_description']()
        self.actions = description.args[0]
        self.group = next(
            action for action in description.args[0]
            if action.kind == 'GroupAction'
            and any(child.kind == 'IncludeLaunchDescription' for child in action.kwargs['actions'])
        )
        self.include = next(child for child in self.group.kwargs['actions']
                            if child.kind == 'IncludeLaunchDescription')
        self.arguments = dict(self.include.kwargs['launch_arguments'])

    def battery_receiver(self):
        group = next(action for action in self.actions
                     if action.kind == 'GroupAction'
                     and any(child.kind == 'PushRosNamespace' and child.args == ('battery_can',)
                             for child in action.kwargs['actions']))
        include = next(child for child in group.kwargs['actions']
                       if child.kind == 'IncludeLaunchDescription')
        return group, include, dict(include.kwargs['launch_arguments'])

    def test_default_drive_and_battery_interfaces_are_separate(self):
        arguments = {action.args[0]: action.kwargs for action in self.actions
                     if action.kind == 'DeclareLaunchArgument'}
        self.assertEqual(arguments['can_interface']['default_value'], 'auto')
        self.assertEqual(resolve(factory(SOURCE)())[1]['can_interface'], 'can0')
        self.assertEqual(arguments['battery_can_interface']['default_value'], 'auto')
        self.assertEqual(resolve(factory(SOURCE)())[1]['battery_can_interface'], 'can1')
        self.assertEqual(arguments['start_battery_socketcan']['default_value'], 'true')

    def test_battery_transport_is_receiver_only_with_acquisition_timestamps(self):
        group, include, arguments = self.battery_receiver()
        self.assertEqual(include.args[0].args[0].args[0][-1], 'socket_can_receiver.launch.py')
        self.assertEqual(arguments['interface'].args, ('battery_can_interface',))
        self.assertEqual(arguments['from_can_bus_topic'], 'can_rx')
        self.assertEqual(arguments['enable_can_fd'], 'false')
        self.assertEqual(arguments['use_bus_time'], 'true')
        self.assertEqual(arguments['filters'], '0:0')
        self.assertNotIn('to_can_bus_topic', arguments)
        pushes = [action for action in group.kwargs['actions'] if action.kind == 'PushRosNamespace']
        self.assertEqual(pushes[0].args[0].args, ('robot_name',))
        self.assertEqual(pushes[1].args, ('battery_can',))

    def test_battery_receiver_requires_hardware_and_battery_and_receiver_enabled(self):
        group, _, _ = self.battery_receiver()
        for stack in ('hardware', 'virtual'):
            for battery in ('true', 'false'):
                for receiver in ('true', 'false'):
                    context = {'stack_type': stack, 'start_battery': battery,
                               'start_battery_socketcan': receiver, 'start_socketcan': 'false'}
                    with self.subTest(**context):
                        self.assertEqual(group.kwargs['condition'].perform(context),
                                         stack == 'hardware' and battery == receiver == 'true')

    def test_monitor_uses_battery_topic_even_when_external_receiver_is_selected(self):
        monitor = next(action for action in self.actions if action.kind == 'Node'
                       and action.kwargs.get('executable') == 'vanguard_battery_monitor')
        self.assertEqual(monitor.kwargs['remappings'], [('can_rx', 'battery_can/can_rx')])
        self.assertTrue(monitor.kwargs['condition'].perform({
            'stack_type': 'hardware', 'start_battery': 'true', 'start_battery_socketcan': 'false',
        }))
        self.assertFalse(monitor.kwargs['condition'].perform({
            'stack_type': 'virtual', 'start_battery': 'true',
        }))

    def test_standalone_battery_launch_has_same_receive_only_topic_contract(self):
        source = SOURCE.parents[2] / 'mobotic_vanguard_battery' / 'launch' / 'vanguard_battery.launch.py'
        env = {'generate_launch_description': resolved_factory(source)}
        actions = env['generate_launch_description']().args[0]
        arguments = {action.args[0]: action for action in actions if action.kind == 'DeclareLaunchArgument'}
        self.assertEqual(arguments['can_interface'].kwargs['default_value'], 'auto')
        group = next(action for action in actions if action.kind == 'GroupAction')
        self.assertTrue(group.kwargs['condition'].perform({'start_socketcan': 'true'}))
        self.assertFalse(group.kwargs['condition'].perform({'start_socketcan': 'false'}))
        include = next(action for action in group.kwargs['actions'] if action.kind == 'IncludeLaunchDescription')
        self.assertEqual(include.args[0].args[0].args[0][-1], 'socket_can_receiver.launch.py')
        transport = dict(include.kwargs['launch_arguments'])
        self.assertEqual(transport['interface'], 'can1')
        self.assertEqual(transport['use_bus_time'], 'true')
        self.assertEqual(transport['from_can_bus_topic'], 'can_rx')
        pushes = [action for action in group.kwargs['actions'] if action.kind == 'PushRosNamespace']
        self.assertEqual(pushes[0].args[0].args, ('robot_name',))
        self.assertEqual(pushes[1].args, ('battery_can',))
        monitor = next(action for action in actions if action.kind == 'Node')
        self.assertEqual(monitor.kwargs['remappings'], [('can_rx', 'battery_can/can_rx')])

    def test_error_frame_reception_is_explicitly_enabled(self):
        filters = self.arguments.get('filters', '0:0').split(',')
        masks = [int(item[1:], 16) for item in filters if item.startswith('#')]
        self.assertEqual(masks, [0x1FFFFFFF])  # Linux CAN_ERR_MASK, all error classes.

    def test_normal_standard_and_extended_traffic_is_preserved(self):
        filters = self.arguments.get('filters', '0:0').split(',')
        normal = [tuple(int(part, 16) for part in item.split(':'))
                  for item in filters if ':' in item]
        self.assertEqual(normal, [(0, 0)])
        for identifier in (0x583, 0x584, 0x707, 0x123, 0x80000000 | 0x18F091F3):
            self.assertTrue(any(identifier & mask == expected & mask for expected, mask in normal))

    def test_topics_namespace_and_acquisition_timestamps_are_unchanged(self):
        self.assertEqual(self.arguments['from_can_bus_topic'], 'can_rx')
        self.assertEqual(self.arguments['to_can_bus_topic'], 'can_tx')
        self.assertEqual(self.arguments['use_bus_time'], 'true')
        self.assertEqual(self.arguments['interface'].args, ('can_interface',))
        namespace_action = self.group.kwargs['actions'][0]
        self.assertEqual(namespace_action.kind, 'PushRosNamespace')
        self.assertEqual(namespace_action.args[0].args, ('robot_name',))

    def test_physical_bridge_is_only_started_when_hardware_and_enabled(self):
        condition = self.group.kwargs['condition']
        for stack, enabled, expected in (
            ('hardware', 'true', True), ('hardware', 'false', False),
            ('virtual', 'true', False), ('virtual', 'false', False),
        ):
            with self.subTest(stack=stack, enabled=enabled):
                self.assertEqual(condition.perform({'stack_type': stack, 'start_socketcan': enabled}), expected)

    def test_virtual_profile_has_all_eight_actuators_with_reference_can_bindings(self):
        nodes = [action for action in namespace['generate_launch_description']().args[0]
                 if action.kind == 'Node' and action.kwargs.get('executable') == 'virtual_mobotic_drive']
        bindings = {node.kwargs['name']: node.kwargs['parameters'][0]['can_node_id'] for node in nodes}
        self.assertEqual(len(nodes), 8)
        self.assertEqual(bindings, {
            'virtual_front_left_steering': '0x01', 'virtual_front_left_traction': '0x02',
            'virtual_front_right_steering': '0x03', 'virtual_front_right_traction': '0x04',
            'virtual_rear_left_steering': '0x07', 'virtual_rear_left_traction': '0x08',
            'virtual_rear_right_steering': '0x05', 'virtual_rear_right_traction': '0x06',
        })
        for node in nodes:
            parameters = node.kwargs['parameters'][0]
            traction = node.kwargs['name'].endswith('_traction')
            self.assertEqual(parameters['drive_type'], 'traction' if traction else 'steering')
            self.assertEqual(parameters['encoder_resolution'], 4096.0)
            self.assertEqual(parameters['gear_ratio'], 16.0 if traction else 121.0)
            self.assertEqual(node.kwargs['remappings'], [('can_rx', 'can_tx'), ('can_tx', 'can_rx')])
            condition = node.kwargs['condition']
            self.assertFalse(condition.perform({'stack_type': 'hardware'}))
            self.assertTrue(condition.perform({'stack_type': 'virtual'}))

    def test_virtual_driver_keeps_the_production_pdo_configuration(self):
        nodes = [action for action in namespace['generate_launch_description']().args[0]
                 if action.kind == 'Node' and action.kwargs.get('executable') == 'mobotic_driver']
        self.assertEqual(len(nodes), 2)
        self.assertEqual(nodes[0].kwargs['parameters'], nodes[1].kwargs['parameters'])

    def test_standalone_virtual_launch_has_explicit_roles_and_reversed_can_topics(self):
        source = SOURCE.parents[2] / 'mobotic_driver' / 'launch' / 'mobotic_drive_virtual.launch.py'
        env = {'generate_launch_description': resolved_factory(source)}
        actions = env['generate_launch_description']().args[0]
        arguments = {action.args[0]: action for action in actions if action.kind == 'DeclareLaunchArgument'}
        self.assertEqual(arguments['can_node_id'].kwargs['default_value'], 'auto')
        self.assertEqual(next(a for a in actions if a.kind == 'Node').kwargs['parameters'][0]['can_node_id'], '0x01')
        self.assertEqual(arguments['drive_type'].kwargs['choices'], ['steering', 'traction'])
        node = next(action for action in actions if action.kind == 'Node')
        self.assertEqual(node.kwargs['remappings'], [('can_rx', 'can_tx'), ('can_tx', 'can_rx')])
        for key in ('encoder_resolution', 'gear_ratio'):
            self.assertIsInstance(node.kwargs['parameters'][0][key], float)

    def test_flexisoft_respawn_is_delayed_in_both_launch_files(self):
        integrated = namespace['generate_launch_description']()
        safety_source = SOURCE.parents[2] / 'mobotic_safety' / 'launch' / 'safety.launch.py'
        safety_namespace = {'generate_launch_description': resolved_factory(safety_source)}
        standalone = safety_namespace['generate_launch_description']()
        for description in (integrated, standalone):
            bridge = next(action for action in description.args[0]
                          if action.kind == 'Node'
                          and action.kwargs.get('executable') == 'flexisoft_tcp_bridge')
            self.assertTrue(bridge.kwargs['respawn'])
            self.assertEqual(bridge.kwargs['respawn_delay'], 1.0)

    def test_odometry_is_enabled_for_both_profiles_and_has_single_tf_owner_switch(self):
        arguments = {action.args[0]: action for action in self.actions if action.kind == 'DeclareLaunchArgument'}
        self.assertEqual(arguments['start_odometry'].kwargs['default_value'], 'true')
        self.assertEqual(arguments['publish_odom_tf'].kwargs['default_value'], 'auto')
        node = next(action for action in self.actions if action.kind == 'Node'
                    and action.kwargs.get('executable') == 'mobotic_odometry')
        self.assertEqual(node.kwargs['namespace'].args, ('robot_name',))
        self.assertEqual(node.kwargs['remappings'], [('/tf', 'tf')])
        self.assertFalse(node.kwargs['respawn'])
        self.assertIs(node.kwargs['parameters'][1]['publish_tf'].kwargs['value_type'], bool)
        self.assertEqual(node.kwargs['parameters'][1]['publish_tf'].args[0].args, ('publish_odom_tf',))
        diagnostics = next(action for action in self.actions if action.kind == 'Node'
                           and action.kwargs.get('executable') == 'platform_diagnostics')
        self.assertEqual(diagnostics.kwargs['parameters'][1]['monitor_odometry'].args[0].args, ('start_odometry',))
        for stack in ('hardware', 'virtual'):
            self.assertTrue(node.kwargs['condition'].perform({'stack_type': stack, 'start_odometry': 'true'}))
            self.assertFalse(node.kwargs['condition'].perform({'stack_type': stack, 'start_odometry': 'false'}))

    def test_standalone_odometry_matches_namespaced_tf_and_no_auto_pose_reset(self):
        source = SOURCE.parents[2] / 'mobotic_odometry' / 'launch' / 'odometry.launch.py'
        env = {'generate_launch_description': resolved_factory(source)}
        node = next(action for action in env['generate_launch_description']().args[0] if action.kind == 'Node')
        self.assertEqual(node.kwargs['remappings'], [('/tf', 'tf')])
        self.assertFalse(node.kwargs['respawn'])
        self.assertTrue(node.kwargs['parameters'][0]['publish_tf'])

    def test_both_scanners_expose_separate_raw_and_laser_scan_topics(self):
        scanners = [action for action in self.actions if action.kind == 'Node'
                    and action.kwargs.get('package') == 'sick_safetyscanners2']
        self.assertEqual(len(scanners), 2)
        for node, location in zip(scanners, ('front_left', 'rear_right')):
            remappings = dict(node.kwargs['remappings'])
            self.assertEqual(remappings['~/raw_data'], f'scanner/{location}/raw_data')
            self.assertEqual(remappings['~/scan'], f'scanner/{location}/scan')


if __name__ == '__main__':
    unittest.main()
