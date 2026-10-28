"""Shared profile regression, rejection and launch-adapter checks, without ROS."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
import yaml
import ast
import contextlib
import io
import os
from unittest.mock import patch
import xml.etree.ElementTree as ET

from mobotic_config.configuration import Configuration, ConfigurationError, read_yaml, validate_platform, config_path, ros_parameters, main
from mobotic_config.description import render_description
from launch_harness import PACKAGES, factory, resolve, environment


class ConfigurationTest(unittest.TestCase):
    def setUp(self):
        self.cfg = Configuration()

    def test_every_previous_parameter_is_preserved(self):
        baseline = json.loads(Path(__file__).with_name('baseline_parameters.json').read_text())
        # Hardware profile now selects CAN; unrelated USB policy/drive limits stay intact.
        baseline['manual']['remdevice_mapping'] = json.dumps(
            {'verified': False, 'node_id': 9, 'axes': {}, 'buttons': {}}, sort_keys=True)
        baseline['supervisor']['require_operator_interlock'] = True
        baseline['supervisor']['operator_scanner_override_enabled'] = False
        baseline['supervisor']['safety_source'] = 'flexisoft'
        baseline['supervisor'].pop('warning_speed_scale')
        baseline['supervisor']['warning_speed_limit'] = 0.3
        baseline['supervisor']['wheel_radius'] = 0.325
        baseline['driver']['physical_sample_skew_budget'] = 0.05
        baseline['driver']['startup_timeout'] = 15.0
        baseline['manual'].update(axis_speed_sign=1, axis_crab_sign=1, axis_steer_sign=1)
        for component, values in baseline.items():
            self.assertEqual(self.cfg.parameters(component), values, component)

    def test_virtual_driver_scaling_comes_from_same_records(self):
        p = self.cfg.parameters('driver')
        for index, (joint, drive) in enumerate(self.cfg.virtual_drives()):
            self.assertEqual(joint, p['drives.names'][index])
            self.assertEqual(drive['can_node_id'], p['drives.can_node_ids'][index])
            self.assertEqual(drive['encoder_resolution'], p['drives.resolutions'][index])
            self.assertEqual(drive['gear_ratio'], p['drives.gear_ratios'][index])

    def test_battery_readiness_requires_booleans_and_noncharging_interlock(self):
        for operational, charge in ((False, False), ('false', True), (True, 1), (None, True)):
            p = copy.deepcopy(self.cfg.platform)
            p['battery']['readiness'] = {'require_operational_status': operational, 'require_not_charging': charge}
            with self.assertRaises(ConfigurationError):
                validate_platform(p)

    def test_scanner_selector_is_restricted_to_commissioned_two_module_hardware(self):
        p = copy.deepcopy(self.cfg.platform)
        p['operator']['mapping'] = {
            'verified': True, 'node_id': 10,
            'axes': {role: {'channel': channel, 'zero': 0, 'full_scale': 255, 'invert': False}
                     for role, channel in (('speed', 3), ('steer', 2), ('crab', 4))},
            'buttons': {'mode': {'signal': 'U5', 'active_high': False},
                        'boost': {'signal': 'SP3', 'active_high': True},
                        'reset': {'signal': 'SP4', 'active_high': True}, 'deadman': None,
                        'scanner_override': {'signal': 'D5', 'active_high': True}}}
        p['can']['master_node_id'] = '0x0B'
        with self.assertRaises(ConfigurationError):
            validate_platform(p)
        p['modules'] = [m for m in p['modules'] if m['name'] in ('front_right', 'rear_left')]
        p['safety'] = {'source': 'hardwired_sto'}
        validate_platform(p)

    def test_battery_count_matches_each_supplied_wheel_module_option(self):
        profiles = (
            (self.cfg.path, 4, ['battery1', 'battery2']),
            (self.cfg.path.with_name('platform-front-right-rear-left.yaml'), 2, ['battery1']),
        )
        for path, module_count, identifiers in profiles:
            with self.subTest(path=path):
                cfg = Configuration(path)
                battery = cfg.parameters('battery')
                self.assertEqual(len(cfg.platform['modules']), module_count)
                self.assertEqual(battery['batteries.ids'], identifiers)
                for key in ('source_addresses', 'status_pgns', 'soc_pgns', 'voltage_pgns'):
                    self.assertEqual(len(battery['batteries.' + key]), len(identifiers))
                self.assertEqual(battery['primary_battery_id'], 'battery1')
                self.assertEqual(cfg.platform['can']['battery'],
                                 {'interface': 'can1', 'bitrate': 500000, 'listen_only': True})

    def test_integrated_and_standalone_battery_launch_follow_selected_pack_count(self):
        for path, identifiers in (
                (self.cfg.path, ['battery1', 'battery2']),
                (self.cfg.path.with_name('platform-front-right-rear-left.yaml'), ['battery1'])):
            with self.subTest(path=path):
                overrides = {'platform_config': str(path)}
                integrated, _ = resolve(
                    factory(PACKAGES / 'mobotic_bringup/launch/moboterra.launch.py')(), overrides)
                standalone, _ = resolve(environment()['standalone_launch']('battery'), overrides)
                for actions in (integrated, standalone):
                    node = next(a for a in actions if a.kind == 'Node'
                                and a.kwargs['executable'] == 'vanguard_battery_monitor')
                    self.assertEqual(node.kwargs['parameters'][0]['batteries.ids'], identifiers)

    def test_battery_topics_and_telegram_bindings_cannot_alias(self):
        for bad_id in ('battery/2', '2battery', 'bad name'):
            profile = copy.deepcopy(self.cfg.platform)
            profile['battery']['participants'][1]['id'] = bad_id
            with self.subTest(identifier=bad_id), self.assertRaises(ConfigurationError):
                validate_platform(profile)
        profile = copy.deepcopy(self.cfg.platform)
        duplicate = dict(profile['battery']['participants'][0], id='another_battery')
        profile['battery']['participants'][1] = duplicate
        with self.assertRaisesRegex(ConfigurationError, 'unique'):
            validate_platform(profile)

    def test_confirmed_two_module_pail_assignments_and_distinct_master(self):
        cfg = Configuration(self.cfg.path.with_name('platform-front-right-rear-left.yaml'))
        mapping = cfg.platform['operator']['mapping']
        self.assertTrue(mapping['verified'])
        self.assertEqual(mapping['node_id'], 10)
        self.assertEqual(int(cfg.platform['can']['master_node_id'], 0), 11)
        self.assertEqual([mapping['axes'][role]['channel'] for role in ('speed', 'steer', 'crab')], [3, 2, 4])
        self.assertEqual(mapping['buttons']['boost']['signal'], 'SP3')
        self.assertEqual(mapping['buttons']['reset']['signal'], 'SP4')
        self.assertTrue(cfg.parameters('supervisor')['require_operator_interlock'])

    def test_invalid_runtime_timing_and_speed_scale_fail_before_launch(self):
        for component, override in (
                ('driver', {'feedback_request_period': .3}),
                ('driver', {'fault_feedback_timeout': .5}),
                ('supervisor', {'control_period': .2}),
                ('supervisor', {'warning_speed_limit': 0}),
                ('supervisor', {'startup_grace': 0}),
                ('supervisor', {'mode_transition_timeout': .1}),
                ('manual', {'publish_period': 1.0})):
            cfg = Configuration()
            cfg.apply_overrides(component, override)
            with self.subTest(component=component, override=override), self.assertRaises(ConfigurationError):
                cfg._validate_policy()

    def test_two_module_profile_propagates_without_omitted_drives(self):
        path = self.cfg.path.with_name('platform-front-right-rear-left.yaml')
        cfg = Configuration(path)
        names = ['front_right', 'rear_left']
        joints = [name + '_' + role for name in names for role in ('steering', 'traction')]
        driver = cfg.parameters('driver')
        self.assertEqual(driver['modules.names'], names)
        self.assertEqual(driver['drives.names'], joints)
        self.assertEqual(driver['drives.can_node_ids'], ['0x03', '0x04', '0x07', '0x08'])
        self.assertEqual(cfg.parameters('kinematics')['module_names'], names)
        self.assertEqual(cfg.parameters('kinematics')['wheels_x'], [0.845, -0.845])
        self.assertEqual(cfg.parameters('kinematics')['wheels_y'], [-0.495, 0.495])
        self.assertEqual(cfg.parameters('supervisor')['expected_module_names'], names)
        self.assertEqual([joint for joint, _ in cfg.virtual_drives()], joints)
        original = copy.deepcopy(self.cfg.platform)
        original['modules'] = [m for m in original['modules'] if m['name'] in names]
        original['can']['drives']['bitrate'] = 250000
        original['can']['master_node_id'] = '0x0B'
        original['operator'] = copy.deepcopy(cfg.platform['operator'])
        original['safety'] = {'source': 'hardwired_sto'}
        original['battery']['participants'] = original['battery']['participants'][:1]
        original['commissioning_pending'] = [item.replace(
            'Confirm FlexiSoft/ELMO independent STO and enable bit mapping.',
            'Verify Siemens hardwired STO chain and fresh per-module drive STO feedback.')
            for item in original['commissioning_pending']]
        original['commissioning_pending'] = [item.replace(
            'Capture PAIL controls and verify axis calibration, RP1/B1/B2/STOP and radio-loss behavior.',
            'Verify PAIL STOP/radio-loss and neutral/rearm behavior on the installed robot.')
            for item in original['commissioning_pending']]
        self.assertEqual(cfg.platform, original)
        for stack in ('hardware', 'virtual'):
            actions, _ = resolve(factory(PACKAGES / 'mobotic_bringup/launch/moboterra.launch.py')(),
                                 {'platform_config': str(path), 'stack_type': stack})
            node = next(a for a in actions if a.kind == 'Node' and a.kwargs['name'] == 'mobotic_driver')
            self.assertEqual(node.kwargs['parameters'][0]['drives.names'], joints)
            simulated = [a.kwargs['parameters'][0]['can_node_id'] for a in actions
                         if a.kind == 'Node' and a.kwargs['executable'] == 'virtual_mobotic_drive']
            self.assertEqual(simulated, ['0x03', '0x04', '0x07', '0x08'])
        robot = ET.fromstring(render_description(cfg))
        moving = {j.get('name') for j in robot.findall('joint') if j.get('type') != 'fixed'}
        self.assertEqual(moving, set(joints))
        links = {link.get('name') for link in robot.findall('link')}
        self.assertNotIn('front_left_wheel', links)
        self.assertNotIn('rear_right_wheel', links)
        self.assertIn('front_left_scan', links)
        self.assertIn('rear_right_scan', links)

    def test_hardwired_sto_source_has_no_gateway_settings_and_keeps_monitor(self):
        path = self.cfg.path.with_name('platform-front-right-rear-left.yaml')
        cfg = Configuration(path)
        self.assertEqual(cfg.parameters('supervisor')['safety_source'], 'hardwired_sto')
        self.assertEqual(cfg.diagnostic_parameters()['safety_source'], 'hardwired_sto')
        self.assertEqual(cfg.parameters('safety')['expected_module_names'], ['front_right', 'rear_left'])
        for source in ('mobotic_bringup/launch/moboterra.launch.py', 'mobotic_safety/launch/safety.launch.py'):
            actions, _ = resolve(factory(PACKAGES / source)(), {'platform_config': str(path)})
            executables = [a.kwargs['executable'] for a in actions if a.kind == 'Node']
            self.assertNotIn('flexisoft_tcp_bridge', executables)
            self.assertIn('safety_monitor', executables)
            self.assertIn('scanner_safety_adapter', executables)
        for safety in ({'source': 'none'}, {'source': 'hardwired_sto', 'port': 9100}):
            profile = copy.deepcopy(cfg.platform)
            profile['safety'] = safety
            with self.assertRaises(ConfigurationError):
                validate_platform(profile)

    def test_four_modules_cannot_select_no_plc_telemetry_mode(self):
        profile = copy.deepcopy(self.cfg.platform)
        self.assertEqual(len(profile['modules']), 4)
        profile['safety'] = {'source': 'hardwired_sto'}
        with self.assertRaisesRegex(ConfigurationError, 'exactly two'):
            validate_platform(profile)

    def test_four_module_launch_retains_flexisoft(self):
        self.assertEqual(self.cfg.parameters('supervisor')['safety_source'], 'flexisoft')
        self.assertEqual(self.cfg.diagnostic_parameters()['safety_source'], 'flexisoft')
        for source in ('mobotic_bringup/launch/moboterra.launch.py', 'mobotic_safety/launch/safety.launch.py'):
            actions, _ = resolve(factory(PACKAGES / source)(), {'platform_config': str(self.cfg.path)})
            executables = [a.kwargs['executable'] for a in actions if a.kind == 'Node']
            self.assertIn('flexisoft_tcp_bridge', executables)

    def test_module_subsets_reject_missing_duplicate_unknown_and_coincident_axes(self):
        modules = self.cfg.platform['modules']
        for selection in ([], modules[:1], [modules[1], modules[1]], modules[1:3][::-1]):
            p = copy.deepcopy(self.cfg.platform)
            p['modules'] = copy.deepcopy(selection)
            with self.subTest(selection=[m['name'] for m in selection]), self.assertRaises(ConfigurationError):
                validate_platform(p)
        for mutation in ('unknown', 'coincident', 'frame_collision'):
            p = copy.deepcopy(self.cfg.platform)
            p['modules'] = p['modules'][1:3]
            if mutation == 'unknown': p['modules'][0]['name'] = 'unknown'
            if mutation == 'coincident': p['modules'][1]['cad_steering_xyz'] = p['modules'][0]['cad_steering_xyz'][:]
            if mutation == 'frame_collision': p['frames']['base'] = 'front_right_wheel'
            with self.subTest(mutation=mutation), self.assertRaises(ConfigurationError):
                validate_platform(p)

    def test_ordered_three_module_subset_is_supported(self):
        p = copy.deepcopy(self.cfg.platform)
        p['modules'] = p['modules'][:3]
        validate_platform(p)

    def test_each_two_module_combination_and_four_module_layout_use_the_same_launch(self):
        from itertools import combinations
        modules = self.cfg.platform['modules']
        selections = list(combinations(modules, 2)) + [tuple(modules)]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'platform.yaml'
            for selection in selections:
                profile = copy.deepcopy(self.cfg.platform)
                profile['modules'] = list(selection)
                path.write_text(yaml.safe_dump(profile))
                cfg = Configuration(path)
                expected = cfg.parameters('driver')['drives.names']
                actions, _ = resolve(factory(PACKAGES / 'mobotic_bringup/launch/moboterra.launch.py')(),
                                     {'platform_config': str(path), 'stack_type': 'virtual'})
                self.assertEqual(len([a for a in actions if a.kind == 'Node'
                                      and a.kwargs['executable'] == 'virtual_mobotic_drive']), len(expected))
                robot = ET.fromstring(render_description(cfg))
                self.assertEqual({j.get('name') for j in robot.findall('joint') if j.get('type') != 'fixed'},
                                 set(expected))
                standalone, _ = resolve(environment()['virtual_launch'](), {'platform_config': str(path)})
                node = next(a for a in standalone if a.kind == 'Node')
                self.assertEqual(node.kwargs['parameters'][0]['can_node_id'], selection[0]['steering']['can_node_id'])

    def test_synchronized_profile_rejects_missing_pdos_and_invalid_skew(self):
        for overrides in ({'traction_pdo_enabled': False}, {'telemetry_pdo_enabled': False},
                          {'sample_skew_budget': 0}, {'sample_skew_budget': 0.02}):
            cfg = Configuration()
            cfg.apply_overrides('driver', overrides)
            with self.subTest(overrides=overrides), self.assertRaises(ConfigurationError):
                cfg._validate_policy()
        for mode in ('autonomous', 'kinematicoff'):
            cfg = Configuration()
            cfg.apply_overrides('supervisor', {'initial_mode': mode})
            cfg._validate_policy()

    def test_invalid_physical_profiles_fail(self):
        mutations = [lambda p: p.update(schema_version=True),
                     lambda p: p['geometry'].update(wheel_radius=float('nan')),
                     lambda p: p['modules'][0]['traction'].update(can_node_id='0x01'),
                     lambda p: p['modules'].reverse(),
                     lambda p: p['can']['battery'].update(interface='can0'),
                     lambda p: p['can']['battery'].update(listen_only='false'),
                     lambda p: p['motion_limits'].update(max_traction_velocity=1000),
                     lambda p: p['safety'].pop('emergency_stop_mask'),
                     lambda p: p['safety'].update(sto_mask=16),
                     lambda p: p['scanners']['units'][0].update(sensor_ip='bad'),
                     lambda p: p['battery'].update(primary_id='absent')]
        for mutation in mutations:
            with self.subTest(mutation=mutation):
                p = copy.deepcopy(self.cfg.platform)
                mutation(p)
                with self.assertRaises(ConfigurationError):
                    validate_platform(p)

    def test_battery_ack_mode_requires_an_explicit_boolean(self):
        for value in (True, False):
            profile = copy.deepcopy(self.cfg.platform)
            profile['can']['battery']['listen_only'] = value
            validate_platform(profile)
        for value in ('false', 'true', 0, 1, None):
            profile = copy.deepcopy(self.cfg.platform)
            profile['can']['battery']['listen_only'] = value
            with self.subTest(value=value), self.assertRaises(ConfigurationError):
                validate_platform(profile)

    def test_duplicate_yaml_keys_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'duplicate.yaml'
            path.write_text('outer:\n  value: 1\n  value: 2\n')
            with self.assertRaisesRegex(ConfigurationError, 'duplicate YAML'):
                read_yaml(path)

    def test_conflicting_legacy_hardware_is_rejected(self):
        with self.assertRaisesRegex(ConfigurationError, 'conflicts'):
            self.cfg.apply_overrides('kinematics', {'wheel_radius': 0.1125})
        with self.assertRaisesRegex(ConfigurationError, 'cannot exceed'):
            self.cfg.apply_overrides('manual', {'max_velocity_linear': 2.0})
        self.cfg.apply_overrides('manual', {'max_velocity_linear': 0.75})
        self.assertEqual(self.cfg.parameters('manual')['max_velocity_linear'], 0.75)
        self.assertEqual(self.cfg.parameters('supervisor')['max_velocity_linear'], 1.25)
        self.cfg.apply_overrides('driver', {'drives.can_node_ids': [1, 2, 3, 4, 7, 8, 5, 6]})

    def test_custom_profile_propagates_to_all_consumers(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'platform.yaml'
            p = copy.deepcopy(self.cfg.platform)
            p['geometry']['wheel_radius'] = 0.33
            p['frames']['base'] = 'custom_base'
            p['modules'][0]['cad_steering_xyz'][0] = 0.9
            p['modules'][0]['traction']['gear_ratio'] = 15.0
            path.write_text(yaml.safe_dump(p))
            cfg = Configuration(path)
            self.assertEqual(cfg.parameters('kinematics')['wheels_x'][0], 0.9)
            self.assertEqual(cfg.parameters('kinematics')['wheel_radius'], 0.33)
            self.assertEqual(cfg.parameters('odometry')['base_frame_id'], 'custom_base')
            self.assertEqual(cfg.virtual_drives()[1][1]['gear_ratio'], 15.0)

    def test_integrated_and_standalone_effective_parameters_match(self):
        actions, context = resolve(factory(PACKAGES / 'mobotic_bringup/launch/moboterra.launch.py')())
        self.assertEqual(context['robot_name'], 'moboterra')
        self.assertEqual((context['can_interface'], context['battery_can_interface']), ('can0', 'can1'))
        env = environment()
        executable = {'driver': 'mobotic_driver', 'kinematics': 'mobotic_kinematics',
                      'manual': 'remdevice_control', 'supervisor': 'mobotic_supervisor',
                      'battery': 'vanguard_battery_monitor', 'odometry': 'mobotic_odometry'}
        for component, name in executable.items():
            standalone, _ = resolve(env['standalone_launch'](component))
            node = next(a for a in standalone if a.kind == 'Node' and a.kwargs['executable'] == name)
            integrated = next(a for a in actions if a.kind == 'Node' and a.kwargs['executable'] == name)
            expected = dict(node.kwargs['parameters'][0])
            if component == 'supervisor':
                expected['required_health_sources'] = ['kinematics', 'odometry']
            self.assertEqual(expected, integrated.kwargs['parameters'][0])

    def test_can_operator_and_usb_fallback_select_one_standalone_provider(self):
        env = environment()
        actions, _ = resolve(env['standalone_launch']('manual'))
        nodes = [a.kwargs for a in actions if a.kind == 'Node']
        self.assertEqual([node['executable'] for node in nodes], ['remdevice_control'])
        self.assertEqual(nodes[0]['name'], 'operator')
        with tempfile.TemporaryDirectory() as directory:
            p = copy.deepcopy(self.cfg.platform)
            p['operator'] = {'source': 'usb'}
            path = Path(directory) / 'usb.yaml'
            path.write_text(yaml.safe_dump(p))
            actions, _ = resolve(env['standalone_launch']('manual'), {'platform_config': str(path)})
            nodes = [a.kwargs for a in actions if a.kind == 'Node']
            self.assertEqual([node['executable'] for node in nodes], ['game_controller_node', 'manual_control'])
            self.assertEqual(nodes[1]['name'], 'operator')

    def test_can_operator_rejects_node_collision_and_unknown_bus_setting(self):
        for mutation in (lambda p: p['operator']['mapping'].update(node_id=10),
                         lambda p: p['operator']['mapping'].update(node_id=1),
                         lambda p: p['operator'].update(interface='can2')):
            p = copy.deepcopy(self.cfg.platform)
            mutation(p)
            with self.assertRaises(ConfigurationError):
                validate_platform(p)

    def test_pail_usb_uses_generic_driver_and_inhibits_unverified_mapping(self):
        with tempfile.TemporaryDirectory() as directory:
            p = copy.deepcopy(self.cfg.platform)
            p['operator']['source'] = 'usb'
            path = Path(directory) / 'pail-usb.yaml'
            path.write_text(yaml.safe_dump(p))
            cfg = Configuration(path)
            self.assertFalse(cfg.parameters('manual')['input_mapping_verified'])
            actions, _ = resolve(environment()['standalone_launch']('manual'), {'platform_config': str(path)})
            nodes = [a.kwargs for a in actions if a.kind == 'Node']
            self.assertEqual([node['executable'] for node in nodes], ['joy_node', 'manual_control'])
            integrated, _ = resolve(factory(PACKAGES / 'mobotic_bringup/launch/moboterra.launch.py')(),
                                    {'platform_config': str(path)})
            joystick = next(a for a in integrated if a.kind == 'Node' and a.kwargs['name'] == 'joystick')
            self.assertEqual(joystick.kwargs['executable'], 'joy_node')

    def test_usb_driver_and_verification_types_are_validated(self):
        for change in (dict(usb_driver='unknown'), dict(usb_mapping_verified='true')):
            p = copy.deepcopy(self.cfg.platform)
            p['operator'].update(change)
            with self.assertRaises(ConfigurationError):
                validate_platform(p)

    def test_namespace_root_and_endpoint_overrides(self):
        desc = factory(PACKAGES / 'mobotic_bringup/launch/moboterra.launch.py')()
        actions, context = resolve(desc, {'robot_name': '', 'can_interface': 'vcan0',
                                          'battery_can_interface': 'vcan1', 'front_scanner_ip': '10.60.20.54'})
        self.assertEqual(context['robot_name'], '')
        scanner = next(a for a in actions if a.kind == 'Node' and a.kwargs['name'] == 'front_left_scanner')
        self.assertEqual(scanner.kwargs['parameters'][0]['sensor_ip'], '10.60.20.54')
        with self.assertRaises(ConfigurationError):
            resolve(desc, {'battery_can_interface': 'can0'})

    def test_tf_auto_respects_yaml_and_explicit_override(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'odom.yaml'
            path.write_text('/**:\n  ros__parameters:\n    publish_tf: false\n')
            desc = factory(PACKAGES / 'mobotic_bringup/launch/moboterra.launch.py')()
            _, ctx = resolve(desc, {'odometry_parameters_file': str(path)})
            self.assertEqual(ctx['publish_odom_tf'], 'false')
            _, ctx = resolve(desc, {'odometry_parameters_file': str(path), 'publish_odom_tf': 'true'})
            self.assertEqual(ctx['publish_odom_tf'], 'true')

    def test_diagnostics_follow_effective_policy(self):
        self.cfg.apply_overrides('supervisor', {'manual_timeout': 0.2, 'battery_timeout': 0.8})
        self.cfg.apply_overrides('driver', {'supervisor_timeout': 0.4})
        params = self.cfg.diagnostic_parameters()
        self.assertEqual((params['manual_timeout'], params['battery_timeout'], params['mode_timeout']), (.2, .8, .4))

    def test_yaml_files_have_one_owner_and_no_physical_duplicates(self):
        files = list(PACKAGES.rglob('*.yaml'))
        expected = {'platform.yaml', 'platform-front-right-rear-left.yaml'}
        expected.update(Path(config_path(component)).name for component in self.cfg._parameters)
        self.assertEqual({path.name for path in files}, expected)
        self.assertTrue(all(path.parent == PACKAGES / 'mobotic_config/config' for path in files))
        for component, derived in self.cfg._derived.items():
            self.assertFalse(set(ros_parameters(config_path(component))) & set(derived), component)

    def test_wrong_overlay_types_and_nonfinite_lists_fail(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'overlay.yaml'
            for component, values in [('driver', {'traction_pdo_enabled': 'false'}),
                                      ('supervisor', {'manual_timeout': -1}),
                                      ('odometry', {'initial_pose': [0, 0, float('inf')]}),
                                      ('manual', {'button_mode_switch': True}),
                                      ('manual', {'buttons_boost': []}),
                                      ('driver', {'telemetry_pdo_sync_divider': 0}),
                                      ('supervisor', {'warning_speed_scale': 2}),
                                      ('safety', {'made_up_mask': 1})]:
                with self.subTest(component=component, values=values):
                    path.write_text(yaml.safe_dump({'/**': {'ros__parameters': values}}))
                    with self.assertRaises(ConfigurationError):
                        Configuration(overrides={component: str(path)})

    def test_description_custom_profile_tracks_control_and_scanner_parameters(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'platform.yaml'
            p = copy.deepcopy(self.cfg.platform)
            p['geometry'].update(wheel_radius=0.33, wheel_width=0.32)
            p['frames']['base'] = 'custom_base'
            p['modules'][0]['cad_steering_xyz'] = [0.9, 0.5, 0.8]
            p['modules'][0]['mounting_yaw'] = 0.1
            p['modules'][0]['steering']['resolution'] = 8192
            p['scanners']['units'][0].update(frame_id='custom_scan', cad_xyz=[1.2, 0.8, 0.26])
            path.write_text(yaml.safe_dump(p))
            cfg = Configuration(path)
            robot = ET.fromstring(render_description(cfg))
            joints = {j.get('name'): j for j in robot.findall('joint')}
            self.assertIsNotNone(robot.find("link[@name='custom_base']"))
            self.assertEqual(joints['base_to_chassis'].find('origin').get('xyz'), '0 0 -0.33')
            xyz = [float(v) for v in joints['front_left_mount_fixed'].find('origin').get('xyz').split()]
            k = cfg.parameters('kinematics')
            self.assertEqual(xyz, [k['wheels_x'][0], k['wheels_y'][0], 0.8])
            self.assertAlmostEqual(float(joints['front_left_traction'].find('origin').get('xyz').split()[2]), -.47)
            cylinders = robot.findall('.//geometry/cylinder')
            self.assertTrue(all(float(c.get('radius')) == .33 and float(c.get('length')) == .32 for c in cylinders))
            self.assertEqual(cfg.scanner_parameters('front_left')['frame_id'], 'custom_scan')
            self.assertIn('custom_scan_fixed', joints)
            self.assertEqual(cfg.parameters('supervisor')['direct_steering_encoder_resolutions'][0], 8192.)
            self.assertEqual(cfg.virtual_drives()[0][1]['encoder_resolution'], 8192.)

    def test_namespace_environment_precedence(self):
        with patch.dict(os.environ, {'ROBOT_NAME': 'environment_robot'}):
            desc = factory(PACKAGES / 'mobotic_bringup/launch/moboterra.launch.py')()
            self.assertEqual(resolve(desc)[1]['robot_name'], 'environment_robot')
            self.assertEqual(resolve(desc, {'robot_name': 'explicit_robot'})[1]['robot_name'], 'explicit_robot')
            self.assertEqual(resolve(desc, {'robot_name': ''})[1]['robot_name'], '')

    def test_legacy_launch_arguments_warn_and_reject_divergent_geometry(self):
        env = environment()
        with self.assertWarns(UserWarning):
            actions, _ = resolve(env['standalone_launch']('driver'), {'can_heartbeat_period': '0.01'})
        node = next(a for a in actions if a.kind == 'Node')
        self.assertEqual(node.kwargs['parameters'][0]['can_heartbeat_period'], .01)
        with self.assertWarns(UserWarning), self.assertRaises(ConfigurationError):
            resolve(env['standalone_launch']('driver'), {'drives.gear_ratios': '[1.0] '})
        with self.assertWarns(UserWarning), self.assertRaises(ConfigurationError):
            resolve(env['virtual_launch'](), {'gear_ratio': '16'})
        actions, _ = resolve(env['virtual_launch'](), {'module_name': 'rear_right', 'drive_type': 'traction'})
        self.assertEqual(next(a for a in actions if a.kind == 'Node').kwargs['parameters'][0]['can_node_id'], '0x06')

    def test_cli_is_readonly_and_checks_policy_overlays(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'supervisor.yaml'
            path.write_text('/**:\n  ros__parameters:\n    battery_timeout: 0.8\n')
            stream = io.StringIO()
            with contextlib.redirect_stdout(stream):
                main(['--override', 'supervisor=' + str(path)])
            result = json.loads(stream.getvalue())
            self.assertEqual(result['diagnostics']['battery_timeout'], .8)
            self.assertTrue(result['commissioning_pending'])

    def test_packaging_installs_all_configuration_and_launch_dependencies(self):
        package = PACKAGES / 'mobotic_config'
        recorded = {}
        source = package / 'setup.py'
        nodes = [n for n in ast.parse(source.read_text()).body if not isinstance(n, (ast.Import, ast.ImportFrom))]
        env = {'setup': lambda **kwargs: recorded.update(kwargs), 'find_packages': lambda **_: ['mobotic_config'],
               'glob': lambda pattern: [str(p.relative_to(package)) for p in package.glob(pattern)]}
        exec(compile(ast.Module(body=nodes, type_ignores=[]), str(source), 'exec'), env)
        for _, paths in recorded['data_files']:
            for path in paths:
                self.assertTrue((package / path).is_file(), path)
        self.assertEqual(recorded['entry_points']['console_scripts'], ['check_config = mobotic_config.configuration:main'])
        for name in ('mobotic_bringup', 'mobotic_driver', 'mobotic_kinematics', 'mobotic_manual_control',
                     'mobotic_supervisor', 'mobotic_vanguard_battery', 'mobotic_safety', 'mobotic_odometry', 'mobotic_description'):
            deps = {d.text for d in ET.parse(PACKAGES / name / 'package.xml').getroot().findall('exec_depend')}
            self.assertIn('mobotic_config', deps)


if __name__ == '__main__':
    unittest.main()
