"""Shared profile regression, rejection and launch-adapter checks, without ROS."""
import copy
import json
import math
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
from platform_fixtures import four_module_platform


class ConfigurationTest(unittest.TestCase):
    def setUp(self):
        self.cfg = Configuration()
        self.fixture_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.fixture_directory.cleanup)
        self.verified_path = Path(self.fixture_directory.name) / 'four-modules.yaml'
        self.verified_path.write_text(yaml.safe_dump(four_module_platform()))

    def test_removed_hold_button_policy_is_rejected(self):
        for index in (-1, 8):
            with self.subTest(index=index), self.assertRaisesRegex(ConfigurationError, 'unknown override'):
                self.cfg.apply_overrides('manual', {'button_deadman': index})

    def test_controller_selection_defaults_overrides_and_virtual_nodes(self):
        platform = copy.deepcopy(self.cfg.platform)
        for module in platform['modules']:
            module['traction'].pop('controller', None)
        platform['actuators']['traction']['controller'] = 'miControl'
        platform['actuators']['traction']['min_traction_velocity'] = -1700
        platform['actuators']['traction']['max_traction_velocity'] = 1700
        platform['modules'][1]['traction'].update(controller='miControl')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'platform.yaml'
            path.write_text(yaml.safe_dump(platform))
            cfg = Configuration(path)
            expected = ['micontrol'] * 8
            self.assertEqual(cfg.parameters('driver')['drives.controller_types'], expected)
            self.assertEqual([p['controller_type'] for _, p in cfg.virtual_drives()], expected)
            self.assertEqual(cfg.parameters('driver')['drives.modes'], ['position', 'velocity'] * 4)

    def test_neutral_homing_geometry_and_driver_ceiling(self):
        platform = four_module_platform()
        platform['motion_limits'].update(max_steering_velocity=.3,
            steering_return_home_on_neutral=True, steering_neutral_home_velocity=1.2)
        self.verified_path.write_text(yaml.safe_dump(platform))
        cfg = Configuration(self.verified_path)
        geometry = json.loads(cfg.parameters('manual')['steering_geometry'])
        self.assertTrue(geometry['steering_return_home_on_neutral'])
        self.assertEqual(geometry['steering_neutral_home_velocity'], 1.2)
        self.assertEqual(geometry['max_steering_velocity'], .3)
        self.assertEqual(cfg.parameters('driver')['max_steering_velocity'], 1.2)
        self.assertEqual(cfg.parameters('kinematics')['max_steering_velocity'], .3)
        with self.assertRaisesRegex(ConfigurationError, 'conflicts with platform_config'):
            cfg.apply_overrides('driver', {'max_steering_velocity': .3})

    def test_invalid_neutral_homing_configuration_is_rejected(self):
        for key, value in [('steering_return_home_on_neutral', 'true'),
                           ('steering_neutral_home_velocity', 0.),
                           ('steering_neutral_home_velocity', -1.),
                           ('steering_neutral_home_velocity', float('nan')),
                           ('steering_neutral_home_velocity', 100.)]:
            with self.subTest(key=key, value=value):
                platform = four_module_platform()
                platform['motion_limits'].update(steering_return_home_on_neutral=True,
                                                 steering_neutral_home_velocity=1.2)
                platform['motion_limits'][key] = value
                self.verified_path.write_text(yaml.safe_dump(platform))
                with self.assertRaises(ConfigurationError):
                    Configuration(self.verified_path)

    def test_steering_alignment_policy_bounds(self):
        self.cfg.apply_overrides('driver', {'steering_align_on_enable': True,
                                           'steering_alignment_velocity': 0.3,
                                           'steering_alignment_timeout': 10.0})
        self.cfg._validate_policy()
        for key, value in (('steering_alignment_velocity', 0), ('steering_alignment_velocity', 100),
                           ('steering_alignment_timeout', 0), ('steering_alignment_timeout', 15)):
            cfg = Configuration()
            cfg.apply_overrides('driver', {'steering_align_on_enable': True, key: value})
            with self.assertRaises(ConfigurationError):
                cfg._validate_policy()

    def test_installed_micontrol_steering_uses_user_units_not_encoder_counts(self):
        directory = Path(__file__).resolve().parents[1] / 'config'
        for profile in ('platform.yaml', 'platform-four-micontrol.yaml'):
            with self.subTest(profile=profile):
                cfg = Configuration(directory / profile)
                driver = cfg.parameters('driver')
                resolutions = dict(zip(driver['drives.names'], driver['drives.resolutions']))
                steering = cfg.parameters('supervisor')['expected_steering_joint_names']
                traction = cfg.parameters('supervisor')['expected_traction_joint_names']
                self.assertEqual([resolutions[n] for n in steering], [121.0] * 4)
                self.assertEqual([resolutions[n] for n in traction], [4096.0] * 4)
                for name in steering:
                    self.assertEqual(round((math.pi/2)*resolutions[name]/(2*math.pi)), 30)
                    self.assertAlmostEqual(30*(2*math.pi)/resolutions[name]*180/math.pi, 89.2561983471)
                geometry = json.loads(cfg.parameters('manual')['steering_geometry'])
                self.assertEqual(geometry['steering_position_tolerances'], [math.radians(1.)] * 4)
                self.assertEqual(geometry['steering_position_resolutions'], [121.] * 4)
                virtual = dict(cfg.virtual_drives())
                for name in steering:
                    self.assertEqual(virtual[name]['encoder_resolution'], resolutions[name])

    def test_one_degree_cap_keeps_finer_encoder_tolerances(self):
        platform = four_module_platform()
        for module, resolution in zip(platform['modules'], [121, 4096, 720, 8192]):
            module['steering']['resolution'] = resolution
        self.verified_path.write_text(yaml.safe_dump(platform))
        cfg = Configuration(self.verified_path)
        for component, key in [('manual', 'steering_geometry'), ('supervisor', 'manual_steering_geometry')]:
            geometry = json.loads(cfg.parameters(component)[key])
            self.assertEqual(geometry['steering_position_resolutions'], [121., 4096., 720., 8192.])
            self.assertEqual(geometry['steering_position_tolerances'],
                [min(math.radians(1.), 2*math.pi/r) for r in [121, 4096, 720, 8192]])

    def test_homing_can_run_twice_as_fast_as_manual_steering(self):
        platform = copy.deepcopy(self.cfg.platform)
        platform['motion_limits']['max_steering_velocity'] = 0.3
        platform['motion_limits']['steering_return_home_on_neutral'] = False
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / 'slow-manual.yaml'
        path.write_text(yaml.safe_dump(platform))
        cfg = Configuration(path)
        cfg.apply_overrides('driver', {'steering_align_on_enable': True,
                                      'steering_alignment_velocity': 0.6})
        cfg._validate_policy()
        self.assertEqual(cfg.parameters('driver')['max_steering_velocity'], 0.3)
        self.assertEqual(cfg.parameters('driver')['steering_alignment_velocity'], 0.6)
        geometry = json.loads(cfg.parameters('manual')['steering_geometry'])
        self.assertEqual(geometry['max_steering_velocity'], 0.3)

    def test_controller_validation_and_native_velocity_limits(self):
        for bad in ('elmo', 'ELMO', 'unknown', '', None, 1):
            platform = copy.deepcopy(self.cfg.platform)
            platform['modules'][0]['traction']['controller'] = bad
            with self.assertRaises(ConfigurationError):
                validate_platform(platform)

        platform = copy.deepcopy(self.cfg.platform)
        platform['modules'][0]['steering']['controller'] = 'ELMO'
        with self.assertRaises(ConfigurationError):
            validate_platform(platform)
        platform = copy.deepcopy(self.cfg.platform)
        platform['modules'][0]['traction'].update(controller='miControl',
            min_traction_velocity=-100, max_traction_velocity=100)
        with self.assertRaisesRegex(ConfigurationError, 'traction limits'):
            validate_platform(platform)

    def test_optional_180_degree_crab_range_reaches_both_manual_authorities(self):
        platform=copy.deepcopy(self.cfg.platform)
        platform['motion_limits']['max_crab_angle']=math.pi
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'wide-crab.yaml'
            path.write_text(yaml.safe_dump(platform))
            cfg=Configuration(path)
            manual=json.loads(cfg.parameters('manual')['steering_geometry'])
            supervisor=json.loads(cfg.parameters('supervisor')['manual_steering_geometry'])
            self.assertEqual(manual['max_crab_angle'],math.pi)
            self.assertEqual(manual,supervisor)
        for bad in (True,0.,math.pi/4,math.pi+.001,float('nan')):
            platform['motion_limits']['max_crab_angle']=bad
            with self.subTest(angle=bad),self.assertRaises(ConfigurationError):
                validate_platform(platform)

    def test_steering_angle_limits_can_be_disabled_for_both_manual_authorities(self):
        platform=copy.deepcopy(self.cfg.platform)
        platform['motion_limits']['steering_angle_limits_enabled']=False
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'continuous-steering.yaml'
            path.write_text(yaml.safe_dump(platform))
            cfg=Configuration(path)
            manual=json.loads(cfg.parameters('manual')['steering_geometry'])
            supervisor=json.loads(cfg.parameters('supervisor')['manual_steering_geometry'])
            self.assertIs(manual['steering_angle_limits_enabled'],False)
            self.assertEqual(manual,supervisor)
        for value in (None,0,1,'false'):
            platform['motion_limits']['steering_angle_limits_enabled']=value
            with self.subTest(value=value),self.assertRaises(ConfigurationError):
                validate_platform(platform)

    def test_homing_speed_can_double_to_1_2_without_changing_manual_speed(self):
        platform=copy.deepcopy(self.cfg.platform)
        platform['motion_limits']['max_steering_velocity']=.3
        platform['motion_limits']['steering_return_home_on_neutral']=False
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'homing.yaml'
            path.write_text(yaml.safe_dump(platform))
            cfg=Configuration(path)
            cfg.apply_overrides('driver',{'steering_align_on_enable':True,'steering_alignment_velocity':1.2})
            cfg._validate_policy()
            self.assertEqual(cfg.parameters('driver')['steering_alignment_velocity'],1.2)
            self.assertEqual(cfg.parameters('driver')['max_steering_velocity'],.3)

    def test_prepared_four_module_micontrol_profile(self):
        path = Path(__file__).resolve().parents[1] / 'config/platform-four-micontrol.yaml'
        cfg = Configuration(path)
        self.assertEqual(len(cfg.platform['modules']), 4)
        self.assertEqual(cfg.parameters('driver')['drives.controller_types'], ['micontrol'] * 8)
        self.assertEqual(cfg.parameters('driver')['drives.max_traction_velocities'][1::2], [1700] * 4)
        self.assertEqual(cfg.parameters('supervisor')['direct_traction_control_mode'], 'current')
        self.assertTrue(cfg.parameters('supervisor')['kinematicoff_current_supported'])
        self.assertEqual(cfg.parameters('driver')['max_traction_current'], 4.0)

    def test_micontrol_current_mode_is_available(self):
        cfg = Configuration()
        self.assertTrue(cfg.parameters('supervisor')['kinematicoff_current_supported'])
        cfg.apply_overrides('supervisor', {'initial_mode': 'kinematicoff'})
        cfg._validate_policy()

    def test_profiles_without_controller_option_use_micontrol(self):
        platform = copy.deepcopy(self.cfg.platform)
        for role in ('steering', 'traction'):
            platform['actuators'][role].pop('controller', None)
            for module in platform['modules']:
                module[role].pop('controller', None)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'legacy.yaml'
            path.write_text(yaml.safe_dump(platform))
            self.assertEqual(Configuration(path).parameters('driver')['drives.controller_types'],
                             ['micontrol'] * 8)

    def test_actual_configuration_preserves_pre_migration_effective_parameters(self):
        # Captured from the commissioned profile and its native overrides
        # before centralization; all nine component configurations must agree.
        baseline = json.loads(Path(__file__).with_name('baseline_parameters.json').read_text())
        actual = {component: self.cfg.parameters(component) for component in self.cfg._parameters}
        self.assertEqual(actual, baseline)

    def test_component_input_deadline_tracks_health_producer_override(self):
        with tempfile.TemporaryDirectory() as directory:
            overlay = Path(directory) / 'diagnostics.yaml'
            overlay.write_text(yaml.safe_dump({'/**': {'ros__parameters': {'odometry_timeout': .65}}}))
            cfg = Configuration(overrides={'diagnostics': str(overlay)})
            self.assertEqual(cfg.parameters('supervisor')['health_input_timeout'], .65)
            self.assertEqual(cfg.parameters('diagnostics')['odometry_timeout'], .65)
            self.assertEqual(cfg.parameters('supervisor')['health_timeout'], .3)
            self.assertEqual(cfg.parameters('supervisor')['velocity_feedback_timeout'], .35)

    def test_battery_filters_preserve_telemetry_priorities_and_charging_guards(self):
        filters = []
        for entry in self.cfg.battery_can_filters().split(','):
            identifier, mask = entry.split(':')
            self.assertEqual((len(identifier), len(mask)), (8, 8))
            # ros2_socketcan's eight-digit form sets the extended-frame flag.
            self.assertTrue(int(mask, 16) & 0x80000000)
            filters.append((int(identifier, 16) | 0x80000000, int(mask, 16)))

        def accepts(identifier):
            return any(identifier & mask == expected & mask for expected, mask in filters)

        for pack in self.cfg.platform['battery']['participants']:
            for field in ('status_pgn', 'soc_pgn', 'voltage_pgn'):
                binding = (int(str(pack[field]), 0) << 8) | int(str(pack['source_address']), 0)
                for priority in range(8):
                    self.assertTrue(accepts(0x80000000 | (priority << 26) | binding))
                self.assertFalse(accepts(binding & 0x7FF))
        for identifier in (0x18FF10F3, 0x0CFFBC49):
            self.assertTrue(accepts(0x80000000 | identifier))
        for identifier in (0x181, 0x581, 0x18A, 0x28A, 0x70B, 0x80000000 | 0x18FF00A5):
            self.assertFalse(accepts(identifier))

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

    def test_four_module_scanner_selector_requires_verified_controls(self):
        profile = four_module_platform()
        validate_platform(profile)
        profile['operator']['mapping']['verified'] = False
        with self.assertRaisesRegex(ConfigurationError, 'verified'):
            validate_platform(profile)

    def test_battery_count_matches_each_supplied_wheel_module_option(self):
        profiles = (
            (self.cfg.path, 4, ['battery1', 'battery2']),
            (self.cfg.path.with_name('platform-four-micontrol.yaml'), 4, ['battery1', 'battery2']),
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
                                 {'interface': 'can0', 'bitrate': 500000, 'listen_only': False}
                                 if path == self.cfg.path else
                                 {'interface': 'can1', 'bitrate': 500000, 'listen_only': True})

    def test_integrated_and_standalone_battery_launch_follow_selected_pack_count(self):
        for path, identifiers in (
                (self.cfg.path, ['battery1', 'battery2']),
                (self.cfg.path.with_name('platform-four-micontrol.yaml'), ['battery1', 'battery2'])):
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

    def test_confirmed_four_module_pail_assignments_and_distinct_master(self):
        cfg = Configuration(self.verified_path)
        mapping = cfg.platform['operator']['mapping']
        self.assertTrue(mapping['verified'])
        self.assertEqual(mapping['node_id'], 10)
        self.assertEqual(int(cfg.platform['can']['master_node_id'], 0), 11)
        self.assertEqual([mapping['axes'][role]['channel'] for role in ('speed', 'steer', 'crab')], [3, 2, 4])
        self.assertEqual(mapping['buttons']['boost']['signal'], 'SP3')
        self.assertEqual(mapping['buttons']['reset']['signal'], 'SP4')
        self.assertFalse(cfg.parameters('supervisor')['require_operator_interlock'])
        self.assertFalse(cfg.parameters('manual')['require_neutral_rearm'])

    def test_invalid_runtime_timing_and_speed_scale_fail_before_launch(self):
        for component, override in (
                ('driver', {'feedback_request_period': .35}),
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

    def test_four_module_profile_propagates_all_eight_drives(self):
        cfg = Configuration(self.cfg.path.with_name('platform-four-micontrol.yaml'))
        names = ['front_left', 'front_right', 'rear_left', 'rear_right']
        joints = [n + '_' + role for n in names for role in ('steering', 'traction')]
        self.assertEqual(cfg.parameters('driver')['modules.names'], names)
        self.assertEqual(cfg.parameters('driver')['drives.names'], joints)
        self.assertEqual(cfg.parameters('driver')['drives.can_node_ids'], ['0x01','0x02','0x03','0x04','0x07','0x08','0x05','0x06'])
        self.assertEqual(cfg.parameters('kinematics')['module_names'], names)
        self.assertEqual(cfg.parameters('supervisor')['expected_module_names'], names)
        self.assertEqual([name for name, _ in cfg.virtual_drives()], joints)
        robot = ET.fromstring(render_description(cfg))
        self.assertEqual({j.get('name') for j in robot.findall('joint') if j.get('type') != 'fixed'}, set(joints))

    def test_hardware_scanner_flag_omits_adapter_but_virtual_keeps_its_adapter(self):
        path = self.verified_path
        for stack, enabled, expected in (('hardware', 'false', False), ('hardware', 'true', True),
                                         ('virtual', 'false', True), ('virtual', 'true', True)):
            with self.subTest(stack=stack, enabled=enabled):
                actions, context = resolve(factory(PACKAGES / 'mobotic_bringup/launch/moboterra.launch.py')(),
                    {'platform_config': str(path), 'stack_type': stack, 'start_scanners': enabled})
                adapter = next(a for a in actions if a.kind == 'Node' and
                               a.kwargs['executable'] == 'scanner_safety_adapter')
                self.assertEqual(adapter.kwargs['condition'].perform(context), expected)
                supervisor = next(a for a in actions if a.kind == 'Node' and
                                  a.kwargs['executable'] == 'mobotic_supervisor')
                self.assertTrue(supervisor.kwargs['parameters'][0]['operator_scanner_override_enabled'])

    def test_manual_hardware_requires_scanners_and_wires_their_timeout(self):
        launch = PACKAGES / 'mobotic_bringup/launch/moboterra.launch.py'
        overrides = {'platform_config': str(self.verified_path), 'stack_type': 'hardware',
                     'control_mode': 'manual_only', 'start_scanners': 'false'}
        with self.assertRaisesRegex(ValueError, 'requires start_scanners:=true'):
            resolve(factory(launch)(), overrides)
        overrides['start_scanners'] = 'true'
        actions, context = resolve(factory(launch)(), overrides)
        nodes = [a for a in actions if a.kind == 'Node']
        authority = next(a for a in nodes if a.kwargs['executable'] == 'manual_drive_control')
        self.assertFalse(any(a.kwargs['executable'] == 'mobotic_supervisor' for a in nodes))
        adapter = next(a for a in nodes if a.kwargs['executable'] == 'scanner_safety_adapter')
        self.assertTrue(adapter.kwargs['condition'].perform(context))
        self.assertEqual(authority.kwargs['parameters'][0]['scanner_timeout'],
                         adapter.kwargs['parameters'][0]['status_timeout'])
        self.assertEqual(sum(a.kwargs['executable'] == 'sick_safetyscanners2_node'
                             and a.kwargs['condition'].perform(context) for a in nodes), 2)

    def test_retired_safety_source_is_rejected(self):
        profile = copy.deepcopy(self.cfg.platform)
        profile['safety'] = {'source': 'hardwired_sto'}
        with self.assertRaisesRegex(ConfigurationError, '(?i)flexisoft'):
            validate_platform(profile)

    def test_four_module_rp2_without_scanners_keeps_plc_and_verified_mapping(self):
        profile = copy.deepcopy(self.cfg.platform)
        reference = read_yaml(self.verified_path)
        profile['operator'] = copy.deepcopy(reference['operator'])
        profile['can']['master_node_id'] = '0x0B'
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'four-rp2.yaml'
            path.write_text(yaml.safe_dump(profile))
            cfg = Configuration(path)
            self.assertTrue(cfg.parameters('supervisor')['operator_scanner_override_enabled'])
            self.assertEqual(len(cfg.parameters('supervisor')['expected_module_names']), 4)
            self.assertEqual(cfg.parameters('battery')['batteries.ids'], ['battery1', 'battery2'])
            actions, context = resolve(factory(PACKAGES / 'mobotic_bringup/launch/moboterra.launch.py')(),
                {'platform_config': str(path), 'stack_type': 'hardware', 'start_scanners': 'false'})
            nodes = [a for a in actions if a.kind == 'Node']
            bridge = next(a for a in nodes if a.kwargs['executable'] == 'flexisoft_tcp_bridge')
            self.assertTrue(bridge.kwargs['condition'].perform(context))
            scanner = next(a for a in nodes if a.kwargs['executable'] == 'scanner_safety_adapter')
            self.assertFalse(scanner.kwargs['condition'].perform(context))
            supervisor = next(a for a in nodes if a.kwargs['executable'] == 'mobotic_supervisor')
            self.assertEqual(supervisor.kwargs['parameters'][0]['safety_source'], 'flexisoft')
            self.assertTrue(supervisor.kwargs['parameters'][0]['operator_scanner_override_enabled'])
            operator = next(a for a in nodes if a.kwargs['executable'] == 'remdevice_control')
            mapping = json.loads(operator.kwargs['parameters'][0]['remdevice_mapping'])
            self.assertEqual(mapping['buttons']['scanner_override'], {'signal': 'D5', 'active_high': True})

    def test_four_module_launch_retains_flexisoft(self):
        self.assertEqual(self.cfg.parameters('supervisor')['safety_source'], 'flexisoft')
        self.assertFalse(self.cfg.parameters('supervisor')['require_flexisoft_telemetry'])
        self.assertEqual(self.cfg.diagnostic_parameters()['safety_source'], 'flexisoft')
        for source in ('mobotic_bringup/launch/moboterra.launch.py', 'mobotic_safety/launch/safety.launch.py'):
            actions, _ = resolve(factory(PACKAGES / source)(), {'platform_config': str(self.cfg.path)})
            executables = [a.kwargs['executable'] for a in actions if a.kind == 'Node']
            self.assertIn('flexisoft_tcp_bridge', executables)

    def test_flexisoft_telemetry_role_is_validated_and_derived_only_for_supervisor(self):
        for selected in (True, False, None):
            profile = copy.deepcopy(self.cfg.platform)
            if selected is None:
                profile['safety'].pop('telemetry_required')
            else:
                profile['safety']['telemetry_required'] = selected
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / 'platform.yaml'
                path.write_text(yaml.safe_dump(profile))
                cfg = Configuration(path)
                self.assertEqual(cfg.parameters('supervisor')['require_flexisoft_telemetry'],
                                 selected if selected is not None else True)
                self.assertNotIn('telemetry_required', cfg.parameters('safety'))
                actions, _ = resolve(factory(PACKAGES / 'mobotic_bringup/launch/moboterra.launch.py')(),
                                     {'platform_config': str(path)})
                supervisor = next(a for a in actions if a.kind == 'Node' and
                                  a.kwargs['executable'] == 'mobotic_supervisor')
                self.assertEqual(supervisor.kwargs['parameters'][0]['require_flexisoft_telemetry'],
                                 cfg.parameters('supervisor')['require_flexisoft_telemetry'])
                for executable in ('flexisoft_tcp_bridge', 'safety_monitor', 'scanner_safety_adapter'):
                    self.assertTrue(any(a.kind == 'Node' and a.kwargs['executable'] == executable
                                        for a in actions))
        for invalid in ('false', 0, 1, None):
            profile = copy.deepcopy(self.cfg.platform)
            profile['safety']['telemetry_required'] = invalid
            with self.assertRaisesRegex(ConfigurationError, 'telemetry_required must be boolean'):
                validate_platform(profile)

    def test_four_module_names_geometry_and_frames_are_validated(self):
        for mutation, error in (
                ('duplicate', 'all four names'), ('reordered', 'all four names'),
                ('unknown', 'all four names'), ('invalid_axis', 'CAD steering datum'),
                ('frame_collision', 'conflict with generated links')):
            profile = copy.deepcopy(self.cfg.platform)
            if mutation == 'duplicate':
                profile['modules'][1]['name'] = profile['modules'][0]['name']
            elif mutation == 'reordered':
                profile['modules'].reverse()
            elif mutation == 'unknown':
                profile['modules'][0]['name'] = 'unknown'
            elif mutation == 'invalid_axis':
                profile['modules'][0]['cad_steering_xyz'][0] = float('nan')
            else:
                profile['frames']['base'] = 'front_right_wheel'
            with self.subTest(mutation=mutation), self.assertRaisesRegex(ConfigurationError, error):
                validate_platform(profile)

    def test_partial_module_layouts_are_rejected(self):
        for count in (0, 1, 2, 3):
            profile = copy.deepcopy(self.cfg.platform)
            profile['modules'] = profile['modules'][:count]
            with self.subTest(count=count), self.assertRaisesRegex(ConfigurationError, 'four'):
                validate_platform(profile)

    def test_four_module_layout_uses_all_eight_virtual_drives(self):
        expected = self.cfg.parameters('driver')['drives.names']
        actions, _ = resolve(factory(PACKAGES / 'mobotic_bringup/launch/moboterra.launch.py')(),
                             {'platform_config': str(self.cfg.path), 'stack_type': 'virtual'})
        simulated = [action for action in actions if action.kind == 'Node'
                     and action.kwargs['executable'] == 'virtual_mobotic_drive']
        self.assertEqual(len(simulated), 8)
        self.assertEqual(len(expected), 8)
        robot = ET.fromstring(render_description(self.cfg))
        self.assertEqual({joint.get('name') for joint in robot.findall('joint') if joint.get('type') != 'fixed'},
                         set(expected))
        standalone, _ = resolve(environment()['virtual_launch'](), {'platform_config': str(self.cfg.path)})
        node = next(action for action in standalone if action.kind == 'Node')
        self.assertEqual(node.kwargs['parameters'][0]['can_node_id'],
                         self.cfg.platform['modules'][0]['steering']['can_node_id'])

    def test_profile_accepts_pdo_requests(self):
        for traction, telemetry in ((True, False), (False, True), (True, True)):
            with self.subTest(traction=traction, telemetry=telemetry):
                cfg = Configuration()
                cfg.apply_overrides('driver', {'traction_pdo_enabled': traction,
                                               'telemetry_pdo_enabled': telemetry})
                cfg._validate_policy()
                self.assertEqual(cfg.parameters('driver')['traction_pdo_enabled'], traction)
                self.assertEqual(cfg.parameters('driver')['telemetry_pdo_enabled'], telemetry)

    def test_profile_rejects_invalid_skew(self):
        for overrides in ({'sample_skew_budget': 0}, {'sample_skew_budget': 0.36}):
            cfg = Configuration()
            cfg.apply_overrides('driver', overrides)
            with self.subTest(overrides=overrides), self.assertRaises(ConfigurationError):
                cfg._validate_policy()
        for mode in ('autonomous', 'kinematicoff'):
            cfg = Configuration(Path(__file__).resolve().parents[1] / 'config/platform-four-micontrol.yaml')
            cfg.apply_overrides('supervisor', {'initial_mode': mode})
            cfg._validate_policy()

    def test_physical_sdo_only_profile_preserves_feedback_guards(self):
        cfg = Configuration(Path(__file__).resolve().parents[1] / 'config/platform-four-micontrol.yaml')
        cfg.apply_overrides('driver', {'traction_pdo_enabled': False, 'telemetry_pdo_enabled': False})
        cfg._validate_policy()
        self.assertFalse(cfg.parameters('driver')['traction_pdo_enabled'])
        self.assertFalse(cfg.parameters('driver')['telemetry_pdo_enabled'])
        cfg.apply_overrides('driver', {'physical_sample_skew_budget': 0.36})
        with self.assertRaises(ConfigurationError):
            cfg._validate_policy()

    def test_invalid_physical_profiles_fail(self):
        mutations = [lambda p: p.update(schema_version=True),
                     lambda p: p['geometry'].update(wheel_radius=float('nan')),
                     lambda p: p['modules'][0]['traction'].update(can_node_id='0x01'),
                     lambda p: p['modules'].reverse(),
                     lambda p: p['can']['battery'].update(interface=p['can']['drives']['interface'], listen_only=True),
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

    def test_shared_can_profile_resolves_both_receivers_and_rejects_conflicting_link_settings(self):
        profile = copy.deepcopy(self.cfg.platform)
        profile['can']['battery'].update(interface=profile['can']['drives']['interface'],
                                         bitrate=profile['can']['drives']['bitrate'], listen_only=False)
        validate_platform(profile)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'shared-can.yaml'
            path.write_text(yaml.safe_dump(profile))
            _, context = resolve(factory(PACKAGES / 'mobotic_bringup/launch/moboterra.launch.py')(),
                                 {'platform_config': str(path)})
            self.assertEqual((context['can_interface'], context['battery_can_interface']), ('can0', 'can0'))
            standalone, _ = resolve(environment()['standalone_launch']('battery'),
                                    {'platform_config': str(path)})
            transport = next(a for a in standalone if a.kind == 'GroupAction')
            include = next(a for a in transport.kwargs['actions'] if a.kind == 'IncludeLaunchDescription')
            self.assertEqual(dict(include.kwargs['launch_arguments'])['interface'], 'can0')
        for change, message in (({'bitrate': 250000}, 'matching bitrates'),
                                ({'listen_only': True}, 'cannot be listen-only')):
            conflicting = copy.deepcopy(profile)
            conflicting['can']['battery'].update(change)
            with self.subTest(change=change), self.assertRaisesRegex(ConfigurationError, message):
                validate_platform(conflicting)

    def test_battery_ack_mode_requires_an_explicit_boolean(self):
        for value in (True, False):
            profile = copy.deepcopy(self.cfg.platform)
            profile['can']['battery']['interface'] = 'can1'
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
            self.cfg.apply_overrides('manual', {'max_velocity_linear': 3.0})
        self.cfg.apply_overrides('manual', {'max_velocity_linear': 0.75})
        self.assertEqual(self.cfg.parameters('manual')['max_velocity_linear'], 0.75)
        self.assertEqual(self.cfg.parameters('supervisor')['max_velocity_linear'], 2.0)
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
        self.assertEqual((context['can_interface'], context['battery_can_interface']), ('can0', 'can0'))
        env = environment()
        executable = {'driver': 'mobotic_driver',
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

    def test_integrated_kinematics_uses_original_romb_and_adapter(self):
        from mobotic_kinematics.launching import kinematics_nodes
        actions, _ = resolve(factory(PACKAGES / 'mobotic_bringup/launch/moboterra.launch.py')())
        expected = kinematics_nodes(lambda **kw: kw, 'moboterra', self.cfg.parameters('kinematics'))
        for node in expected:
            actual = next(a.kwargs for a in actions if a.kind == 'Node' and
                          a.kwargs['executable'] == node['executable'])
            self.assertEqual(actual['package'], 'mobotic_kinematics')
            self.assertEqual(actual['parameters'], node['parameters'])

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
        for mutation in (lambda p: p['operator']['mapping'].update(node_id=int(p['can']['master_node_id'], 0)),
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
            p['operator']['mapping']['buttons'].pop('scanner_override', None)
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
        for change in (dict(usb_driver='unknown'), dict(usb_mapping_verified='true'),
                       dict(require_enable_interlock='false'), dict(require_neutral_rearm=1),
                       dict(receiver_can_overrun_advisory='true'),
                       dict(receiver_errors_advisory='true'),
                       dict(receiver_auto_communication_reset='false')):
            p = copy.deepcopy(self.cfg.platform)
            p['operator'].update(change)
            with self.assertRaises(ConfigurationError):
                validate_platform(p)

    def test_pail_overrun_advisory_is_opt_in_and_reaches_the_operator(self):
        cfg = Configuration(self.verified_path)
        self.assertFalse(cfg.parameters('manual')['receiver_can_overrun_advisory'])
        self.assertFalse(cfg.parameters('manual')['receiver_auto_communication_reset'])
        p = copy.deepcopy(cfg.platform)
        p['operator']['receiver_can_overrun_advisory'] = True
        p['operator']['receiver_auto_communication_reset'] = True
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'pail.json'
            path.write_text(json.dumps(p))
            self.assertTrue(Configuration(path).parameters('manual')['receiver_can_overrun_advisory'])
            self.assertTrue(Configuration(path).parameters('manual')['receiver_auto_communication_reset'])

    def test_pail_pdo_based_diagnostic_policy_reaches_operator_and_launch(self):
        cfg = Configuration(self.verified_path)
        self.assertFalse(cfg.parameters('manual')['receiver_errors_advisory'])
        platform = copy.deepcopy(cfg.platform)
        platform['operator']['receiver_errors_advisory'] = True
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'pdo-policy.json'
            path.write_text(json.dumps(platform))
            self.assertTrue(Configuration(path).parameters('manual')['receiver_errors_advisory'])
            actions, _ = resolve(factory(PACKAGES / 'mobotic_bringup/launch/moboterra.launch.py')(),
                                 {'platform_config': str(path)})
            operator = next(action for action in actions if action.kind == 'Node'
                            and action.kwargs.get('executable') == 'remdevice_control')
            self.assertTrue(operator.kwargs['parameters'][0]['receiver_errors_advisory'])

    def test_namespace_root_and_endpoint_overrides(self):
        desc = factory(PACKAGES / 'mobotic_bringup/launch/moboterra.launch.py')()
        actions, context = resolve(desc, {'robot_name': '', 'can_interface': 'vcan0',
                                          'battery_can_interface': 'vcan1', 'front_scanner_ip': '10.60.20.54'})
        self.assertEqual(context['robot_name'], '')
        scanner = next(a for a in actions if a.kind == 'Node' and a.kwargs['name'] == 'front_left_scanner')
        self.assertEqual(scanner.kwargs['parameters'][0]['sensor_ip'], '10.60.20.54')
        with self.assertRaises(ConfigurationError):
            resolve(desc, {'battery_can_interface': 'invalid/interface'})

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
        expected = {'platform.yaml', 'platform-four-micontrol.yaml'}
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
