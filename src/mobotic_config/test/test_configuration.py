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
        for component, values in baseline.items():
            self.assertEqual(self.cfg.parameters(component), values, component)

    def test_virtual_driver_scaling_comes_from_same_records(self):
        p = self.cfg.parameters('driver')
        for index, (joint, drive) in enumerate(self.cfg.virtual_drives()):
            self.assertEqual(joint, p['drives.names'][index])
            self.assertEqual(drive['can_node_id'], p['drives.can_node_ids'][index])
            self.assertEqual(drive['encoder_resolution'], p['drives.resolutions'][index])
            self.assertEqual(drive['gear_ratio'], p['drives.gear_ratios'][index])

    def test_invalid_physical_profiles_fail(self):
        mutations = [lambda p: p.update(schema_version=True),
                     lambda p: p['geometry'].update(wheel_radius=float('nan')),
                     lambda p: p['modules'][0]['traction'].update(can_node_id='0x01'),
                     lambda p: p['modules'].reverse(),
                     lambda p: p['can']['battery'].update(interface='can0'),
                     lambda p: p['can']['battery'].update(listen_only=False),
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
                      'manual': 'manual_control', 'supervisor': 'mobotic_supervisor',
                      'battery': 'vanguard_battery_monitor', 'odometry': 'mobotic_odometry'}
        for component, name in executable.items():
            standalone, _ = resolve(env['standalone_launch'](component))
            node = next(a for a in standalone if a.kind == 'Node' and a.kwargs['executable'] == name)
            integrated = next(a for a in actions if a.kind == 'Node' and a.kwargs['executable'] == name)
            self.assertEqual(node.kwargs['parameters'][0], integrated.kwargs['parameters'][0])

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
        self.assertEqual(len(files), 10)
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
