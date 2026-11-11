"""ROS-free cross-checks of the four-wheel controller profile and units."""

import ast
import math
from pathlib import Path
import re
import unittest
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'mobotic_config'))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'mobotic_config/test'))
from mobotic_config.configuration import Configuration
from launch_harness import factory, resolve


PACKAGES = Path(__file__).resolve().parents[2]
DRIVER = PACKAGES / 'mobotic_driver'


def config_value(package, filename, key):
    component = {'mobotic_bringup': 'driver', 'mobotic_kinematics': 'kinematics',
                 'mobotic_supervisor': 'supervisor'}[package]
    return repr(Configuration().parameters(component)[key])


def driver_value(key):
    return ast.literal_eval(config_value('mobotic_bringup', 'moboterra_driver.yaml', key))


class FourWheelDriverProfileTest(unittest.TestCase):
    def test_heartbeat_matches_four_wheel_reference_and_legacy_battery_watchdog(self):
        heartbeat = driver_value('can_heartbeat_period')
        self.assertEqual(heartbeat, 0.01)
        self.assertLess(heartbeat, 0.5)

    def test_all_four_traction_drives_use_motor_rpm_limits(self):
        lower = driver_value('drives.min_traction_velocities')
        upper = driver_value('drives.max_traction_velocities')
        self.assertEqual(len(lower), 8)
        self.assertEqual(len(upper), 8)
        for index in (1, 3, 5, 7):
            self.assertEqual((lower[index], upper[index]), (-1700, 1700))

    def test_installed_micontrol_ramps_are_not_driver_configuration(self):
        parameters = Configuration().parameters('driver')
        for key in ('traction_max_profile_velocity', 'traction_profile_accel', 'traction_profile_decel'):
            self.assertNotIn(key, parameters)

    def test_kinematics_and_direct_contact_speed_fit_native_rpm_limits(self):
        cfg = Configuration()
        radius = cfg.platform['geometry']['wheel_radius']
        for speed in (cfg.parameters('kinematics')['max_traction_velocity'],
                      cfg.parameters('supervisor')['max_direct_traction_velocity']):
            for index in (1, 3, 5, 7):
                rpm = speed / radius * driver_value('drives.gear_ratios')[index] * 60 / (2 * math.pi)
                self.assertLessEqual(rpm, driver_value('drives.max_traction_velocities')[index])

    def test_driver_fallback_motion_defaults_match_integrated_profile(self):
        source = (DRIVER / 'src/mobotic_driver.cpp').read_text()
        for parameter in ('max_traction_velocity', 'max_steering_velocity', 'wheel_radius', 'max_traction_current'):
            match = re.search(r'declare_parameter\("' + parameter + r'", ([0-9.]+)\)', source)
            self.assertIsNotNone(match)
            self.assertEqual(float(match.group(1)), driver_value(parameter))

    def test_standalone_launch_forwards_all_eight_drive_parameters(self):
        actions, _ = resolve(factory(DRIVER / 'launch/mobotic_driver.launch.py')())
        node = next(a for a in actions if a.kind == 'Node')
        parameters = node.kwargs['parameters'][0]
        self.assertEqual(parameters, Configuration().parameters('driver'))
        self.assertEqual(len(parameters['drives.names']), 8)
        self.assertEqual(len(parameters['modules.names']), 4)


if __name__ == '__main__':
    unittest.main()
