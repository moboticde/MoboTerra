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

    def test_traction_limits_and_controller_profile_share_reference_ceiling(self):
        ceiling = driver_value('traction_max_profile_velocity')
        self.assertEqual(ceiling, 116053)
        lower = driver_value('drives.min_traction_velocities')
        upper = driver_value('drives.max_traction_velocities')
        self.assertEqual(len(lower), 8)
        self.assertEqual(len(upper), 8)
        for index in (1, 3, 5, 7):
            self.assertEqual(lower[index], -ceiling)
            self.assertEqual(upper[index], ceiling)

    def test_acceleration_and_deceleration_use_actual_wheel_radius_and_scaling(self):
        radius = float(config_value('mobotic_kinematics', 'moboterra_kinematics.yaml', 'wheel_radius'))
        self.assertEqual(radius, 0.325)
        resolutions = driver_value('drives.resolutions')
        gearing = driver_value('drives.gear_ratios')
        for index in (1, 3, 5, 7):
            increments_per_metre = resolutions[index] * gearing[index] / (2.0 * math.pi * radius)
            for key in ('traction_profile_accel', 'traction_profile_decel'):
                self.assertEqual(driver_value(key), round(increments_per_metre))
                self.assertAlmostEqual(driver_value(key) / increments_per_metre, 1.0, places=4)

    def test_kinematics_and_direct_joint_limits_fit_controller_ceiling(self):
        ceiling = driver_value('traction_max_profile_velocity')
        for package, filename, key in (
            ('mobotic_kinematics', 'moboterra_kinematics.yaml', 'max_traction_velocity'),
            ('mobotic_supervisor', 'supervisor.yaml', 'max_direct_traction_velocity'),
        ):
            wheel_radians_per_second = float(config_value(package, filename, key))
            for index in (1, 3, 5, 7):
                ticks_per_second = wheel_radians_per_second * driver_value('drives.resolutions')[index] * driver_value('drives.gear_ratios')[index] / (2.0 * math.pi)
                self.assertLessEqual(ticks_per_second, ceiling)

    def test_driver_fallback_defaults_match_integrated_profile(self):
        source = (DRIVER / 'src' / 'mobotic_driver.cpp').read_text(encoding='utf-8')
        for constant, parameter in (
            ('DEFAULT_TRACTION_PROFILE_ACCEL', 'traction_profile_accel'),
            ('DEFAULT_TRACTION_MAX_PROFILE_VELOCITY', 'traction_max_profile_velocity'),
        ):
            match = re.search(r'constexpr int ' + constant + r' = (\d+);', source)
            self.assertIsNotNone(match)
            self.assertEqual(int(match.group(1)), driver_value(parameter))
        self.assertTrue('DEFAULT_TRACTION_PROFILE_DECEL = DEFAULT_TRACTION_PROFILE_ACCEL' in source)
        self.assertEqual(driver_value('traction_profile_decel'), driver_value('traction_profile_accel'))

    def test_standalone_launch_defaults_and_parameter_forwarding_match(self):
        actions, _ = resolve(factory(DRIVER / 'launch/mobotic_driver.launch.py')())
        node = next(a for a in actions if a.kind == 'Node')
        self.assertEqual(node.kwargs['parameters'][0], Configuration().parameters('driver'))
        for key in ('can_heartbeat_period', 'traction_max_profile_velocity', 'traction_profile_accel', 'traction_profile_decel'):
            self.assertEqual(node.kwargs['parameters'][0][key], driver_value(key))


if __name__ == '__main__':
    unittest.main()
