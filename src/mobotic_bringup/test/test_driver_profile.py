"""ROS-free cross-checks of the four-wheel controller profile and units."""

import math
from pathlib import Path
import unittest
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'mobotic_config'))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'mobotic_config/test'))
from mobotic_config.configuration import Configuration
from launch_harness import factory, resolve


PACKAGES = Path(__file__).resolve().parents[2]
DRIVER = PACKAGES / 'mobotic_driver'


class FourWheelDriverProfileTest(unittest.TestCase):
    def test_heartbeat_matches_four_wheel_reference_and_legacy_battery_watchdog(self):
        heartbeat = Configuration().parameters('driver')['can_heartbeat_period']
        self.assertEqual(heartbeat, 0.1)
        self.assertLess(heartbeat, 0.5)

    def test_all_four_traction_drives_use_motor_rpm_limits(self):
        parameters = Configuration().parameters('driver')
        lower = parameters['drives.min_traction_velocities']
        upper = parameters['drives.max_traction_velocities']
        self.assertEqual(len(lower), 8)
        self.assertEqual(len(upper), 8)
        for index in (1, 3, 5, 7):
            self.assertEqual((lower[index], upper[index]), (-2469, 2469))

    def test_installed_micontrol_ramps_are_not_driver_configuration(self):
        parameters = Configuration().parameters('driver')
        for key in ('traction_max_profile_velocity', 'traction_profile_accel', 'traction_profile_decel'):
            self.assertNotIn(key, parameters)

    def test_kinematics_and_direct_contact_speed_fit_native_rpm_limits(self):
        cfg = Configuration()
        driver = cfg.parameters('driver')
        radius = cfg.platform['geometry']['wheel_radius']
        for speed in (cfg.parameters('kinematics')['max_traction_velocity'],
                      cfg.parameters('supervisor')['max_direct_traction_velocity']):
            for index in (1, 3, 5, 7):
                rpm = speed / radius * driver['drives.gear_ratios'][index] * 60 / (2 * math.pi)
                self.assertLessEqual(rpm, driver['drives.max_traction_velocities'][index])

    def test_hardware_launch_applies_integrated_motion_limits(self):
        launch = PACKAGES / 'mobotic_bringup/launch/moboterra.launch.py'
        actions, _ = resolve(factory(launch)(), {'stack_type': 'hardware'})
        driver = next(a for a in actions if a.kind == 'Node'
                      and a.kwargs.get('name') == 'mobotic_driver')
        parameters = driver.kwargs['parameters'][0]
        self.assertEqual(parameters['max_traction_velocity'], 2.0)
        self.assertEqual(parameters['max_steering_velocity'], 1.2)
        self.assertEqual(parameters['wheel_radius'], 0.325)
        self.assertEqual(parameters['max_traction_current'], 4.0)

    def test_standalone_launch_forwards_all_eight_drive_parameters(self):
        actions, _ = resolve(factory(DRIVER / 'launch/mobotic_driver.launch.py')())
        node = next(a for a in actions if a.kind == 'Node')
        parameters = node.kwargs['parameters'][0]
        self.assertEqual(parameters, Configuration().parameters('driver'))
        self.assertEqual(len(parameters['drives.names']), 8)
        self.assertEqual(len(parameters['modules.names']), 4)


if __name__ == '__main__':
    unittest.main()
