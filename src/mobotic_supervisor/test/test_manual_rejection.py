"""PAIL input rejection must identify interlocks without hiding input faults."""
import math
from types import SimpleNamespace
import unittest

import test_command_recovery as recovery
import test_enable_gate as harness


class ManualRejectionTest(unittest.TestCase):
    def setUp(self):
        self.fixture = recovery.CommandRecoveryTest()
        self.fixture.setUp()
        self.node = self.fixture.node
        self.node.selected_mode = harness.MODES.MANUAL
        self.warnings = []
        self.node.get_logger = lambda: SimpleNamespace(warning=self.warnings.append)

    def test_fresh_pail_input_reports_disabled_drives_without_buffering_motion(self):
        self.node.operator_drives_disabled = True
        for _ in range(50):
            self.node._manual_command_callback(self.fixture.velocity())
            self.fixture.advance(0.02)
        self.assertIsNone(self.node.manual_command)
        self.assertEqual(len(self.warnings), 1)
        self.assertIn('drives disabled by B2', self.warnings[-1])
        self.assertNotIn('stale', self.warnings[-1])

        self.node.operator_drives_disabled = False
        self.node.wheel_status.modules[0].startup_complete = False
        self.node._manual_command_callback(self.fixture.velocity())
        self.assertEqual(len(self.warnings), 2)  # New blocker is reported immediately.
        self.assertIn('startup not complete', self.warnings[-1])
        self.assertIsNone(self.node.manual_command)

    def test_actual_timestamp_and_value_errors_are_distinct_from_hardware_inhibition(self):
        for error, expected in (('stale', 'timestamp'), ('future', 'future'),
                                ('nonfinite', 'non-finite')):
            with self.subTest(error=error):
                self.setUp()
                command = self.fixture.velocity()
                if error == 'stale':
                    command.header.stamp = harness.Time(9.7).to_msg()
                elif error == 'future':
                    command.header.stamp = harness.Time(10.05).to_msg()
                else:
                    command.twist.linear.x = math.nan
                self.node._manual_command_callback(command)
                self.assertIsNone(self.node.manual_command)
                self.assertIn(expected, self.warnings[-1])

    def test_recovery_requires_new_acquisition_stamp_then_accepts_fresh_pail_input(self):
        self.node.operator_drives_disabled = True
        old = self.fixture.velocity()
        self.node._manual_command_callback(old)
        self.fixture.advance(0.05)
        self.node.operator_drives_disabled = False
        self.node._manual_command_callback(old)
        self.assertIsNone(self.node.manual_command)
        self.assertIn('predates readiness/mode change', self.warnings[-1])
        fresh = self.fixture.velocity()
        self.node._manual_command_callback(fresh)
        self.assertIs(self.node.manual_command, fresh)
        self.fixture.advance(0.02)
        self.node._control()
        self.assertGreater(self.node.cmd_vel_publisher.messages[-1].twist.linear.x, 0)

    def test_persistent_fault_logs_are_bounded_and_include_latched_fault_codes(self):
        self.node.fault_codes.add('DRIVE_STARTUP_FAILED:front')
        self.node._manual_command_callback(self.fixture.velocity())
        self.assertIn('DRIVE_STARTUP_FAILED:front', self.warnings[-1])
        self.fixture.advance(2.0)
        self.node._manual_command_callback(self.fixture.velocity())
        self.assertEqual(len(self.warnings), 2)
        self.assertIsNone(self.node.manual_command)
