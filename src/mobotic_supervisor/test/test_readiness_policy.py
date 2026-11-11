"""Four-module boundaries between motion stopping, enable and B2 recovery."""
import unittest
from types import SimpleNamespace

import test_enable_gate as harness
import test_four_modules as four_modules


class ReadinessPolicyTest(unittest.TestCase):
    def setUp(self):
        self.fixture = four_modules.FourModuleSupervisorTest()
        self.fixture.setUp()
        self.node = self.fixture.node
        self.now = self.fixture.now
        self.node.started_at = harness.Time(1)
        self.node.startup_grace = 0
        self.node.selected_mode = harness.MODES.MANUAL
        self.node.manual_state = SimpleNamespace(
            header=SimpleNamespace(stamp=self.now.to_msg()), connected=True,
            command_active=False, controls_neutral=True, estop_state_known=True,
            estop_pressed=False, status_message='neutral operator; no motion command')
        self.node.manual_state_received = self.now

    def test_absent_motion_command_stops_without_withdrawing_enable(self):
        self.node.manual_command = None
        self.node._control()
        self.assertTrue(self.node.module_command_publisher.messages[-1].enable)
        self.assertFalse(getattr(self.node, 'operator_drives_disabled', False))
        output = self.node.cmd_vel_publisher.messages[-1].twist
        self.assertEqual((output.linear.x, output.linear.y, output.angular.z), (0, 0, 0))

    def test_stale_feedback_on_each_module_requires_b2_even_after_feedback_recovers(self):
        for index in range(4):
            with self.subTest(module=index):
                self.setUp()
                self.assertTrue(self.node._refresh_motion_readiness(self.now)[0])
                self.node.wheel_status.modules[index].feedback_fresh = False
                self.node._control()
                self.assertFalse(self.node.module_command_publisher.messages[-1].enable)
                self.assertTrue(self.node.operator_drives_disabled)
                self.node.wheel_status.modules[index].feedback_fresh = True
                self.node._control()
                self.assertFalse(self.node.module_command_publisher.messages[-1].enable)

    def test_recent_status_publication_does_not_hide_stale_acquisition_or_wrong_modules(self):
        for invalid in ('old_stamp', 'old_receipt', 'missing_module', 'duplicate_module'):
            with self.subTest(invalid=invalid):
                self.setUp()
                if invalid == 'old_stamp':
                    self.node.wheel_status.header.stamp = harness.Time(9).to_msg()
                elif invalid == 'old_receipt':
                    self.node.wheel_status_received = harness.Time(9)
                elif invalid == 'missing_module':
                    self.node.wheel_status.modules.pop()
                else:
                    self.node.wheel_status.modules[1].name = self.node.wheel_status.modules[0].name
                self.assertFalse(self.node._enable_gate(self.now)[0])
                self.assertFalse(self.node._base_gate(self.now)[0])
                self.assertFalse(self.node._motion_hardware_gate(self.now)[0])

    def test_callback_evaluation_does_not_reuse_previous_readiness(self):
        self.assertTrue(self.node._motion_hardware_gate(self.now)[0])
        self.node.wheel_status.modules[0].feedback_fresh = False
        self.assertFalse(self.node._source_gate(self.now)[0])
        self.node.wheel_status.modules[0].feedback_fresh = True
        self.assertTrue(self.node._motion_hardware_gate(self.now)[0])

    def test_fault_codes_are_unique_without_hiding_distinct_losses(self):
        node = self.node
        node.require_flexisoft_telemetry = True
        node.drive_startup_requested = True
        node.safety_received = harness.Time(9)
        node.safety_state.communication_ok = False
        node.safety_state.header.stamp = harness.Time(9).to_msg()
        node.wheel_status.modules[0].feedback_fresh = False
        errors, _ = node._system_conditions(self.now)
        self.assertEqual(errors.count('SAFETY_MONITOR_LOST'), 1)
        self.assertIn('DRIVE_FEEDBACK_LOST:front_left', errors)
