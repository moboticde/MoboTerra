"""Two-module hardware STO needs no PLC data; other layouts keep FlexiSoft."""
import copy
from types import SimpleNamespace
import unittest

import test_enable_gate as harness


class SafetyVariantsTest(unittest.TestCase):
    def setUp(self):
        harness.EnableGateTest.setUp(self)
        self.node.safety_source = 'hardwired_sto'
        self.node.started_at = harness.Time(1.0)
        self.node.startup_grace = 0.0
        self.node.selected_mode = harness.MODES.AUTO_VELOCITY

    def no_plc(self):
        self.node.safety_state = self.node.safety_io_state = None
        self.node.safety_received = self.node.safety_io_received = None

    def test_missing_or_unhealthy_plc_messages_do_not_change_two_module_gates(self):
        node = self.node
        node.safety_state.communication_ok = False
        node.safety_state.emergency_stop_active = node.safety_state.sto_active = True
        node.safety_state.warning_field_active = node.safety_state.safety_override_active = True
        node.safety_io_state.safety_enable_active = False
        node.safety_io_state.front_left_ossd_active = False
        for absent in (False, True):
            with self.subTest(absent=absent):
                if absent:
                    self.no_plc()
                self.assertTrue(node._enable_gate(self.now)[0])
                self.assertEqual(node._system_conditions(self.now), ([], False))
                self.assertEqual(node._speed_scale(), 1.0)
                self.assertFalse(node._io_warning_active())

    def test_four_modules_always_require_plc_even_if_source_incorrectly_set(self):
        self.node.expected_module_names = ['front_left', 'front_right', 'rear_left', 'rear_right']
        for source in ('flexisoft', 'hardwired_sto'):
            self.node.safety_source = source
            self.no_plc()
            self.assertFalse(self.node._enable_gate(self.now)[0])
            errors, _ = self.node._system_conditions(self.now)
            self.assertIn('SAFETY_MONITOR_LOST', errors)
            self.assertIn('SAFETY_IO_LOST', errors)

    def test_two_module_flexisoft_profile_still_requires_telemetry(self):
        self.node.safety_source = 'flexisoft'
        self.no_plc()
        self.assertFalse(self.node._enable_gate(self.now)[0])
        self.assertIn('SAFETY_MONITOR_LOST', self.node._system_conditions(self.now)[0])

    def test_known_active_drive_sto_inhibits_two_module_enable(self):
        self.no_plc()
        module = self.node.wheel_status.modules[0]
        module.sto_state_known = module.sto_active = True
        self.assertFalse(self.node._enable_gate(self.now)[0])
        self.assertTrue(self.node._system_conditions(self.now)[1])

    def test_drive_battery_and_operator_checks_remain_without_plc(self):
        self.no_plc()
        node = self.node
        for field, value in (('feedback_fresh', False), ('initialized', False),
                             ('position_valid', False), ('error_code', 1)):
            original = getattr(node.wheel_status.modules[0], field)
            setattr(node.wheel_status.modules[0], field, value)
            self.assertEqual(node._enable_gate(self.now)[0], field != 'error_code', field)
            self.assertFalse(node._base_gate(self.now)[0], field)
            setattr(node.wheel_status.modules[0], field, original)
        original = copy.deepcopy(node.wheel_status.modules)
        node.wheel_status.modules.pop()
        self.assertFalse(node._enable_gate(self.now)[0])
        node.wheel_status.modules = original
        node.wheel_status_received = harness.Time(9.0)
        self.assertFalse(node._enable_gate(self.now)[0])
        node.wheel_status_received = self.now
        node.battery_state.high_voltage_connected = False
        self.assertFalse(node._enable_gate(self.now)[0])
        node.battery_state.high_voltage_connected = True
        node.require_operator_interlock = True
        self.assertFalse(node._enable_gate(self.now)[0])
        node.manual_state = SimpleNamespace(header=SimpleNamespace(stamp=self.now.to_msg()),
            connected=True, estop_state_known=True, estop_pressed=True)
        node.manual_state_received = self.now
        self.assertFalse(node._enable_gate(self.now)[0])
        node.manual_state.estop_pressed = False
        self.assertTrue(node._enable_gate(self.now)[0])
        node.fault_codes.add('DRIVE_FAULT')
        self.assertFalse(node._enable_gate(self.now)[0])

    def test_direct_mode_needs_real_joint_feedback_but_no_plc(self):
        self.no_plc()
        self.node.selected_mode = harness.MODES.AUTO_DIRECT
        for module in self.node.wheel_status.modules:
            module.enabled = True
        ready, reason = self.node._motion_hardware_gate(self.now)
        self.assertFalse(ready)
        self.assertIn('joint-state', reason)
        self.node.joint_state = SimpleNamespace(header=SimpleNamespace(stamp=self.now.to_msg()))
        self.node.joint_state_received = self.now
        self.assertTrue(self.node._motion_hardware_gate(self.now)[0])


if __name__ == '__main__':
    unittest.main()
