"""Exercise the monitoring and interlock roles of FlexiSoft telemetry."""
import copy
from types import SimpleNamespace
import unittest

import test_enable_gate as harness


class SafetyVariantsTest(unittest.TestCase):
    def setUp(self):
        harness.EnableGateTest.setUp(self)
        self.node.safety_source = 'flexisoft'
        self.node.require_flexisoft_telemetry = False
        self.node.expected_module_names = ['front_left', 'front_right', 'rear_left', 'rear_right']
        template = self.node.wheel_status.modules[0]
        self.node.wheel_status.modules = [copy.deepcopy(template) for _ in self.node.expected_module_names]
        for module, name in zip(self.node.wheel_status.modules, self.node.expected_module_names):
            module.name = name
        self.node.started_at = harness.Time(1.0)
        self.node.startup_grace = 0.0
        self.node.selected_mode = harness.MODES.AUTONOMOUS

    def no_plc(self):
        self.node.safety_state = self.node.safety_io_state = None
        self.node.safety_received = self.node.safety_io_received = None

    def test_monitoring_policy_does_not_gate_on_plc_messages(self):
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


    def test_interlock_policy_requires_telemetry(self):
        self.node.require_flexisoft_telemetry = True
        self.no_plc()
        self.assertFalse(self.node._enable_gate(self.now)[0])
        self.assertIn('SAFETY_MONITOR_LOST', self.node._system_conditions(self.now)[0])

    def test_known_active_drive_sto_inhibits_four_module_enable(self):
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
        self.node.selected_mode = harness.MODES.KINEMATICOFF
        for module in self.node.wheel_status.modules:
            module.enabled = True
        ready, reason = self.node._motion_hardware_gate(self.now)
        self.assertFalse(ready)
        self.assertIn('joint-state', reason)
        self.node.joint_state = SimpleNamespace(header=SimpleNamespace(stamp=self.now.to_msg()))
        self.node.joint_state_received = self.now
        self.assertTrue(self.node._motion_hardware_gate(self.now)[0])


class MonitorOnlyFlexisoftTest(unittest.TestCase):
    def setUp(self):
        harness.EnableGateTest.setUp(self)
        node = self.node
        node.require_flexisoft_telemetry = False
        node.expected_module_names = ['front_left', 'front_right', 'rear_left', 'rear_right']
        template = node.wheel_status.modules[0]
        node.wheel_status.modules = [copy.deepcopy(template) for _ in node.expected_module_names]
        for module, name in zip(node.wheel_status.modules, node.expected_module_names):
            module.name, module.enabled = name, True
        node.started_at = harness.Time(1.0)
        node.startup_grace = 0.0
        node.selected_mode = harness.MODES.AUTONOMOUS

    def test_absent_stale_or_stop_telemetry_does_not_control_four_modules(self):
        node = self.node
        node.safety_state.communication_ok = False
        node.safety_state.emergency_stop_active = node.safety_state.sto_active = True
        node.safety_state.warning_field_active = node.safety_state.safety_override_active = True
        node.safety_io_state.safety_enable_active = False
        node.safety_io_state.front_left_ossd_active = False
        node.safety_io_state.front_left_warning_field_clear = False
        for state in ('reported_stop', 'stale', 'absent'):
            with self.subTest(state=state):
                if state == 'stale':
                    node.safety_received = node.safety_io_received = harness.Time(1.0)
                elif state == 'absent':
                    node.safety_state = node.safety_io_state = None
                    node.safety_received = node.safety_io_received = None
                self.assertTrue(node._enable_gate(self.now)[0])
                self.assertTrue(node._base_gate(self.now)[0])
                self.assertEqual(node._system_conditions(self.now), ([], False))
                self.assertEqual(node._speed_scale(), 1.0)
                self.assertFalse(node._io_warning_active())

    def test_explicit_interlock_role_still_requires_flexisoft(self):
        self.node.require_flexisoft_telemetry = True
        self.node.safety_state = self.node.safety_io_state = None
        self.assertFalse(self.node._enable_gate(self.now)[0])
        errors, _ = self.node._system_conditions(self.now)
        self.assertIn('SAFETY_MONITOR_LOST', errors)
        self.assertIn('SAFETY_IO_LOST', errors)

    def test_scanner_red_unknown_stale_and_yellow_still_apply(self):
        node = self.node
        original = copy.deepcopy(node.scanner_states['front_left'])
        field = node.scanner_states['front_left'][0]
        field.warning_field_clear = False
        self.assertTrue(node._enable_gate(self.now)[0])
        self.assertTrue(node._warning_active())
        self.assertEqual(node._reduced_speed_limit(), .3)
        field.protective_field_clear = False
        self.assertFalse(node._enable_gate(self.now)[0])
        self.assertTrue(node._system_conditions(self.now)[1])
        for replacement in ((None, None), (original[0], harness.Time(1.0))):
            node.scanner_states['front_left'] = replacement
            self.assertFalse(node._enable_gate(self.now)[0])
            self.assertIn('SCANNER_MONITOR_LOST:front_left', node._system_conditions(self.now)[0])

    def test_drive_sto_battery_operator_and_feedback_still_inhibit(self):
        node = self.node
        module = node.wheel_status.modules[0]
        module.sto_state_known = module.sto_active = True
        self.assertFalse(node._enable_gate(self.now)[0])
        self.assertTrue(node._system_conditions(self.now)[1])
        module.sto_active = False
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
        module.feedback_fresh = False
        self.assertFalse(node._base_gate(self.now)[0])
        self.assertIn('DRIVE_FEEDBACK_LOST:front_left', node._system_conditions(self.now)[0])
        module.feedback_fresh = True
        node.fault_codes.add('DRIVE_FAULT:front_left')
        self.assertFalse(node._enable_gate(self.now)[0])


if __name__ == '__main__':
    unittest.main()
