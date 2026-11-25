"""Commissioned RP2 selector cannot bypass operator/drive interlocks or speed caps."""
import math
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import test_enable_gate as h
import test_interface_update as interfaces


class OperatorScannerOverrideTest(unittest.TestCase):
    def setUp(self):
        fixture = interfaces.InterfaceUpdateTest()
        fixture.setUp()
        self.f = fixture.fixture
        self.n = fixture.node
        self.n.selected_mode = h.MODES.MANUAL
        self.n.operator_scanner_override_enabled = True
        self.n.get_clock = lambda: SimpleNamespace(now=lambda: self.f.now)
        self.n.output_frame_id = 'base_link'
        self.n.require_operator_interlock = True
        self.n.manual_state = SimpleNamespace(header=SimpleNamespace(stamp=self.f.now.to_msg()),
            source_id='remdevice', connected=True, estop_state_known=True, estop_pressed=False,
            controls_neutral=True, command_active=True, status_message='neutral')
        self.n.manual_state_received = self.f.now
        self.select(True)

    def select(self, active):
        self.n.operator_selector = SimpleNamespace(header=SimpleNamespace(stamp=self.f.now.to_msg(), frame_id='base_link'),
            buttons=[0]*10+[int(active)])
        self.n.operator_selector_received = self.f.now

    def test_disabled_ignores_red_and_missing_scanner_fields(self):
        for name in self.n.scanner_states:
            self.n.scanner_states[name] = (None, None)
        self.assertTrue(self.n._scanner_gate(self.f.now)[0])
        errors, stop = self.n._system_conditions(self.f.now)
        self.assertFalse(any(x.startswith('SCANNER_') for x in errors))
        self.assertFalse(stop)

    def test_enabled_requires_both_scanners(self):
        self.select(False)
        self.assertTrue(self.n._scanner_gate(self.f.now)[0])
        self.n.scanner_states['front_left'] = (None, None)
        self.assertFalse(self.n._scanner_gate(self.f.now)[0])

    def test_unavailable_selector_requires_scanners_instead_of_exempting_them(self):
        for field, value in (('connected', False), ('estop_pressed', True), ('estop_state_known', False)):
            original = getattr(self.n.manual_state, field)
            setattr(self.n.manual_state, field, value)
            self.assertTrue(self.n._scanner_gate(self.f.now)[0])
            self.n.scanner_states['front_left'] = (None, None)
            self.assertFalse(self.n._scanner_gate(self.f.now)[0])
            self.n.scanner_states['front_left'] = (SimpleNamespace(
                header=SimpleNamespace(stamp=self.f.now.to_msg()), scanner_id='front_left',
                communication_ok=True, protective_state_known=True, warning_state_known=True,
                protective_field_clear=True, warning_field_clear=True), self.f.now)
            setattr(self.n.manual_state, field, original)
        self.n.operator_selector.header.stamp = h.Time(9).to_msg()
        self.assertTrue(self.n._scanner_gate(self.f.now)[0])

    def test_autonomy_cannot_use_scanner_exemption(self):
        for mode in (h.MODES.AUTONOMOUS, h.MODES.KINEMATICOFF):
            self.n.selected_mode = mode
            self.assertFalse(self.n._scanner_gate(self.f.now)[0])

    def test_battery_and_drive_sto_checks_remain_required(self):
        self.n.battery_state.system_ready = False
        self.assertFalse(self.n._enable_gate(self.f.now)[0])
        self.n.battery_state.system_ready = True
        self.n.wheel_status.modules[0].sto_state_known = True
        self.n.wheel_status.modules[0].sto_active = True
        self.assertFalse(self.n._enable_gate(self.f.now)[0])

    def test_diagonal_translation_cap_even_if_warning_policy_is_higher(self):
        self.n.manual_state.controls_neutral = False
        self.n.warning_speed_limit = 1.0
        self.n.max_acceleration_linear = 100
        source = h.TwistStamped()
        source.header.stamp = self.f.now.to_msg()
        source.twist.linear.x = source.twist.linear.y = 1
        self.n.last_control_time = h.Time(9.9)
        self.n._publish_velocity(source, self.f.now)
        output = self.n.cmd_vel_publisher.messages[-1].twist.linear
        self.assertAlmostEqual(math.hypot(output.x, output.y), .3)

    def test_switch_edge_discards_old_commands_and_zeroes_output(self):
        self.n.manual_command = h.TwistStamped()
        self.n.last_output = [.8, .2, .1]
        self.f.now = h.Time(10.001)
        msg = SimpleNamespace(header=SimpleNamespace(stamp=self.f.now.to_msg(), frame_id='base_link'), buttons=[0]*11)
        self.n._operator_selector_callback(msg)
        self.assertIsNone(self.n.manual_command)
        self.assertEqual(self.n.last_output, [0, 0, 0])

    def test_final_wheel_speed_cap_preserves_reverse_and_turn_ratio(self):
        self.n.warning_speed_limit = 1.0
        msg = h.JointState()
        msg.name = self.n.expected_steering_joint_names + self.n.expected_traction_joint_names
        msg.position = [0]*4
        msg.velocity = [1, 1, 3, -1.5]
        self.n._emit_final(msg, self.f.now.to_msg(), self.f.now, h.WheelModuleCommandArray.MOTION)
        final = self.n.final_command_publisher.messages[-1]
        self.assertAlmostEqual(final.modules[0].traction_velocity, .3)
        self.assertAlmostEqual(final.modules[1].traction_velocity, -.15)

    def test_malformed_selector_does_not_exempt_scanners(self):
        for buttons in ([0]*10, [0]*10+[2], []):
            self.n.operator_selector.buttons = buttons
            self.assertTrue(self.n._scanner_gate(self.f.now)[0])
            self.n.scanner_states['front_left'] = (None, None)
            self.assertFalse(self.n._scanner_gate(self.f.now)[0])
            self.n.scanner_states['front_left'] = (SimpleNamespace(
                header=SimpleNamespace(stamp=self.f.now.to_msg()), scanner_id='front_left',
                communication_ok=True, protective_state_known=True, warning_state_known=True,
                protective_field_clear=True, warning_field_clear=True), self.f.now)


class FourModuleOperatorScannerOverrideTest(unittest.TestCase):
    def setUp(self):
        import copy
        fixture = OperatorScannerOverrideTest()
        fixture.setUp()
        self.n, self.f = fixture.n, fixture.f
        self.select = fixture.select
        n = self.n
        names = ['front_left', 'front_right', 'rear_left', 'rear_right']
        n.expected_module_names = names
        n.expected_steering_joint_names = [name + '_steering' for name in names]
        n.expected_traction_joint_names = [name + '_traction' for name in names]
        template = n.wheel_status.modules[0]
        n.wheel_status.modules = [copy.deepcopy(template) for name in names]
        for name, module in zip(names, n.wheel_status.modules):
            module.name = name
        n.joint_state.name = n.expected_steering_joint_names + n.expected_traction_joint_names
        n.joint_state.position = [0.0] * 8
        n.joint_state.velocity = [0.0] * 8
        n.scanner_states = {name: (None, None) for name in n.scanner_states}

    def test_stale_selector_or_operator_cannot_authorize_bypass(self):
        for cause in ('selector_stamp', 'selector_receipt', 'operator_stamp',
                      'operator_receipt', 'radio_loss', 'operator_estop', 'invalid_selector'):
            with self.subTest(cause=cause):
                self.setUp()
                if cause.endswith('_stamp'):
                    obj = {'selector_stamp': self.n.operator_selector,
                           'operator_stamp': self.n.manual_state}[cause]
                    obj.header.stamp = h.Time(9).to_msg()
                elif cause.endswith('_receipt'):
                    field = {'selector_receipt': 'operator_selector_received',
                             'operator_receipt': 'manual_state_received'}[cause]
                    setattr(self.n, field, h.Time(9))
                elif cause == 'radio_loss': self.n.manual_state.connected = False
                elif cause == 'operator_estop': self.n.manual_state.estop_pressed = True
                else: self.n.operator_selector.buttons[-1] = 2
                self.assertFalse(self.n._scanner_gate(self.f.now)[0])

    def test_red_scanner_callback_does_not_stop_authorized_bypass(self):
        self.f.now = h.Time(10.001)
        msg = SimpleNamespace(header=SimpleNamespace(stamp=self.f.now.to_msg()),
            scanner_id='front_left', communication_ok=True, protective_state_known=True,
            warning_state_known=True, protective_field_clear=False, warning_field_clear=False)
        self.n._scanner_callback('front_left', msg)
        self.assertTrue(self.n._scanner_gate(self.f.now)[0])
        self.assertFalse(self.n._system_conditions(self.f.now)[1])

    def test_auto_requests_and_current_auto_motion_are_rejected(self):
        for mode in (h.MODES.AUTONOMOUS, h.MODES.KINEMATICOFF):
            with self.subTest(mode=mode):
                self.assertFalse(self.n._request_mode(mode)[0])
                self.n.selected_mode = mode
                self.assertFalse(self.n._scanner_gate(self.f.now)[0])
                self.n.selected_mode = h.MODES.MANUAL

    def test_battery_and_each_drive_still_required(self):
        self.assertTrue(self.n._base_gate(self.f.now)[0])
        self.n.battery_state.system_ready = False
        self.assertFalse(self.n._enable_gate(self.f.now)[0])
        self.n.battery_state.system_ready = True
        for module in self.n.wheel_status.modules:
            for field, value in (('sto_active', True), ('error_code', 1), ('enabled', False),
                                 ('feedback_fresh', False)):
                with self.subTest(module=module.name, field=field):
                    if field == 'sto_active': module.sto_state_known = True
                    previous = getattr(module, field, False)
                    setattr(module, field, value)
                    self.assertFalse(self.n._base_gate(self.f.now)[0])
                    setattr(module, field, previous)

    def test_all_four_wheels_and_diagonal_motion_are_capped(self):
        n = self.n
        n.warning_speed_limit = 1.0
        n.last_control_time = h.Time(9)
        n.max_acceleration_linear = 100
        source = h.TwistStamped()
        source.twist.linear.x = source.twist.linear.y = 1
        n._publish_velocity(source, self.f.now)
        v = n.cmd_vel_publisher.messages[-1].twist.linear
        self.assertAlmostEqual(math.hypot(v.x, v.y), .3)
        msg = h.JointState()
        msg.name = n.expected_steering_joint_names + n.expected_traction_joint_names
        msg.position = [0.0] * 8
        msg.velocity = [1.0] * 4 + [8.0, -4.0, 2.0, -1.0]
        n._emit_final(msg, self.f.now.to_msg(), self.f.now, h.WheelModuleCommandArray.MOTION)
        speeds = [m.traction_velocity for m in n.final_command_publisher.messages[-1].modules]
        self.assertEqual(len(speeds), 4)
        for speed, expected in zip(speeds, (.3, -.15, .075, -.0375)):
            self.assertAlmostEqual(speed, expected)

    def test_returning_to_safety_enabled_restores_scanner_gate_and_clears_motion(self):
        self.n.manual_command = h.TwistStamped()
        self.n.last_output = [.3, 0, 0]
        self.f.now = h.Time(10.001)
        msg = SimpleNamespace(header=SimpleNamespace(stamp=self.f.now.to_msg(), frame_id='base_link'),
                              buttons=[0] * 11)
        self.n._operator_selector_callback(msg)
        self.assertIsNone(self.n.manual_command)
        self.assertEqual(self.n.last_output, [0, 0, 0])
        self.assertFalse(self.n._scanner_gate(self.f.now)[0])
        self.assertTrue(all(m.traction_velocity == 0
                            for m in self.n.final_command_publisher.messages[-1].modules))


class FourModuleScannerAvailabilityTest(unittest.TestCase):
    """Physical four-miControl profile with disconnected Ethernet scanners."""

    def setUp(self):
        fixture = FourModuleOperatorScannerOverrideTest()
        fixture.setUp()
        self.n, self.f = fixture.n, fixture.f
        self.select = fixture.select
        # Exercise fault reporting beyond discovery and feedback grace periods.
        self.n.started_at = h.Time(1)
        self.n.startup_grace = 0.0
        self.n.drive_startup_requested = True
        self.n.drive_startup_completed_at = h.Time(1)

    def test_disconnected_scanners_allow_manual_enable_without_scanner_faults(self):
        self.assertEqual(len(self.n.expected_module_names), 4)
        self.assertTrue(all(state == (None, None) for state in self.n.scanner_states.values()))
        self.assertTrue(self.n._scanner_gate(self.f.now)[0])
        self.assertTrue(self.n._enable_gate(self.f.now)[0])
        self.assertTrue(self.n._base_gate(self.f.now)[0])
        self.assertEqual(self.n._system_conditions(self.f.now), ([], False))

    def test_scanner_disconnect_callback_does_not_withdraw_enable(self):
        self.f.now = h.Time(10.001)
        for name in self.n.scanner_states:
            msg = SimpleNamespace(header=SimpleNamespace(stamp=self.f.now.to_msg()),
                scanner_id=name, communication_ok=False, protective_state_known=False,
                warning_state_known=False, protective_field_clear=False,
                warning_field_clear=False)
            self.n._scanner_callback(name, msg)
        self.assertTrue(self.n._enable_gate(self.f.now)[0])
        self.assertEqual(self.n._system_conditions(self.f.now), ([], False))

    def test_safety_enabled_restores_both_missing_scanner_faults(self):
        self.select(False)
        self.assertFalse(self.n._enable_gate(self.f.now)[0])
        errors, _ = self.n._system_conditions(self.f.now)
        for name in self.n.scanner_states:
            self.assertIn('SCANNER_MONITOR_LOST:' + name, errors)

    def test_receiver_fault_still_inhibits_with_safety_disabled(self):
        self.n.manual_state.connected = False
        self.n.manual_state.status_message = 'joystick receiver fault EMCY=0x8150'
        self.assertIsNone(self.n._operator_selector_value(self.f.now))
        self.assertFalse(self.n._enable_gate(self.f.now)[0])
        self.assertIn('OPERATOR_LOST', self.n._system_conditions(self.f.now)[0])

    def test_unknown_selector_does_not_exempt_disconnected_scanners(self):
        self.n.operator_selector.header.stamp = h.Time(9).to_msg()
        self.assertIsNone(self.n._operator_selector_value(self.f.now))
        self.assertFalse(self.n._enable_gate(self.f.now)[0])


class RestrictedManualFeedbackTest(unittest.TestCase):
    def setUp(self):
        fixture = FourModuleScannerAvailabilityTest()
        fixture.setUp()
        self.n, self.f = fixture.n, fixture.f
        self.select = fixture.select
        del self.n._source_gate, self.n._publish_mode_state
        mode_state = patch.dict(h.namespace, VehicleModeState=interfaces.State)
        mode_state.start()
        self.addCleanup(mode_state.stop)
        self.n.manual_state.controls_neutral = False
        self.n._control()
        self.assertTrue(self.n.module_command_publisher.messages[-1].enable)

    def command(self):
        cmd = h.TwistStamped()
        cmd.header.stamp = self.f.now.to_msg()
        cmd.twist.linear.x = 1.0
        self.n._manual_command_callback(cmd)

    def assert_feedback_pause(self, code):
        self.n._control()
        self.assertTrue(self.n.module_command_publisher.messages[-1].enable)
        state = self.n.mode_state_publisher.messages[-1]
        self.assertFalse(state.motion_permitted or state.error_active)
        self.assertIn(code, state.reason_codes)
        self.assertNotIn('B2_ENABLE_REQUIRED', state.reason_codes)
        self.assertIsNone(self.n.manual_command)
        self.assertEqual(self.n.cmd_vel_publisher.messages[-1].twist.linear.x, 0.0)

    def test_status_timeout_clears_motion_until_fresh_feedback_and_neutral(self):
        self.command()
        self.n._control()
        self.assertTrue(self.n.mode_state_publisher.messages[-1].motion_permitted)
        self.n.wheel_status_received = h.Time(9)
        self.assert_feedback_pause('DRIVER_MONITOR_LOST')

        self.n._wheel_status_callback(self.n.wheel_status)
        self.command()
        self.n._control()
        self.assertFalse(self.n.mode_state_publisher.messages[-1].motion_permitted)

        self.f.now = h.Time(10.01)
        self.n.manual_state.header.stamp = self.f.now.to_msg()
        self.n.manual_state.controls_neutral = True
        self.n._manual_state_callback(self.n.manual_state)
        self.assertIsNone(self.n.manual_command)
        self.f.now = h.Time(10.02)
        self.n.manual_state.header.stamp = self.f.now.to_msg()
        self.n.manual_state.controls_neutral = False
        self.n._manual_state_callback(self.n.manual_state)
        self.command()
        self.n._control()
        self.assertTrue(self.n.mode_state_publisher.messages[-1].motion_permitted)
        self.assertTrue(self.n.module_command_publisher.messages[-1].enable)

    def test_drive_position_and_feedback_loss_pause_motion_during_startup_grace(self):
        self.n.started_at = self.f.now
        self.n.startup_grace = 1.0
        module = self.n.wheel_status.modules[0]
        self.command()
        module.position_valid = module.feedback_fresh = False
        self.assert_feedback_pause('POSITION_CONTINUITY_UNKNOWN:' + module.name)
        self.assertIn('DRIVE_FEEDBACK_LOST:' + module.name,
                      self.n.mode_state_publisher.messages[-1].reason_codes)

    def test_kinematics_timeout_pauses_but_active_fault_withdraws_enable(self):
        self.command()
        self.n.required_health_sources = ['kinematics']
        self.n.component_health = {}
        self.n.health_timeout = 0.3
        self.assert_feedback_pause('COMPONENT_HEALTH_LOST:kinematics')
        record = SimpleNamespace(header=SimpleNamespace(stamp=self.f.now.to_msg()),
                                 monitoring_valid=True, fault_active=True)
        self.n.component_health['kinematics'] = (record, self.f.now)
        self.n._control()
        self.assertFalse(self.n.module_command_publisher.messages[-1].enable)
        self.assertTrue(self.n.operator_drives_disabled)

    def test_charger_interlock_withdraws_enable_during_feedback_pause(self):
        self.n.wheel_status_received = h.Time(9)
        self.assert_feedback_pause('DRIVER_MONITOR_LOST')
        self.n.battery_state.system_ready = False
        self.n.battery_state.status_message = 'charging or charger mains connected'
        self.n._control()
        self.assertFalse(self.n.module_command_publisher.messages[-1].enable)
        self.assertTrue(self.n.operator_drives_disabled)

    def test_safety_enabled_requires_b2_after_feedback_timeout(self):
        for name in self.n.scanner_states:
            state = SimpleNamespace(
                header=SimpleNamespace(stamp=self.f.now.to_msg()), scanner_id=name,
                communication_ok=True, protective_state_known=True, warning_state_known=True,
                protective_field_clear=True, warning_field_clear=True)
            self.n._scanner_callback(name, state)
        self.select(False)
        self.n.wheel_status_received = h.Time(9)
        self.n._control()
        self.assertFalse(self.n.module_command_publisher.messages[-1].enable)
        self.assertIn('B2_ENABLE_REQUIRED', self.n.mode_state_publisher.messages[-1].reason_codes)


if __name__ == '__main__':
    unittest.main()
