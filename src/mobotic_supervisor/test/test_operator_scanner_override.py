"""Commissioned RP2 selector cannot bypass operator/drive interlocks or speed caps."""
import math
from types import SimpleNamespace
import unittest
import test_enable_gate as h
import test_interface_update as interfaces


class OperatorScannerOverrideTest(unittest.TestCase):
    def setUp(self):
        fixture = interfaces.InterfaceUpdateTest()
        fixture.setUp()
        self.f = fixture.fixture
        self.n = fixture.node
        self.n.safety_source = 'hardwired_sto'
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
        self.assertTrue(self.n._safety_gate(self.f.now)[0])
        errors, stop = self.n._system_conditions(self.f.now)
        self.assertFalse(any(x.startswith('SCANNER_') for x in errors))
        self.assertFalse(stop)

    def test_enabled_requires_both_scanners(self):
        self.select(False)
        self.assertTrue(self.n._safety_gate(self.f.now)[0])
        self.n.scanner_states['front_left'] = (None, None)
        self.assertFalse(self.n._safety_gate(self.f.now)[0])

    def test_stop_link_loss_unknown_estop_and_stale_selector_never_exempt_scanners(self):
        for field, value in (('connected', False), ('estop_pressed', True), ('estop_state_known', False)):
            original = getattr(self.n.manual_state, field)
            setattr(self.n.manual_state, field, value)
            self.assertFalse(self.n._safety_gate(self.f.now)[0])
            setattr(self.n.manual_state, field, original)
        self.n.operator_selector.header.stamp = h.Time(9).to_msg()
        self.assertFalse(self.n._safety_gate(self.f.now)[0])

    def test_autonomy_cannot_use_scanner_exemption(self):
        for mode in (h.MODES.AUTO_VELOCITY, h.MODES.AUTO_DIRECT):
            self.n.selected_mode = mode
            self.assertFalse(self.n._safety_gate(self.f.now)[0])

    def test_battery_and_drive_sto_checks_remain_required(self):
        self.n.battery_state.system_ready = False
        self.assertFalse(self.n._enable_gate(self.f.now)[0])
        self.n.battery_state.system_ready = True
        self.n.wheel_status.modules[0].sto_state_known = True
        self.n.wheel_status.modules[0].sto_active = True
        self.assertFalse(self.n._enable_gate(self.f.now)[0])

    def test_diagonal_translation_cap_even_if_warning_policy_is_higher(self):
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
        self.assertAlmostEqual(final.modules[0].traction_velocity*self.n.wheel_radius, .3)
        self.assertAlmostEqual(final.modules[1].traction_velocity*self.n.wheel_radius, -.15)

    def test_malformed_selector_does_not_exempt_scanners(self):
        for buttons in ([0]*10, [0]*10+[2], []):
            self.n.operator_selector.buttons = buttons
            self.assertFalse(self.n._safety_gate(self.f.now)[0])


if __name__ == '__main__':
    unittest.main()
