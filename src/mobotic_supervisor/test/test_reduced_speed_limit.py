"""Absolute 0.3 m/s cap at body and wheel authority boundaries, for both layouts."""
import math
from types import SimpleNamespace
import unittest

import test_enable_gate as h
import test_interface_update as interfaces


class ReducedSpeedLimitTest(unittest.TestCase):
    def configure(self, count, condition):
        fixture = interfaces.InterfaceUpdateTest()
        fixture.setUp()
        self.node = n = fixture.node
        self.now = fixture.fixture.now
        names = ['front_right', 'rear_left'] if count == 2 else [
            'front_left', 'front_right', 'rear_left', 'rear_right']
        n.expected_module_names = names
        n.expected_steering_joint_names = [name + '_steering' for name in names]
        n.expected_traction_joint_names = [name + '_traction' for name in names]
        n.safety_source = 'flexisoft'
        n.direct_traction_control_mode = 'velocity'
        n.warning_speed_limit = 1.0  # An overlay must never raise the absolute cap.
        n.max_acceleration_linear = n.max_acceleration_angular = 100
        n.last_control_time = h.Time(9)
        n.last_output = [1.0, 1.0, 1.0]
        n.override_speed_scale = .01  # Legacy percentage must not set the limit.
        if condition == 'scanner_yellow':
            n.scanner_states['front_left'][0].warning_field_clear = False
        elif condition == 'plc_yellow':
            n.safety_state.warning_field_active = True
        elif condition == 'plc_io_yellow':
            n.safety_io_state.rear_right_warning_field_clear = False
        elif condition == 'flexisoft_disabled':
            n.safety_state.safety_override_active = True
        elif condition == 'rp2_disabled':
            n.operator_scanner_override_enabled = True
            n.manual_state = SimpleNamespace(header=SimpleNamespace(stamp=self.now.to_msg()),
                source_id='remdevice', connected=True, estop_state_known=True, estop_pressed=False)
            n.manual_state_received = self.now
            n.operator_selector = SimpleNamespace(
                header=SimpleNamespace(stamp=self.now.to_msg(), frame_id=n.output_frame_id),
                buttons=[0] * 10 + [1])
            n.operator_selector_received = self.now
        msg = h.JointState()
        msg.header.stamp = self.now.to_msg()
        msg.name = n.expected_steering_joint_names + n.expected_traction_joint_names
        msg.position = [0.0] * (2 * count)
        msg.velocity = [1.0] * count + [8.0 * (-.5) ** i for i in range(count)]
        msg.effort = []
        n.joint_state = msg
        n.last_final = dict(zip(names, msg.velocity[count:]))
        return msg

    def cases(self):
        for count in (2, 4):
            for condition in ('scanner_yellow', 'plc_yellow', 'plc_io_yellow', 'flexisoft_disabled'):
                yield count, condition
        yield 2, 'rp2_disabled'
        yield 4, 'rp2_disabled'

    def test_final_wheels_are_bounded_for_every_warning_source_and_supported_mode(self):
        for count, condition in self.cases():
            for mode in (h.MODES.MANUAL, h.MODES.AUTONOMOUS, h.MODES.KINEMATICOFF):
                with self.subTest(count=count, condition=condition, mode=mode):
                    msg = self.configure(count, condition)
                    self.node.selected_mode = mode
                    if condition == 'rp2_disabled' and mode != h.MODES.MANUAL:
                        continue  # RP2 bypass does not authorize automatic modes.
                    self.node._emit_final(msg, self.now.to_msg(), self.now, h.WheelModuleCommandArray.MOTION)
                    wheels = self.node.final_command_publisher.messages[-1].modules
                    speeds = [wheel.traction_velocity for wheel in wheels]
                    self.assertAlmostEqual(max(map(abs, speeds)), .3)
                    for i, speed in enumerate(speeds):
                        self.assertAlmostEqual(speed / speeds[0], (-.5) ** i)

    def test_diagonal_and_turn_commands_from_previous_high_speed_are_capped_immediately(self):
        for count, condition in self.cases():
            with self.subTest(count=count, condition=condition):
                self.configure(count, condition)
                source = h.TwistStamped()
                source.twist.linear.x = source.twist.linear.y = source.twist.angular.z = 1
                self.node._publish_velocity(source, self.now)
                out = self.node.cmd_vel_publisher.messages[-1].twist
                self.assertAlmostEqual(math.hypot(out.linear.x, out.linear.y), .3)
                self.assertLessEqual(abs(out.angular.z),
                    self.node.max_velocity_angular * .3 / self.node.max_velocity_linear + 1e-12)

    def test_direct_wheel_commands_and_controlled_stops_obey_the_same_cap(self):
        for count, condition in self.cases():
            for direct in (False, True):
                with self.subTest(count=count, condition=condition, direct=direct):
                    msg = self.configure(count, condition)
                    if direct:
                        self.node._publish_direct(msg, self.now)
                    else:
                        self.node._emit_final(msg, self.now.to_msg(), self.now,
                            h.WheelModuleCommandArray.CONTROLLED_STOP)
                    wheels = self.node.final_command_publisher.messages[-1].modules
                    self.assertTrue(all(abs(m.traction_velocity) <= .3 + 1e-12
                                        for m in wheels))

    def test_lower_configured_limits_and_slow_commands_remain_lower(self):
        for count, condition in self.cases():
            with self.subTest(count=count, condition=condition):
                msg = self.configure(count, condition)
                self.node.warning_speed_limit = .15
                self.node._emit_final(msg, self.now.to_msg(), self.now, h.WheelModuleCommandArray.MOTION)
                self.assertAlmostEqual(max(abs(m.traction_velocity)
                    for m in self.node.final_command_publisher.messages[-1].modules), .15)
                for i in range(count, 2 * count):
                    msg.velocity[i] = .1
                self.node._emit_final(msg, self.now.to_msg(), self.now, h.WheelModuleCommandArray.MOTION)
                self.assertTrue(all(abs(m.traction_velocity - .1) < 1e-12
                    for m in self.node.final_command_publisher.messages[-1].modules))

    def test_contact_speed_cap_is_independent_of_radius_and_clear_policy(self):
        for count in (2, 4):
            for radius in (.1, .325, .5):
                with self.subTest(count=count, radius=radius):
                    msg = self.configure(count, 'flexisoft_disabled')
                    self.node.wheel_radius = radius
                    self.node._emit_final(msg, self.now.to_msg(), self.now, h.WheelModuleCommandArray.MOTION)
                    self.assertAlmostEqual(max(abs(m.traction_velocity)
                        for m in self.node.final_command_publisher.messages[-1].modules), .3)
                    self.node.safety_state.safety_override_active = False
                    self.node._emit_final(msg, self.now.to_msg(), self.now, h.WheelModuleCommandArray.MOTION)
                    self.assertEqual([m.traction_velocity for m in self.node.final_command_publisher.messages[-1].modules],
                        msg.velocity[count:])


if __name__ == '__main__':
    unittest.main()
