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

    def test_manual_ackermann_speed_caps_preserve_requested_curvature(self):
        from mobotic_kinematics.manual_steering import ManualSteering
        for count, condition in list(self.cases()) + [(2, 'clear'), (4, 'clear')]:
            for velocity in (-2., -1., .1, 1., 2.):
                for curvature in (-.6, -.22827042, .22827042, .6):
                    with self.subTest(count=count, condition=condition, velocity=velocity, curvature=curvature):
                        self.configure(count, condition)
                        n = self.node
                        n.max_velocity_linear = 2.
                        n.selected_mode = h.MODES.MANUAL
                        n.manual_steering_policy = ManualSteering({
                            'wheels_x': [.845 if name.startswith('front') else -.845 for name in n.expected_module_names],
                            'wheels_y': [.495 if name.endswith('left') else -.495 for name in n.expected_module_names],
                            'steering_joint_names': n.expected_steering_joint_names,
                            'traction_joint_names': n.expected_traction_joint_names,
                            'wheel_mount_angles': [0. if name.endswith('left') else math.pi for name in n.expected_module_names],
                            'max_steering_velocity': .3})
                        n.last_output = [0., 0., 0.]
                        source = h.TwistStamped()
                        source.header.stamp = self.now.to_msg()
                        source.twist.linear.x = velocity
                        source.twist.angular.z = velocity*curvature
                        n._publish_velocity(source, self.now)
                        output = n.cmd_vel_publisher.messages[-1].twist
                        expected_speed = abs(velocity) if condition == 'clear' else min(abs(velocity), .3)
                        self.assertAlmostEqual(output.linear.x, math.copysign(expected_speed, velocity))
                        self.assertAlmostEqual(output.angular.z/output.linear.x, curvature)
                        self.assertEqual(output.linear.y, 0.)

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

    def test_green_normal_boost_and_reduced_speed_matrix(self):
        for count in (2, 4):
            for condition in ('clear', 'scanner_yellow', 'plc_yellow', 'plc_io_yellow',
                              'flexisoft_disabled', 'rp2_disabled'):
                for requested in (1., 2.):
                    with self.subTest(count=count, condition=condition, requested=requested):
                        msg = self.configure(count, condition)
                        self.node.max_velocity_linear = self.node.max_direct_traction_velocity = 2.
                        source = h.TwistStamped()
                        source.twist.linear.x = requested
                        self.node.last_output = [0., 0., 0.]
                        self.node._publish_velocity(source, self.now)
                        expected = requested if condition == 'clear' else .3
                        velocity = self.node.cmd_vel_publisher.messages[-1].twist
                        self.assertAlmostEqual(velocity.linear.x, expected)
                        for index in range(count, 2 * count):
                            msg.velocity[index] = requested * (-1) ** index
                        self.node._emit_final(msg, self.now.to_msg(), self.now,
                                              h.WheelModuleCommandArray.MOTION)
                        self.assertAlmostEqual(max(abs(module.traction_velocity) for module in
                            self.node.final_command_publisher.messages[-1].modules), expected)

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

    def test_green_scanners_and_enabled_rp2_remove_the_reduced_speed_cap(self):
        self.configure(4,'rp2_disabled')
        self.assertEqual(self.node._speed_limit_reasons(),['SAFETY_DISABLED_RP2'])
        self.node.operator_selector.buttons[10]=0
        self.assertEqual(self.node._speed_limit_reasons(),[])
        self.assertFalse(self.node._warning_active())
        self.node.max_velocity_linear=2.
        self.node.last_output=[0.,0.,0.]
        source=h.TwistStamped()
        source.twist.linear.x=2.
        self.node._publish_velocity(source,self.now)
        self.assertEqual(self.node.cmd_vel_publisher.messages[-1].twist.linear.x,2.)


if __name__ == '__main__':
    unittest.main()
