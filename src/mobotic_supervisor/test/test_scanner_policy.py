"""Scanner traffic controls absolute speed, immediate stops and safe recovery."""
import copy
import math
from types import SimpleNamespace
import unittest

import test_enable_gate as harness
import test_interface_update as interfaces


class ScannerPolicyTest(unittest.TestCase):
    def setUp(self):
        self.fixture = interfaces.InterfaceUpdateTest()
        self.fixture.setUp()
        self.platform = self.fixture.fixture
        self.node = self.fixture.node
        self.node.safety_source = 'hardwired_sto'
        self.node.selected_mode = harness.MODES.AUTO_VELOCITY
        self.node.motion_inputs_ready = True
        del self.node._source_gate
        self.node.direct_traction_control_mode = 'velocity'

    def field(self, name='front_left', *, red=False, yellow=False, known=True):
        state = copy.deepcopy(self.node.scanner_states[name][0])
        self.platform.now = harness.Time(self.platform.now.nanoseconds / 1e9 + .001)
        state.header.stamp = self.platform.now.to_msg()
        state.communication_ok = known
        state.protective_state_known = state.warning_state_known = known
        state.protective_field_clear, state.warning_field_clear = not red, not yellow
        self.node._scanner_callback(name, state)
        return state

    def command(self, vx=1.0, vy=0.0, wz=0.0):
        source = harness.TwistStamped()
        source.header.stamp = self.platform.now.to_msg()
        source.twist.linear.x, source.twist.linear.y, source.twist.angular.z = vx, vy, wz
        return source

    def publish(self, source, seconds=.1):
        self.platform.now = harness.Time(self.platform.now.nanoseconds / 1e9 + seconds)
        self.platform.feedback()
        self.node._publish_velocity(source, self.platform.now)
        return self.node.cmd_vel_publisher.messages[-1].twist

    def test_yellow_ramps_to_absolute_point_three_instead_of_percentage(self):
        self.node.last_output = [1.0, 0.0, 0.0]
        self.field(yellow=True)
        source = self.command()
        self.assertAlmostEqual(self.publish(source).linear.x, .9495)
        for _ in range(20):
            out = self.publish(source)
        self.assertAlmostEqual(out.linear.x, .3)
        self.assertTrue(self.node._enable_gate(self.platform.now)[0])

    def test_yellow_is_a_cap_and_preserves_slow_commands(self):
        self.field(yellow=True)
        for vx, vy in ((.1, 0), (-.2, 0), (0, .2), (.1, .1), (0, 0)):
            with self.subTest(vx=vx, vy=vy):
                self.node.last_output = [0.0] * 3
                source = self.command(vx, vy)
                for _ in range(8):
                    out = self.publish(source)
                self.assertAlmostEqual(out.linear.x, vx)
                self.assertAlmostEqual(out.linear.y, vy)

    def test_diagonal_reverse_and_sideways_speed_are_bounded(self):
        self.field(yellow=True)
        for vx, vy in ((1, 1), (-1, -1), (0, 1), (0, -1)):
            self.node.last_output = [0.0] * 3
            for _ in range(10):
                out = self.publish(self.command(vx, vy))
            self.assertAlmostEqual(math.hypot(out.linear.x, out.linear.y), .3)
            self.assertGreaterEqual(out.linear.x * vx, 0)
            self.assertGreaterEqual(out.linear.y * vy, 0)

    def test_clear_fields_restore_normal_ramp_and_platform_limits(self):
        self.field(yellow=True)
        self.node.last_output = [.3, 0, 0]
        self.field()
        out = self.publish(self.command())
        self.assertGreater(out.linear.x, .3)
        self.assertLess(out.linear.x, 1)
        for _ in range(20):
            out = self.publish(self.command())
        self.assertAlmostEqual(out.linear.x, 1)

    def test_clear_never_increases_a_lower_platform_limit(self):
        self.node.max_velocity_linear = .2
        self.field(yellow=True)
        for _ in range(10):
            out = self.publish(self.command())
        self.assertAlmostEqual(out.linear.x, .2)

    def test_red_zeros_final_wheels_and_rotation_in_the_receive_callback(self):
        self.node.last_output = [.8, .4, .6]
        self.node.last_final = {'front': 3.0, 'rear': -2.0}
        self.node.velocity_ledger[123] = (7, harness.WheelModuleCommandArray.MOTION, 123)
        self.field(red=True, yellow=True)
        self.assertEqual(self.node.last_output, [0.0] * 3)
        final = self.node.final_command_publisher.messages[-1]
        self.assertTrue(all(m.traction_velocity == 0 and m.steering_profile_velocity == 0 for m in final.modules))
        self.assertFalse(self.node._safety_gate(self.platform.now)[0])
        self.assertTrue(self.node._system_conditions(self.platform.now)[1])
        self.assertNotIn(123, self.node.velocity_ledger)

    def test_red_from_either_scanner_has_priority_over_yellow_and_override(self):
        self.node.safety_source = 'flexisoft'
        self.node.safety_state.safety_override_active = True
        self.field(yellow=True)
        self.field('rear_right', red=True)
        self.assertFalse(self.node._safety_gate(self.platform.now)[0])
        self.field('rear_right')
        self.assertTrue(self.node._safety_gate(self.platform.now)[0])
        self.assertTrue(self.node._warning_active())
        self.field()
        self.assertFalse(self.node._warning_active())

    def test_missing_invalid_or_stale_either_scanner_inhibits_motion(self):
        for name in self.node.scanner_states:
            original = self.node.scanner_states[name]
            self.node.scanner_states[name] = (None, None)
            self.assertFalse(self.node._safety_gate(self.platform.now)[0])
            self.node.scanner_states[name] = original
            self.field(name, known=False)
            self.assertFalse(self.node._safety_gate(self.platform.now)[0])
            self.field(name)
        self.platform.now = harness.Time(10.4)
        self.assertFalse(self.node._safety_gate(self.platform.now)[0])

    def test_heartbeats_do_not_extend_original_acquisition_lifetime(self):
        state = self.field()
        self.platform.now = harness.Time(10.25)
        self.node._scanner_callback('front_left', state)
        self.assertAlmostEqual(self.node.scanner_states['front_left'][1].nanoseconds / 1e9, 10.001)
        self.platform.now = harness.Time(10.302)
        self.assertFalse(self.node._scanner_gate(self.platform.now)[0])

    def test_red_recovery_requires_new_command_and_keeps_yellow_limit(self):
        self.node._autonomy_command_callback(self.command())
        self.field(red=True)
        self.assertIsNone(self.node.autonomy_command)
        self.field(yellow=True)
        self.assertIsNone(self.node.autonomy_command)
        self.assertFalse(self.node._source_gate(self.platform.now)[0])
        self.node._autonomy_command_callback(self.command())
        self.assertTrue(self.node._source_gate(self.platform.now)[0])
        for _ in range(10):
            out = self.publish(self.command())
        self.assertAlmostEqual(out.linear.x, .3)

    def test_yellow_discards_inflight_unrestricted_kinematics(self):
        msg = self.fixture.joints()
        self.node.velocity_ledger[msg.header.stamp] = (self.node.authority_generation,
            harness.WheelModuleCommandArray.MOTION, msg.header.stamp)
        self.field(yellow=True)
        self.node._kinematics_candidate_callback(msg)
        self.assertFalse(self.node.final_command_publisher.messages)

    def test_yellow_does_not_change_driver_authority_and_force_a_stop(self):
        generation = self.node.authority_generation
        self.field(yellow=True)
        self.assertEqual(self.node.authority_generation, generation)
        self.assertTrue(self.node._safety_gate(self.platform.now)[0])
        self.field()
        self.assertEqual(self.node.authority_generation, generation)

    def test_red_announces_driver_inhibition_during_a_mode_transition(self):
        self.node.transition_in_progress = True
        self.field(red=True)
        now_mode, transitioning, permitted, reason = self.platform.states[-1]
        self.assertTrue(transitioning)
        self.assertFalse(permitted)
        self.assertIn('red', reason)
        self.assertTrue(all(m.traction_velocity == 0 for m in self.node.final_command_publisher.messages[-1].modules))

    def test_direct_mode_yellow_ramps_wheel_peripheral_speed_then_clear_restores(self):
        node = self.node
        node.selected_mode = harness.MODES.AUTO_DIRECT
        self.field('rear_right', yellow=True)
        node.last_final = {'front': 3.0, 'rear': -3.0}
        source = self.fixture.joints()
        source.velocity = [1.0, 1.0, 3.0, -3.0]
        for step in range(20):
            self.platform.now = harness.Time(self.platform.now.nanoseconds / 1e9 + .1)
            self.platform.feedback()
            node._publish_direct(source, self.platform.now)
            if step == 0:
                speed = node.final_command_publisher.messages[-1].modules[0].traction_velocity * node.wheel_radius
                self.assertGreater(speed, .3)
                self.assertLess(speed, 3 * node.wheel_radius)
                self.assertAlmostEqual(3 * node.wheel_radius - speed, .0505)
        final = node.final_command_publisher.messages[-1]
        self.assertTrue(all(abs(m.traction_velocity * node.wheel_radius) <= .3 + 1e-12 for m in final.modules))
        self.assertAlmostEqual(final.modules[0].traction_velocity * node.wheel_radius, .3)
        self.assertTrue(node._motion_hardware_gate(self.platform.now)[0])
        self.field('rear_right')
        node._publish_direct(source, self.platform.now)
        self.assertEqual([m.traction_velocity for m in node.final_command_publisher.messages[-1].modules], [3, -3])

    def test_direct_mode_red_stops_without_waiting_for_a_control_tick(self):
        self.node.selected_mode = harness.MODES.AUTO_DIRECT
        self.field(red=True)
        self.assertTrue(all(m.traction_velocity == 0 for m in self.node.final_command_publisher.messages[-1].modules))


if __name__ == '__main__':
    unittest.main()
