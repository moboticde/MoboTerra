"""Autonomy steering admission, constrained ramps and authority transitions."""
import copy
from types import SimpleNamespace
import unittest

from mobotic_kinematics.manual_steering import ManualSteering

import test_enable_gate as h
import test_command_recovery as recovery
import test_interface_update as final


class AutonomySteeringTest(unittest.TestCase):
    def setUp(self):
        self.fixture = recovery.CommandRecoveryTest()
        self.fixture.setUp()
        self.platform = self.fixture.platform
        self.node = self.fixture.node
        self.node.steering_mode = self.node.requested_steering_mode = 0
        self.node.steering_transition_in_progress = False
        self.node.manual_steering_policy = ManualSteering({'wheels_x': [1., -1.], 'wheels_y': [1/3, -1/3]})
        self.node.autonomy_command_rejection = ''
        self.node.requested_mode = h.MODES.AUTONOMOUS
        self.node.authority_generation = 1
        self.node.velocity_ledger = {}

    def command(self, x=0., y=0., yaw=0.):
        msg = self.fixture.velocity()
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = x, y, yaw
        return msg

    def select(self, mode):
        response = self.node._set_steering_service(SimpleNamespace(mode=mode), SimpleNamespace())
        self.assertTrue(response.accepted, response.message)
        return response

    def finish(self):
        self.node._control()
        self.platform.advance(.25)
        self.assertFalse(self.node.transition_in_progress)

    def output(self):
        msg = self.node.cmd_vel_publisher.messages[-1]
        return msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z

    def test_standard_default_accepts_combined_translation_and_rotation(self):
        self.assertEqual(self.select(0).active_mode, 0)
        msg = self.command(.001, .002, .003)
        self.node._autonomy_command_callback(msg)
        self.fixture.advance(.02)
        self.node._control()
        self.assertEqual(self.output(), (.001, .002, .003))

    def test_crab_accepts_both_sideways_and_diagonal_directions(self):
        self.node.steering_mode = 1
        for x, y in ((0., .2), (0., -.2), (.2, .2), (-.2, -.2)):
            with self.subTest(x=x, y=y):
                msg = self.command(x, y)
                self.node._autonomy_command_callback(msg)
                self.assertIs(self.node.autonomy_command, msg)
                self.assertTrue(self.node._source_gate(self.platform.now)[0])

    def test_ackermann_accepts_forward_reverse_straight_and_zero(self):
        self.node.steering_mode = 2
        for x, yaw in ((.2, .15), (-.2, .15), (-.2, -.15), (.2, 0.), (0., 0.)):
            with self.subTest(x=x, yaw=yaw):
                msg = self.command(x=x, yaw=yaw)
                self.node._autonomy_command_callback(msg)
                self.assertIs(self.node.autonomy_command, msg)
                self.assertTrue(self.node._source_gate(self.platform.now)[0])

    def test_incompatible_input_revokes_old_command_and_requests_controlled_stop(self):
        for mode, command, reason in ((1, (0., .2, .1), 'CRAB'),
                                      (2, (.2, .1, 0.), 'linear.y'),
                                      (2, (0., 0., .1), 'rotation in place'),
                                      (2, (.2, 0., .16), 'curvature')):
            with self.subTest(mode=mode, command=command):
                self.setUp()
                self.node.steering_mode = mode
                self.node._autonomy_command_callback(self.command(x=.2))
                self.fixture.advance(.1)
                self.node._control()
                before = self.output()[0]
                old_generation = self.node.authority_generation
                self.node._autonomy_command_callback(self.command(*command))
                self.assertIsNone(self.node.autonomy_command)
                self.assertGreater(self.node.authority_generation, old_generation)
                self.assertEqual(self.node.velocity_ledger, {})
                self.assertIn(reason, self.node._source_gate(self.platform.now)[1])
                self.fixture.advance(.02)
                self.node._control()
                self.assertGreater(self.output()[0], 0.)
                self.assertLess(self.output()[0], before)
                entry = self.node.velocity_ledger[self.platform.now.nanoseconds]
                self.assertEqual(entry[1], h.WheelModuleCommandArray.CONTROLLED_STOP)
                self.assertFalse(self.node.fault_codes)
                self.assertTrue(self.node.module_command_publisher.messages[-1].enable)
                # Repeated invalid publications must not revoke stop results.
                generation = self.node.authority_generation
                ledger = copy.deepcopy(self.node.velocity_ledger)
                self.node._autonomy_command_callback(self.command(*command))
                self.assertEqual(self.node.authority_generation, generation)
                self.assertEqual(self.node.velocity_ledger, ledger)
                self.node._autonomy_command_callback(self.command())
                self.assertEqual(self.node.autonomy_command_rejection, '')

    def test_ackermann_ramps_and_reversal_never_create_rotation_in_place(self):
        self.node.steering_mode = 2
        self.node.max_acceleration_linear = 10.
        self.node.max_acceleration_angular = .1
        self.node.last_output = [.1, 0., .075]
        self.node.last_control_time = self.platform.now
        # Linear deceleration reaches zero sooner than independent yaw slew.
        self.fixture.advance(.02)
        self.node._autonomy_command_callback(self.command())
        self.node._control()
        vx, vy, yaw = self.output()
        self.assertGreater(vx, 0.)
        self.assertEqual(vy, 0.)
        self.assertAlmostEqual(yaw, .073)
        self.assertLessEqual(abs(yaw), abs(vx) * .75)
        for x, yaw in ((-.2, -.15), (.2, .15), (0., 0.)):
            self.fixture.advance(.02)
            self.node._autonomy_command_callback(self.command(x=x, yaw=yaw))
            self.node._control()
            vx, vy, wz = self.output()
            self.assertEqual(vy, 0.)
            self.assertLessEqual(abs(wz), abs(vx) * .75)

    def test_ackermann_reversal_obeys_both_acceleration_limits(self):
        self.node.steering_mode = 2
        self.node.last_output = [.1, 0., .075]
        previous = self.node.last_output[:]
        reached_zero = False
        for _ in range(60):
            self.fixture.advance(.02)
            self.node._autonomy_command_callback(self.command(x=-.1, yaw=.075))
            self.node._control()
            vx, vy, yaw = self.output()
            reached_zero |= vx == 0. and yaw == 0.
            self.assertEqual(vy, 0.)
            self.assertLessEqual(abs(yaw), abs(vx) * .75 + 1e-12)
            self.assertLessEqual(abs(vx - previous[0]), .5 * .02 + 1e-12)
            self.assertLessEqual(abs(yaw - previous[2]), .52 * .02 + 1e-12)
            previous = [vx, vy, yaw]
        self.assertTrue(reached_zero)
        self.assertAlmostEqual(vx, -.1)
        self.assertAlmostEqual(yaw, .075)

    def test_tolerated_numerical_noise_is_zeroed(self):
        for mode, command in ((1, (.01, .01, 1e-10)), (2, (.01, 1e-10, 0.))):
            self.node.steering_mode = mode
            self.node._autonomy_command_callback(self.command(*command))
            self.fixture.advance(.02)
            self.node._control()
            self.assertEqual(self.output()[2 if mode == 1 else 1], 0.)

    def test_switch_stops_then_commits_and_discards_pre_switch_input(self):
        self.node._autonomy_command_callback(self.command(.1, .1, .05))
        self.node.velocity_ledger[123] = ('old motion',)
        self.node.last_output = [.1, .1, .05]
        response = self.select(1)
        self.assertTrue(response.transition_started)
        self.assertEqual(response.active_mode, 0)
        self.assertEqual(self.node.requested_steering_mode, 1)
        self.assertEqual(self.node.velocity_ledger, {})
        self.assertIsNone(self.node.autonomy_command)
        self.platform.advance(.1, vx=.05)
        self.assertEqual(self.node.steering_mode, 0)
        self.assertTrue(self.node.transition_in_progress)
        for _ in range(4):
            self.platform.advance(.1)
        old = self.command(y=.1)
        self.node._autonomy_command_callback(old)
        self.platform.advance(.25)
        self.assertFalse(self.node.transition_in_progress)
        self.assertEqual(self.node.steering_mode, 1)
        self.assertFalse(self.node.steering_transition_in_progress)
        self.assertIsNone(self.node.autonomy_command)
        self.node._autonomy_command_callback(old)
        self.assertIsNone(self.node.autonomy_command)
        self.node._autonomy_command_callback(self.command(y=.1))
        self.assertIsNotNone(self.node.autonomy_command)

    def test_repeated_selection_is_idempotent_and_conflicting_transitions_reject(self):
        self.assertFalse(self.select(0).transition_started)
        self.select(1)
        self.assertFalse(self.select(1).transition_started)
        self.assertFalse(self.node._request_steering(2)[0])
        self.assertFalse(self.node._request_mode(h.MODES.MANUAL)[0])
        self.finish()
        generation = self.node.authority_generation
        self.assertFalse(self.select(1).transition_started)
        self.assertEqual(self.node.authority_generation, generation)

    def test_all_three_selections_can_be_changed_back(self):
        for mode in (1, 2, 0):
            self.assertTrue(self.select(mode).transition_started)
            self.finish()
            self.assertEqual(self.node.steering_mode, mode)

    def test_invalid_mode_recovery_and_direct_mode_reject_without_mutation(self):
        for mode in (-1, 3, 255):
            self.assertFalse(self.node._request_steering(mode)[0])
        self.node.selected_mode = h.MODES.KINEMATICOFF
        self.assertFalse(self.node._request_steering(1)[0])
        self.node.selected_mode = h.MODES.AUTONOMOUS
        self.node.reset_started_at = self.platform.now
        self.assertFalse(self.node._request_steering(1)[0])
        self.node.reset_started_at = None
        self.node.fault_codes.add('TEST_FAULT')
        self.assertFalse(self.node._request_steering(1)[0])
        self.assertEqual(self.node.steering_mode, 0)

    def test_manual_preselection_requires_standstill_and_does_not_restrict_manual(self):
        self.node.selected_mode = self.node.requested_mode = h.MODES.MANUAL
        self.platform.feedback(vx=.2)
        self.assertFalse(self.node._request_steering(1)[0])
        self.platform.feedback()
        self.select(1)
        self.finish()
        self.assertEqual(self.node.selected_mode, h.MODES.MANUAL)
        self.assertEqual(self.node._autonomy_steering_reason(self.command(yaw=.2)), '')
        self.node._autonomy_command_callback(self.command(yaw=.2))
        self.assertEqual(self.node.autonomy_command_rejection, '')

    def test_missing_geometry_rejects_ackermann(self):
        self.node.manual_steering_policy = None
        self.assertFalse(self.node._request_steering(2)[0])

    def test_missing_feedback_and_timeout_keep_previous_selection(self):
        self.select(1)
        self.node.measured_velocity_received = None
        self.node._control()
        self.assertTrue(self.node.transition_in_progress)
        self.assertEqual(self.node.steering_mode, 0)
        self.platform.advance(10.1, vx=.1)
        self.assertIn('MODE_TRANSITION_TIMEOUT', self.node.fault_codes)
        self.assertEqual(self.node.steering_mode, 0)
        self.assertFalse(self.node._request_steering(2)[0])

    def test_rp2_loss_cancels_pending_selection(self):
        self.platform.selector(False)
        self.select(1)
        self.node.operator_selector = None
        self.node._control()
        self.assertFalse(self.node.transition_in_progress)
        self.assertFalse(self.node.steering_transition_in_progress)
        self.assertEqual(self.node.requested_steering_mode, self.node.steering_mode)
        self.assertEqual(self.node.steering_mode, 0)

    def test_cached_command_rules_are_rechecked(self):
        self.node.steering_mode = 1
        msg = self.command(y=.1)
        self.node._autonomy_command_callback(msg)
        msg.twist.angular.z = .1
        self.assertFalse(self.node._source_gate(self.platform.now)[0])

    def test_pending_motion_result_is_revoked_but_stop_result_reaches_driver(self):
        fixture = final.InterfaceUpdateTest()
        fixture.setUp()
        node, platform = fixture.node, fixture.fixture
        del node._source_gate
        node.selected_mode = node.requested_mode = h.MODES.AUTONOMOUS
        node.steering_mode = 1
        command = h.TwistStamped()
        command.header.stamp = platform.now.to_msg()
        command.twist.linear.x = .2
        node._autonomy_command_callback(command)
        node.last_output = [.2, 0., 0.]
        node.last_final = {'front': .2, 'rear': .2}
        node._control()
        delayed = fixture.joints()
        self.assertIn(delayed.header.stamp, node.velocity_ledger)
        command = copy.deepcopy(command)
        command.twist.angular.z = .1
        node._autonomy_command_callback(command)
        platform.now = h.Time(10.02)
        platform.feedback()
        node._control()
        node._kinematics_candidate_callback(delayed)
        self.assertFalse(node.final_command_publisher.messages)
        stop = fixture.joints()
        stop.velocity[2:] = [.19, .19]
        node._kinematics_candidate_callback(stop)
        output = node.final_command_publisher.messages[-1]
        self.assertEqual(output.purpose, h.WheelModuleCommandArray.CONTROLLED_STOP)
        self.assertTrue(all(0. <= m.traction_velocity <= .2 for m in output.modules))
        self.assertTrue(all(m.steering_profile_velocity == 0. for m in output.modules))

    def test_selection_survives_vehicle_mode_changes_and_recovery_wait(self):
        self.select(1)
        self.finish()
        for mode in (h.MODES.MANUAL, h.MODES.AUTONOMOUS):
            self.assertTrue(self.node._request_mode(mode)[0])
            self.finish()
            self.assertEqual(self.node.steering_mode, 1)
        self.node._wait_for_b2(self.platform.now, 'test readiness loss')
        self.assertEqual(self.node.steering_mode, 1)
        self.assertEqual(self.node.requested_steering_mode, 1)
        self.assertFalse(self.node.steering_transition_in_progress)

    def test_state_reports_active_pending_and_rejected_commands(self):
        fixture = final.InterfaceUpdateTest()
        fixture.setUp()
        node = fixture.node
        node.steering_mode, node.requested_steering_mode = 0, 1
        node.steering_transition_in_progress = True
        state = fixture.publish_state()
        self.assertEqual((state.steering_mode, state.requested_steering_mode), (0, 1))
        self.assertTrue(state.steering_transition_in_progress)
        node.steering_mode = 1
        node.selected_mode = h.MODES.AUTONOMOUS
        node.autonomy_command_rejection = 'CRAB requires angular.z = 0'
        state = fixture.publish_state()
        self.assertIn('AUTONOMY_STEERING_COMMAND_REJECTED', state.reason_codes)


if __name__ == '__main__':
    unittest.main()
