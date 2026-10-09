"""Scenario tests for verified standstill and blocked transition recovery.

This intentionally uses a two-module fixture to exercise configurable topology;
test_four_modules.py covers the actual four-module MoboTerra platform contract.
"""

from types import SimpleNamespace
import unittest

import test_enable_gate as harness


class ModeTransitionTest(unittest.TestCase):
    def setUp(self):
        harness.EnableGateTest.setUp(self)
        node = self.node
        # Use the real velocity/direct stop publishers for these scenarios.
        del node._publish_velocity
        del node._publish_direct_stop
        node.cmd_vel_publisher = harness.Publisher()
        node.direct_command_publisher = harness.Publisher()
        self.states = []
        node._publish_mode_state = lambda now, permitted, reason: self.states.append(
            (node.current_mode, node.transition_in_progress, permitted, reason)
        )
        node.get_logger = lambda: SimpleNamespace(
            info=lambda message: None, warning=lambda message: None
        )
        node.command_accept_after = None
        node.transition_duration = 0.25
        node.velocity_feedback_timeout = 0.3
        node.standstill_linear_velocity = 0.02
        node.standstill_angular_velocity = 0.02
        node.standstill_joint_velocity = 0.1
        node.max_velocity_linear = 1.25
        node.max_velocity_angular = 1.51
        node.max_acceleration_linear = 0.5
        node.max_acceleration_angular = 0.52
        node.warning_speed_scale = 0.3
        node.override_speed_scale = 0.2
        node.output_frame_id = 'base_link'
        node.safety_io_state.front_left_warning_field_clear = True
        node.safety_io_state.rear_right_warning_field_clear = True
        node.expected_steering_joint_names = ['front_steering', 'rear_steering']
        node.direct_steering_encoder_resolutions = [4096.0, 4096.0]
        node.expected_traction_joint_names = ['front_traction', 'rear_traction']
        node.last_control_time = self.now
        node.manual_command = None
        node.manual_command_received = None
        node.autonomy_command = None
        node.autonomy_command_received = None
        node.autonomy_joint_command = None
        node.autonomy_joint_received = None
        node.manual_timeout = node.autonomy_timeout = 0.25
        self.feedback()

    def feedback(self, *, vx=0.0, vy=0.0, wz=0.0, joint_velocity=0.0):
        node = self.node
        node.safety_received = node.safety_io_received = self.now
        node.battery_received = node.wheel_status_received = self.now
        for state in (node.safety_state, node.safety_io_state, node.battery_state):
            state.header.stamp = self.now.to_msg()
        node.wheel_status = SimpleNamespace(header=SimpleNamespace(stamp=self.now.to_msg()), modules=[
            SimpleNamespace(name=name, enabled=True, error_code=0, feedback_fresh=True)
            for name in node.expected_module_names
        ])
        velocity = harness.TwistStamped()
        velocity.header.stamp = self.now.to_msg()
        velocity.twist.linear.x = vx
        velocity.twist.linear.y = vy
        velocity.twist.angular.z = wz
        node._velocity_feedback_callback(velocity)
        joints = harness.JointState()
        joints.header.stamp = self.now.to_msg()
        joints.name = node.expected_steering_joint_names + node.expected_traction_joint_names
        joints.position = [0.7, -0.4, 0.0, 0.0]
        joints.velocity = [joint_velocity] * 4
        node._joint_state_callback(joints)

    def advance(self, seconds, **feedback):
        self.now = harness.Time(self.now.nanoseconds / 1.0e9 + seconds)
        self.feedback(**feedback)
        self.node._control()

    def request(self, mode=harness.MODES.AUTO_VELOCITY):
        accepted, started, _ = self.node._request_mode(mode)
        self.assertTrue(accepted)
        self.assertTrue(started)

    def test_moving_vehicle_does_not_switch_after_old_fixed_delay(self):
        self.request()
        self.advance(0.3, vx=0.5)
        self.advance(0.5, wz=0.2)
        self.assertEqual(self.node.current_mode, harness.MODES.MANUAL)
        self.assertTrue(self.node.transition_in_progress)
        self.assertFalse(self.states[-1][2])

    def test_stale_can_blocks_transition_despite_fresh_zero_joint_feedback(self):
        self.request()
        self.now = harness.Time(10.4)
        self.feedback()
        self.node.wheel_status.modules[0].feedback_fresh = False
        self.node._control()
        self.assertTrue(self.node.transition_in_progress)
        self.assertEqual(self.node.current_mode, harness.MODES.MANUAL)
        self.assertIsNone(self.node.transition_standstill_since)
        self.assertIn('feedback stale', self.states[-1][3])

    def test_controlled_stop_ramps_velocity_before_settling(self):
        self.node.last_output = [1.0, 0.0, 0.0]
        self.request()
        self.advance(0.1)
        self.assertAlmostEqual(self.node.cmd_vel_publisher.messages[-1].twist.linear.x, 0.95)
        self.assertIsNone(self.node.transition_standstill_since)
        self.assertTrue(self.node.transition_in_progress)
        for _ in range(21):
            self.advance(0.1)
        self.advance(0.3)
        self.assertEqual(self.node.current_mode, harness.MODES.AUTO_VELOCITY)

    def test_standstill_hold_completes_only_with_new_feedback(self):
        self.request()
        self.node._control()
        self.advance(0.2)
        self.assertTrue(self.node.transition_in_progress)
        self.advance(0.05)
        self.assertFalse(self.node.transition_in_progress)
        self.assertEqual(self.node.current_mode, harness.MODES.AUTO_VELOCITY)
        self.assertFalse(self.states[-1][2])

    def test_motion_resets_standstill_hold(self):
        self.request()
        self.node._control()
        self.advance(0.2, vy=0.03)
        self.advance(0.1)
        self.advance(0.2)
        self.assertTrue(self.node.transition_in_progress)
        self.advance(0.05)
        self.assertFalse(self.node.transition_in_progress)

    def test_joint_motion_prevents_false_vehicle_standstill(self):
        self.request()
        self.advance(0.5, joint_velocity=0.2)
        self.assertTrue(self.node.transition_in_progress)
        self.assertIn('joints are still moving', self.states[-1][3])

    def test_movement_between_control_ticks_resets_hold(self):
        self.request()
        self.node._control()
        self.now = harness.Time(10.2)
        self.feedback(vx=0.3)
        self.feedback()
        self.node._control()
        self.advance(0.1)
        self.assertTrue(self.node.transition_in_progress)
        self.advance(0.15)
        self.assertFalse(self.node.transition_in_progress)

    def test_missing_or_stale_feedback_blocks_switch(self):
        for attr in ('measured_velocity_received', 'joint_state_received'):
            for received in (None, harness.Time(1.0)):
                with self.subTest(attr=attr, received=received):
                    self.request()
                    self.node._control()
                    self.now = harness.Time(self.now.nanoseconds / 1.0e9 + 0.5)
                    self.feedback()
                    setattr(self.node, attr, received)
                    self.node._control()
                    self.assertTrue(self.node.transition_in_progress)
                    self.assertIsNone(self.node.transition_standstill_since)
                    self.node.transition_in_progress = False

    def test_pre_request_feedback_is_not_standstill_proof(self):
        self.now = harness.Time(10.1)
        self.request()
        self.node._control()
        self.assertTrue(self.node.transition_in_progress)
        self.assertIsNone(self.node.transition_standstill_since)

    def test_frozen_or_replayed_feedback_cannot_complete_hold(self):
        self.request()
        self.node._control()
        self.now = harness.Time(10.25)
        self.node.measured_velocity_received = self.node.joint_state_received = self.now
        self.node._control()
        self.assertTrue(self.node.transition_in_progress)
        self.now = harness.Time(10.31)
        self.node.safety_received = self.node.safety_io_received = self.now
        self.node.battery_received = self.node.wheel_status_received = self.now
        self.node.measured_velocity_received = self.node.joint_state_received = self.now
        self.node._control()
        self.assertTrue(self.node.transition_in_progress)
        self.assertIsNone(self.node.transition_standstill_since)

    def test_invalid_feedback_clears_cached_standstill(self):
        self.request()
        self.node._control()
        self.node.measured_velocity.twist.linear.x = float('nan')
        self.node._velocity_feedback_callback(self.node.measured_velocity)
        self.assertIsNone(self.node.measured_velocity)
        self.assertIsNone(self.node.transition_standstill_since)
        self.feedback()
        self.node.joint_state.velocity = []
        self.node._joint_state_callback(self.node.joint_state)
        self.assertIsNone(self.node.joint_state)
        self.node._control()
        self.assertTrue(self.node.transition_in_progress)

    def test_timeout_stays_blocked_until_explicit_retry(self):
        self.request()
        self.advance(10.0, vx=0.3)
        self.assertTrue(self.node.transition_timed_out)
        self.advance(0.3)
        self.advance(0.3)
        self.assertTrue(self.node.transition_in_progress)
        self.assertEqual(self.node.current_mode, harness.MODES.MANUAL)
        self.assertIn('timed out', self.states[-1][3])
        self.request()
        self.node._control()
        self.advance(0.25)
        self.assertFalse(self.node.transition_in_progress)
        self.assertEqual(self.node.current_mode, harness.MODES.AUTO_VELOCITY)

    def test_safety_loss_disables_and_immediately_stops(self):
        self.node.last_output = [1.0, 0.2, 0.1]
        self.request()
        self.node.safety_state.motion_permitted = False
        self.node._control()
        self.assertEqual(self.node.last_output, [0.0, 0.0, 0.0])
        self.assertFalse(self.node.module_command_publisher.messages[-1].enable)
        self.advance(0.5)
        self.assertTrue(self.node.transition_in_progress)

    def test_direct_mode_stop_uses_only_direct_joint_input(self):
        self.node.current_mode = harness.MODES.AUTO_DIRECT
        self.request(harness.MODES.MANUAL)
        self.node._control()
        self.assertFalse(self.node.cmd_vel_publisher.messages)
        stop = self.node.direct_command_publisher.messages[-1]
        self.assertEqual(stop.position[:2], [0.7, -0.4])
        self.assertEqual(stop.velocity, [0.0] * 4)
        self.assertEqual(stop.effort, [0.0] * 4)
        self.advance(0.25)
        self.assertEqual(self.node.current_mode, harness.MODES.MANUAL)

    def test_pre_switch_commands_are_cleared_and_rejected(self):
        self.request()
        old = harness.TwistStamped()
        old.header.stamp = self.now.to_msg()
        old.twist.linear.x = 0.5
        self.node._autonomy_command_callback(old)
        self.node._control()
        self.advance(0.25)
        self.assertIsNone(self.node.autonomy_command)
        self.node._autonomy_command_callback(old)
        self.assertIsNone(self.node.autonomy_command)
        new = harness.TwistStamped()
        new.header.stamp = self.now.to_msg()
        self.node._autonomy_command_callback(new)
        self.assertIs(self.node.autonomy_command, new)

    def test_entering_direct_mode_stops_through_existing_velocity_path(self):
        self.request(harness.MODES.AUTO_DIRECT)
        self.node._control()
        self.assertTrue(self.node.cmd_vel_publisher.messages)
        self.assertFalse(self.node.direct_command_publisher.messages)
        self.advance(0.25)
        self.assertEqual(self.node.current_mode, harness.MODES.AUTO_DIRECT)

    def test_same_active_mode_can_recover_after_timeout(self):
        self.request()
        self.advance(10.0)
        self.request(harness.MODES.MANUAL)
        self.node._control()
        self.advance(0.25)
        self.assertFalse(self.node.transition_in_progress)
        self.assertEqual(self.node.current_mode, harness.MODES.MANUAL)

    def test_reselecting_current_mode_also_stops_and_verifies_standstill(self):
        self.node.last_output = [0.5, 0.0, 0.0]
        self.request(harness.MODES.MANUAL)
        self.advance(0.1, vx=0.3)
        self.assertTrue(self.node.transition_in_progress)
        self.assertFalse(self.states[-1][2])

    def test_conflicting_requests_cannot_select_two_modes_during_transition(self):
        self.request(harness.MODES.AUTO_DIRECT)
        accepted, started, _ = self.node._request_mode(harness.MODES.AUTO_VELOCITY)
        self.assertFalse(accepted)
        self.assertFalse(started)
        self.assertEqual(self.node.current_mode, harness.MODES.MANUAL)
        self.assertEqual(self.node.requested_mode, harness.MODES.AUTO_DIRECT)

    def test_topic_and_service_requests_share_stop_before_switch_path(self):
        for via_service in (False, True):
            with self.subTest(via_service=via_service):
                self.setUp()
                request = SimpleNamespace(header=SimpleNamespace(stamp=self.now.to_msg()),
                                          requested_mode=harness.MODES.AUTO_VELOCITY)
                if via_service:
                    response = self.node._set_mode_service(request, SimpleNamespace())
                    self.assertTrue(response.accepted)
                    self.assertTrue(response.transition_started)
                else:
                    self.node._mode_request_callback(request)
                self.assertEqual(self.node.current_mode, harness.MODES.MANUAL)
                self.advance(0.3, joint_velocity=0.2)
                self.assertTrue(self.node.transition_in_progress)
                self.advance(0.02)
                self.advance(0.25)
                self.assertEqual(self.node.current_mode, harness.MODES.AUTO_VELOCITY)
                self.assertFalse(self.node.transition_in_progress)

    def test_auto_direct_is_rejected_on_topic_but_available_through_service(self):
        request = SimpleNamespace(header=SimpleNamespace(stamp=self.now.to_msg()),
                                  requested_mode=harness.MODES.AUTO_DIRECT)
        self.node._mode_request_callback(request)
        self.assertFalse(self.node.transition_in_progress)
        self.assertEqual(self.node.current_mode, harness.MODES.MANUAL)
        response = self.node._set_mode_service(request, SimpleNamespace())
        self.assertTrue(response.accepted)
        self.assertTrue(response.transition_started)
        self.node._control()
        self.advance(0.25)
        self.assertEqual(self.node.current_mode, harness.MODES.AUTO_DIRECT)

    def test_all_mode_pairs_require_standstill_before_switching(self):
        for old in (harness.MODES.MANUAL, harness.MODES.AUTO_VELOCITY, harness.MODES.AUTO_DIRECT):
            for new in (harness.MODES.MANUAL, harness.MODES.AUTO_VELOCITY, harness.MODES.AUTO_DIRECT):
                with self.subTest(old=old, new=new):
                    self.setUp()
                    self.node.current_mode = old
                    self.request(new)
                    self.advance(0.3, vx=0.5)
                    self.assertEqual(self.node.current_mode, old)
                    self.assertTrue(self.node.transition_in_progress)
                    self.advance(0.02)
                    self.advance(0.25)
                    self.assertEqual(self.node.current_mode, new)
                    self.assertFalse(self.node.transition_in_progress)

    def test_request_discards_buffered_commands_without_discarding_stop_ramp(self):
        self.node.manual_command = self.node.autonomy_command = harness.TwistStamped()
        self.node.autonomy_joint_command = harness.JointState()
        self.node.last_output = [0.5, 0.0, 0.0]
        self.request(harness.MODES.AUTO_DIRECT)
        self.assertIsNone(self.node.manual_command)
        self.assertIsNone(self.node.autonomy_command)
        self.assertIsNone(self.node.autonomy_joint_command)
        self.assertEqual(self.node.last_output, [0.5, 0.0, 0.0])


if __name__ == '__main__':
    unittest.main()
