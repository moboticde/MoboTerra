"""Stationary manual steering must retain the existing final motion authority."""
import copy
import math
import unittest

import test_enable_gate as harness
import test_interface_update as final


class ManualSteeringTest(unittest.TestCase):
    def setUp(self):
        self.fixture = final.InterfaceUpdateTest()
        self.fixture.setUp()
        self.node = self.fixture.node
        self.now = self.fixture.fixture.now
        self.node.manual_stationary_steering_enabled = True
        self.node.manual_steering_requests = {}
        self.request = harness.JointState()
        self.request.header.stamp = self.now.to_msg()
        self.request.header.frame_id = 'base_link'
        self.request.name = ['front_steering', 'rear_steering', 'front_traction', 'rear_traction']
        self.request.position = [0.4, -0.4, 0.0, 0.0]
        self.request.velocity = [1.0, 1.0, 0.0, 0.0]

    def accept(self):
        self.node._manual_steering_callback(self.request)
        self.assertIn(self.request.header.stamp, self.node.manual_steering_requests)

    def candidate(self, *, zero=True, source_stamp=None):
        msg = self.fixture.joints()
        msg.velocity[2:] = [0.0, 0.0]
        self.node.velocity_ledger[msg.header.stamp] = (
            self.node.authority_generation, harness.WheelModuleCommandArray.MOTION,
            self.request.header.stamp if source_stamp is None else source_stamp, zero)
        self.node._kinematics_candidate_callback(msg)
        return self.node.final_command_publisher.messages[-1]

    def test_fresh_stationary_request_changes_angles_with_zero_traction(self):
        self.accept()
        output = self.candidate()
        self.assertEqual([m.steering_position for m in output.modules], [0.4, -0.4])
        self.assertEqual([m.traction_velocity for m in output.modules], [0.0, 0.0])
        self.assertEqual(output.supervisor_instance, 'test-supervisor')
        self.assertEqual(output.authority_generation, 7)
        self.assertEqual(output.source_stamp, self.request.header.stamp)

    def test_invalid_or_driving_requests_are_rejected(self):
        for field, value in (
                ('velocity', [1.0, 1.0, 0.1, 0.0]),
                ('velocity', [-1.0, 1.0, 0.0, 0.0]),
                ('velocity', [5.0, 1.0, 0.0, 0.0]),
                ('position', [math.nan, 0.0, 0.0, 0.0]),
                ('position', [0.4, -0.4, 1.0, 0.0]),
                ('name', ['front_steering'] * 4)):
            with self.subTest(field=field, value=value):
                self.setUp()
                setattr(self.request, field, value)
                self.node._manual_steering_callback(self.request)
                self.assertFalse(self.node.manual_steering_requests)

    def test_disabled_hardware_path_rejects_steering_source(self):
        self.node.manual_stationary_steering_enabled = False
        self.node._manual_steering_callback(self.request)
        self.assertFalse(self.node.manual_steering_requests)

    def test_stop_or_other_source_gate_cannot_be_bypassed(self):
        self.accept()
        self.node._source_gate = lambda now: (False, 'STOP')
        msg = self.fixture.joints()
        self.node.velocity_ledger[msg.header.stamp] = (7, harness.WheelModuleCommandArray.MOTION,
                                                     self.request.header.stamp, True)
        self.node._kinematics_candidate_callback(msg)
        self.assertFalse(self.node.final_command_publisher.messages)

    def test_moving_or_stale_inputs_keep_normal_kinematics_angles(self):
        for case in ('moving_command', 'moving_traction', 'moving_body', 'stale_request',
                     'different_sample', 'autonomy', 'missing_feedback'):
            with self.subTest(case=case):
                self.setUp()
                self.accept()
                if case == 'moving_traction': self.node.joint_state.velocity[2] = 0.2
                if case == 'moving_body': self.node.measured_velocity.twist.linear.x = 0.1
                if case == 'stale_request':
                    self.node.manual_steering_requests[self.request.header.stamp] = (self.request, harness.Time(9.0))
                if case == 'autonomy': self.node.selected_mode = harness.MODES.AUTO_VELOCITY
                if case == 'missing_feedback': self.node.measured_velocity = None
                output = self.candidate(zero=case != 'moving_command',
                    source_stamp=self.request.header.stamp - 1 if case == 'different_sample' else None)
                self.assertEqual([m.steering_position for m in output.modules], [0.2, -0.1])

    def test_steering_motion_does_not_interrupt_zero_traction_presteering(self):
        self.accept()
        self.node.joint_state.velocity[0] = 1.0
        output = self.candidate()
        self.assertAlmostEqual(output.modules[0].steering_position, 0.4)
        self.assertEqual(output.modules[0].traction_velocity, 0.0)

    def test_readiness_loss_discards_buffered_steering(self):
        self.accept()
        self.node._invalidate_motion_commands(self.now)
        self.assertFalse(self.node.manual_steering_requests)

    def test_newer_request_does_not_hide_current_velocity_source_sample(self):
        old_stamp = self.request.header.stamp
        self.accept()
        self.now = harness.Time(self.now.nanoseconds / 1e9 + 0.01)
        self.fixture.fixture.now = self.now
        newer = copy.deepcopy(self.request)
        newer.header.stamp = self.now.to_msg()
        newer.position[0] = 0.8
        self.node._manual_steering_callback(newer)
        output = self.candidate(source_stamp=old_stamp)
        self.assertAlmostEqual(output.modules[0].steering_position, 0.4)

    def test_zero_yaw_translation_waits_for_all_wheels_to_align(self):
        msg = self.fixture.joints()
        entry = (7, harness.WheelModuleCommandArray.MOTION, self.now.to_msg(), False, True)
        self.node.last_output = [0.0, 0.5, 0.0]
        output = self.node._aligned_manual_candidate(msg, entry, self.now)
        self.assertEqual(output.position, msg.position)
        self.assertEqual(output.velocity[2:], [0.0, 0.0])
        self.assertEqual(self.node.last_output, [0.0, 0.0, 0.0])
        self.node.joint_state.position[:2] = msg.position[:2]
        output = self.node._aligned_manual_candidate(msg, entry, self.now)
        self.assertEqual(output.velocity, msg.velocity)
        self.node.joint_state.position[1] += 0.1
        output = self.node._aligned_manual_candidate(msg, entry, self.now)
        self.assertEqual(output.velocity[2:], [0.0, 0.0])

    def test_alignment_gate_is_not_enabled_for_hardware_or_turning_commands(self):
        msg = self.fixture.joints()
        for enabled, yaw_zero in ((False, True), (True, False)):
            self.node.manual_stationary_steering_enabled = enabled
            entry = (7, harness.WheelModuleCommandArray.MOTION, self.now.to_msg(), False, yaw_zero)
            self.assertIs(self.node._aligned_manual_candidate(msg, entry, self.now), msg)


if __name__ == '__main__':
    unittest.main()
