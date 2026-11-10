"""Absolute manual steering envelopes at the final supervisor output."""
import math
import unittest

import test_enable_gate as harness
import test_four_modules as four_modules
import test_manual_steering as manual


class ManualSteeringLimitsTest(unittest.TestCase):
    def setUp(self):
        self.fixture = manual.ManualSteeringTest()
        self.fixture.setUp()
        self.node = self.fixture.node
        self.now = self.fixture.now

    def candidate(self, angles, *, crab=True, positions=None, purpose=None):
        msg = self.fixture.fixture.joints()
        msg.position[:2] = angles
        if positions is not None:
            self.node.joint_state.position[:2] = positions
        self.node.velocity_ledger[msg.header.stamp] = (
            self.node.authority_generation,
            harness.WheelModuleCommandArray.MOTION if purpose is None else purpose,
            self.now.to_msg(), False, crab)
        self.node._kinematics_candidate_callback(msg)
        return self.node.final_command_publisher.messages[-1]

    def test_held_crab_stops_steering_at_both_90_degree_limits(self):
        for direction in (-1, 1):
            angle = direction * math.pi / 2
            for _ in range(100):
                output = self.candidate([angle, angle], positions=[angle, angle])
                self.assertEqual([m.steering_position for m in output.modules], [angle, angle])
                self.assertEqual([m.steering_profile_velocity for m in output.modules], [0.0, 0.0])

    def test_held_ackermann_stops_steering_at_both_45_degree_limits(self):
        for direction in (-1, 1):
            limit = direction * math.pi / 4
            for _ in range(100):
                output = self.candidate([direction * 1.2] * 2, crab=False,
                                        positions=[limit] * 2)
                self.assertEqual([m.steering_position for m in output.modules], [limit] * 2)
                self.assertEqual([m.steering_profile_velocity for m in output.modules], [0.0] * 2)

    def test_full_and_half_turn_candidates_never_escape_absolute_bounds(self):
        for crab, limit in ((True, math.pi / 2), (False, math.pi / 4)):
            for turns in range(-20, 21):
                for delta in (-1.2, -math.pi / 2, 0.0, math.pi / 2, 1.2):
                    output = self.candidate([turns * math.pi + delta] * 2, crab=crab,
                                            positions=[turns * math.pi] * 2)
                    self.assertTrue(all(abs(m.steering_position) <= limit for m in output.modules))

    def test_equivalent_half_turn_reverses_traction(self):
        # Disable the separate virtual alignment gate to inspect the projection.
        self.node.manual_stationary_steering_enabled = False
        output = self.candidate([math.pi + 0.2, -math.pi - 0.1], positions=[0.2, -0.1])
        self.assertAlmostEqual(output.modules[0].steering_position, 0.2)
        self.assertAlmostEqual(output.modules[1].steering_position, -0.1)
        self.assertAlmostEqual(output.modules[0].traction_velocity, -2.0)
        self.assertAlmostEqual(output.modules[1].traction_velocity, 3.0)

    def test_sideways_equivalence_holds_the_existing_endpoint(self):
        for direction in (-1, 1):
            angle = direction * math.pi / 2
            output = self.candidate([-angle, -angle], positions=[angle, angle])
            self.assertEqual([m.steering_position for m in output.modules], [angle, angle])
            self.assertEqual([m.steering_profile_velocity for m in output.modules], [0.0] * 2)

    def test_mount_rounding_near_90_does_not_trigger_a_half_turn(self):
        for direction in (-1, 1):
            angle = direction * math.pi / 2
            for error in (-0.0001, 0.0001):
                output = self.candidate([-angle + error] * 2, positions=[angle] * 2)
                self.assertEqual([m.steering_position for m in output.modules], [angle] * 2)
                self.assertEqual([m.steering_profile_velocity for m in output.modules], [0.0] * 2)

    def test_reverse_input_moves_away_from_limit(self):
        for crab, limit in ((True, math.pi / 2), (False, math.pi / 4)):
            output = self.candidate([-0.3, 0.3], crab=crab, positions=[limit, -limit])
            self.assertEqual([m.steering_position for m in output.modules], [-0.3, 0.3])
            self.assertEqual([m.steering_profile_velocity for m in output.modules], [1.0] * 2)

    def test_stationary_presteering_does_not_follow_multi_turn_feedback(self):
        for direction in (-1, 1):
            self.setUp()
            self.fixture.request.position[:2] = [direction * 1.2, -direction * 1.2]
            self.node.joint_state.position[:2] = [direction * 2 * math.pi] * 2
            self.fixture.accept()
            output = self.fixture.candidate()
            self.assertEqual([m.steering_position for m in output.modules],
                             [direction * math.pi / 4, -direction * math.pi / 4])
            self.assertEqual([m.traction_velocity for m in output.modules], [0.0] * 2)

    def test_saturation_never_increases_traction_speed(self):
        self.node.manual_stationary_steering_enabled = False
        output = self.candidate([1.2, -1.2], crab=False)
        self.assertLessEqual(abs(output.modules[0].traction_velocity), 2.0)
        self.assertLessEqual(abs(output.modules[1].traction_velocity), 3.0)

    def test_autonomy_and_stop_candidates_retain_their_existing_policy(self):
        msg = self.fixture.fixture.joints()
        msg.position[:2] = [2.0, -2.0]
        for mode, purpose in (
                (harness.MODES.AUTONOMOUS, harness.WheelModuleCommandArray.MOTION),
                (harness.MODES.MANUAL, harness.WheelModuleCommandArray.CONTROLLED_STOP),
                (harness.MODES.MANUAL, harness.WheelModuleCommandArray.HOLD)):
            self.node.selected_mode = mode
            entry = (7, purpose, self.now.to_msg(), False, False)
            self.assertIs(self.node._bounded_manual_candidate(msg, entry, self.now), msg)

    def test_all_four_module_joint_pairs_are_bounded_regardless_of_wire_order(self):
        platform = four_modules.FourModuleSupervisorTest()
        platform.setUp()
        node = platform.node
        node.selected_mode = harness.MODES.MANUAL
        msg = harness.JointState()
        msg.name = node.expected_steering_joint_names + node.expected_traction_joint_names
        msg.position = [2 * math.pi + 1.2] * 4 + [0.0] * 4
        msg.velocity = [1.0] * 4 + [2.0] * 4
        msg.name.reverse()
        msg.position.reverse()
        msg.velocity.reverse()
        entry = (7, harness.WheelModuleCommandArray.MOTION, platform.now.to_msg(), False, False)
        output = node._bounded_manual_candidate(msg, entry, platform.now)
        indices = {name: i for i, name in enumerate(output.name)}
        for name in node.expected_steering_joint_names:
            self.assertAlmostEqual(output.position[indices[name]], math.pi / 4)
        self.assertEqual(msg.position, [0.0] * 4 + [2 * math.pi + 1.2] * 4)


if __name__ == '__main__':
    unittest.main()
