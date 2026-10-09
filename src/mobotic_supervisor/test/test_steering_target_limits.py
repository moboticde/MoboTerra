"""CAN position bounds in the actual supervisor methods, without a ROS runtime."""
import copy
import math
from pathlib import Path
import re
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'mobotic_config'))
from mobotic_config.configuration import Configuration

import test_enable_gate as harness
import test_four_modules as four_modules

representable = harness.namespace['steering_position_representable']
MIN_TICKS, MAX_TICKS = -(2**31), 2**31 - 1


def angle_for_ticks(ticks, resolution=4096.0):
    return ticks * (2.0 * math.pi) / resolution


class SteeringTargetLimitsTest(unittest.TestCase):
    def setUp(self):
        self.platform = four_modules.FourModuleSupervisorTest()
        self.platform.setUp()
        self.node = self.platform.node
        self.node.max_direct_steering_velocity = 4.32
        self.node.max_direct_traction_velocity = 11.12
        self.node.max_direct_traction_current = 4.0
        self.node.control_period = 0.02
        self.node.motion_inputs_ready = True

    def command(self):
        msg = harness.JointState()
        msg.header.stamp = self.platform.now.to_msg()
        msg.name = self.node.expected_steering_joint_names + self.node.expected_traction_joint_names
        msg.position = [0.1] * 4 + [0.0] * 4
        msg.velocity = [1.0] * 4 + [2.0] * 4
        msg.effort = [0.0] * 8
        return msg

    def test_normal_and_multi_turn_angles_are_not_wrapped(self):
        for angle in (0.0, -0.1, math.pi / 2, -math.pi / 2, 20 * math.pi):
            with self.subTest(angle=angle):
                self.assertTrue(representable(angle, 4096.0))
        msg = self.command()
        msg.position[0] = 20 * math.pi
        self.assertTrue(self.node._joint_command_valid(msg))
        self.node._publish_direct(msg, self.platform.now)
        self.assertEqual(self.node.direct_command_publisher.messages[-1].position[0], 20 * math.pi)

    def test_signed_integer_and_rounding_boundaries(self):
        for resolution in (1.0, 4096.0, 8192.0, 1_000_000.0):
            for ticks, expected in (
                (MIN_TICKS, True), (MAX_TICKS, True),
                (MIN_TICKS - 0.25, True), (MAX_TICKS + 0.25, True),
                (MIN_TICKS - 0.75, False), (MAX_TICKS + 0.75, False),
                (MIN_TICKS - 1, False), (MAX_TICKS + 1, False),
            ):
                with self.subTest(resolution=resolution, ticks=ticks):
                    self.assertEqual(representable(angle_for_ticks(ticks, resolution), resolution), expected)

    def test_invalid_and_intermediate_overflow_values(self):
        for angle in (math.nan, math.inf, -math.inf, sys.float_info.max, -sys.float_info.max, 1e100, -1e100):
            with self.subTest(angle=angle):
                self.assertFalse(representable(angle, 4096.0))
        for resolution in (0.0, -1.0, math.nan, math.inf):
            self.assertFalse(representable(0.1, resolution))
        self.assertFalse(representable(10.0, sys.float_info.max))

    def test_any_unsafe_steering_joint_rejects_the_whole_direct_command(self):
        for mode in ('velocity', 'current'):
            self.node.direct_traction_control_mode = mode
            for index in range(4):
                for value in (1e100, -1e100, angle_for_ticks(MAX_TICKS + 1)):
                    with self.subTest(mode=mode, index=index, value=value):
                        msg = self.command()
                        msg.position[index] = value
                        self.assertFalse(self.node._joint_command_valid(msg))

    def test_joint_name_order_and_each_encoder_resolution(self):
        self.node.direct_steering_encoder_resolutions = [1.0, 4096.0, 8192.0, 65536.0]
        msg = self.command()
        msg.position[:4] = [angle_for_ticks(MAX_TICKS, resolution)
                            for resolution in self.node.direct_steering_encoder_resolutions]
        msg.name.reverse(); msg.position.reverse(); msg.velocity.reverse(); msg.effort.reverse()
        self.assertTrue(self.node._joint_command_valid(msg))
        bad_index = msg.name.index(self.node.expected_steering_joint_names[-1])
        msg.position[bad_index] = angle_for_ticks(MAX_TICKS + 1, 65536.0)
        self.assertFalse(self.node._joint_command_valid(msg))

    def test_invalid_replacement_clears_cached_command_and_stops(self):
        for mode in ('velocity', 'current'):
            with self.subTest(mode=mode):
                self.setUp()
                node = self.node
                node.current_mode = harness.MODES.AUTO_DIRECT
                node.direct_traction_control_mode = mode
                del node._source_gate  # Use the real selected-source gate.
                good = self.command()
                node._autonomy_joint_callback(good)
                self.assertIs(node.autonomy_joint_command, good)
                node._control()
                bad = copy.deepcopy(good)
                bad.position[3] = 1e100
                node._autonomy_joint_callback(bad)
                self.assertIsNone(node.autonomy_joint_command)
                self.assertIsNone(node.autonomy_joint_received)
                node._control()
                stop = node.direct_command_publisher.messages[-1]
                self.assertEqual(stop.velocity, [0.0] * 8)
                self.assertEqual(stop.effort, [0.0] * 8)
                self.assertEqual(stop.position[:4], node.joint_state.position[:4])
                self.assertTrue(node.module_command_publisher.messages[-1].enable)

    def test_unrepresentable_feedback_cannot_be_used_for_hold_position(self):
        msg = copy.deepcopy(self.node.joint_state)
        msg.position[3] = 1e100
        self.assertFalse(self.node._joint_feedback_valid(msg))
        self.node._joint_state_callback(msg)
        self.assertIsNone(self.node.joint_state)
        self.assertIsNone(self.node.joint_state_received)

    def test_invalid_resolution_configuration_fails_closed(self):
        for resolutions in ([4096.0], [4096.0] * 3, [4096.0] * 5,
                            [0.0] * 4, [-1.0] * 4, [math.nan] * 4, [math.inf] * 4):
            with self.subTest(resolutions=resolutions):
                self.node.direct_steering_encoder_resolutions = resolutions
                with self.assertRaisesRegex(ValueError, 'direct_steering_encoder_resolutions'):
                    self.node._validate_parameters()
                self.assertFalse(self.node._joint_command_valid(self.command()))

    def test_supplied_supervisor_and_driver_resolutions_match(self):
        cfg = Configuration()
        driver, supervisor = cfg.parameters('driver'), cfg.parameters('supervisor')
        driver_resolutions = dict(zip(driver['drives.names'], driver['drives.resolutions']))
        names = supervisor['expected_steering_joint_names']
        resolutions = supervisor['direct_steering_encoder_resolutions']
        self.assertEqual(len(names), len(resolutions))
        self.assertEqual(resolutions, [driver_resolutions[name] for name in names])


if __name__ == '__main__':
    unittest.main()
