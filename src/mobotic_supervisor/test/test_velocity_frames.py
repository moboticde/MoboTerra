"""Velocity candidates must already be expressed in the configured body frame."""

import unittest

import test_enable_gate as harness
import test_command_recovery as recovery_harness


class VelocityFrameTest(unittest.TestCase):
    def setUp(self):
        self.platform = recovery_harness.CommandRecoveryTest()
        self.platform.setUp()
        self.node = self.platform.node

    def velocity(self, frame='base_link'):
        command = self.platform.velocity()
        command.header.frame_id = frame
        return command

    def test_wrong_or_missing_autonomy_frame_is_rejected(self):
        for frame in ('map', 'odom', '', '/base_link', ' base_link', 'base_link '):
            with self.subTest(frame=frame):
                self.setUp()
                self.node._autonomy_command_callback(self.velocity(frame))
                self.assertIsNone(self.node.autonomy_command)
                self.assertIsNone(self.node.autonomy_command_received)
                self.platform.advance(0.02)
                self.node._control()
                self.assertEqual(self.node.cmd_vel_publisher.messages[-1].twist.linear.x, 0.0)

    def test_matching_autonomy_frame_preserves_planar_components(self):
        command = self.velocity()
        command.twist.linear.x = 0.001
        command.twist.linear.y = 0.002
        command.twist.angular.z = 0.003
        self.node._autonomy_command_callback(command)
        self.assertIs(self.node.autonomy_command, command)
        self.platform.advance(0.02)
        self.node._control()
        output = self.node.cmd_vel_publisher.messages[-1]
        self.assertEqual(output.header.frame_id, 'base_link')
        self.assertEqual(output.twist.linear.x, 0.001)
        self.assertEqual(output.twist.linear.y, 0.002)
        self.assertEqual(output.twist.angular.z, 0.003)

    def test_wrong_frame_clears_previous_autonomy_command_until_valid_input(self):
        self.node._autonomy_command_callback(self.velocity())
        self.platform.advance(0.02)
        self.node._control()
        self.assertGreater(self.node.cmd_vel_publisher.messages[-1].twist.linear.x, 0.0)
        self.node._autonomy_command_callback(self.velocity('map'))
        self.assertIsNone(self.node.autonomy_command)
        self.assertIsNone(self.node.autonomy_command_received)
        self.node._control()
        self.assertEqual(self.node.cmd_vel_publisher.messages[-1].twist.linear.x, 0.0)
        self.assertTrue(self.node.module_command_publisher.messages[-1].enable)
        self.node._autonomy_command_callback(self.velocity())
        self.platform.advance(0.02)
        self.node._control()
        self.assertGreater(self.node.cmd_vel_publisher.messages[-1].twist.linear.x, 0.0)

    def test_manual_commands_obey_same_frame_contract(self):
        self.node.current_mode = harness.MODES.MANUAL
        command = self.velocity()
        self.node._manual_command_callback(command)
        self.assertIs(self.node.manual_command, command)
        self.platform.advance(0.02)
        self.node._control()
        self.assertGreater(self.node.cmd_vel_publisher.messages[-1].twist.linear.x, 0.0)
        for frame in ('map', 'odom', ''):
            with self.subTest(frame=frame):
                self.node._manual_command_callback(self.velocity(frame))
                self.assertIsNone(self.node.manual_command)
                self.assertIsNone(self.node.manual_command_received)
                self.node._control()
                self.assertEqual(self.node.cmd_vel_publisher.messages[-1].twist.linear.x, 0.0)

    def test_configured_frame_is_used_instead_of_hardcoded_base_link(self):
        self.node.output_frame_id = 'base_footprint'
        command = self.velocity('base_footprint')
        self.node._autonomy_command_callback(command)
        self.assertIs(self.node.autonomy_command, command)
        self.platform.advance(0.02)
        self.node._control()
        self.assertEqual(self.node.cmd_vel_publisher.messages[-1].header.frame_id, 'base_footprint')
        self.node._autonomy_command_callback(self.velocity('base_link'))
        self.assertIsNone(self.node.autonomy_command)

    def test_source_gate_rechecks_cached_velocity_frame(self):
        for mode, callback_name in (
            (harness.MODES.MANUAL, '_manual_command_callback'),
            (harness.MODES.AUTO_VELOCITY, '_autonomy_command_callback'),
        ):
            with self.subTest(mode=mode):
                self.setUp()
                self.node.current_mode = mode
                command = self.velocity()
                getattr(self.node, callback_name)(command)
                command.header.frame_id = 'map'
                ready, reason = self.node._source_gate(self.platform.platform.now)
                self.assertFalse(ready)
                self.assertIn('frame mismatch', reason)
                self.node._control()
                self.assertEqual(self.node.cmd_vel_publisher.messages[-1].twist.linear.x, 0.0)

    def test_invalid_inactive_source_does_not_clear_selected_manual_command(self):
        self.node.current_mode = harness.MODES.MANUAL
        command = self.velocity()
        self.node._manual_command_callback(command)
        self.node._autonomy_command_callback(self.velocity('map'))
        self.assertIs(self.node.manual_command, command)
        self.platform.advance(0.02)
        self.node._control()
        self.assertGreater(self.node.cmd_vel_publisher.messages[-1].twist.linear.x, 0.0)

    def test_empty_or_untrimmed_configured_frame_is_invalid(self):
        self.node.control_period = 0.02
        for frame in ('', ' ', '\t', ' base_link', 'base_link '):
            with self.subTest(frame=frame):
                self.node.output_frame_id = frame
                with self.assertRaisesRegex(ValueError, 'output_frame_id'):
                    self.node._validate_parameters()
        self.node.output_frame_id = 'base_link'
        self.node._validate_parameters()


if __name__ == '__main__':
    unittest.main()
