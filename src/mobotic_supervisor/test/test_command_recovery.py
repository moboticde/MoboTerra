"""Source commands must be new after motion hardware readiness recovers."""
import copy
from types import SimpleNamespace
import unittest

import test_enable_gate as harness
import test_mode_transition as transition_harness


class CommandRecoveryTest(unittest.TestCase):
    def setUp(self):
        self.platform = transition_harness.ModeTransitionTest()
        self.platform.setUp()
        self.node = self.platform.node
        del self.node._source_gate
        self.node.current_mode = harness.MODES.AUTO_VELOCITY
        self.node.direct_traction_control_mode = 'velocity'
        self.node.max_direct_steering_velocity = 4.32
        self.node.max_direct_traction_velocity = 11.12
        self.node.max_direct_traction_current = 4.0
        self.node.manual_state = SimpleNamespace(
            header=SimpleNamespace(stamp=self.platform.now.to_msg()),
            connected=True, deadman_pressed=True, command_active=True,
            status_message='manual active',
        )
        self.node.manual_state_received = self.platform.now

    def advance(self, seconds):
        self.platform.now = harness.Time(
            self.platform.now.nanoseconds / 1e9 + seconds
        )
        self.platform.feedback()
        self.node.manual_state_received = self.platform.now
        self.node.manual_state.header.stamp = self.platform.now.to_msg()

    def velocity(self, stamp=None):
        command = harness.TwistStamped()
        command.header.stamp = (
            self.platform.now.to_msg() if stamp is None else stamp
        )
        command.twist.linear.x = 0.5
        return command

    def test_manual_motion_without_physical_deadman_still_requires_active_connected_source(self):
        self.node.current_mode = harness.MODES.MANUAL
        self.node.manual_state.deadman_pressed = False
        self.node._manual_command_callback(self.velocity())
        self.assertTrue(self.node._source_gate(self.platform.now)[0])
        self.advance(0.02)
        self.node._control()
        self.assertGreater(self.node.cmd_vel_publisher.messages[-1].twist.linear.x, 0)
        self.node.manual_state.command_active = False
        self.assertFalse(self.node._source_gate(self.platform.now)[0])
        self.node._control()
        self.assertEqual(self.node.cmd_vel_publisher.messages[-1].twist.linear.x, 0)
        self.node.manual_state.command_active = True
        self.node.manual_state.connected = False
        self.assertFalse(self.node._source_gate(self.platform.now)[0])

    def test_only_selected_source_is_forwarded_when_all_sources_publish(self):
        for mode in (harness.MODES.MANUAL, harness.MODES.AUTO_VELOCITY, harness.MODES.AUTO_DIRECT):
            with self.subTest(mode=mode):
                self.setUp()
                self.node.current_mode = mode
                manual, automatic, direct = self.velocity(), self.velocity(), self.joints()
                self.node._manual_command_callback(manual)
                self.node._autonomy_command_callback(automatic)
                self.node._autonomy_joint_callback(direct)
                outputs = []
                self.node._publish_velocity = lambda source, *args, **kwargs: outputs.append(('velocity', source))
                self.node._publish_direct = lambda source, *args: outputs.append(('direct', source))
                self.node._control()
                selected = {harness.MODES.MANUAL: ('velocity', manual),
                            harness.MODES.AUTO_VELOCITY: ('velocity', automatic),
                            harness.MODES.AUTO_DIRECT: ('direct', direct)}[mode]
                self.assertEqual(len(outputs), 1)
                self.assertEqual(outputs[0][0], selected[0])
                self.assertIs(outputs[0][1], selected[1])

    def joints(self, stamp=None):
        command = harness.JointState()
        command.header.stamp = (
            self.platform.now.to_msg() if stamp is None else stamp
        )
        command.name = (
            self.node.expected_steering_joint_names
            + self.node.expected_traction_joint_names
        )
        command.position = [0.7, -0.4, 0.0, 0.0]
        command.velocity = [1.0, 1.0, 2.0, 2.0]
        command.effort = [0.0, 0.0, 1.0, 1.0]
        return command

    def test_velocity_recovery_requires_new_original_source_command(self):
        for mode in (harness.MODES.MANUAL, harness.MODES.AUTO_VELOCITY):
            with self.subTest(mode=mode):
                self.setUp()
                node = self.node
                node.current_mode = mode
                callback = (node._manual_command_callback if mode == harness.MODES.MANUAL
                            else node._autonomy_command_callback)
                old = self.velocity()
                callback(old)
                self.advance(0.02)
                node._control()
                self.assertGreater(node.cmd_vel_publisher.messages[-1].twist.linear.x, 0)
                node.wheel_status.modules[0].feedback_fresh = False
                node._control()
                self.assertEqual(node.cmd_vel_publisher.messages[-1].twist.linear.x, 0)
                # Safety/battery still request enable; only motion is blocked.
                self.assertTrue(node.module_command_publisher.messages[-1].enable)
                self.advance(0.02)
                node._control()
                self.assertEqual(node.cmd_vel_publisher.messages[-1].twist.linear.x, 0)
                callback(old)  # Delayed/replayed pre-recovery input cannot restart.
                node._control()
                self.assertEqual(node.cmd_vel_publisher.messages[-1].twist.linear.x, 0)
                callback(self.velocity())
                self.advance(0.02)
                node._control()
                self.assertGreater(node.cmd_vel_publisher.messages[-1].twist.linear.x, 0)

    def test_direct_recovery_requires_new_joint_command_in_both_traction_modes(self):
        for traction_mode in ('velocity', 'current'):
            with self.subTest(traction_mode=traction_mode):
                self.setUp()
                node = self.node
                node.current_mode = harness.MODES.AUTO_DIRECT
                node.direct_traction_control_mode = traction_mode
                old = self.joints()
                node._autonomy_joint_callback(old)
                node._control()
                self.assertTrue(any(node.direct_command_publisher.messages[-1].velocity))
                node.wheel_status.modules[0].error_code = 1
                node._control()
                self.assertEqual(node.direct_command_publisher.messages[-1].velocity, [0.0] * 4)
                self.advance(0.02)
                node._control()
                stop = node.direct_command_publisher.messages[-1]
                self.assertEqual(stop.velocity, [0.0] * 4)
                self.assertEqual(stop.effort, [0.0] * 4)
                node._autonomy_joint_callback(old)
                self.assertIsNone(node.autonomy_joint_command)
                node._autonomy_joint_callback(self.joints())
                node._control()
                resumed = node.direct_command_publisher.messages[-1]
                self.assertEqual(resumed.velocity[2:] if traction_mode == 'velocity'
                                 else resumed.effort[2:], [2.0, 2.0] if traction_mode == 'velocity'
                                 else [1.0, 1.0])
                self.assertFalse(node.cmd_vel_publisher.messages)

    def test_commands_arriving_while_blocked_are_not_buffered(self):
        self.node.wheel_status.modules[0].feedback_fresh = False
        self.node._control()
        # Even a stamp slightly in the future cannot queue motion while blocked.
        stamp = harness.Time(10.09).to_msg()
        self.node._manual_command_callback(self.velocity(stamp))
        self.node._autonomy_command_callback(self.velocity(stamp))
        self.node._autonomy_joint_callback(self.joints(stamp))
        self.assertIsNone(self.node.manual_command)
        self.assertIsNone(self.node.autonomy_command)
        self.assertIsNone(self.node.autonomy_joint_command)
        self.advance(0.02)
        self.node._control()
        self.assertEqual(self.node.cmd_vel_publisher.messages[-1].twist.linear.x, 0)

    def test_readiness_loss_and_recovery_between_ticks_invalidate_commands(self):
        for input_name, bad_field, value, callback_name in (
            ('wheel_status', 'feedback_fresh', False, '_wheel_status_callback'),
            ('safety_state', 'motion_permitted', False, '_safety_callback'),
            ('safety_io_state', 'emergency_stop_active', True, '_safety_io_callback'),
            ('battery_state', 'system_ready', False, '_battery_callback'),
        ):
            with self.subTest(input=input_name):
                self.setUp()
                self.node._autonomy_command_callback(self.velocity())
                good = copy.deepcopy(getattr(self.node, input_name))
                good.header = SimpleNamespace(stamp=self.platform.now.to_msg())
                bad = copy.deepcopy(good)
                if input_name == 'wheel_status':
                    setattr(bad.modules[0], bad_field, value)
                else:
                    setattr(bad, bad_field, value)
                callback = getattr(self.node, callback_name)
                callback(bad)
                self.advance(0.01)
                good.header.stamp = self.platform.now.to_msg()
                callback(good)
                self.node._control()
                self.assertIsNone(self.node.autonomy_command)
                self.assertEqual(self.node.cmd_vel_publisher.messages[-1].twist.linear.x, 0)

    def test_direct_reduced_speed_recovery_requires_new_command(self):
        self.node.current_mode = harness.MODES.AUTO_DIRECT
        self.node._autonomy_joint_callback(self.joints())
        self.node.safety_state.warning_field_active = True
        self.node._control()
        self.node.safety_state.warning_field_active = False
        self.advance(0.02)
        self.node._control()
        self.assertEqual(self.node.direct_command_publisher.messages[-1].velocity, [0.0] * 4)
        self.assertIsNone(self.node.autonomy_joint_command)

    def test_direct_joint_feedback_recovery_requires_new_command(self):
        self.node.current_mode = harness.MODES.AUTO_DIRECT
        self.node._autonomy_joint_callback(self.joints())
        invalid = harness.JointState()
        invalid.header.stamp = self.platform.now.to_msg()
        self.node._joint_state_callback(invalid)
        self.advance(0.02)
        self.node._control()
        self.assertIsNone(self.node.autonomy_joint_command)
        self.assertEqual(self.node.direct_command_publisher.messages[-1].velocity, [0.0] * 4)

    def test_healthy_feedback_does_not_discard_active_commands(self):
        old = self.velocity()
        self.node._autonomy_command_callback(old)
        for _ in range(3):
            self.advance(0.02)
            self.node._control()
            self.assertIs(self.node.autonomy_command, old)
            self.assertGreater(self.node.cmd_vel_publisher.messages[-1].twist.linear.x, 0)

    def test_startup_ready_transition_discards_commands_queued_before_ready(self):
        self.node.wheel_status_received = None
        old = self.velocity()
        self.node._autonomy_command_callback(old)
        self.assertIsNone(self.node.autonomy_command)
        self.advance(0.02)
        self.node._control()
        self.assertEqual(self.node.cmd_vel_publisher.messages[-1].twist.linear.x, 0)


if __name__ == '__main__':
    unittest.main()
