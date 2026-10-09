"""Receipt time must not extend the original timestamp's validity window."""

import copy
from types import SimpleNamespace
import unittest

import test_enable_gate as harness
import test_command_recovery as recovery_harness


class MessageFreshnessTest(unittest.TestCase):
    def setUp(self):
        self.platform = recovery_harness.CommandRecoveryTest()
        self.platform.setUp()
        self.node = self.platform.node
        # Readiness was established before these delayed commands were generated.
        self.node.command_accept_after = harness.Time(9.5)

    def delayed_command(self, mode):
        self.node.current_mode = mode
        stamp = harness.Time(9.76).to_msg()  # 240 ms old at receipt.
        if mode == harness.MODES.AUTO_DIRECT:
            msg = self.platform.joints(stamp)
            self.node._autonomy_joint_callback(msg)
            self.assertIs(self.node.autonomy_joint_command, msg)
        elif mode == harness.MODES.MANUAL:
            msg = self.platform.velocity(stamp)
            self.node._manual_command_callback(msg)
            self.assertIs(self.node.manual_command, msg)
        else:
            msg = self.platform.velocity(stamp)
            self.node._autonomy_command_callback(msg)
            self.assertIs(self.node.autonomy_command, msg)
        return msg

    def test_original_command_stamp_expires_before_receipt_timeout(self):
        for mode, traction_mode in (
            (harness.MODES.MANUAL, 'velocity'),
            (harness.MODES.AUTO_VELOCITY, 'velocity'),
            (harness.MODES.AUTO_DIRECT, 'velocity'),
            (harness.MODES.AUTO_DIRECT, 'current'),
        ):
            with self.subTest(mode=mode, traction_mode=traction_mode):
                self.setUp()
                self.node.direct_traction_control_mode = traction_mode
                self.delayed_command(mode)
                self.assertTrue(self.node._source_gate(self.platform.platform.now)[0])
                self.platform.advance(0.02)
                self.node._control()
                self.assertFalse(self.node._source_gate(self.platform.platform.now)[0])
                if mode == harness.MODES.AUTO_DIRECT:
                    stop = self.node.direct_command_publisher.messages[-1]
                    self.assertEqual(stop.velocity, [0.0] * 4)
                    self.assertEqual(stop.effort, [0.0] * 4)
                else:
                    self.assertEqual(self.node.cmd_vel_publisher.messages[-1].twist.linear.x, 0.0)
                self.assertTrue(self.node.module_command_publisher.messages[-1].enable)

    def test_manual_state_original_stamp_also_expires(self):
        self.node.current_mode = harness.MODES.MANUAL
        self.node._manual_command_callback(self.platform.velocity())
        state = copy.deepcopy(self.node.manual_state)
        state.header.stamp = harness.Time(9.76).to_msg()
        original_stamp = state.header.stamp
        self.node._manual_state_callback(state)
        self.platform.advance(0.02)
        # Advance refreshes the normal state; restore the delayed original stamp.
        self.node.manual_state.header.stamp = original_stamp
        ready, reason = self.node._source_gate(self.platform.platform.now)
        self.assertFalse(ready)
        self.assertIn('manual-control state stale', reason)

    def test_readiness_original_stamp_expiry_disables_and_clears_motion(self):
        for name, callback, timeout in (
            ('safety_state', '_safety_callback', 0.3),
            ('safety_io_state', '_safety_io_callback', 0.3),
            ('battery_state', '_battery_callback', 0.6),
        ):
            with self.subTest(input=name):
                self.setUp()
                self.node._autonomy_command_callback(self.platform.velocity())
                delayed = copy.deepcopy(getattr(self.node, name))
                delayed.header.stamp = harness.Time(10.0 - timeout + 0.01).to_msg()
                original_stamp = delayed.header.stamp
                getattr(self.node, callback)(delayed)
                self.assertTrue(self.node._enable_gate(self.platform.platform.now)[0])
                self.platform.advance(0.02)
                delayed.header.stamp = original_stamp
                setattr(self.node, name, delayed)
                self.node._control()
                self.assertFalse(self.node.module_command_publisher.messages[-1].enable)
                self.assertIsNone(self.node.autonomy_command)
                self.assertEqual(self.node.cmd_vel_publisher.messages[-1].twist.linear.x, 0.0)

    def test_wheel_report_stamp_expiry_blocks_motion_not_enable(self):
        delayed = copy.deepcopy(self.node.wheel_status)
        delayed.header.stamp = harness.Time(9.71).to_msg()
        self.node._wheel_status_callback(delayed)
        self.node._autonomy_command_callback(self.platform.velocity())
        self.platform.advance(0.02)
        self.node.wheel_status = delayed
        self.node._control()
        self.assertTrue(self.node.module_command_publisher.messages[-1].enable)
        self.assertIsNone(self.node.autonomy_command)
        self.assertEqual(self.node.cmd_vel_publisher.messages[-1].twist.linear.x, 0.0)

    def test_direct_joint_feedback_stamp_expiry_clears_motion(self):
        self.node.current_mode = harness.MODES.AUTO_DIRECT
        delayed = copy.deepcopy(self.node.joint_state)
        delayed.header.stamp = harness.Time(9.71).to_msg()
        self.node._joint_state_callback(delayed)
        self.node._autonomy_joint_callback(self.platform.joints())
        self.platform.advance(0.02)
        self.node.joint_state = delayed
        self.node._control()
        self.assertIsNone(self.node.autonomy_joint_command)
        self.assertEqual(self.node.direct_command_publisher.messages[-1].velocity, [0.0] * 4)
        self.assertTrue(self.node.module_command_publisher.messages[-1].enable)

    def test_timestamp_and_receipt_boundaries(self):
        now = harness.Time(10.0)
        for stamp, received, expected in (
            (9.75, 10.0, True),   # Exact original age limit.
            (9.749, 10.0, False),
            (10.0, 9.75, True),   # Exact receipt age limit.
            (10.0, 9.749, False),
            (10.1, 10.0, True),   # Retain existing 100 ms header skew allowance.
            (10.101, 10.0, False),
            (0.0, 10.0, False),
            (10.0, 10.001, False),  # Backward clock: negative receipt age.
        ):
            with self.subTest(stamp=stamp, received=received):
                message = SimpleNamespace(header=SimpleNamespace(stamp=harness.Time(stamp).to_msg()))
                self.assertEqual(
                    self.node._message_fresh(message, harness.Time(received), 0.25, now), expected,
                )
        self.assertFalse(self.node._message_fresh(None, now, 0.25, now))
        self.assertFalse(self.node._message_fresh(message, None, 0.25, now))

    def test_refreshing_receipt_cannot_keep_replayed_command_alive(self):
        old = self.delayed_command(harness.MODES.AUTO_VELOCITY)
        self.platform.advance(0.02)
        self.node.autonomy_command_received = self.platform.platform.now
        self.node._autonomy_command_callback(old)  # Expired original is rejected.
        self.node._control()
        self.assertEqual(self.node.cmd_vel_publisher.messages[-1].twist.linear.x, 0.0)
        self.node._autonomy_command_callback(self.platform.velocity())
        self.platform.advance(0.02)
        self.node._control()
        self.assertGreater(self.node.cmd_vel_publisher.messages[-1].twist.linear.x, 0.0)

    def test_receipt_clock_reversal_disables_even_with_valid_header(self):
        self.node.safety_received = harness.Time(10.01)
        ready, reason = self.node._enable_gate(self.platform.platform.now)
        self.assertFalse(ready)
        self.assertIn('safety state stale', reason)

    def test_standstill_still_requires_unexpired_original_feedback(self):
        self.node._request_mode(harness.MODES.MANUAL)
        self.node.transition_started_at = harness.Time(9.5)
        self.node.measured_velocity.header.stamp = harness.Time(9.71).to_msg()
        self.platform.platform.now = harness.Time(10.02)
        self.node.measured_velocity_received = self.platform.platform.now
        ready, reason = self.node._standstill_gate(self.platform.platform.now)
        self.assertFalse(ready)
        self.assertIn('vehicle velocity feedback timestamp', reason)


if __name__ == '__main__':
    unittest.main()
