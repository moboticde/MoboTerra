"""Real DDS regression checks; run in an isolated domain without a CAN bridge."""
import json
from pathlib import Path
import sys
import time
import unittest

# Prefer the checked-out provider when validating before installation.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from can_msgs.msg import Frame
from mobotic_interfaces.msg import OperatorState
from mobotic_manual_control.remdevice import RemdeviceControl


class RombTest(unittest.TestCase):
    def setUp(self):
        mapping = {'verified': True, 'node_id': 10,
            'axes': {role: {'channel': channel, 'zero': 0, 'full_scale': 255, 'invert': False}
                     for role, channel in [('speed', 3), ('steer', 2), ('crab', 4)]},
            'buttons': {'mode': {'signal': 'U5', 'active_high': False},
                        'boost': {'signal': 'SP3', 'active_high': True},
                        'reset': {'signal': 'SP4', 'active_high': True}}}
        rclpy.init(args=['--ros-args', '-p', 'remdevice_mapping:='+json.dumps(json.dumps(mapping)),
                        '-p', 'receiver_auto_recovery:=false',
                        '-p', 'receiver_can_overrun_advisory:=true'])
        self.operator = RemdeviceControl()
        self.operator.timer.cancel()
        self.bus = Node('operator_romb_test_bus')
        self.controls = self.bus.create_publisher(Frame, 'can_rx', 100)
        # The old combined reader is used automatically when checking the
        # original source, so the burst regression demonstrates its PDO loss.
        event_topic = 'can_events' if hasattr(self.operator, 'can_event_subscription') else 'can_rx'
        self.events = self.bus.create_publisher(Frame, event_topic, 100)
        self.states = []
        self.state_sub = self.bus.create_subscription(OperatorState, 'operator/state', self.states.append, 100)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.operator)
        self.executor.add_node(self.bus)
        self.spin_until(lambda: self.controls.get_subscription_count() and self.events.get_subscription_count(), 3)
        self.assertGreater(self.controls.get_subscription_count(), 0)
        self.assertGreater(self.events.get_subscription_count(), 0)

    def tearDown(self):
        self.executor.shutdown()
        self.operator.destroy_node()
        self.bus.destroy_node()
        rclpy.shutdown()

    def spin_until(self, predicate, seconds=.15):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=.001)
            if predicate():
                return True
        return False

    def frame(self, cob, data, stamp):
        msg = Frame()
        msg.id, msg.dlc = cob, len(data)
        msg.data = list(data)+[0]*(8-len(data))
        msg.header.stamp = Time(nanoseconds=stamp).to_msg()
        return msg

    def burst(self, *, age=.01):
        stamp = self.bus.get_clock().now().nanoseconds-int(age*1e9)
        self.controls.publish(self.frame(0x18a, [0x10, 0x10, 0, 0, 0, 0, 0, 0x38], stamp))
        self.controls.publish(self.frame(0x28a, [0]*8, stamp+1000))
        for n in range(24):
            payload = [0x10, 0x81, 0x11, 0, 0, 0, 0, 0] if n % 2 == 0 else [0]*8
            self.events.publish(self.frame(0x8a, payload, stamp+2000+n))
        # Allow DDS histories to fill before the executor resumes, as during
        # the observed scheduling delays. Acquisition stamps are never renewed.
        time.sleep(.025)
        return stamp

    def state(self):
        self.operator._publish()
        count = len(self.states)
        self.spin_until(lambda: len(self.states)>count)
        return self.states[-1]

    def test_diagnostic_burst_does_not_evict_neutral_pair(self):
        stamp = self.burst()
        self.spin_until(lambda: False, .08)
        state = self.state()
        self.assertTrue(state.connected)
        self.assertTrue(state.controls_neutral)
        self.assertLessEqual(Time.from_msg(state.header.stamp).nanoseconds, stamp+1000)

    def test_queued_stale_inputs_cannot_become_neutral_proof(self):
        self.burst(age=.35)
        self.spin_until(lambda: False, .08)
        self.assertFalse(self.state().connected)
        self.assertFalse(self.state().controls_neutral)

    def test_stop_digital_inhibits_without_waiting_for_analog(self):
        self.burst()
        self.spin_until(lambda: False, .06)
        stamp = self.bus.get_clock().now().nanoseconds
        self.controls.publish(self.frame(0x18a, [0]*7+[0xa8], stamp))
        self.spin_until(lambda: False, .03)
        state = self.state()
        self.assertTrue(state.estop_pressed)
        self.assertFalse(state.connected)
        self.assertFalse(state.controls_neutral)

    def test_collision_on_event_channel_still_blocks_fresh_controls(self):
        self.burst()
        self.spin_until(lambda: False, .06)
        stamp = self.bus.get_clock().now().nanoseconds
        self.events.publish(self.frame(0x8a, [0x50, 0x81, 0x11, 0, 0, 0, 0, 0], stamp))
        self.spin_until(lambda: False, .03)
        self.assertFalse(self.state().connected)
        self.assertFalse(self.state().controls_neutral)

    def test_pair_skew_limit_remains_enforced(self):
        stamp = self.bus.get_clock().now().nanoseconds
        self.controls.publish(self.frame(0x18a, [0]*7+[0x28], stamp-100_000_000))
        self.controls.publish(self.frame(0x28a, [0]*8, stamp-1_000_000))
        self.spin_until(lambda: False, .03)
        self.assertFalse(self.state().controls_neutral)


if __name__ == '__main__':
    unittest.main()
