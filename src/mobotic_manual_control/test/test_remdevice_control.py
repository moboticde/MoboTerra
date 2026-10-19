"""Actual CAN operator methods with the existing ROS-free controller doubles."""
import ast
import json
from pathlib import Path
from types import SimpleNamespace
import unittest

import test_manual_control as base
from mobotic_config.operator_inputs import RemdeviceMapping
from mobotic_manual_control.remdevice_protocol import RemdeviceReceiver, signed_axis


class Time(base.Time):
    def __init__(self, seconds=0, *, nanoseconds=None):
        self.nanoseconds = round(seconds * 1e9) if nanoseconds is None else nanoseconds

    @staticmethod
    def from_msg(stamp):
        return Time(nanoseconds=stamp)


class Message:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None, frame_id='')


SOURCE = Path(__file__).resolve().parents[1] / 'mobotic_manual_control/remdevice.py'
tree = ast.parse(SOURCE.read_text(encoding='utf-8-sig'))
cls = next(item for item in tree.body if isinstance(item, ast.ClassDef))
cls.body = [item for item in cls.body if isinstance(item, ast.FunctionDef) and item.name != '__init__']
namespace = {'ManualControl': base.Controller, 'Time': Time, 'Joy': Message, 'Frame': Message,
             'String': SimpleNamespace, 'Trigger': SimpleNamespace(Request=lambda: None),
             'signed_axis': signed_axis, 'json': json}
exec(compile(ast.Module(body=[cls], type_ignores=[]), str(SOURCE), 'exec'), namespace)
Control = namespace['RemdeviceControl']


def synthetic_mapping():
    return {'verified': True, 'node_id': 9,
            'axes': {role: {'channel': channel, 'zero': 0, 'full_scale': 255, 'invert': False}
                     for role, channel in (('speed', 1), ('steer', 2), ('crab', 3))},
            'buttons': {'mode': {'signal': 'SP1', 'active_high': True},
                        'boost': {'signal': 'SP2', 'active_high': True},
                        'reset': {'signal': 'SP3', 'active_high': True}, 'deadman': None}}


class RemdeviceControlTest(unittest.TestCase):
    def setUp(self):
        fixture = base.ManualControlTest()
        fixture.setUp()
        self.now = Time(10)
        self.node = fixture.node
        self.node.__class__ = Control
        node = self.node
        node.get_clock = lambda: SimpleNamespace(now=lambda: self.now)
        node.get_logger = lambda: SimpleNamespace(error=lambda _: None, warning=lambda _: None, info=lambda _: None)
        node.mapping = RemdeviceMapping(synthetic_mapping())
        node.decoder = RemdeviceReceiver(9)
        node.buttons_boost = [9]
        node.last_reset = None
        node.reset_future = None
        node.last_start_ns = None
        node.input_error = ''
        node.raw_publisher = base.Publisher()
        node.joy_publisher = base.Publisher()
        node.can_publisher = base.Publisher()
        self.resets = []
        future = SimpleNamespace(done=lambda: False, add_done_callback=lambda cb: None)
        node.reset_client = SimpleNamespace(service_is_ready=lambda: True,
                                            call_async=lambda req: self.resets.append(req) or future)

    def send(self, cob, data, *, dt=0.001):
        self.now = Time(nanoseconds=self.now.nanoseconds + round(dt * 1e9))
        self.node._can_callback(SimpleNamespace(id=cob, dlc=len(data), data=data,
                                header=SimpleNamespace(stamp=self.now.to_msg()),
                                is_extended=False, is_rtr=False, is_error=False))

    def pair(self, *, speed=0, buttons=0, status=0x28, up=None):
        self.send(0x189, [int(speed > 0) if up is None else up, 0, buttons, 0, 0, 0, 0, status], dt=0.049)
        self.send(0x289, [speed, 0, 0, 0, 0, 0, 0, 0])

    def state(self):
        self.node._publish()
        return self.node.state_publisher.messages[-1]

    def test_unmapped_captures_but_never_drives_switches_or_resets(self):
        self.node.mapping = RemdeviceMapping({'verified': False, 'node_id': 9, 'axes': {}, 'buttons': {}})
        for buttons in (0, 7, 0, 7):
            self.pair(speed=255, buttons=buttons)
        state = self.state()
        self.assertFalse(state.command_active or state.connected or state.estop_state_known)
        self.assertIn('unverified', state.status_message)
        self.assertEqual(self.node.command, [0, 0, 0])
        self.assertFalse(self.resets or self.node.mode_publisher.messages or self.node.can_publisher.messages)
        self.assertIsNotNone(json.loads(self.node.raw_publisher.messages[-1].data)['digital'])

    def test_neutral_then_stick_drives_without_hold_button(self):
        self.pair(speed=255)
        self.assertFalse(self.state().command_active)
        self.pair()
        self.pair(speed=255)
        state = self.state()
        self.assertTrue(state.connected and state.command_active and state.estop_state_known)
        self.assertFalse(state.deadman_pressed or state.deadman_state_known)
        self.assertGreater(self.node.command[0], 0)

    def test_configured_can_deadman_requires_hold_and_release_stops(self):
        mapping = synthetic_mapping()
        mapping['buttons']['deadman'] = {'signal': 'SP4', 'active_high': True}
        self.node.mapping = RemdeviceMapping(mapping)
        self.node.button_deadman = 8
        self.pair(buttons=8)
        self.assertFalse(self.state().command_active)
        self.pair(buttons=0)
        self.pair(buttons=8)
        self.pair(speed=255, buttons=8)
        self.assertTrue(self.state().deadman_pressed)
        self.assertGreater(self.node.command[0], 0)
        self.pair(speed=255, buttons=0)
        self.assertEqual(self.node.command, [0, 0, 0])
        state = self.state()
        self.assertTrue(state.deadman_state_known)
        self.assertFalse(state.deadman_pressed or state.command_active)

    def test_radio_loss_digital_stops_without_waiting_for_analog(self):
        self.pair(); self.pair(speed=255)
        self.assertGreater(self.node.command[0], 0)
        self.send(0x189, [0] * 8)
        self.assertEqual(self.node.command, [0, 0, 0])
        state = self.state()
        self.assertFalse(state.command_active or state.estop_state_known)

    def test_stop_reports_pressed_even_when_radio_emgy_is_off(self):
        self.pair(); self.pair(speed=255)
        self.send(0x189, [0] * 7 + [0x80])
        state = self.state()
        self.assertTrue(state.estop_pressed and state.estop_state_known)
        self.assertFalse(state.command_active)
        self.assertEqual(self.node.command, [0, 0, 0])

    def test_reconnection_with_deflected_stick_requires_neutral(self):
        self.pair(); self.pair(speed=255)
        self.now = Time(11)
        self.state()
        self.pair(speed=255); self.pair(speed=255)
        self.assertFalse(self.state().command_active)
        self.pair(); self.pair(speed=255)
        self.assertTrue(self.state().command_active)

    def test_neutral_deadzone_is_exact_zero_motion(self):
        self.pair(speed=20); self.pair(speed=20)
        self.assertTrue(self.state().controls_neutral)
        self.assertEqual(self.node.command, [0, 0, 0])

    def test_partial_pair_cannot_renew_published_source_stamp(self):
        self.pair(); old = self.state().header.stamp
        self.send(0x189, [0] * 7 + [0x28], dt=0.05)
        self.assertEqual(self.state().header.stamp, old)

    def test_reset_held_at_start_is_primed_and_new_neutral_edge_calls_supervisor(self):
        self.pair(buttons=4); self.pair(buttons=4)
        self.assertFalse(self.resets)
        self.pair(); self.pair(buttons=4)
        self.assertEqual(len(self.resets), 1)
        self.pair(buttons=4)
        self.assertEqual(len(self.resets), 1)

    def test_reset_while_controls_deflected_is_ignored(self):
        self.pair(); self.pair(speed=255, buttons=4)
        self.assertFalse(self.resets)

    def test_rp1_position_requests_only_modes_zero_one(self):
        self.pair(); self.pair(buttons=1); self.pair(buttons=1); self.pair()
        self.assertEqual([m.requested_mode for m in self.node.mode_publisher.messages], [1, 0])

    def test_nmt_start_is_targeted_rate_limited_and_only_auto_when_verified(self):
        self.state(); self.state()
        self.assertEqual(len(self.node.can_publisher.messages), 1)
        frame = self.node.can_publisher.messages[0]
        self.assertEqual((frame.id, frame.dlc, frame.data[:2]), (0, 2, [1, 9]))

    def test_can_error_without_timestamp_still_clears_motion(self):
        self.pair(); self.pair(speed=255)
        self.node._can_callback(SimpleNamespace(is_error=True))
        self.assertEqual(self.node.command, [0, 0, 0])


if __name__ == '__main__':
    unittest.main()
