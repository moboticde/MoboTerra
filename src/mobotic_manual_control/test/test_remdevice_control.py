"""Actual CAN operator methods with the existing ROS-free controller doubles."""
import ast
import json
import math
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
             'signed_axis': signed_axis, 'json': json, 'math': math, 'JointState': Message}
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
        node.reset_observed_pair = None
        node.receiver_recovery = None
        node.receiver_reset_status = ''
        node.last_start_ns = None
        node.input_error = ''
        node.raw_publisher = base.Publisher()
        node.joy_publisher = base.Publisher()
        node.selector_publisher = base.Publisher()
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

    def test_commissioned_d5_selector_has_separate_normalized_button(self):
        mapping = synthetic_mapping()
        mapping['buttons']['scanner_override'] = {'signal': 'D5', 'active_high': True}
        self.node.mapping = RemdeviceMapping(mapping)
        self.send(0x189, [0, 16, 0, 0, 0, 0, 0, 0x28])
        self.send(0x289, [0]*8)
        self.assertEqual(self.node.joy_publisher.messages[-1].buttons[10], 1)
        self.send(0x189, [0, 0, 0, 0, 0, 0, 0, 0x28])
        self.send(0x289, [0]*8)
        self.assertEqual(self.node.joy_publisher.messages[-1].buttons[10], 0)

    def test_physical_selector_is_fresh_during_emcy_without_faking_joy_or_rearm(self):
        mapping = synthetic_mapping()
        mapping['buttons']['scanner_override'] = {'signal': 'D5', 'active_high': True}
        self.node.mapping = RemdeviceMapping(mapping)
        self.pair()
        self.send(0x89, [0x10, 0x81, 0x11, 0, 0, 0, 0, 0])
        count = len(self.node.joy_publisher.messages)
        for enabled in (True, False, True):
            self.send(0x189, [1, 16 if enabled else 0, 0, 0, 0, 0, 0, 0x28], dt=.1)
            state = self.state()
            selector = self.node.selector_publisher.messages[-1]
            self.assertEqual(selector.buttons, [0] * 10 + [int(enabled)])
            self.assertEqual(selector.axes, [])
            self.assertEqual(selector.header.stamp, self.now.to_msg())
            self.assertTrue(state.connected)
            self.assertFalse(state.command_active)
            self.assertEqual(len(self.node.joy_publisher.messages), count)
            self.assertEqual(self.node.rearm_phase, 'neutral')
        old = selector.header.stamp
        self.now = Time(nanoseconds=self.now.nanoseconds + 1_000_000)
        self.state()
        self.assertEqual(self.node.selector_publisher.messages[-1].header.stamp, old)
        count = len(self.node.selector_publisher.messages)
        self.now = Time(nanoseconds=self.now.nanoseconds + 251_000_000)
        self.assertFalse(self.state().connected)
        self.assertEqual(len(self.node.selector_publisher.messages), count)

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

    def measured_pair(self, *, steer=0, crab=0, speed=0, status=0x28):
        # Confirmed PAIL: left paddle A2; right cross joystick A3/A4.
        up = (2 if steer > 0 else 0) | (4 if speed > 0 else 0) | (8 if crab > 0 else 0)
        down = (2 if steer < 0 else 0) | (4 if speed < 0 else 0) | (8 if crab < 0 else 0)
        self.send(0x189, [up, down, 0, 0, 0, 0, 0, status], dt=0.049)
        self.send(0x289, [0, abs(steer), abs(speed), abs(crab), 0, 0, 0, 0])

    def enable_stationary_steering(self):
        mapping = synthetic_mapping()
        for role, channel in (('speed', 3), ('steer', 2), ('crab', 4)):
            mapping['axes'][role]['channel'] = channel
        self.node.mapping = RemdeviceMapping(mapping)
        self.node.stationary_steering_enabled = True
        self.node.steering_geometry = {
            'steering_joint_names': ['front_left_steering', 'front_right_steering',
                                     'rear_left_steering', 'rear_right_steering'],
            'traction_joint_names': ['front_left_traction', 'front_right_traction',
                                     'rear_left_traction', 'rear_right_traction'],
            'wheels_x': [0.845, 0.845, -0.845, -0.845],
            'wheels_y': [0.495, -0.495, 0.495, -0.495],
            'wheel_mount_angles': [0.0, math.pi, 0.0, math.pi],
            'max_steering_velocity': 4.32}
        self.node.steering_publisher = base.Publisher()

    def test_left_paddle_turns_wheels_with_zero_body_and_traction_velocity(self):
        self.enable_stationary_steering()
        self.measured_pair()
        self.measured_pair(steer=255)
        request = self.node.steering_publisher.messages[-1]
        self.assertEqual(self.node.command, [0.0, 0.0, 0.0])
        self.assertEqual(request.velocity[4:], [0.0] * 4)
        self.assertGreater(request.position[0], 0.0)
        self.assertLess(request.position[2], 0.0)
        self.assertGreater(request.position[0], request.position[1] + math.pi)
        self.assertEqual(request.header.stamp, self.node.last_joy_stamp)

    def test_right_stick_crabs_both_directions_with_zero_yaw(self):
        for direction in (-255, 255):
            with self.subTest(direction=direction):
                self.setUp()
                self.enable_stationary_steering()
                self.measured_pair()
                count = len(self.node.steering_publisher.messages)
                self.measured_pair(crab=direction)
                self.assertEqual(self.node.command[0], 0.0)
                self.assertGreater(self.node.command[1] * direction, 0.0)
                self.assertEqual(self.node.command[2], 0.0)
                self.assertEqual(len(self.node.steering_publisher.messages), count)

    def test_stationary_steering_requires_neutral_rearm_and_healthy_receiver(self):
        self.enable_stationary_steering()
        self.measured_pair(steer=255)
        self.assertFalse(self.node.steering_publisher.messages)
        self.measured_pair()
        count = len(self.node.steering_publisher.messages)
        self.measured_pair(steer=255, status=0xa8)  # STOP set.
        self.assertEqual(len(self.node.steering_publisher.messages), count)
        self.assertFalse(self.state().connected)
        self.assertTrue(self.state().estop_pressed)

    def test_ros_fixed_array_is_normalized_before_protocol_validation(self):
        class WireByte(int):
            pass
        class FixedArray:
            def __init__(self, values): self.values = [WireByte(value) for value in values]
            def __len__(self): return len(self.values)
            def __getitem__(self, index): return self.values[index]
            def tolist(self): return [int(value) for value in self.values]
        for speed in (0, 255):
            for cob, data in ((0x189, [int(speed > 0), 0, 0, 0, 0, 0, 0, 0x28]),
                              (0x289, [speed, 0, 0, 0, 0, 0, 0, 0])):
                self.now = Time(nanoseconds=self.now.nanoseconds + 25_000_000)
                self.node._can_callback(SimpleNamespace(id=cob, dlc=8, data=FixedArray(data),
                    header=SimpleNamespace(stamp=self.now.to_msg()),
                    is_extended=False, is_rtr=False, is_error=False))
        self.assertTrue(self.state().connected)
        self.assertGreater(self.node.command[0], 0)

    def test_publication_with_advancing_clock_preserves_received_pdos(self):
        self.pair()
        def advancing_now():
            self.now = Time(nanoseconds=self.now.nanoseconds + 1000)
            return self.now
        self.node.get_clock = lambda: SimpleNamespace(now=advancing_now)
        for _ in range(10):
            self.assertTrue(self.state().connected)
            self.assertIsNotNone(self.node.decoder.snapshot(advancing_now().nanoseconds))
            self.assertNotEqual(self.node.decoder.reason, 'clock moved backwards')

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
        self.pair(); self.state()
        old = self.node.command_publisher.messages[-1].header.stamp
        self.send(0x189, [0] * 7 + [0x28], dt=0.05)
        self.assertEqual(self.state().header.stamp, self.now.to_msg())
        self.assertEqual(self.node.command_publisher.messages[-1].header.stamp, old)

    def test_pending_analog_preserves_last_complete_fresh_input(self):
        self.pair(); self.pair(speed=255)
        self.state()
        old = self.node.command_publisher.messages[-1].header.stamp
        self.send(0x189, [1, 0, 0, 0, 0, 0, 0, 0x28], dt=0.1)
        state = self.state()
        self.assertTrue(state.connected and state.command_active)
        self.assertEqual(state.header.stamp, self.now.to_msg())
        self.assertEqual(self.node.command_publisher.messages[-1].header.stamp, old)
        self.send(0x289, [255, 0, 0, 0, 0, 0, 0, 0])
        self.assertTrue(self.state().command_active)

    def test_pending_pair_does_not_mask_stop_or_extend_input_deadline(self):
        for stop in (False, True):
            with self.subTest(stop=stop):
                self.setUp()
                self.pair(); self.pair(speed=255)
                self.state()
                old = self.node.command_publisher.messages[-1].header.stamp
                self.send(0x189, [1, 0, 0, 0, 0, 0, 0, 0xa8 if stop else 0x28], dt=0.1)
                if stop:
                    self.assertFalse(self.state().command_active)
                    self.assertTrue(self.state().estop_pressed)
                else:
                    self.send(0x189, [1, 0, 0, 0, 0, 0, 0, 0x28], dt=0.1)
                    self.state()
                    self.assertEqual(self.node.command_publisher.messages[-1].header.stamp, old)
                    self.send(0x189, [1, 0, 0, 0, 0, 0, 0, 0x28], dt=0.06)
                    self.assertFalse(self.state().command_active)

    def test_physical_status_keeps_its_own_stamp_across_motion_input_expiry(self):
        self.pair(); self.pair(speed=255)
        for dt in (.1, .1, .06):
            self.send(0x189, [1, 0, 0, 0, 0, 0, 0, 0x28], dt=dt)
            state = self.state()
            self.assertTrue(state.connected and state.estop_state_known)
            self.assertEqual(state.header.stamp, self.now.to_msg())
        self.assertFalse(state.command_active)
        self.assertEqual(self.node.command, [0, 0, 0])
        self.now = Time(nanoseconds=self.now.nanoseconds + 251_000_000)
        self.assertFalse(self.state().connected or self.state().estop_state_known)

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

    def test_emcy_preserves_fresh_physical_link_but_stops_every_motion_command(self):
        self.pair(); self.pair(speed=255)
        self.assertGreater(self.node.command[0], 0)
        self.send(0x89, [0x10, 0x81, 0x11, 0, 0, 0, 0, 0])
        state = self.state()
        self.assertTrue(state.connected and state.estop_state_known)
        self.assertFalse(state.command_active or state.controls_neutral)
        self.assertEqual(self.node.command, [0, 0, 0])
        self.assertNotEqual(self.node.decoder.emergency_register, 0)
        self.send(0x89, [0] * 8)
        self.assertTrue(self.state().connected)
        self.assertFalse(self.state().command_active)
        self.pair(speed=255)
        self.assertFalse(self.state().command_active)
        self.pair(); self.pair(speed=255)
        self.assertTrue(self.state().command_active)

    def test_physical_link_stamp_does_not_renew_command_or_survive_radio_loss(self):
        self.pair(); self.pair(speed=255)
        old_command_stamp = self.node.last_joy_stamp
        self.send(0x89, [0x10, 0x81, 0x11, 0, 0, 0, 0, 0])
        self.send(0x189, [0] * 7 + [0x28], dt=.1)
        state = self.state()
        self.assertTrue(state.connected)
        self.assertGreater(state.header.stamp, old_command_stamp)
        self.assertFalse(state.command_active)
        self.assertIsNone(self.node.last_joy_stamp)  # The pre-fault command was discarded.
        self.assertEqual(self.node.command_publisher.messages[-1].twist.linear.x, 0.0)
        self.send(0x189, [0] * 8)
        self.assertFalse(self.state().connected or self.state().estop_state_known)
        self.send(0x189, [0] * 7 + [0x80])
        self.assertTrue(self.state().estop_pressed)
        self.now = Time(nanoseconds=self.now.nanoseconds + 251_000_000)
        self.assertFalse(self.state().connected or self.state().estop_state_known)

    def receiver_fault(self):
        self.pair()
        self.send(0x89, [0x10, 0x81, 0x11, 0, 0, 0, 0, 0])
        self.pair()  # Fresh neutral controls and released B2 after the fault.
        self.assertTrue(self.state().connected)
        self.assertFalse(self.state().command_active)

    def begin_receiver_reset(self):
        self.receiver_fault()
        self.pair(buttons=4)
        request = self.node.can_publisher.messages[-1]
        self.assertEqual((request.id, request.dlc, request.data),
                         (0x609, 8, [0x40, 1, 0x10, 0, 0, 0, 0, 0]))
        self.assertFalse(self.resets)
        self.assertEqual(self.node.command, [0, 0, 0])

    def receiver_reply(self, error=0):
        self.send(0x589, [0x4f, 1, 0x10, 0, error, 0, 0, 0])

    def test_b2_recovers_verified_receiver_then_requests_supervisor_with_new_input(self):
        self.begin_receiver_reset()
        self.receiver_reply()
        self.assertEqual(self.node.decoder.emergency_register, 0)
        self.assertFalse(self.resets)
        self.assertFalse(self.state().command_active)
        self.pair(buttons=4)
        self.assertEqual(len(self.resets), 1)
        self.assertEqual(self.node.command, [0, 0, 0])
        self.pair(speed=255, buttons=4)
        self.assertFalse(self.state().command_active)
        self.assertEqual(self.node.command, [0, 0, 0])

    def test_receiver_active_error_aborted_or_wrong_width_reply_never_clears_latch(self):
        for response in ([0x4f, 1, 0x10, 0, 0x11, 0, 0, 0],
                         [0x80, 1, 0x10, 0, 0, 0, 2, 6],
                         [0x4b, 1, 0x10, 0, 0, 0, 0, 0]):
            with self.subTest(response=response):
                self.setUp(); self.begin_receiver_reset()
                self.send(0x589, response)
                self.assertEqual(self.node.decoder.emergency_register, 0x11)
                self.assertFalse(self.resets or self.state().command_active)
                self.assertIsNone(self.node.receiver_recovery)

    def test_unrelated_stale_future_extended_and_rtr_replies_cannot_acknowledge(self):
        cases = ((0x588, None, False, False), (0x589, -1, False, False),
                 (0x589, 1_000_000, False, False), (0x589, None, True, False),
                 (0x589, None, False, True))
        for cob, offset, extended, rtr in cases:
            with self.subTest(case=(cob, offset, extended, rtr)):
                self.setUp(); self.begin_receiver_reset()
                request_ns = self.node.receiver_recovery['started_ns']
                self.now = Time(nanoseconds=self.now.nanoseconds + 1000)
                stamp = self.now.nanoseconds if offset is None else request_ns + offset
                self.node._can_callback(SimpleNamespace(id=cob, dlc=8,
                    data=[0x4f, 1, 0x10, 0, 0, 0, 0, 0],
                    header=SimpleNamespace(stamp=stamp), is_error=False,
                    is_extended=extended, is_rtr=rtr))
                self.assertEqual(self.node.decoder.emergency_register, 0x11)
                self.assertFalse(self.resets)

    def test_new_emergency_during_b2_verification_is_not_cleared(self):
        self.begin_receiver_reset()
        self.send(0x89, [0x50, 0x81, 0x11, 0, 0, 0, 0, 0])
        self.receiver_reply()
        self.assertEqual(self.node.decoder.emergency_code, 0x8150)
        self.assertFalse(self.resets)
        self.assertIsNone(self.node.receiver_recovery)

    def test_stop_link_loss_and_movement_during_verification_block_recovery(self):
        for status, speed in ((0xa8, 0), (0, 0), (0x28, 255)):
            with self.subTest(status=status, speed=speed):
                self.setUp(); self.begin_receiver_reset()
                self.pair(status=status, speed=speed, buttons=4)
                self.receiver_reply()
                self.assertEqual(self.node.decoder.emergency_register, 0x11)
                self.assertFalse(self.resets or self.state().command_active)

    def test_timeout_does_not_acknowledge_and_held_b2_does_not_retry(self):
        self.begin_receiver_reset()
        self.now = Time(nanoseconds=self.now.nanoseconds + 1_100_000_000)
        self.state()
        self.assertIsNone(self.node.receiver_recovery)
        self.receiver_reply()
        self.pair(buttons=4); self.pair(buttons=4)
        reads = [f for f in self.node.can_publisher.messages if f.id == 0x609]
        self.assertEqual(len(reads), 1)
        self.assertFalse(self.resets)
        self.assertEqual(self.node.decoder.emergency_register, 0x11)

    def test_b2_held_when_fault_arrives_requires_release_and_new_press(self):
        self.pair()
        self.send(0x89, [0x10, 0x81, 0x11, 0, 0, 0, 0, 0])
        self.pair(buttons=4); self.pair(buttons=4)
        self.assertFalse([f for f in self.node.can_publisher.messages if f.id == 0x609])
        self.pair(); self.pair(buttons=4)
        self.assertEqual(len([f for f in self.node.can_publisher.messages if f.id == 0x609]), 1)

    def test_supervisor_reset_result_requires_another_neutral_rearm(self):
        self.begin_receiver_reset(); self.receiver_reply(); self.pair(buttons=4)
        future = SimpleNamespace(result=lambda: SimpleNamespace(success=True, message='accepted'))
        self.node.reset_future = future
        self.node._reset_done(future)
        self.pair(speed=255)
        self.assertFalse(self.state().command_active)
        self.pair(); self.pair(speed=255)
        self.assertGreater(self.node.command[0], 0)

    def test_receiver_recovery_never_sends_nmt_reset_or_sdo_download(self):
        self.begin_receiver_reset(); self.receiver_reply(); self.pair()
        requests = self.node.can_publisher.messages
        self.assertTrue(requests)
        self.assertTrue(all(f.id == 0x609 and f.data == [0x40, 1, 0x10, 0, 0, 0, 0, 0]
                            for f in requests))

    def test_recovery_with_advancing_callback_clock_does_not_invalidate_input(self):
        def advancing_now():
            self.now = Time(nanoseconds=self.now.nanoseconds + 1000)
            return self.now
        self.node.get_clock = lambda: SimpleNamespace(now=advancing_now)
        self.begin_receiver_reset(); self.receiver_reply(); self.pair(buttons=4)
        self.assertEqual(len(self.resets), 1)
        self.assertNotEqual(self.node.decoder.reason, 'clock moved backwards')

    def test_missing_supervisor_service_times_out_without_enabling_motion(self):
        self.begin_receiver_reset(); self.receiver_reply()
        self.node.reset_client.service_is_ready = lambda: False
        self.pair(buttons=4)
        self.assertFalse(self.resets or self.state().command_active)
        self.now = Time(nanoseconds=self.now.nanoseconds + 1_100_000_000)
        self.state()
        self.assertIsNone(self.node.receiver_recovery)
        self.assertFalse(self.state().command_active)

    def test_b2_held_across_timer_detected_disconnect_must_be_released(self):
        self.pair()
        self.now = Time(nanoseconds=self.now.nanoseconds + 1_000_000_000)
        self.state()
        self.pair(buttons=4); self.pair(buttons=4)
        self.assertFalse(self.resets)
        self.pair(); self.pair(buttons=4)
        self.assertEqual(len(self.resets), 1)

    def test_pending_supervisor_request_still_blocks_motion_after_new_can_error(self):
        self.pair(); self.pair(buttons=4)
        self.assertEqual(len(self.resets), 1)
        self.node._can_callback(SimpleNamespace(is_error=True))
        self.pair(); self.pair(speed=255)
        self.assertFalse(self.state().command_active)
        self.assertEqual(self.node.command, [0, 0, 0])

    def test_rp1_position_requests_only_modes_zero_one(self):
        self.pair(); self.pair(buttons=1); self.pair(buttons=1); self.pair()
        self.assertEqual([m.requested_mode for m in self.node.mode_publisher.messages], [1, 0])

    def test_confirmed_active_low_u5_selects_manual_high_and_autonomous_low(self):
        mapping = synthetic_mapping()
        mapping['buttons']['mode'] = {'signal': 'U5', 'active_high': False}
        self.node.mapping = RemdeviceMapping(mapping)
        self.pair(up=0)  # Startup primes even when the switch indicates auto.
        self.assertFalse(self.node.mode_publisher.messages)
        self.pair(up=16)  # Raw U5 high -> logical 0 -> MANUAL.
        self.pair(up=0)   # Raw U5 low -> logical 1 -> AUTONOMOUS.
        self.assertEqual([m.requested_mode for m in self.node.mode_publisher.messages], [0, 1])

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
