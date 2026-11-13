"""Actual CAN operator methods with the existing ROS-free controller doubles."""
import ast
import json
import inspect
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
                        'reset': {'signal': 'SP3', 'active_high': True}}}


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
        node.drive_toggle_status = ''
        node.last_start_ns = None
        node.receiver_transmit_enabled = True
        # Existing tests isolate B2 and command behavior from automatic recovery.
        node.receiver_auto_recovery = False
        node.receiver_auto_communication_reset = False
        node.receiver_poll_ns = None
        node.receiver_read = None
        node.receiver_register_ns = None
        node.receiver_register_value = None
        node.receiver_reset_ns = None
        node.receiver_last_reset_ns = None
        node.receiver_reset_attempts = 0
        node.input_error = ''
        node.raw_publisher = base.Publisher()
        node.joy_publisher = base.Publisher()
        node.selector_publisher = base.Publisher()
        node.can_publisher = base.Publisher()
        self.auto_supervisor_status = True
        self.supervisor_status()
        self.resets = []
        future = SimpleNamespace(done=lambda: False, add_done_callback=lambda cb: None)
        node.reset_client = SimpleNamespace(service_is_ready=lambda: True,
                                            call_async=lambda req: self.resets.append(req) or future)

    def send(self, cob, data, *, dt=0.001):
        self.now = Time(nanoseconds=self.now.nanoseconds + round(dt * 1e9))
        self.node._can_callback(SimpleNamespace(id=cob, dlc=len(data), data=data,
                                header=SimpleNamespace(stamp=self.now.to_msg()),
                                is_extended=False, is_rtr=False, is_error=False))

    def supervisor_status(self, *, enabled=None, resetting=None):
        previous = self.node.vehicle_mode_state
        self.node._vehicle_mode_callback(SimpleNamespace(
            header=SimpleNamespace(stamp=self.now.to_msg()), current_mode=0,
            drives_enabled=(getattr(previous, 'drives_enabled', False) if enabled is None else enabled),
            reset_in_progress=(getattr(previous, 'reset_in_progress', False) if resetting is None else resetting)))

    def pair(self, *, speed=0, buttons=0, status=0x28, up=None):
        if self.auto_supervisor_status:
            self.supervisor_status()
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
            self.assertFalse(state.connected)  # EMCY must withdraw drive enable.
            self.assertTrue(state.estop_state_known)
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

    def test_left_paddle_requests_body_yaw_without_speed_stick(self):
        self.enable_stationary_steering()
        self.measured_pair()
        count = len(self.node.steering_publisher.messages)
        self.measured_pair(steer=255)
        self.assertEqual(self.node.command[:2], [0.0, 0.0])
        self.assertGreater(self.node.command[2], 0.0)
        self.assertEqual(len(self.node.steering_publisher.messages), count)

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

    def test_pending_b2_preserves_physical_neutral_proof_without_motion(self):
        self.pair()
        self.pair(buttons=4)
        self.assertEqual(len(self.resets), 1)
        self.assertIsNone(self.node.last_joy_stamp)
        state = self.state()
        self.assertTrue(state.connected)
        self.assertTrue(state.controls_neutral)
        self.assertFalse(state.command_active)
        self.pair(speed=255)
        state = self.state()
        self.assertFalse(state.controls_neutral)
        self.assertFalse(state.command_active)

    def test_reset_while_controls_deflected_is_ignored(self):
        self.pair(); self.pair(speed=255, buttons=4)
        self.assertFalse(self.resets)

    def test_emcy_withdraws_readiness_but_preserves_physical_estop_evidence(self):
        self.pair(); self.pair(speed=255)
        self.assertGreater(self.node.command[0], 0)
        self.send(0x89, [0x10, 0x81, 0x11, 0, 0, 0, 0, 0])
        state = self.state()
        self.assertFalse(state.connected)
        self.assertTrue(state.estop_state_known)
        self.assertIn('EMCY=0x8110', state.status_message)
        self.assertIn('CAN receive overrun', state.status_message)
        self.assertIn('CAN traffic load', state.status_message)
        self.assertIn('restart joystick/receiver', state.status_message)
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

    def test_advisory_overrun_retains_controls_and_allows_explicit_b2(self):
        self.node.decoder.can_overrun_advisory = True
        warnings = []
        self.node.get_logger = lambda: SimpleNamespace(
            error=lambda _: None, warning=warnings.append, info=lambda _: None)
        self.pair(); self.pair(speed=255)
        stamp = self.node.last_joy_stamp
        self.send(0x89, [0x10, 0x81, 0x11, 0, 0, 0, 0, 0])
        state = self.state()
        self.assertTrue(state.connected and state.command_active)
        self.assertEqual(self.node.last_joy_stamp, stamp)
        self.assertIn('overrun warning', state.status_message)
        self.assertTrue(any('overrun warning' in warning for warning in warnings))
        self.assertFalse(self.node.can_publisher.messages)
        self.pair(); self.pair(buttons=4)
        self.assertEqual(len(self.resets), 1)

    def test_advisory_overrun_still_stops_for_stop_link_loss_invalid_axes_and_stale_input(self):
        for cause in ('stop', 'link', 'invalid_axis', 'stale', 'analog_missing', 'collision'):
            with self.subTest(cause=cause):
                self.setUp()
                self.node.decoder.can_overrun_advisory = True
                self.pair(); self.pair(speed=255)
                self.send(0x89, [0x10, 0x81, 0x11, 0, 0, 0, 0, 0])
                if cause == 'stop':
                    self.send(0x189, [0]*7 + [0xA8])
                elif cause == 'link':
                    self.send(0x189, [0]*7 + [0x08])
                elif cause == 'invalid_axis':
                    self.send(0x189, [1, 1, 0, 0, 0, 0, 0, 0x28])
                    self.send(0x289, [255]+[0]*7)
                elif cause == 'analog_missing':
                    for _ in range(6):
                        self.send(0x189, [0]*7+[0x28], dt=.05)
                elif cause == 'collision':
                    self.send(0x89, [0x50, 0x81, 0x11, 0, 0, 0, 0, 0])
                else:
                    self.now = Time(nanoseconds=self.now.nanoseconds + 251_000_000)
                state = self.state()
                self.assertFalse(state.connected or state.command_active)
                self.assertEqual(self.node.command, [0, 0, 0])
                if cause == 'stop':
                    self.assertTrue(state.estop_state_known and state.estop_pressed)
                    self.assertIn('STOP', state.status_message)

    def test_fresh_error_register_with_additional_faults_ends_overrun_advisory(self):
        self.node.decoder.can_overrun_advisory = True
        self.auto_recovery_fault(code=0x8110)
        self.error_register_reply(0x15)
        self.assertTrue(self.node.decoder.has_blocking_error())
        self.assertFalse(self.state().connected)

    def test_pdo_based_policy_diagnostics_do_not_drop_motion_or_require_rearm(self):
        self.node.decoder.errors_advisory = True
        warnings, infos = [], []
        self.node.get_logger = lambda: SimpleNamespace(error=warnings.append,
            warning=warnings.append, info=infos.append)
        self.pair(); self.pair(speed=255)
        self.state()
        initial_infos = len(infos)
        for code, register in ((0x8110, 0x11), (0x8150, 0x11), (0x6000, 1), (0, 0)):
            stamp = self.node.last_joy_stamp
            count = len(self.node.joy_publisher.messages)
            self.send(0x89, [code & 255, code >> 8, register, 0, 0, 0, 0, 0])
            state = self.state()
            self.assertTrue(state.connected and state.command_active)
            self.assertEqual(self.node.last_joy_stamp, stamp)
            self.assertEqual(len(self.node.joy_publisher.messages), count)
            self.assertNotIn('EMCY=', state.status_message)
            self.assertNotIn('diagnostic warning', state.status_message)
            raw = json.loads(self.node.raw_publisher.messages[-1].data)
            self.assertEqual(raw['receiver_error_code'], code)
            self.assertEqual(raw['receiver_error_register'], register)
            self.pair(speed=255)
            self.assertTrue(self.state().command_active)
        self.assertFalse(self.resets)
        self.assertFalse(warnings)
        self.assertEqual(len(infos), initial_infos)

    def test_pdo_based_policy_register_errors_never_reset_healthy_receiver(self):
        self.node.decoder.errors_advisory = True
        self.auto_recovery_fault()
        for register in (0x11, 0x15, 0):
            if self.node.receiver_read is None:
                self.now = Time(nanoseconds=self.now.nanoseconds + 1_000_000_000)
                self.pair(); self.state()
            self.error_register_reply(register)
            self.assertTrue(self.state().connected)
            self.assertIsNone(self.node.receiver_reset_ns)
            self.assertFalse(self.node.decoder.has_blocking_error())
        self.assertEqual(self.node.receiver_reset_attempts, 0)
        self.assertTrue(all(frame.id != 0 for frame in self.node.can_publisher.messages))

    def test_pdo_based_policy_keeps_stop_link_and_pdo_validation(self):
        for cause in ('stop', 'link', 'invalid_axis', 'stale', 'analog_missing', 'bad_dlc'):
            with self.subTest(cause=cause):
                self.setUp()
                self.node.decoder.errors_advisory = True
                self.pair(); self.pair(speed=255)
                self.send(0x89, [0x50, 0x81, 0x11, 0, 0, 0, 0, 0])
                if cause == 'stop':
                    self.send(0x189, [0] * 7 + [0xA8])
                elif cause == 'link':
                    self.send(0x189, [0] * 7 + [0x08])
                elif cause == 'invalid_axis':
                    self.send(0x189, [1, 1, 0, 0, 0, 0, 0, 0x28])
                    self.send(0x289, [255] + [0] * 7)
                elif cause == 'analog_missing':
                    for _ in range(6):
                        self.send(0x189, [0] * 7 + [0x28], dt=.05)
                elif cause == 'bad_dlc':
                    self.send(0x289, [0])
                else:
                    self.now = Time(nanoseconds=self.now.nanoseconds + 251_000_000)
                state = self.state()
                self.assertFalse(state.connected or state.command_active)
                self.assertEqual(self.node.command, [0, 0, 0])

    def test_pdo_based_policy_can_error_indication_does_not_clear_fresh_input(self):
        self.node.decoder.errors_advisory = True
        warnings = []
        self.node.get_logger = lambda: SimpleNamespace(error=warnings.append,
            warning=warnings.append, info=lambda _: None)
        self.pair(); self.pair(speed=255)
        stamp = self.node.last_joy_stamp
        self.node._can_callback(SimpleNamespace(id=0x20000040, dlc=8, data=[0] * 8,
            header=SimpleNamespace(stamp=self.now.to_msg()), is_extended=False, is_rtr=False, is_error=True))
        self.assertTrue(self.state().connected)
        self.assertEqual(self.node.last_joy_stamp, stamp)
        self.assertEqual(self.node.decoder.can_error_count, 1)
        self.assertFalse(warnings)

    def test_receiver_fault_is_logged_without_b2_and_not_hidden_by_pending_enable(self):
        warnings = []
        self.node.get_logger = lambda: SimpleNamespace(
            error=lambda _: None, warning=warnings.append, info=lambda _: None)
        self.pair()
        self.state()
        self.node.drive_toggle_status = 'B2 drive initialization/enable requested'
        self.send(0x89, [0x50, 0x81, 0x11, 0, 0, 0, 0, 0])
        for _ in range(5):
            state = self.state()
            self.assertFalse(state.connected)
            self.assertIn('EMCY=0x8150', state.status_message)
        reports = [message for message in warnings if message.startswith('Joystick input unavailable:')]
        self.assertEqual(len(reports), 1)
        self.assertIn('EMCY=0x8150', reports[0])

    def test_physical_link_stamp_does_not_renew_command_or_survive_radio_loss(self):
        self.pair(); self.pair(speed=255)
        old_command_stamp = self.node.last_joy_stamp
        self.send(0x89, [0x10, 0x81, 0x11, 0, 0, 0, 0, 0])
        self.send(0x189, [0] * 7 + [0x28], dt=.1)
        state = self.state()
        self.assertFalse(state.connected)
        self.assertTrue(state.estop_state_known)
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

    def auto_recovery_fault(self, code=0x8150, *, communication_reset=True):
        self.node.receiver_auto_recovery = True
        self.node.receiver_auto_communication_reset = communication_reset
        self.pair()
        self.send(0x89, [code & 255, code >> 8, 0x11, 0, 0, 0, 0, 0])
        self.pair()
        self.state()
        self.assertEqual(self.node.can_publisher.messages[-1].id, 0x609)
        self.assertEqual(self.node.can_publisher.messages[-1].data,
                         [0x40, 1, 0x10, 0, 0, 0, 0, 0])

    def error_register_reply(self, register, **kwargs):
        self.send(0x589, [0x4f, 1, 0x10, 0, register, 0, 0, 0], **kwargs)

    def test_startup_reads_current_register_without_waiting_for_emcy(self):
        self.node.receiver_auto_recovery = True
        self.pair()
        self.state()
        raw = json.loads(self.node.raw_publisher.messages[-1].data)
        self.assertIsNone(raw['receiver_error_register_read_value'])
        self.assertIsNone(raw['receiver_error_register_read_stamp_ns'])
        self.assertEqual(raw['receiver_nmt_state'], 5)
        request = self.node.can_publisher.messages[-1]
        self.assertEqual((request.id, request.data), (0x609, [0x40, 1, 0x10, 0, 0, 0, 0, 0]))
        self.error_register_reply(0)
        self.assertTrue(self.state().connected)
        raw = json.loads(self.node.raw_publisher.messages[-1].data)
        self.assertEqual(raw['receiver_error_register_read_value'], 0)
        self.assertEqual(raw['receiver_error_register_read_stamp_ns'], self.now.nanoseconds)
        # Timer callbacks cannot turn this into extra traffic or renew the read.
        self.now = Time(nanoseconds=self.now.nanoseconds + 100_000_000)
        self.state()
        self.assertEqual(len(self.node.can_publisher.messages), 1)
        self.assertLess(self.node.receiver_register_ns, self.now.nanoseconds)
        self.now = Time(nanoseconds=self.now.nanoseconds + 900_000_000)
        self.state()
        self.assertEqual(len(self.node.can_publisher.messages), 2)
        self.assertTrue(all(m.id == 0x609 for m in self.node.can_publisher.messages))

    def test_missed_startup_emcy_detected_by_current_register_then_real_clear(self):
        self.node.receiver_auto_recovery = True
        self.pair()
        self.state()
        self.error_register_reply(0x11)
        self.assertFalse(self.state().connected)
        self.assertEqual(self.node.decoder.emergency_code, 0)
        self.assertIn('EMCY detail not received', self.node._receiver_fault_message())
        self.pair(speed=255)
        self.assertFalse(self.state().command_active)
        self.now = Time(nanoseconds=self.now.nanoseconds + 1_000_000_000)
        self.state()
        self.error_register_reply(0)
        self.pair(speed=255)
        self.assertFalse(self.state().command_active)
        self.pair()
        self.pair(speed=255)
        self.assertTrue(self.state().command_active)
        self.assertFalse(self.resets)

    def test_healthy_current_register_does_not_log_every_second(self):
        self.node.receiver_auto_recovery = True
        logs = []
        self.node.get_logger = lambda: SimpleNamespace(info=logs.append, warning=lambda _: None)
        for _ in range(3):
            self.pair()
            self.state()
            self.error_register_reply(0)
            self.now = Time(nanoseconds=self.now.nanoseconds + 1_000_000_000)
        self.assertEqual(len([m for m in logs if 'error register' in m]), 1)

    def test_automatic_clear_checks_device_and_requires_new_neutral_input(self):
        self.auto_recovery_fault()
        self.assertFalse(self.state().command_active)
        self.error_register_reply(0)
        self.assertEqual(self.node.decoder.emergency_code, 0)
        self.assertEqual(self.node.decoder.emergency_register, 0)
        self.assertIsNone(self.node.last_joy_stamp)
        self.pair(speed=255, buttons=4)
        self.assertFalse(self.state().command_active)
        self.assertFalse(self.resets)
        self.pair()
        self.pair(speed=255)
        self.assertTrue(self.state().command_active)
        self.assertFalse(self.resets)

    def test_historical_collision_does_not_block_b2_after_current_error_clears(self):
        self.auto_recovery_fault()
        pending = self.node.receiver_read
        # 0x1003:1 retains a previous 0x8150 event with register 0x11.
        # A history response must not masquerade as the current 0x1001 read.
        history = [0x43, 3, 0x10, 1, 0x50, 0x81, 0x11, 0]
        self.send(0x589, history)
        self.assertEqual(self.node.receiver_read, pending)
        self.assertEqual(self.node.decoder.emergency_register, 0x11)
        self.error_register_reply(0)
        self.assertEqual(self.node.decoder.emergency_code, 0)
        self.assertEqual(self.node.decoder.emergency_register, 0)
        self.pair()
        self.send(0x589, history)
        self.pair(buttons=4)
        self.assertEqual(len(self.resets), 1)
        self.assertTrue(self.state().connected)
        self.assertEqual(self.node.receiver_reset_attempts, 0)

    def test_read_only_recovery_never_resets_pail_even_with_repeated_b2(self):
        self.auto_recovery_fault(communication_reset=False)
        for _ in range(5):
            self.error_register_reply(0x11)
            self.pair(); self.pair(buttons=4)
            self.now = Time(nanoseconds=self.now.nanoseconds + 1_000_000_000)
            self.pair()
            self.state()
        self.assertEqual(self.node.receiver_reset_attempts, 0)
        self.assertIsNone(self.node.receiver_reset_ns)
        self.assertFalse(self.resets)
        self.assertTrue(all(m.id == 0x609 and m.data == [0x40, 1, 0x10, 0, 0, 0, 0, 0]
                            for m in self.node.can_publisher.messages))
        # A real zero error register still clears the local latch; no reset write.
        self.error_register_reply(0)
        self.assertEqual(self.node.decoder.emergency_code, 0)

    def test_b2_healthy_enable_never_sends_pail_nmt_or_sdo(self):
        self.node.receiver_auto_recovery = True
        self.pair()
        self.state()  # Independent periodic health monitor, before B2.
        count = len(self.node.can_publisher.messages)
        self.pair(buttons=4)
        self.assertEqual(len(self.resets), 1)
        self.assertEqual(len(self.node.can_publisher.messages), count)
        for _ in range(5):
            self.pair(); self.pair(buttons=4)
        self.assertEqual(len(self.resets), 1)
        self.assertEqual(len(self.node.can_publisher.messages), count)

    def test_error_register_numpy_reply_stays_json_serializable(self):
        try:
            import numpy as np
        except ImportError:
            self.skipTest('NumPy is used by installed ROS fixed-size CAN arrays')
        self.auto_recovery_fault(communication_reset=False)
        self.send(0x589, np.array([0x4f, 1, 0x10, 0, 0x11, 0, 0, 0], dtype=np.uint8))
        self.assertIs(type(self.node.decoder.emergency_register), int)
        self.state()
        diagnostic = json.loads(self.node.raw_publisher.messages[-1].data)
        self.assertEqual(diagnostic['receiver_error_register'], 0x11)
        self.assertEqual(self.node.receiver_reset_attempts, 0)

    def test_collision_recovery_resets_only_receiver_and_starts_after_delay(self):
        self.auto_recovery_fault()
        self.error_register_reply(0x11)
        reset = self.node.can_publisher.messages[-1]
        self.assertEqual((reset.id, reset.dlc, reset.data),
                         (0, 2, [0x82, 9, 0, 0, 0, 0, 0, 0]))
        self.assertEqual(self.node.receiver_reset_attempts, 1)
        self.assertFalse(self.state().command_active)
        count = len(self.node.can_publisher.messages)
        self.state()
        self.assertEqual(len(self.node.can_publisher.messages), count)
        # A zero EMCY during reset cannot re-enable commands before restart.
        self.send(0x89, [0] * 8)
        self.pair(); self.pair(speed=255)
        state = self.state()
        self.assertFalse(state.command_active or state.connected)
        self.now = Time(nanoseconds=self.now.nanoseconds + 600_000_000)
        self.supervisor_status(enabled=False, resetting=False)
        self.state()
        start = self.node.can_publisher.messages[-1]
        self.assertEqual((start.id, start.data[:2]), (0, [1, 9]))
        self.assertFalse(self.node.decoder.samples)
        self.pair(speed=255)
        self.assertFalse(self.state().command_active)
        self.pair(); self.pair(speed=255)
        self.assertTrue(self.state().command_active)
        self.assertFalse(self.resets)

    def test_collision_reset_requires_disabled_drives_neutral_input_and_current_mode(self):
        for condition in ('enabled', 'resetting', 'stale', 'missing', 'deflected',
                          'stop', 'radio', 'pending_toggle'):
            with self.subTest(condition=condition):
                self.setUp()
                self.auto_recovery_fault()
                if condition == 'enabled':
                    self.supervisor_status(enabled=True)
                elif condition == 'resetting':
                    self.supervisor_status(resetting=True)
                elif condition == 'missing':
                    self.node.vehicle_mode_state = None
                elif condition == 'stale':
                    self.node.vehicle_mode_received = Time(1)
                elif condition == 'deflected':
                    self.pair(speed=255)
                elif condition in ('stop', 'radio'):
                    self.pair(status=0xa8 if condition == 'stop' else 0)
                else:
                    self.node.reset_future = SimpleNamespace(done=lambda: False)
                self.error_register_reply(0x11)
                self.assertEqual(self.node.receiver_reset_attempts, 0)
                self.assertTrue(all(msg.id == 0x609 for msg in self.node.can_publisher.messages))
                self.assertEqual(self.node.decoder.emergency_code, 0x8150)

    def test_eeprom_and_other_errors_are_not_reset_or_saved_automatically(self):
        for code in (0x6000, 0x8110, 0x1000):
            with self.subTest(code=code):
                self.setUp()
                self.auto_recovery_fault(code)
                self.error_register_reply(0x11)
                self.assertEqual(self.node.decoder.emergency_code, code)
                self.assertEqual(self.node.receiver_reset_attempts, 0)
                self.assertEqual(len(self.node.can_publisher.messages), 1)

    def test_receive_only_blocks_automatic_and_explicit_nmt_in_every_receiver_state(self):
        self.node.receiver_transmit_enabled = False
        self.node.receiver_auto_recovery = True
        for status in (None, 0, 127, 4, 5):
            self.node.decoder.nmt_state = status
            self.now = Time(nanoseconds=self.now.nanoseconds + 1_100_000_000)
            self.state()
            self.assertFalse(self.node._send_start())
            response = self.node._start_receiver(None, SimpleNamespace())
            self.assertFalse(response.success)
            self.assertIn('receive-only', response.message)
        self.assertFalse(self.node.can_publisher.messages)
        self.assertIsNone(self.node.last_start_ns)
        self.assertIsNone(self.node.receiver_poll_ns)

    def test_receive_only_preserves_stick_input_and_stop_detection(self):
        self.node.receiver_transmit_enabled = False
        self.node.receiver_auto_recovery = True
        self.pair()
        self.pair(speed=255)
        state = self.state()
        self.assertTrue(state.connected and state.command_active)
        self.assertGreater(self.node.command[0], 0)
        self.pair(status=0xa8)
        state = self.state()
        self.assertFalse(state.connected or state.command_active)
        self.assertTrue(state.estop_pressed)
        self.assertFalse(self.node.can_publisher.messages)
        self.assertFalse(json.loads(self.node.raw_publisher.messages[-1].data)['receiver_transmit_enabled'])

    def test_receive_only_b2_still_requests_drive_enable_without_receiver_tx(self):
        self.node.receiver_transmit_enabled = False
        self.node.receiver_auto_recovery = True
        self.pair()
        self.pair(buttons=4)
        self.assertEqual(len(self.resets), 1)
        self.state()
        self.assertFalse(self.node.can_publisher.messages)

    def test_receive_only_faults_remain_blocking_without_poll_or_communication_reset(self):
        self.node.receiver_transmit_enabled = False
        self.node.receiver_auto_recovery = self.node.receiver_auto_communication_reset = True
        self.pair()
        self.send(0x89, [0x50, 0x81, 0x11, 0, 0, 0, 0, 0])
        for _ in range(3):
            self.pair()
            self.assertFalse(self.state().connected)
        self.assertEqual(self.node.decoder.emergency_code, 0x8150)
        self.assertEqual(self.node.receiver_reset_attempts, 0)
        self.assertFalse(self.node.can_publisher.messages)

    def test_receiver_poll_is_rate_limited_and_can_be_disabled(self):
        self.auto_recovery_fault()
        self.state(); self.state()
        self.assertEqual(len(self.node.can_publisher.messages), 1)
        self.now = Time(nanoseconds=self.now.nanoseconds + 1_000_000_000)
        self.state()
        self.assertEqual(len(self.node.can_publisher.messages), 2)
        self.node.receiver_auto_recovery = False
        self.error_register_reply(0)
        self.assertEqual(self.node.decoder.emergency_code, 0x8150)
        self.now = Time(nanoseconds=self.now.nanoseconds + 1_000_000_000)
        self.state()
        self.assertEqual(len(self.node.can_publisher.messages), 2)

    def test_unsolicited_wrong_stale_and_aborted_sdo_cannot_clear_fault(self):
        for condition in ('unsolicited', 'wrong_node', 'wrong_object', 'stale',
                          'old_acquisition', 'abort', 'bad_dlc', 'new_fault'):
            with self.subTest(condition=condition):
                self.setUp()
                self.auto_recovery_fault()
                data = [0x4f, 1, 0x10, 0, 0, 0, 0, 0]
                cob = 0x589
                if condition == 'unsolicited':
                    self.node.receiver_read = None
                elif condition == 'wrong_node':
                    cob = 0x58a
                elif condition == 'wrong_object':
                    data[1] = 3
                elif condition == 'stale':
                    self.now = Time(nanoseconds=self.now.nanoseconds + 251_000_000)
                elif condition == 'old_acquisition':
                    self.node.receiver_read = (self.now.nanoseconds + 10_000_000,
                        self.node.decoder.last_stamps[self.node.decoder.emergency_id])
                elif condition == 'abort':
                    data[0] = 0x80
                elif condition == 'bad_dlc':
                    data.pop()
                elif condition == 'new_fault':
                    self.send(0x89, [0x50, 0x81, 0x11, 0, 0, 0, 0, 0])
                self.send(cob, data)
                self.assertEqual(self.node.decoder.emergency_code, 0x8150)
                self.assertFalse(self.state().command_active)

    def test_collision_reset_is_bounded_and_spaced_five_seconds_apart(self):
        self.auto_recovery_fault()
        for attempt in range(3):
            if attempt:
                self.now = Time(nanoseconds=self.now.nanoseconds + 5_000_000_000)
                self.pair(); self.state()
            self.error_register_reply(0x11)
            self.assertEqual(self.node.receiver_reset_attempts, attempt + 1)
            self.now = Time(nanoseconds=self.now.nanoseconds + 600_000_000)
            self.supervisor_status(enabled=False, resetting=False)
            self.state()
            self.pair()
            self.now = Time(nanoseconds=self.now.nanoseconds + 500_000_000)
            self.pair(); self.state()
            self.error_register_reply(0x11)
            self.assertEqual(self.node.receiver_reset_attempts, attempt + 1)
        self.now = Time(nanoseconds=self.now.nanoseconds + 5_000_000_000)
        self.pair(); self.state(); self.error_register_reply(0x11)
        self.assertEqual(self.node.receiver_reset_attempts, 3)
        self.assertEqual(self.node.decoder.emergency_code, 0x8150)
        self.assertFalse(self.state().command_active)

    def test_unverified_mapping_never_polls_or_resets_receiver(self):
        self.auto_recovery_fault()
        self.node.mapping = RemdeviceMapping({'verified': False, 'node_id': 9,
                                             'axes': {}, 'buttons': {}})
        self.error_register_reply(0)
        self.now = Time(nanoseconds=self.now.nanoseconds + 1_000_000_000)
        self.state()
        self.assertEqual(len(self.node.can_publisher.messages), 1)
        self.assertEqual(self.node.decoder.emergency_code, 0x8150)

    def receiver_fault(self):
        self.pair()
        self.send(0x89, [0x10, 0x81, 0x11, 0, 0, 0, 0, 0])
        self.pair()  # Fresh neutral controls and released B2 after the fault.
        self.assertFalse(self.state().connected)
        self.assertTrue(self.state().estop_state_known)
        self.assertFalse(self.state().command_active)

    def test_b2_never_reads_or_clears_receiver_faults_or_restarts_communication(self):
        self.receiver_fault()
        warnings = []
        self.node.get_logger = lambda: SimpleNamespace(warning=warnings.append)
        for _ in range(3):
            self.pair(); self.pair(buttons=4); self.state()
        self.assertTrue(any('CAN receive overrun' in message
                            and 'restart joystick/receiver' in message for message in warnings))
        # Unsolicited SDO replies cannot clear an EMCY either.
        self.send(0x589, [0x4f, 1, 0x10, 0, 0, 0, 0, 0])
        self.assertEqual(self.node.decoder.emergency_register, 0x11)
        self.assertFalse(self.resets or self.node.can_publisher.messages)
        self.assertFalse(self.state().command_active)

    def test_b2_never_disables_enabled_drives_even_with_deflected_or_faulted_input(self):
        for fault in (False, True):
            with self.subTest(fault=fault):
                self.setUp()
                if fault:
                    self.receiver_fault()
                self.now = Time(nanoseconds=self.now.nanoseconds + 1000)
                self.supervisor_status(enabled=True)
                self.pair(speed=255); self.pair(speed=255, buttons=4)
                self.assertEqual(len(self.resets), 0)
                self.assertFalse(self.node.can_publisher.messages)

    def test_b2_cannot_cancel_pending_initialization(self):
        self.now = Time(nanoseconds=self.now.nanoseconds + 1000)
        self.supervisor_status(resetting=True)
        self.pair(speed=255); self.pair(speed=255, buttons=4)
        self.assertEqual(len(self.resets), 0)
        self.assertTrue(self.node.vehicle_mode_state.reset_in_progress)

    def test_pending_request_ignores_repeated_edges_and_all_motion(self):
        self.pair(); self.pair(buttons=4)
        for _ in range(3):
            self.pair(); self.pair(speed=255, buttons=4)
            self.assertFalse(self.state().command_active)
        self.assertEqual(len(self.resets), 1)
        self.assertEqual(self.node.command, [0, 0, 0])

    def complete_toggle(self, *, success=True):
        future = SimpleNamespace(result=lambda: SimpleNamespace(success=success, message='result'))
        self.node.reset_future = future
        self.node._reset_done(future)

    def test_toggle_result_requires_new_neutral_rearm_on_success_rejection_or_exception(self):
        for outcome in ('success', 'rejection', 'exception'):
            with self.subTest(outcome=outcome):
                self.setUp()
                self.pair(); self.pair(buttons=4)
                if outcome == 'exception':
                    def fail():
                        raise RuntimeError('service failed')
                    future = SimpleNamespace(result=fail)
                    self.node.reset_future = future
                    self.node._reset_done(future)
                else:
                    self.complete_toggle(success=outcome == 'success')
                self.assertIsNone(self.node.reset_future)
                self.pair(speed=255, buttons=4)
                self.assertFalse(self.state().command_active)
                self.assertEqual(len(self.resets), 1)
                self.pair(); self.pair(speed=255)
                self.assertGreater(self.node.command[0], 0)

    def test_external_disable_requires_release_and_new_neutral_b2_enable_press(self):
        self.now = Time(nanoseconds=self.now.nanoseconds + 1000)
        self.supervisor_status(enabled=True)
        self.pair(); self.pair(speed=255, buttons=4)
        self.assertEqual(len(self.resets), 0)
        self.now = Time(nanoseconds=self.now.nanoseconds + 1000)
        self.supervisor_status(enabled=False)
        self.pair(buttons=4); self.pair(buttons=4)
        self.assertEqual(len(self.resets), 0)
        self.pair(); self.pair(buttons=4)
        self.assertEqual(len(self.resets), 1)

    def test_enable_rejects_deflection_stop_radio_loss_and_incoherent_axes(self):
        for status, speed, up in ((0x28, 255, None), (0xa8, 0, None),
                                  (0, 0, None)):
            with self.subTest(status=status, speed=speed, up=up):
                self.setUp()
                self.pair(); self.pair(status=status, speed=speed, up=up, buttons=4)
                self.assertFalse(self.resets)

    def test_contradictory_axis_directions_block_enable(self):
        self.pair()
        self.send(0x189, [1, 1, 4, 0, 0, 0, 0, 0x28], dt=.049)
        self.send(0x289, [255, 0, 0, 0, 0, 0, 0, 0])
        self.state()
        self.assertFalse(self.resets)
        self.assertEqual(self.node.command, [0, 0, 0])

    def test_missing_or_stale_supervisor_status_blocks_toggle_without_retries(self):
        for missing in (True, False):
            with self.subTest(missing=missing):
                self.setUp()
                self.auto_supervisor_status = False
                if missing:
                    self.node.vehicle_mode_state = None
                else:
                    self.now = Time(nanoseconds=self.now.nanoseconds + 600_000_000)
                self.pair(); self.pair(buttons=4)
                self.assertFalse(self.resets)
                self.now = Time(nanoseconds=self.now.nanoseconds + 1000)
                self.supervisor_status()
                self.pair(buttons=4)
                self.assertFalse(self.resets)
                self.pair(); self.pair(buttons=4)
                self.assertEqual(len(self.resets), 1)

    def test_service_submission_failure_drops_motion_and_requires_new_press(self):
        call = self.node.reset_client.call_async
        def fail(request):
            raise RuntimeError('service unavailable')
        self.node.reset_client.call_async = fail
        self.pair(); self.pair(buttons=4)
        self.assertIsNone(self.node.reset_future)
        self.assertEqual(self.node.command, [0, 0, 0])
        self.node.reset_client.call_async = call
        self.pair(buttons=4)
        self.assertFalse(self.resets)
        self.pair(); self.pair(buttons=4)
        self.assertEqual(len(self.resets), 1)

    def test_missing_supervisor_service_requires_new_press_after_recovery(self):
        self.node.reset_client.service_is_ready = lambda: False
        self.pair(); self.pair(buttons=4)
        self.assertFalse(self.resets)
        self.node.reset_client.service_is_ready = lambda: True
        self.pair(buttons=4)
        self.assertFalse(self.resets)
        self.pair(); self.pair(buttons=4)
        self.assertEqual(len(self.resets), 1)

    def test_receiver_clear_requires_new_input_and_release_before_b2_enable(self):
        self.receiver_fault()
        self.pair(buttons=4)
        self.assertFalse(self.resets)
        self.send(0x89, [0] * 8)
        self.pair(buttons=4); self.pair(buttons=4)
        self.assertFalse(self.resets)
        self.pair(); self.pair(buttons=4)
        self.assertEqual(len(self.resets), 1)
        self.assertFalse(self.node.can_publisher.messages)

    def test_single_pdo_cannot_generate_toggle_or_refresh_button_edge(self):
        self.pair()
        self.send(0x189, [0, 0, 4, 0, 0, 0, 0, 0x28])
        self.state()
        self.assertFalse(self.resets)
        self.send(0x289, [0] * 8)
        self.assertEqual(len(self.resets), 1)

    def test_toggle_with_advancing_clock_does_not_invalidate_input(self):
        def advancing_now():
            self.now = Time(nanoseconds=self.now.nanoseconds + 1000)
            return self.now
        self.node.get_clock = lambda: SimpleNamespace(now=advancing_now)
        self.pair(); self.pair(buttons=4)
        self.assertEqual(len(self.resets), 1)
        self.assertNotEqual(self.node.decoder.reason, 'clock moved backwards')

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

    def test_reset_accept_then_reject_use_distinct_rclpy_log_sites(self):
        sites, failures, logged = {}, [], []
        def record(level, message):
            caller = inspect.currentframe().f_back.f_back
            site = (caller.f_code.co_filename, caller.f_lineno)
            if site in sites and sites[site] != level:
                raise ValueError('Logger severity cannot be changed between calls.')
            sites[site] = level
            logged.append((level, message))
        logger = SimpleNamespace(info=lambda msg: record('info', msg),
            warning=lambda msg: record('warning', msg), error=failures.append)
        self.node.get_logger = lambda: logger
        for success, message in ((True, 'accepted'), (False, 'driver startup failed')):
            future = SimpleNamespace(result=lambda: SimpleNamespace(success=success, message=message))
            self.node.reset_future = future
            self.node._reset_done(future)
        self.assertFalse(failures)
        self.assertEqual([entry[0] for entry in logged], ['info', 'warning'])
        self.assertIn('driver startup failed', logged[-1][1])

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

    def test_missing_samples_and_local_can_error_do_not_restart_operational_receiver(self):
        self.pair()
        self.assertEqual(self.node.decoder.nmt_state, 5)
        self.node._can_callback(SimpleNamespace(is_error=True))
        self.state()
        self.now = Time(nanoseconds=self.now.nanoseconds + 2_000_000_000)
        self.state()
        self.assertFalse(self.node.can_publisher.messages)
        self.pair()
        self.now = Time(nanoseconds=self.now.nanoseconds + 2_000_000_000)
        self.assertFalse(self.state().connected)
        self.state()
        self.assertFalse(self.node.can_publisher.messages)

    def test_boot_preoperational_and_stopped_allow_targeted_rate_limited_start(self):
        self.pair()
        for status in (0, 127, 4):
            self.send(0x709, [status], dt=1.1)
            self.state(); self.state()
        self.assertEqual(len(self.node.can_publisher.messages), 3)
        self.assertTrue(all(m.id == 0 and m.data[:2] == [1, 9]
                            for m in self.node.can_publisher.messages))
        self.send(0x709, [5])
        self.now = Time(nanoseconds=self.now.nanoseconds + 2_000_000_000)
        self.state()
        self.assertEqual(len(self.node.can_publisher.messages), 3)

    def test_can_error_without_timestamp_still_clears_motion(self):
        self.pair(); self.pair(speed=255)
        self.node._can_callback(SimpleNamespace(is_error=True))
        self.assertEqual(self.node.command, [0, 0, 0])


if __name__ == '__main__':
    unittest.main()
