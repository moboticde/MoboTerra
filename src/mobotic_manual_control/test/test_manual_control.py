"""Exercise actual manual-control methods with ROS-free clock/message doubles."""

import ast
import math
from pathlib import Path
from types import SimpleNamespace
import unittest


class Time:
    def __init__(self, seconds):
        self.nanoseconds = round(seconds * 1e9)

    def __sub__(self, other):
        return SimpleNamespace(nanoseconds=self.nanoseconds - other.nanoseconds)

    def to_msg(self):
        return self.nanoseconds

    @staticmethod
    def from_msg(stamp):
        return Time(stamp / 1e9)


class TwistStamped:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None, frame_id='')
        self.twist = SimpleNamespace(
            linear=SimpleNamespace(x=0.0, y=0.0),
            angular=SimpleNamespace(z=0.0),
        )


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class VehicleModeRequest:
    MANUAL = 0
    AUTO_VELOCITY = 1
    AUTO_DIRECT = 2

    def __init__(self):
        self.header = SimpleNamespace(stamp=None, frame_id='')
        self.requested_mode = 255


class Duration:
    def __init__(self, *, seconds):
        self.nanoseconds = round(seconds * 1e9)

    def __ge__(self, other):
        return self.nanoseconds >= other.nanoseconds


SOURCE = Path(__file__).resolve().parents[1] / 'mobotic_manual_control' / 'manual_control.py'
tree = ast.parse(SOURCE.read_text(encoding='utf-8'))
controller_class = next(
    item for item in tree.body
    if isinstance(item, ast.ClassDef) and item.name == 'ManualControl'
)
controller_class.bases = []
controller_class.body = [
    method for method in controller_class.body
    if isinstance(method, ast.FunctionDef) and method.name != '__init__'
]
namespace = {
    'Duration': Duration, 'TwistStamped': TwistStamped,
    'ManualControlState': SimpleNamespace, 'math': math,
    'Time': Time, 'VehicleModeRequest': VehicleModeRequest,
}
exec(compile(ast.Module(body=[controller_class], type_ignores=[]), str(SOURCE), 'exec'), namespace)
Controller = namespace['ManualControl']


class ManualControlTest(unittest.TestCase):
    def setUp(self):
        self.now = Time(10.0)
        self.node = Controller()
        node = self.node
        node.get_clock = lambda: SimpleNamespace(now=lambda: self.now)
        node.get_logger = lambda: SimpleNamespace(error=lambda message: None)
        node.axis_speed = 3
        node.axis_steer = 0
        node.axis_crab = 2
        node.button_deadman = -1
        node.buttons_boost = [9, 10]
        node.button_mode_switch = 4
        # Position-input fixtures exercise future RP1 semantics, not its CAN mapping.
        node.mode_switch_type = 'position'
        node.mode_switch_manual_value = 0
        node.mode_switch_auto_velocity_value = 1
        node.mode_state_timeout = 0.5
        node.vehicle_mode_state = None
        node.vehicle_mode_received = None
        node.mode_switch_state = None
        node.last_mode_joy_stamp = None
        node.mode_publisher = Publisher()
        node.max_velocity_linear = 1.25
        node.max_velocity_angular = 1.51
        node.max_acceleration_linear = 0.5
        node.max_acceleration_angular = 0.52
        node.scale_linear = 0.4
        node.scale_angular = 0.4
        node.joystick_timeout = 0.25
        node.publish_period = 0.02
        node.frame_id = 'base_link'
        node.last_joy_time = None
        node.last_update_time = self.now
        node.deadman_pressed = False
        node.turbo_active = False
        node.command = [0.0, 0.0, 0.0]
        node.warned_invalid_layout = False
        node.command_publisher = Publisher()
        node.state_publisher = Publisher()

    def joy(self, *, moving=False, deadman=False, turbo=False):
        buttons = [0] * 15
        buttons[8] = int(deadman)  # Optional-deadman test fixture only.
        buttons[10] = int(turbo)
        return SimpleNamespace(
            header=SimpleNamespace(stamp=self.now.to_msg()),
            axes=[0.0, 0.0, 0.0, 1.0 if moving else 0.0, 0.0, 0.0],
            buttons=buttons,
        )

    def seed_motion(self):
        self.node.last_joy_time = Time(10.0)
        self.node.command = [0.3, 0.2, 0.15]
        self.node.deadman_pressed = True
        self.node.turbo_active = True

    def output(self):
        msg = self.node.command_publisher.messages[-1]
        return [msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z]

    def test_startup_without_joy_is_stopped(self):
        self.node._publish()
        self.assertEqual(self.output(), [0.0] * 3)
        self.assertFalse(self.node.state_publisher.messages[-1].connected)

    def test_timeout_clears_command_and_acceleration_state(self):
        self.seed_motion()
        self.now = Time(10.3)
        self.node._publish()
        self.assertEqual(self.output(), [0.0] * 3)
        self.assertEqual(self.node.command, [0.0] * 3)
        self.assertEqual(self.node.last_update_time.nanoseconds, self.now.nanoseconds)
        self.assertFalse(self.node.deadman_pressed)
        self.assertFalse(self.node.turbo_active)
        state = self.node.state_publisher.messages[-1]
        self.assertFalse(state.connected)
        self.assertFalse(state.command_active)

    def test_neutral_reconnection_after_timeout_does_not_restore_motion(self):
        self.seed_motion()
        self.now = Time(10.3)
        self.node._publish()
        self.now = Time(10.32)
        self.node._joy_callback(self.joy())
        self.node._publish()
        self.assertEqual(self.node.command, [0.0] * 3)
        self.assertEqual(self.output(), [0.0] * 3)
        self.assertTrue(self.node.state_publisher.messages[-1].connected)

    def test_neutral_reconnection_before_timer_runs_does_not_restore_motion(self):
        self.seed_motion()
        self.now = Time(10.3)
        self.node._joy_callback(self.joy())
        self.node._publish()
        self.assertEqual(self.output(), [0.0] * 3)

    def test_moving_reconnection_restarts_acceleration_from_zero(self):
        self.seed_motion()
        self.now = Time(11.0)
        self.node._joy_callback(self.joy(moving=True))
        self.node._publish()
        self.assertEqual(self.output(), [0.0] * 3)
        self.now = Time(11.02)
        self.node._joy_callback(self.joy(moving=True))
        self.node._publish()
        self.assertAlmostEqual(self.output()[0], 0.01)
        self.assertEqual(self.output()[1:], [0.0, 0.0])

    def test_healthy_input_keeps_normal_acceleration(self):
        self.node._joy_callback(self.joy(moving=True))
        for time, expected in ((10.1, 0.05), (10.2, 0.1)):
            self.now = Time(time)
            self.node._joy_callback(self.joy(moving=True))
            self.node._publish()
            self.assertAlmostEqual(self.output()[0], expected)
            self.assertTrue(self.node.state_publisher.messages[-1].command_active)

    def test_invalid_layout_clears_flags_and_acceleration_state(self):
        self.seed_motion()
        self.now = Time(10.1)
        self.node._joy_callback(SimpleNamespace(axes=[], buttons=[]))
        self.node._publish()
        self.assertEqual(self.output(), [0.0] * 3)
        self.assertFalse(self.node.deadman_pressed)
        self.assertFalse(self.node.turbo_active)
        self.assertEqual(self.node.last_update_time.nanoseconds, self.now.nanoseconds)
        self.assertFalse(self.node.state_publisher.messages[-1].command_active)

    def test_optional_deadman_still_stops_when_explicitly_configured(self):
        self.node.button_deadman = 8
        self.seed_motion()
        self.now = Time(10.1)
        self.node._joy_callback(self.joy(moving=True, deadman=False))
        self.node._publish()
        self.assertEqual(self.node.command, [0.0] * 3)
        self.assertEqual(self.output(), [0.0] * 3)
        self.assertFalse(self.node.state_publisher.messages[-1].command_active)

    def test_clock_reversal_clears_old_motion(self):
        self.seed_motion()
        self.now = Time(9.9)
        self.node._publish()
        self.assertEqual(self.output(), [0.0] * 3)
        self.assertFalse(self.node.state_publisher.messages[-1].connected)
        self.now = Time(10.1)
        self.node._publish()
        self.assertFalse(self.node.state_publisher.messages[-1].connected)
        self.node._joy_callback(self.joy())
        self.node._publish()
        self.assertEqual(self.output(), [0.0] * 3)

    def test_exact_timeout_boundary_is_connected(self):
        self.seed_motion()
        self.now = Time(10.25)
        self.node._publish()
        self.assertEqual(self.output(), [0.3, 0.2, 0.15])
        self.assertTrue(self.node.state_publisher.messages[-1].connected)

    def switch(self, value):
        self.now = Time(self.now.nanoseconds / 1e9 + 0.02)
        msg = self.joy(deadman=False)
        msg.buttons[self.node.button_mode_switch] = value
        self.node._joy_callback(msg)

    def test_switch_changes_request_only_manual_and_auto_velocity(self):
        self.switch(0)
        for value in (1, 1, 0, 0, 1):
            self.switch(value)
        self.assertEqual([msg.requested_mode for msg in self.node.mode_publisher.messages], [1, 0, 1])
        self.assertEqual(self.node.mode_publisher.messages[-1].header.stamp, self.now.to_msg())

    def test_position_at_startup_or_reconnection_does_not_request_mode(self):
        self.switch(1)
        self.assertFalse(self.node.mode_publisher.messages)
        self.now = Time(11.0)
        self.switch(0)
        self.assertFalse(self.node.mode_publisher.messages)
        self.switch(1)
        self.assertEqual(self.node.mode_publisher.messages[-1].requested_mode, 1)

    def test_invalid_switch_value_reprimes_without_requesting_direct_mode(self):
        self.switch(0)
        self.switch(2)
        self.switch(1)
        self.assertFalse(self.node.mode_publisher.messages)
        self.switch(0)
        self.assertEqual(self.node.mode_publisher.messages[-1].requested_mode, 0)
        self.assertFalse(self.node._request_joystick_mode(2, self.now))
        self.assertEqual(len(self.node.mode_publisher.messages), 1)

    def test_stale_zero_or_future_joy_cannot_request_mode(self):
        for stamp in (0, Time(9).to_msg(), Time(10.1).to_msg()):
            with self.subTest(stamp=stamp):
                self.setUp()
                self.switch(0)
                msg = self.joy()
                msg.header.stamp = stamp
                msg.buttons[4] = 1
                self.node._joy_callback(msg)
                self.assertFalse(self.node.mode_publisher.messages)

    def test_disabled_or_unavailable_switch_does_not_break_manual_driving(self):
        for index in (-1, 20):
            with self.subTest(index=index):
                self.setUp()
                self.node.button_mode_switch = index
                self.node._joy_callback(self.joy(moving=True))
                self.now = Time(10.1)
                self.node._joy_callback(self.joy(moving=True))
                self.node._publish()
                self.assertGreater(self.output()[0], 0)
                self.assertFalse(self.node.mode_publisher.messages)

    def test_switch_configuration_cannot_overlap_motion_buttons(self):
        for index in (9, 10, -2):
            with self.subTest(index=index):
                self.node.button_mode_switch = index
                with self.assertRaises(ValueError):
                    self.node._validate_parameters()

    def test_switch_polarity_is_configurable_and_requires_binary_distinct_values(self):
        self.node.mode_switch_manual_value = 1
        self.node.mode_switch_auto_velocity_value = 0
        self.node._validate_parameters()
        self.switch(1)
        self.switch(0)
        self.switch(1)
        self.assertEqual([msg.requested_mode for msg in self.node.mode_publisher.messages], [1, 0])
        for manual, auto in ((0, 0), (1, 1), (-1, 1), (0, 2)):
            self.node.mode_switch_manual_value, self.node.mode_switch_auto_velocity_value = manual, auto
            with self.assertRaises(ValueError):
                self.node._validate_parameters()

    def test_replayed_switch_samples_and_unchanged_position_cannot_override_terminal_mode(self):
        self.switch(0)
        msg = self.joy()
        msg.buttons[4] = 1
        self.node._joy_callback(msg)  # Same acquisition timestamp: replay.
        self.assertFalse(self.node.mode_publisher.messages)
        for _ in range(5):
            self.switch(0)
        self.assertFalse(self.node.mode_publisher.messages)
        self.switch(1)
        self.assertEqual(self.node.mode_publisher.messages[-1].requested_mode, 1)

    def mode_feedback(self, mode=0, transitioning=False, stamp=None):
        self.node._vehicle_mode_callback(SimpleNamespace(
            header=SimpleNamespace(stamp=self.now.to_msg() if stamp is None else stamp),
            current_mode=mode, transition_in_progress=transitioning))

    def test_usb_back_toggles_from_reported_mode_not_local_state(self):
        self.node.mode_switch_type = 'toggle'
        self.switch(0)
        for current, requested in ((0, 1), (1, 0), (2, 0)):
            self.mode_feedback(current)
            self.switch(1)
            self.assertEqual(self.node.mode_publisher.messages[-1].requested_mode, requested)
            count = len(self.node.mode_publisher.messages)
            self.switch(1)
            self.switch(0)
            self.assertEqual(len(self.node.mode_publisher.messages), count)

    def test_usb_toggle_needs_fresh_mode_feedback_and_ignores_pending_transition(self):
        self.node.mode_switch_type = 'toggle'
        self.switch(0)
        self.switch(1)  # No supervisor yet.
        self.switch(0)
        self.mode_feedback(transitioning=True)
        self.switch(1)
        self.assertFalse(self.node.mode_publisher.messages)
        self.switch(0)
        self.mode_feedback()
        self.now = Time(10.7)
        self.switch(0)
        self.switch(1)
        self.assertFalse(self.node.mode_publisher.messages)
        self.mode_feedback()
        self.switch(1)  # Holding cannot retry automatically.
        self.assertFalse(self.node.mode_publisher.messages)
        self.switch(0)
        self.switch(1)
        self.assertEqual(len(self.node.mode_publisher.messages), 1)

    def test_invalid_or_replayed_mode_feedback_cannot_change_toggle_target(self):
        self.node.mode_switch_type = 'toggle'
        self.mode_feedback(1)
        for mode, stamp in ((0, self.now.to_msg()), (255, Time(10.01).to_msg()),
                            (0, 0), (0, Time(9).to_msg()), (0, Time(11).to_msg())):
            self.mode_feedback(mode, stamp=stamp)
            self.assertEqual(self.node.vehicle_mode_state.current_mode, 1)
        self.switch(0)
        self.switch(1)
        self.assertEqual(self.node.mode_publisher.messages[-1].requested_mode, 0)

    def test_face_buttons_do_nothing_and_right_stick_crabs_without_forward_input(self):
        self.node.mode_switch_type = 'toggle'
        self.mode_feedback()
        self.node._joy_callback(self.joy())
        self.now = Time(10.2)
        msg = self.joy()
        msg.buttons[:4] = [1] * 4
        msg.axes[2] = 1.0
        self.node._joy_callback(msg)
        self.assertFalse(self.node.mode_publisher.messages)
        self.assertEqual(self.node.command[0], 0)
        self.assertGreater(self.node.command[1], 0)
        self.assertEqual(self.node.command[2], 0)

    def test_steering_yaw_follows_forward_reverse_direction(self):
        for speed in (-1.0, 1.0):
            self.setUp()
            self.node._joy_callback(self.joy())
            self.now = Time(10.2)
            msg = self.joy()
            msg.axes[3] = speed
            msg.axes[0] = 1.0
            self.node._joy_callback(msg)
            self.assertGreater(self.node.command[2] * speed, 0)

    def test_nonfinite_or_out_of_range_axis_stops_motion(self):
        for value in (float('nan'), float('inf'), 1.1):
            self.seed_motion()
            msg = self.joy()
            msg.axes[3] = value
            self.node._joy_callback(msg)
            self.assertEqual(self.node.command, [0.0] * 3)

    def test_manual_motion_needs_no_shoulder_and_reports_no_fake_deadman(self):
        self.node._joy_callback(self.joy(moving=True))
        self.now = Time(10.1)
        self.node._joy_callback(self.joy(moving=True))
        self.node._publish()
        self.assertGreater(self.output()[0], 0)
        state = self.node.state_publisher.messages[-1]
        self.assertTrue(state.connected)
        self.assertTrue(state.command_active)
        self.assertFalse(state.deadman_pressed)
        self.assertFalse(state.turbo_active)

    def test_either_shoulder_boosts_and_both_do_not_stack(self):
        for buttons, expected in (((), 0.5), ((9,), 1.25), ((10,), 1.25), ((9, 10), 1.25)):
            with self.subTest(buttons=buttons):
                self.setUp()
                self.node.max_acceleration_linear = 100.0
                self.node._joy_callback(self.joy(moving=True))
                self.now = Time(10.1)
                msg = self.joy(moving=True)
                for button in buttons:
                    msg.buttons[button] = 1
                self.node._joy_callback(msg)
                self.node._publish()
                self.assertAlmostEqual(self.output()[0], expected)
                self.assertEqual(self.node.turbo_active, bool(buttons))
                self.assertTrue(self.node.state_publisher.messages[-1].command_active)

    def test_unused_left_vertical_axis_cannot_command_motion(self):
        self.node._joy_callback(self.joy())
        self.now = Time(10.1)
        msg = self.joy()
        msg.axes[1] = 1.0
        self.node._joy_callback(msg)
        self.assertEqual(self.node.command, [0.0] * 3)

    def test_invalid_layout_is_inactive_even_without_deadman_requirement(self):
        self.node._joy_callback(self.joy(moving=True))
        self.node._joy_callback(SimpleNamespace(axes=[], buttons=[]))
        self.node._publish()
        self.assertEqual(self.output(), [0.0] * 3)
        self.assertFalse(self.node.state_publisher.messages[-1].connected)
        self.assertFalse(self.node.state_publisher.messages[-1].command_active)

    def test_boost_and_optional_deadman_configuration_is_validated(self):
        for buttons in ([], [-1], [9, 9]):
            self.node.buttons_boost = buttons
            with self.assertRaises(ValueError):
                self.node._validate_parameters()
        self.node.buttons_boost = [9, 10]
        self.node.button_deadman = -2
        with self.assertRaises(ValueError):
            self.node._validate_parameters()
        self.node.button_deadman = 9
        with self.assertRaises(ValueError):
            self.node._validate_parameters()
        self.node.button_deadman = -1
        self.node.button_mode_switch = -1
        self.node._validate_parameters()


if __name__ == '__main__':
    unittest.main()
