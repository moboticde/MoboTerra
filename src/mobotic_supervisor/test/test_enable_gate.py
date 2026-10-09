"""Exercise the actual supervisor gates/control cycle without ROS installed.

Extract the relevant class methods and supply only clock/message test doubles.
This keeps the readiness scenarios runnable on the Windows development host.
"""

import ast
import copy
import math
from pathlib import Path
from types import SimpleNamespace
import unittest


class Duration:
    def __init__(self, *, seconds):
        self.nanoseconds = round(seconds * 1.0e9)

    def __le__(self, other):
        return self.nanoseconds <= other.nanoseconds


class Time:
    def __init__(self, seconds):
        self.nanoseconds = round(seconds * 1.0e9)

    def __sub__(self, other):
        return Duration(seconds=(self.nanoseconds - other.nanoseconds) / 1.0e9)

    def to_msg(self):
        return self.nanoseconds

    @staticmethod
    def from_msg(stamp):
        return Time(stamp / 1.0e9)


class TwistStamped:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None, frame_id='base_link')
        self.twist = SimpleNamespace(
            linear=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            angular=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        )


class JointState:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None)
        self.name = []
        self.position = []
        self.velocity = []
        self.effort = []


class WheelModuleCommand:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None)
        self.enable = False
        self.clear_errors = False


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


MODES = SimpleNamespace(MANUAL=0, AUTO_VELOCITY=1, AUTO_DIRECT=2, UNKNOWN=255)
SOURCE = Path(__file__).resolve().parents[1] / 'mobotic_supervisor' / 'supervisor.py'
tree = ast.parse(SOURCE.read_text(encoding='utf-8'))
supervisor_class = next(
    node for node in tree.body
    if isinstance(node, ast.ClassDef) and node.name == 'MoboticSupervisor'
)
supervisor_class.bases = []
supervisor_class.body = [
    method for method in supervisor_class.body
    if isinstance(method, ast.FunctionDef) and method.name != '__init__'
]
namespace = {
    'Duration': Duration,
    'Time': Time,
    'TwistStamped': TwistStamped,
    'JointState': JointState,
    'WheelModuleCommand': WheelModuleCommand,
    'VehicleModeState': MODES,
    'math': math,
    'copy': copy,
}
math_source = SOURCE.with_name('control_math.py')
exec(compile(math_source.read_text(encoding='utf-8'), str(math_source), 'exec'), namespace)
exec(compile(ast.Module(body=[supervisor_class], type_ignores=[]), str(SOURCE), 'exec'), namespace)
Supervisor = namespace['MoboticSupervisor']


class EnableGateTest(unittest.TestCase):
    def setUp(self):
        self.now = Time(10.0)
        self.node = Supervisor()
        node = self.node
        node.safety_timeout = 0.3
        node.battery_timeout = 0.6
        node.wheel_status_timeout = 0.3
        node.velocity_feedback_timeout = 0.3
        node.minimum_battery_percentage = 0.2
        node.safety_received = self.now
        node.safety_io_received = self.now
        node.battery_received = self.now
        node.safety_state = SimpleNamespace(
            header=SimpleNamespace(stamp=self.now.to_msg()),
            communication_ok=True, system_ready=True, motion_permitted=True,
            emergency_stop_active=False, sto_active=False,
            protective_stop_active=False, safety_override_active=False,
            warning_field_active=False, status_message='',
        )
        node.safety_io_state = SimpleNamespace(
            header=SimpleNamespace(stamp=self.now.to_msg()),
            emergency_stop_active=False, safety_enable_active=True, sto_active=False,
            front_left_protective_field_clear=True,
            rear_right_protective_field_clear=True,
            front_left_ossd_active=True, rear_right_ossd_active=True,
        )
        node.battery_state = SimpleNamespace(
            header=SimpleNamespace(stamp=self.now.to_msg()),
            communication_ok=True, system_ready=True, high_voltage_connected=True,
            minimum_percentage=0.8, status_message='',
        )
        node.expected_module_names = ['front', 'rear']
        node.wheel_status_received = None
        node.wheel_status = None
        node.get_clock = lambda: SimpleNamespace(now=lambda: self.now)
        node.module_command_publisher = Publisher()
        node.current_mode = MODES.MANUAL
        node.transition_in_progress = False
        node.transition_timeout = 10.0
        node.transition_timed_out = False
        node.transition_standstill_since = None
        node.measured_velocity = None
        node.measured_velocity_received = None
        node.joint_state = None
        node.joint_state_received = None
        node.last_output = [0.0, 0.0, 0.0]
        node.motion_inputs_ready = False
        node.command_accept_after = None
        node._source_gate = lambda now: (False, 'deadman released')
        node._publish_velocity = lambda *args, **kwargs: None
        node._publish_direct_stop = lambda *args: None
        node._publish_mode_state = lambda *args: None

    def test_soc_boundary_and_invalid_values(self):
        for soc, expected in [
            (0.199, False), (0.2, True), (0.201, True), (1.0, True),
            (math.nan, False), (math.inf, False), (-0.1, False), (1.1, False),
        ]:
            with self.subTest(soc=soc):
                self.node.battery_state.minimum_percentage = soc
                self.assertEqual(self.node._enable_gate(self.now)[0], expected)

    def test_missing_and_stale_readiness_disable(self):
        for attr in ('safety_received', 'safety_io_received', 'battery_received'):
            for received in (None, Time(9.0)):
                with self.subTest(attr=attr, received=received):
                    setattr(self.node, attr, received)
                    self.assertFalse(self.node._enable_gate(self.now)[0])
                    setattr(self.node, attr, self.now)

    def test_safety_and_battery_failures_disable(self):
        cases = [
            (self.node.safety_state, 'communication_ok', False),
            (self.node.safety_state, 'system_ready', False),
            (self.node.safety_state, 'motion_permitted', False),
            (self.node.safety_state, 'emergency_stop_active', True),
            (self.node.safety_state, 'sto_active', True),
            (self.node.safety_state, 'protective_stop_active', True),
            (self.node.safety_io_state, 'safety_enable_active', False),
            (self.node.safety_io_state, 'emergency_stop_active', True),
            (self.node.safety_io_state, 'sto_active', True),
            (self.node.safety_io_state, 'front_left_ossd_active', False),
            (self.node.battery_state, 'communication_ok', False),
            (self.node.battery_state, 'system_ready', False),
            (self.node.battery_state, 'high_voltage_connected', False),
        ]
        for state, attr, value in cases:
            with self.subTest(attr=attr):
                previous = getattr(state, attr)
                setattr(state, attr, value)
                self.assertFalse(self.node._enable_gate(self.now)[0])
                setattr(state, attr, previous)

    def test_missing_or_disabled_wheels_block_motion_but_allow_enable(self):
        self.assertTrue(self.node._enable_gate(self.now)[0])
        self.assertFalse(self.node._base_gate(self.now)[0])
        self.node.wheel_status_received = self.now
        self.node.wheel_status = SimpleNamespace(header=SimpleNamespace(stamp=self.now.to_msg()), modules=[
            SimpleNamespace(name=name, enabled=False, error_code=0, feedback_fresh=True)
            for name in self.node.expected_module_names
        ])
        self.assertTrue(self.node._enable_gate(self.now)[0])
        self.assertFalse(self.node._base_gate(self.now)[0])

    def test_warning_field_keeps_enable(self):
        self.node.safety_state.warning_field_active = True
        self.assertTrue(self.node._enable_gate(self.now)[0])

    def test_fresh_publication_with_stale_can_feedback_blocks_motion(self):
        self.node.wheel_status_received = self.now
        self.node.wheel_status = SimpleNamespace(header=SimpleNamespace(stamp=self.now.to_msg()), modules=[
            SimpleNamespace(name=name, enabled=True, error_code=0, feedback_fresh=True)
            for name in self.node.expected_module_names
        ])
        self.assertTrue(self.node._base_gate(self.now)[0])
        self.node.wheel_status.modules[0].feedback_fresh = False
        ready, reason = self.node._base_gate(self.now)
        self.assertFalse(ready)
        self.assertIn('feedback stale', reason)
        self.assertTrue(self.node._enable_gate(self.now)[0])
        self.node.wheel_status.modules[0].feedback_fresh = True
        self.assertTrue(self.node._base_gate(self.now)[0])

    def test_control_publishes_enable_without_active_motion_source(self):
        self.node._control()
        self.node._control()
        messages = self.node.module_command_publisher.messages
        self.assertEqual(len(messages), 2)
        self.assertTrue(all(message.enable for message in messages))
        self.assertTrue(all(not message.clear_errors for message in messages))
        self.assertEqual(messages[-1].header.stamp, self.now.to_msg())

    def test_control_disables_then_recovers_with_readiness(self):
        self.node.battery_received = None
        self.node._control()
        self.node.battery_received = self.now
        self.node._control()
        self.node.battery_state.minimum_percentage = 0.199
        self.node._control()
        self.assertEqual(
            [msg.enable for msg in self.node.module_command_publisher.messages],
            [False, True, False],
        )

    def test_control_disables_during_mode_transition(self):
        self.node.transition_in_progress = True
        self.node.transition_started_at = self.now
        self.node.transition_duration = 0.25
        self.node.transition_from_direct = False
        self.node.requested_mode = MODES.AUTO_VELOCITY
        self.node.safety_state.motion_permitted = False
        self.node._control()
        self.assertFalse(self.node.module_command_publisher.messages[-1].enable)


if __name__ == '__main__':
    unittest.main()
