"""Catch collisions with rclpy's read-only Node API without a ROS installation."""
import ast
import copy
import math
import time
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch


class Instant:
    def __init__(self, seconds):
        self.nanoseconds = round(seconds * 1e9)

    def to_msg(self):
        return SimpleNamespace(sec=self.nanoseconds // 10**9, nanosec=self.nanoseconds % 10**9)

    @staticmethod
    def from_msg(stamp):
        return Instant(stamp.sec + stamp.nanosec * 1e-9)

    def __sub__(self, other):
        return SimpleNamespace(nanoseconds=self.nanoseconds - other.nanoseconds)


class Message:
    OK, WARN, ERROR = 0, 1, 2

    def __init__(self, **kwargs):
        self.header = SimpleNamespace(stamp=Instant(0).to_msg())
        self.communication_ok = False
        self.protective_state_known = self.warning_state_known = self.ossd_state_known = False
        self.protective_field_clear = self.warning_field_clear = self.ossd_active = False
        self.field_set_known = False
        self.scanner_id = self.source = ''
        self.__dict__.update(kwargs)


class ReadOnlyPublisherNode:
    def __init__(self, name):
        self.created_publishers = []
        self.inputs = []
        self.now = 10.0

    @property
    def publishers(self):
        return iter(self.created_publishers)

    def declare_parameter(self, name, value):
        pass

    def get_parameter(self, name):
        return SimpleNamespace(value={'status_timeout': 0.3, 'scanner_source': 'scanner'}[name])

    def create_publisher(self, kind, topic, depth):
        publisher = SimpleNamespace(topic=topic, messages=[])
        publisher.publish = publisher.messages.append
        self.created_publishers.append(publisher)
        return publisher

    def create_subscription(self, *args):
        self.inputs.append(args)
        return object()

    def create_timer(self, *args):
        return object()

    def get_clock(self):
        return SimpleNamespace(now=lambda: Instant(self.now))


class ScannerSafetyTest(unittest.TestCase):
    def setUp(self):
        source = Path(__file__).parents[1] / 'mobotic_safety/scanner_safety.py'
        cls = next(n for n in ast.parse(source.read_text()).body if isinstance(n, ast.ClassDef))
        namespace = dict(Node=ReadOnlyPublisherNode, SafetyIOState=object,
                         ScannerSafetyState=Message, DiagnosticArray=Message,
                         DiagnosticStatus=Message, KeyValue=Message, RawMicroScanData=object,
                         qos_profile_sensor_data=object(), math=math, copy=copy, time=time, Time=Instant)
        exec(compile(ast.Module(body=[cls], type_ignores=[]), str(source), 'exec'), namespace)
        self.node = namespace['ScannerSafetyAdapter']()
        self.monotonic = patch.object(time, 'monotonic', side_effect=lambda: self.node.now)
        self.monotonic.start()
        self.addCleanup(self.monotonic.stop)

    def test_adapter_constructs_with_read_only_node_publishers(self):
        node = self.node
        self.assertEqual({p.topic for p in node.publishers}, {
            'scanner/front_left/safety_zone_status', 'scanner/rear_right/safety_zone_status',
            'scanner/front_left/diagnostics', 'scanner/rear_right/diagnostics',
        })
        self.assertEqual([args[1] for args in node.inputs], [
            'scanner/front_left/raw_data', 'scanner/rear_right/raw_data'])

    def raw(self, *, red=False, yellow=False, sequence=1, valid=None):
        return SimpleNamespace(
            header=SimpleNamespace(sequence_number=sequence),
            general_system_state=SimpleNamespace(run_mode_active=True, standby_mode_active=False,
                application_error=False, device_error=False, contamination_error=False),
            application_data=SimpleNamespace(outputs=SimpleNamespace(
                evaluation_path_outputs_eval_out=[not red, not yellow, True],
                evaluation_path_outputs_is_safe=[True, False, False],
                evaluation_path_outputs_is_valid=[True, True, False] if valid is None else valid)))

    def latest(self, name='front_left'):
        return self.node.scanner_publishers[name].messages[-1]

    def test_clear_yellow_red_and_clear_again(self):
        for sequence, (red, yellow) in enumerate(((False, False), (False, True),
                                                 (True, True), (False, False)), 1):
            self.node.now += .02
            self.node._receive_scanner('front_left', self.raw(red=red, yellow=yellow, sequence=sequence))
            state = self.latest()
            self.assertTrue(state.communication_ok and state.protective_state_known and state.warning_state_known)
            self.assertEqual(state.protective_field_clear, not red)
            self.assertEqual(state.warning_field_clear, not yellow)
            self.assertFalse(state.ossd_state_known or state.field_set_known)

    def test_scanners_are_independent_and_lost_stream_becomes_unknown(self):
        self.node._receive_scanner('front_left', self.raw())
        self.node._receive_scanner('rear_right', self.raw(yellow=True))
        self.assertTrue(self.latest().warning_field_clear)
        self.assertFalse(self.latest('rear_right').warning_field_clear)
        stamp = self.latest().header.stamp
        self.node.now += .31
        self.node._publish()
        self.assertFalse(self.latest().communication_ok)
        self.assertEqual(self.latest().header.stamp.sec, stamp.sec)
        self.assertEqual(self.latest().header.stamp.nanosec, stamp.nanosec)

    def test_duplicates_and_reordering_do_not_refresh_a_clear_sample(self):
        self.node._receive_scanner('front_left', self.raw(sequence=10))
        for seconds, sequence in ((.1, 10), (.1, 9), (.11, 10), (.1, 10)):
            self.node.now += seconds
            self.node._receive_scanner('front_left', self.raw(sequence=sequence))
        self.node._publish()
        self.assertFalse(self.latest().communication_ok)
        self.node._receive_scanner('front_left', self.raw(sequence=11))
        self.assertTrue(self.latest().communication_ok)

    def test_sequence_rollover_and_restart(self):
        self.node._receive_scanner('front_left', self.raw(sequence=0xffffffff))
        self.node.now += .02
        self.node._receive_scanner('front_left', self.raw(sequence=0))
        self.assertTrue(self.latest().communication_ok)
        self.node._receive_scanner('front_left', self.raw(sequence=100))
        self.node.now += .31
        self.node._receive_scanner('front_left', self.raw(sequence=1))
        self.assertFalse(self.latest().communication_ok)
        self.node.now += .02
        self.node._receive_scanner('front_left', self.raw(sequence=2))
        self.assertTrue(self.latest().communication_ok)

    def test_missing_invalid_paths_and_device_faults_never_report_clear(self):
        self.node._receive_scanner('front_left', self.raw())
        bad = []
        for attr in ('application_error', 'device_error', 'contamination_error', 'standby_mode_active'):
            raw = self.raw()
            setattr(raw.general_system_state, attr, True)
            bad.append(raw)
        bad.append(self.raw(valid=[True, False, False]))
        raw = self.raw()
        raw.application_data.outputs.evaluation_path_outputs_is_safe = []
        bad.append(raw)
        for seq, raw in enumerate(bad, 2):
            raw.header.sequence_number = seq
            self.node.now += .02
            self.node._receive_scanner('front_left', raw)
            self.assertFalse(self.latest().communication_ok)

    def test_steady_clock_timeout_even_if_ros_time_pauses(self):
        self.node._receive_scanner('front_left', self.raw())
        with patch.object(time, 'monotonic', return_value=10.31):
            self.node._publish()
        self.assertFalse(self.latest().communication_ok)


if __name__ == '__main__':
    unittest.main()
