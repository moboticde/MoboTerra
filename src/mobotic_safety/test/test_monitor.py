"""Test the actual FlexiSoft mapping/interlocks without ROS installed."""
import ast
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


class Message:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None)


source = Path(__file__).parents[1] / 'mobotic_safety' / 'safety_monitor.py'
tree = ast.parse(source.read_text(encoding='utf-8'))
cls = next(node for node in tree.body if isinstance(node, ast.ClassDef))
cls.bases = []
cls.body = [node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name != '__init__']
namespace = dict(SafetyIOState=Message, SafetyState=Message)
exec(compile(ast.Module(body=[cls], type_ignores=[]), str(source), 'exec'), namespace)


class SafetyMonitorTest(unittest.TestCase):
    def setUp(self):
        self.now = Time(10.0)
        self.node = namespace['SafetyMonitor']()
        node = self.node
        node.status_timeout = 0.25
        node.last_status_time = self.now
        node.status_byte = 0x0F
        node.front_ossd_mask, node.rear_ossd_mask = 0x03, 0x0C
        node.estop_mask, node.override_mask = 0x10, 0x20
        node.front_warning_mask, node.rear_warning_mask = 0x80, 0x40
        node.safety_enable_mask = node.sto_mask = 0
        node.estop_implies_sto = True
        node.get_clock = lambda: SimpleNamespace(now=lambda: self.now)
        self.io, self.state = [], []
        node.io_publisher = SimpleNamespace(publish=self.io.append)
        node.state_publisher = SimpleNamespace(publish=self.state.append)

    def test_reference_clear_fields_and_unmapped_sto(self):
        self.node._publish()
        self.assertTrue(self.state[-1].motion_permitted)
        self.assertFalse(self.io[-1].sto_state_known)
        self.assertFalse(self.io[-1].safety_enable_state_known)
        self.assertEqual(self.io[-1].flexisoft_status_raw, 0x0F)

    def test_estop_is_conservative_not_confirmed_sto(self):
        self.node.status_byte = 0x1F
        self.node._publish()
        self.assertTrue(self.io[-1].sto_active)
        self.assertFalse(self.io[-1].sto_state_known)
        self.assertFalse(self.state[-1].system_ready)

    def test_occupied_field_blocks_motion_and_override_is_visible(self):
        self.node.status_byte = 0x0C
        self.node._publish()
        self.assertFalse(self.state[-1].motion_permitted)
        self.node.status_byte = 0x2C
        self.node._publish()
        self.assertTrue(self.state[-1].safety_override_active)
        self.assertTrue(self.state[-1].motion_permitted)

    def test_timeout_and_backward_clock_fail_closed(self):
        for seconds in (10.3, 9.0):
            self.now = Time(seconds)
            self.node._publish()
            self.assertFalse(self.state[-1].communication_ok)
            self.assertFalse(self.state[-1].motion_permitted)
            self.assertTrue(self.io[-1].sto_active)
            self.assertFalse(self.io[-1].sto_state_known)


if __name__ == '__main__':
    unittest.main()
