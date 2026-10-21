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

    @staticmethod
    def from_msg(stamp):
        return Time(stamp / 1e9)


class Message:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None)


source = Path(__file__).parents[1] / 'mobotic_safety' / 'safety_monitor.py'
tree = ast.parse(source.read_text(encoding='utf-8'))
cls = next(node for node in tree.body if isinstance(node, ast.ClassDef))
cls.bases = []
cls.body = [node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name != '__init__']
namespace = dict(SafetyIOState=Message, SafetyState=Message, Time=Time)
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

    def test_one_ossd_channel_cannot_hide_the_other_low_channel(self):
        for value in (0x0D, 0x0E, 0x07, 0x0B):
            self.node.status_byte = value
            self.node._publish()
            self.assertFalse(self.state[-1].motion_permitted)
            self.assertTrue(self.state[-1].protective_stop_active)

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
            self.assertFalse(self.io[-1].sto_active)  # Timeout is unknown physical state, not confirmed STO.
            self.assertFalse(self.io[-1].sto_state_known)


class HardwiredStoMonitorTest(unittest.TestCase):
    def setUp(self):
        SafetyMonitorTest.setUp(self)
        self.node.source = 'hardwired_sto'
        self.node.expected_modules = ['front', 'rear']
        self.node.last_module_time = self.now
        self.node.last_modules = SimpleNamespace(modules=[
            SimpleNamespace(name=name, sto_state_known=True, sto_active=False, feedback_fresh=True)
            for name in self.node.expected_modules])

    def test_clear_sto_does_not_fabricate_scanner_estop_or_enable_inputs(self):
        self.node._publish()
        self.assertTrue(self.state[-1].physical_state_known)
        self.assertTrue(self.io[-1].sto_state_known)
        self.assertFalse(self.io[-1].sto_active)
        self.assertFalse(self.io[-1].scanner_state_known)
        self.assertFalse(self.io[-1].emergency_stop_state_known)
        self.assertFalse(self.io[-1].safety_enable_state_known)
        self.assertFalse(self.io[-1].safety_enable_active)
        self.assertFalse(self.io[-1].front_left_ossd_active)
        self.assertTrue(self.state[-1].warning_field_active)

    def test_unknown_sto_or_missing_feedback_on_either_module_blocks(self):
        for module in self.node.last_modules.modules:
            for field in ('sto_state_known', 'feedback_fresh'):
                setattr(module, field, False)
                self.node._publish()
                self.assertFalse(self.state[-1].motion_permitted)
                self.assertFalse(self.state[-1].physical_state_known)
                setattr(module, field, True)

    def test_missing_duplicate_or_wrong_modules_cannot_confirm_clear(self):
        modules = self.node.last_modules.modules
        for selection in ([], modules[:1], [modules[0], modules[0]]):
            self.node.last_modules.modules = selection
            self.node._publish()
            self.assertFalse(self.state[-1].motion_permitted)
        self.node.last_modules.modules = modules

    def test_one_known_active_sto_inhibits_even_with_partial_feedback(self):
        self.node.last_modules.modules = self.node.last_modules.modules[:1]
        self.node.last_modules.modules[0].sto_active = True
        self.node._publish()
        self.assertTrue(self.io[-1].sto_state_known)
        self.assertTrue(self.state[-1].sto_active)
        self.assertFalse(self.state[-1].motion_permitted)

    def test_stale_and_backward_clock_never_keep_a_clear_sto(self):
        for seconds in (10.3, 9):
            self.now = Time(seconds)
            self.node._publish()
            self.assertFalse(self.state[-1].motion_permitted)
            self.assertFalse(self.io[-1].sto_state_known)

    def test_delayed_or_duplicate_module_messages_do_not_renew_freshness(self):
        for seconds in (9, 10, 11):
            msg = SimpleNamespace(header=SimpleNamespace(stamp=Time(seconds).to_msg()), modules=[])
            self.node._module_callback(msg)
            self.assertEqual(self.node.last_module_time.nanoseconds, Time(10).nanoseconds)


if __name__ == '__main__':
    unittest.main()
