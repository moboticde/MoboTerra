"""Exercise actual diagnostic publication with ROS message/clock doubles."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest
from test_diagnostic_health import health


class Message:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None)
        self.status = []
        self.values = []


source = Path(__file__).parents[1] / 'mobotic_bringup' / 'platform_diagnostics.py'
tree = ast.parse(source.read_text(encoding='utf-8'))
cls = next(node for node in tree.body if isinstance(node, ast.ClassDef))
cls.bases = []
cls.body = [node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name != '__init__']
namespace = dict(DiagnosticArray=Message, DiagnosticStatus=Message,
                 KeyValue=lambda **kwargs: SimpleNamespace(**kwargs),
                 time=SimpleNamespace(monotonic=lambda: 10.0), OK=health.OK, WARN=health.WARN,
                 ERROR=health.ERROR, freshness=health.freshness,
                 battery_health=health.battery_health, safety_health=health.safety_health)
exec(compile(ast.Module(body=[cls], type_ignores=[]), str(source), 'exec'), namespace)


class PlatformDiagnosticsTest(unittest.TestCase):
    def setUp(self):
        self.node = namespace['PlatformDiagnostics']()
        node = self.node
        node.minimum_soc = 0.2
        node.cache = {}
        node.channels = {'safety/io_state': (None, 0.3), 'scanner/front_left/scan': (None, 0.5)}
        node.get_namespace = lambda: '/moboterra'
        now = SimpleNamespace(nanoseconds=int(10e9), to_msg=lambda: None)
        node.get_clock = lambda: SimpleNamespace(now=lambda: now)
        self.outputs = []
        node.publisher = SimpleNamespace(publish=self.outputs.append)

    def test_missing_sources_are_stale(self):
        self.node._publish()
        self.assertTrue(all(status.level == health.STALE for status in self.outputs[-1].status))

    def test_unknown_hardware_signals_are_warn_not_green(self):
        msg = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=0)),
                              sto_state_known=False, safety_enable_state_known=False, flexisoft_status_raw=15)
        self.node._receive('safety/io_state', msg)
        self.node._publish()
        status = self.outputs[-1].status[0]
        self.assertEqual(status.level, health.WARN)
        self.assertIn('unmapped', status.message)
        self.assertEqual(status.values[0].value, '0xf')

    def test_receipt_does_not_make_old_scan_fresh(self):
        msg = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=9, nanosec=0)), ranges=[1.0])
        self.node._receive('scanner/front_left/scan', msg)
        self.node._publish()
        self.assertEqual(self.outputs[-1].status[1].level, health.STALE)
        msg.header.stamp.sec = 10
        self.node._publish()
        self.assertEqual(self.outputs[-1].status[1].level, health.OK)

    def test_fresh_odometry_is_not_treated_as_laser_scan(self):
        self.node.channels = {'odometry': (None, 0.5)}
        self.node._receive('odometry', SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=0))))
        self.node._publish()
        self.assertEqual(self.outputs[-1].status[0].level, health.OK)


if __name__ == '__main__':
    unittest.main()
