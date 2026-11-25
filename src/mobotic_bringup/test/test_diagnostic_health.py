import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest

spec = importlib.util.spec_from_file_location('diagnostic_health', Path(__file__).parents[1] /
                                             'mobotic_bringup' / 'diagnostic_health.py')
health = importlib.util.module_from_spec(spec)
spec.loader.exec_module(health)


class DiagnosticHealthTest(unittest.TestCase):
    def test_missing_and_stale_messages(self):
        self.assertEqual(health.freshness(None, 10.0, 0.3)[0], health.STALE)
        self.assertEqual(health.freshness(9.0, 10.0, 0.3)[0], health.STALE)
        self.assertEqual(health.freshness(10.0, 10.0, 0.3, 0.01)[0], health.OK)

    def test_fresh_transport_does_not_hide_stale_header(self):
        self.assertEqual(health.freshness(10.0, 10.0, 0.3, 1.0)[0], health.STALE)
        self.assertEqual(health.freshness(10.0, 10.0, 0.3, -1.0)[0], health.STALE)

    def test_battery_boundary_and_invalid(self):
        msg = SimpleNamespace(communication_ok=True, system_ready=True,
                              high_voltage_connected=True, minimum_percentage=0.2)
        self.assertEqual(health.battery_health(msg)[0], health.OK)
        for value in (0.199, float('nan'), 1.5):
            msg.minimum_percentage = value
            self.assertEqual(health.battery_health(msg)[0], health.WARN if value == 0.199 else health.ERROR)

if __name__ == '__main__':
    unittest.main()
