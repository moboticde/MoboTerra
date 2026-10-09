import importlib.util
import math
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('telemetry', Path(__file__).parents[1] /
                                             'mobotic_vanguard_battery' / 'telemetry.py')
telemetry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(telemetry)


class BatteryDecodeTest(unittest.TestCase):
    def test_soc_is_fraction(self):
        self.assertAlmostEqual(telemetry.decode_soc(32000), 0.50016)
        self.assertLess(telemetry.decode_soc(12000), 0.2)
        self.assertGreaterEqual(telemetry.decode_soc(12800), 0.2)
        self.assertEqual(telemetry.decode_soc(64000), 1.0)

    def test_reserved_soc_is_not_full(self):
        for raw in (64001, 0xFFFE, 0xFFFF):
            self.assertTrue(math.isnan(telemetry.decode_soc(raw)))

    def test_electrical_decode_and_reserved(self):
        self.assertEqual(telemetry.decode_electrical(960, 32000), (48.0, 0.0))
        voltage, current = telemetry.decode_electrical(0xFFFF, 0xFFFF)
        self.assertFalse(telemetry.values_valid(0.5, voltage, current))

    def test_invalid_soc_cannot_hide_in_minimum(self):
        self.assertFalse(telemetry.values_valid(math.nan, 48.0, 0.0))
        self.assertTrue(telemetry.values_valid(0.5, 48.0, -1.0))


if __name__ == '__main__':
    unittest.main()
