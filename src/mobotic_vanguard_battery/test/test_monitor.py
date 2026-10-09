"""Exercise actual battery monitor methods with ROS message/clock doubles."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from test_telemetry import telemetry


class Time:
    def __init__(self, seconds):
        self.nanoseconds = round(seconds * 1e9)
        self.clock_type = 1
    def __sub__(self, other):
        return SimpleNamespace(nanoseconds=self.nanoseconds - other.nanoseconds)
    def to_msg(self):
        return self.nanoseconds
    @staticmethod
    def from_msg(stamp, *, clock_type=1):
        return Time(stamp / 1e9)


class Message:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None, frame_id='')


for index, name in enumerate(('POWER_SUPPLY_TECHNOLOGY_UNKNOWN', 'POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE',
                             'POWER_SUPPLY_HEALTH_GOOD', 'POWER_SUPPLY_HEALTH_UNSPEC_FAILURE',
                             'POWER_SUPPLY_STATUS_UNKNOWN', 'POWER_SUPPLY_STATUS_CHARGING',
                             'POWER_SUPPLY_STATUS_DISCHARGING', 'POWER_SUPPLY_STATUS_NOT_CHARGING')):
    setattr(Message, name, index)

source = Path(__file__).parents[1] / 'mobotic_vanguard_battery' / 'vanguard_battery_monitor.py'
tree = ast.parse(source.read_text(encoding='utf-8'))
cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'VanguardBatteryMonitor')
cls.bases = []
cls.body = [node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name != '__init__']
namespace = dict(math=math, Time=Time, BatteryState=Message, BatterySystemState=Message,
                 HIGH_VOLTAGE_CONNECTED=1, OPERATIONAL=2,
                 decode_soc=telemetry.decode_soc, decode_electrical=telemetry.decode_electrical,
                 values_valid=telemetry.values_valid)
exec(compile(ast.Module(body=[cls], type_ignores=[]), str(source), 'exec'), namespace)


class BatteryMonitorTest(unittest.TestCase):
    def setUp(self):
        self.now = Time(10.0)
        self.node = namespace['VanguardBatteryMonitor']()
        node = self.node
        node.telemetry_timeout = 0.5
        node.primary_battery_id = 'system'
        node.get_clock = lambda: SimpleNamespace(now=lambda: self.now)
        self.battery = SimpleNamespace(identifier='system', source_address=0xF3, status_pgn=0xF096,
            soc_pgn=0xF091, voltage_pgn=0xF090, high_voltage_status=1, operational_status=2,
            percentage=0.5, voltage=48.0, current=0.0, status_stamp=self.now,
            soc_stamp=self.now, electrical_stamp=self.now, acquisition_stamps={})
        node.batteries = {'system': self.battery}
        self.primary, self.system = [], []
        node.primary_publisher = SimpleNamespace(publish=self.primary.append)
        node.system_state_publisher = SimpleNamespace(publish=self.system.append)

    def test_valid_system_and_single_public_battery(self):
        self.node._publish()
        self.assertTrue(self.system[-1].system_ready)
        self.assertEqual(self.system[-1].minimum_percentage, 0.5)
        self.assertEqual(len(self.primary), 1)

    def test_invalid_soc_blocks_readiness(self):
        self.battery.percentage = math.nan
        self.node._publish()
        self.assertFalse(self.system[-1].system_ready)
        self.assertEqual(self.system[-1].faulted_batteries, ['system'])
        self.assertTrue(math.isnan(self.system[-1].minimum_percentage))

    def test_any_missing_participant_blocks_system(self):
        missing = SimpleNamespace(**vars(self.battery))
        missing.identifier = 'internal'
        missing.soc_stamp = None
        self.node.batteries['internal'] = missing
        self.node._publish()
        self.assertFalse(self.system[-1].communication_ok)
        self.assertEqual(self.system[-1].stale_batteries, ['internal'])

    def test_timeout_and_backward_clock_fail_closed(self):
        for seconds in (10.6, 9.0):
            self.now = Time(seconds)
            self.node._publish()
            self.assertFalse(self.system[-1].system_ready)

    def test_can_soc_path_and_invalid_frames(self):
        frame = SimpleNamespace(id=(0xF091 << 8) | 0xF3, is_extended=True,
                                header=SimpleNamespace(stamp=Time(0.0).to_msg()),
                                is_error=False, is_rtr=False, dlc=8, data=[0, 125, 0, 0, 0, 0, 0, 0])
        self.node._can_callback(frame)
        self.assertAlmostEqual(self.battery.percentage, 0.50016)
        frame.data[:2] = [255, 255]
        frame.is_error = True
        self.node._can_callback(frame)
        self.assertAlmostEqual(self.battery.percentage, 0.50016)
        frame.is_error = False
        self.node._can_callback(frame)
        self.assertTrue(math.isnan(self.battery.percentage))


if __name__ == '__main__':
    unittest.main()
