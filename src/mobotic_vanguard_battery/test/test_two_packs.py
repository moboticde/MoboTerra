"""Two independent packs on one bus must never mask each other's telemetry loss."""
import ast
from dataclasses import dataclass, field
import math
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import test_monitor as harness


class NodeDouble:
    overrides = {}

    def __init__(self, name):
        self.parameters = dict(self.overrides)
        self.messages = {}

    def declare_parameter(self, name, default):
        self.parameters.setdefault(name, default)

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])

    def create_publisher(self, message_type, topic, depth):
        self.messages[topic] = []
        return SimpleNamespace(publish=self.messages[topic].append)

    def create_subscription(self, *args):
        return None

    def create_timer(self, *args):
        return None


tree = ast.parse(harness.source.read_text(encoding='utf-8'))
definitions = [node for node in tree.body if isinstance(node, (ast.ClassDef, ast.FunctionDef))
               and node.name != 'main']
namespace = dict(harness.namespace, __name__=__name__, Node=NodeDouble, Frame=object,
                 Parameter=SimpleNamespace(Type=SimpleNamespace(STRING_ARRAY=None)),
                 dataclass=dataclass, field=field)
exec(compile(ast.Module(body=definitions, type_ignores=[]), str(harness.source), 'exec'), namespace)


class TwoPackTest(unittest.TestCase):
    def setUp(self):
        # The same PGNs at different source addresses must remain independent.
        self.parameters = {
            'batteries.ids': ['battery1', 'battery2'],
            'batteries.source_addresses': ['0xF4', '0xF5'],
            'batteries.status_pgns': ['0xF096', '0xF096'],
            'batteries.soc_pgns': ['0xF091', '0xF091'],
            'batteries.voltage_pgns': ['0xF090', '0xF090'],
            'primary_battery_id': 'battery1',
        }
        self.now = harness.Time(10.0)
        self.node = self.construct()
        self.node.get_clock = lambda: SimpleNamespace(now=lambda: self.now)

    def construct(self):
        with patch.object(NodeDouble, 'overrides', self.parameters):
            return namespace['VanguardBatteryMonitor']()

    def send(self, identifier, *, soc=32000, voltage=960, current=32000, status=0x21,
             fields=('status', 'soc', 'voltage')):
        pack = self.node.batteries[identifier]
        data = {
            'status': [0, 0, 0, 0, 0, status, 0, 0],
            'soc': [soc & 255, soc >> 8, 0, 0, 0, 0, 0, 0],
            'voltage': [0, 0, 0, 0, voltage & 255, voltage >> 8, current & 255, current >> 8],
        }
        for key in fields:
            self.node._can_callback(SimpleNamespace(
                id=(getattr(pack, key + '_pgn') << 8) | pack.source_address,
                header=SimpleNamespace(stamp=self.now.to_msg()),
                is_extended=True, is_error=False, is_rtr=False, dlc=8, data=data[key],
            ))

    def system(self):
        self.node._publish()
        return self.node.messages['battery/system_state'][-1]

    def test_separate_topics_values_and_primary_alias(self):
        self.send('battery1', soc=51200, voltage=1000, current=31980)
        self.send('battery2', soc=16000, voltage=980, current=31960)
        state = self.system()
        first = self.node.messages['battery/battery1/state'][-1]
        second = self.node.messages['battery/battery2/state'][-1]
        self.assertTrue(state.system_ready)
        self.assertEqual((first.location, second.location), ('battery1', 'battery2'))
        self.assertAlmostEqual(first.percentage, 0.800256)
        self.assertAlmostEqual(second.percentage, 0.25008)
        self.assertEqual((first.voltage, second.voltage), (50.0, 49.0))
        self.assertEqual((first.current, second.current), (-1.0, -2.0))
        self.assertEqual(state.minimum_percentage, second.percentage)
        self.assertIs(self.node.messages['battery/state'][-1], first)

    def test_one_pack_profile_becomes_ready_without_a_second_pack(self):
        for key in self.parameters:
            if key.startswith('batteries.'):
                self.parameters[key] = self.parameters[key][:1]
        self.node = self.construct()
        self.node.get_clock = lambda: SimpleNamespace(now=lambda: self.now)
        self.send('battery1')
        state = self.system()
        self.assertTrue(state.communication_ok)
        self.assertTrue(state.system_ready)
        self.assertEqual(state.stale_batteries, [])
        self.assertEqual(set(self.node.messages), {
            'battery/battery1/state', 'battery/state', 'battery/system_state',
        })
        self.assertEqual(state.minimum_percentage,
                         self.node.messages['battery/battery1/state'][-1].percentage)
        self.now = harness.Time(10.6)
        state = self.system()
        self.assertFalse(state.system_ready)
        self.assertEqual(state.stale_batteries, ['battery1'])

    def test_one_pack_cannot_satisfy_both_required_participants(self):
        for identifier in ('battery1', 'battery2'):
            with self.subTest(identifier=identifier):
                self.setUp()
                self.send(identifier)
                state = self.system()
                self.assertFalse(state.communication_ok)
                self.assertFalse(state.system_ready)
                self.assertEqual(state.stale_batteries,
                                 ['battery2' if identifier == 'battery1' else 'battery1'])

    def test_each_pack_each_telegram_expires_independently(self):
        for identifier in ('battery1', 'battery2'):
            for missing in ('status', 'soc', 'voltage'):
                with self.subTest(identifier=identifier, missing=missing):
                    self.setUp()
                    self.send('battery1')
                    self.send('battery2')
                    self.now = harness.Time(10.6)
                    self.send('battery2' if identifier == 'battery1' else 'battery1')
                    self.send(identifier, fields=tuple(
                        key for key in ('status', 'soc', 'voltage') if key != missing))
                    state = self.system()
                    self.assertEqual(state.stale_batteries, [identifier])
                    self.assertFalse(state.system_ready)
                    self.assertTrue(math.isnan(state.minimum_percentage))
                    message = self.node.messages[f'battery/{identifier}/state'][-1]
                    if missing == 'soc':
                        self.assertTrue(math.isnan(message.percentage))
                    elif missing == 'voltage':
                        self.assertTrue(math.isnan(message.voltage))
                    else:
                        self.assertFalse(message.present)

    def test_either_pack_fault_invalid_data_or_hv_loss_blocks_readiness(self):
        for identifier in ('battery1', 'battery2'):
            for values in ({'status': 0x11}, {'status': 0x20}, {'soc': 65535}, {'voltage': 65535}):
                with self.subTest(identifier=identifier, values=values):
                    self.setUp()
                    self.send('battery1')
                    self.send('battery2')
                    self.now = harness.Time(10.01)
                    self.send(identifier, **values)
                    state = self.system()
                    self.assertFalse(state.system_ready)
                    if values == {'status': 0x20}:
                        self.assertFalse(state.high_voltage_connected)
                    else:
                        self.assertEqual(state.faulted_batteries, [identifier])

    def test_lowest_soc_is_exposed_for_supervisor_twenty_percent_gate(self):
        for identifier in ('battery1', 'battery2'):
            with self.subTest(identifier=identifier):
                self.setUp()
                self.send('battery1')
                self.send('battery2')
                self.now = harness.Time(10.01)
                self.send(identifier, soc=6400)
                self.assertAlmostEqual(self.system().minimum_percentage, 0.100032)

    def test_fresh_recovery_requires_the_missing_packs_telemetry(self):
        self.send('battery1')
        self.assertFalse(self.system().system_ready)
        self.send('battery2')
        self.assertTrue(self.system().system_ready)

    def test_duplicate_bindings_invalid_topic_ids_and_bad_pgns_are_rejected(self):
        changes = (
            ('batteries.source_addresses', ['0xF4', '0xF4']),
            ('batteries.ids', ['battery1', 'bad/name']),
            ('batteries.status_pgns', ['0x40000', '0xF096']),
        )
        for key, value in changes:
            with self.subTest(key=key):
                self.setUp()
                self.parameters[key] = value
                with self.assertRaises(ValueError):
                    self.construct()


if __name__ == '__main__':
    unittest.main()
