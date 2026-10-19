"""Synthetic assignments only; these are not the installed PAIL mapping."""
import copy
import unittest
from mobotic_config.operator_inputs import RemdeviceMapping


def example_mapping():
    return {'verified': True, 'node_id': 9,
            'axes': {role: {'channel': channel, 'zero': 0, 'full_scale': 255, 'invert': False}
                     for role, channel in (('speed', 1), ('steer', 2), ('crab', 3))},
            'buttons': {'mode': {'signal': 'SP1', 'active_high': True},
                        'boost': {'signal': 'SP2', 'active_high': True},
                        'reset': {'signal': 'SP3', 'active_high': True}, 'deadman': None}}


class MappingTest(unittest.TestCase):
    def test_unverified_is_capture_only_and_has_no_assignments(self):
        mapping = RemdeviceMapping({'verified': False, 'node_id': 9, 'axes': {}, 'buttons': {}})
        self.assertFalse(mapping.verified)
        self.assertEqual(mapping.node_id, 9)

    def test_verified_axes_no_deadman_and_explicit_polarity(self):
        value = example_mapping()
        value['buttons']['mode']['active_high'] = False
        mapping = RemdeviceMapping(value)
        self.assertFalse(mapping.button('deadman', {}))
        self.assertTrue(mapping.button('mode', {'SP1': False}))
        value['buttons']['mode']['signal'] = 'SP4'
        self.assertTrue(mapping.button('mode', {'SP1': False}))

    def test_invalid_or_ambiguous_control_maps_rejected(self):
        mutations = [lambda m: m.update(node_id=True), lambda m: m.update(node_id=128),
                     lambda m: m.update(verified=False), lambda m: m.update(verified='true'),
                     lambda m: m['axes']['speed'].update(channel=2),
                     lambda m: m['axes']['speed'].update(channel=9),
                     lambda m: m['axes']['speed'].update(zero=255),
                     lambda m: m['axes']['speed'].update(full_scale=True),
                     lambda m: m['axes']['speed'].update(invert=1),
                     lambda m: m['buttons']['reset'].update(signal='SP1'),
                     lambda m: m['buttons']['mode'].update(signal='STOP'),
                     lambda m: m['buttons']['boost'].update(active_high=1),
                     lambda m: m['buttons'].pop('deadman'),
                     lambda m: m['axes'].pop('steer')]
        for change in mutations:
            mapping = copy.deepcopy(example_mapping()); change(mapping)
            with self.subTest(mapping=mapping), self.assertRaises(ValueError):
                RemdeviceMapping(mapping)
