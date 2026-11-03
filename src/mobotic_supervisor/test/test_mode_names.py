"""Public mode constants, configuration spellings and logs use canonical names."""
from pathlib import Path
import re
import unittest

import test_enable_gate as h


class ModeNamesTest(unittest.TestCase):
    def test_public_interfaces_have_only_canonical_mode_constants(self):
        package = Path(__file__).resolve().parents[2] / 'mobotic_interfaces'
        selected = {'MANUAL': 0, 'AUTONOMOUS': 1, 'KINEMATICOFF': 2}
        for relative, extra in (
                ('msg/VehicleModeState.msg', {'SAFETY': 3, 'ERROR': 4, 'UNKNOWN': 255}),
                ('msg/VehicleModeRequest.msg', {'UNKNOWN': 255}),
                ('srv/SetVehicleMode.srv', {})):
            with self.subTest(interface=relative):
                constants = re.findall(r'^uint8\s+([A-Z_]+)=(\d+)$',
                                       (package / relative).read_text(), re.MULTILINE)
                self.assertEqual(len(constants), len(selected) + len(extra))
                self.assertEqual({name: int(value) for name, value in constants}, selected | extra)

    def test_configuration_mode_ids_and_log_names_agree(self):
        for label, value in (('MANUAL', 0), ('AUTONOMOUS', 1), ('KINEMATICOFF', 2)):
            with self.subTest(label=label):
                self.assertEqual(h.Supervisor._mode_from_name(label.lower()), value)
                self.assertEqual(h.Supervisor._mode_name(value), label)


if __name__ == '__main__':
    unittest.main()
