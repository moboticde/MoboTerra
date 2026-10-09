"""ROS-free wiring checks linking the independently tested mode components."""
import ast
from pathlib import Path
import re
import unittest
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'mobotic_config'))
from mobotic_config.configuration import Configuration

PACKAGES = Path(__file__).resolve().parents[2]


class ModeRoutingTest(unittest.TestCase):
    def test_direct_selection_has_no_extra_permission_gate(self):
        source = (PACKAGES / 'mobotic_supervisor/mobotic_supervisor/supervisor.py').read_text()
        config = (PACKAGES / 'mobotic_config/config/supervisor.yaml').read_text()
        self.assertNotIn('allow_auto_direct', source)
        self.assertNotIn('allow_auto_direct', config)

    def test_usb_logitech_defaults_match_standardized_layout_and_no_face_buttons(self):
        source = (PACKAGES / 'mobotic_manual_control/mobotic_manual_control/manual_control.py').read_text()
        config = (PACKAGES / 'mobotic_config/config/manual_control.yaml').read_text()
        tree = ast.parse(source)
        defaults = {call.args[0].value: ast.literal_eval(call.args[1]) for call in ast.walk(tree)
                    if isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute)
                    and call.func.attr == 'declare_parameter' and len(call.args) == 2}
        keys = ('button_mode_switch', 'mode_switch_manual_value', 'mode_switch_auto_velocity_value')
        self.assertEqual([defaults[key] for key in keys], [4, 0, 1])
        for key in keys:
            self.assertRegex(config, rf'(?m)^\s*{key}: {defaults[key]}(?:\s+#.*)?$')
        self.assertEqual(defaults['mode_switch_type'], 'toggle')
        self.assertEqual(defaults['button_deadman'], -1)
        self.assertEqual(defaults['buttons_boost'], [9, 10])
        self.assertEqual((defaults['axis_speed'], defaults['axis_crab'], defaults['axis_steer']), (3, 2, 0))
        for key in ('axis_speed', 'axis_crab', 'axis_steer', 'button_deadman'):
            self.assertRegex(config, rf'(?m)^\s*{key}: {defaults[key]}(?:\s+#.*)?$')
        self.assertRegex(config, r'(?m)^\s*buttons_boost: \[9, 10\](?:\s+#.*)?$')
        for launch in ('mobotic_bringup/launch/moboterra.launch.py',
                       'mobotic_config/mobotic_config/launching.py'):
            wiring = (PACKAGES / launch).read_text()
            self.assertIn("executable='game_controller_node'", wiring)
            self.assertIn("cfg.platform['usb_joystick']", wiring)
        self.assertFalse(Configuration().platform['usb_joystick']['sticky_buttons'])
        for removed in ('button_mode_manual', 'button_mode_auto_velocity', 'button_mode_auto_direct'):
            self.assertNotIn(removed, defaults)
        self.assertIn("VehicleModeRequest, 'vehicle/mode_request'", source)

    def test_driver_labels_each_motion_input_and_consumes_mode_authority(self):
        source = (PACKAGES / 'mobotic_driver/src/mobotic_driver.cpp').read_text()
        for topic, label in (('kinematics/joint_setpoints', 'Kinematics'),
                             ('supervisor/joint_setpoints', 'SupervisorDirect')):
            self.assertRegex(source, re.escape('"' + topic + '"') +
                             r'[^;]*jointSetpointsCallback\(command, MotionSourceGate::Source::' + label)
        self.assertIn('"vehicle/mode_state"', source)
        self.assertIn('motion_source_gate_.permits(source', source)
        self.assertIn('this->enforceModeAuthority();', source)
        self.assertIn('output.header = msg.header;',
                      (PACKAGES / 'mobotic_kinematics/src/mobotic_kinematics.cpp').read_text())


if __name__ == '__main__':
    unittest.main()
