"""Configuration and installation contract; does not emulate a ROS build."""
import ast
from pathlib import Path
import re
import unittest
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'mobotic_config'))
from mobotic_config.configuration import Configuration
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).parents[1]


class PackageContractTest(unittest.TestCase):
    def test_body_frame_and_timeout_agree_with_production_kinematics(self):
        cfg = Configuration()
        odometry, kinematics = cfg.parameters('odometry'), cfg.parameters('kinematics')
        self.assertEqual(odometry['base_frame_id'], kinematics['output_frame_id'])
        self.assertLessEqual(odometry['feedback_timeout'], kinematics['joint_feedback_timeout'])
        self.assertEqual(odometry['odom_frame_id'], 'odom')

    def test_manifest_declares_message_and_tf_runtime_dependencies(self):
        manifest = ET.parse(PACKAGE / 'package.xml').getroot()
        dependencies = {element.text for element in manifest.findall('exec_depend')}
        for name in ('rclpy', 'nav_msgs', 'geometry_msgs', 'diagnostic_msgs', 'std_srvs', 'tf2_ros_py', 'launch', 'launch_ros'):
            self.assertIn(name, dependencies)
        bringup = ET.parse(PACKAGE.parent / 'mobotic_bringup' / 'package.xml').getroot()
        self.assertIn('mobotic_odometry', {item.text for item in bringup.findall('exec_depend')})

    def test_setup_installs_resource_configuration_launch_and_entrypoint(self):
        source = (PACKAGE / 'setup.py').read_text(encoding='utf-8')
        recorded = {}
        def setup(**kwargs):
            recorded.update(kwargs)
        tree = ast.parse(source)
        exec(compile(ast.Module(body=[item for item in tree.body if not isinstance(item, (ast.Import, ast.ImportFrom))],
                                type_ignores=[]), 'setup.py', 'exec'), {'setup': setup, 'find_packages': lambda **_: []})
        for _, files in recorded['data_files']:
            for filename in files:
                self.assertTrue((PACKAGE / filename).is_file(), filename)
        self.assertEqual(recorded['entry_points']['console_scripts'], ['mobotic_odometry = mobotic_odometry.odometry_node:main'])
        self.assertIn('install_scripts=$base/lib/mobotic_odometry', (PACKAGE / 'setup.cfg').read_text())


if __name__ == '__main__':
    unittest.main()
