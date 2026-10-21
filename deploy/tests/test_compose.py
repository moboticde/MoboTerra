"""Render real Compose commands and check optional ROS policy-file defaults."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/mobotic_config'))
from mobotic_config.configuration import COMPONENT_FILES, Configuration


class ComposeLaunchTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not shutil.which('docker'):
            raise unittest.SkipTest('Docker Compose CLI is not installed')
        if subprocess.run(['docker', 'compose', 'version'], capture_output=True).returncode:
            raise unittest.SkipTest('Docker Compose CLI is not installed')

    def render(self, overlays):
        # Compose config only parses YAML; it does not contact the Docker daemon
        # or start containers, and needs no root privileges or CAN adapters.
        env = {key: value for key, value in os.environ.items()
               if not key.endswith('_PARAMETERS_FILE')}
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / 'robot.env'
            values = dict(MOBOTERRA_IMAGE='moboterra:test', INPUT_GID='1000',
                          ROS_DOMAIN_ID='42', **overlays)
            config.write_text(''.join(f'{key}={value}\n' for key, value in values.items()))
            result = subprocess.run(['docker', 'compose', '--env-file', str(config),
                                     '--file', str(ROOT / 'deploy/compose.yaml'),
                                     'config', '--format', 'json'],
                                    env=env, capture_output=True, text=True, check=True)
        command = json.loads(result.stdout)['services']['robot']['command']
        self.assertFalse(any(arg.endswith(':=') for arg in command), command)
        return dict(arg.split(':=', 1) for arg in command if ':=' in arg)

    def test_unset_and_empty_overlays_preserve_default_policy(self):
        variables = ('DRIVER', 'SUPERVISOR', 'MANUAL', 'SAFETY', 'BATTERY',
                     'KINEMATICS', 'DIAGNOSTICS', 'ODOMETRY', 'SCANNER')
        for supplied in ({}, {key + '_PARAMETERS_FILE': '' for key in variables}):
            with self.subTest(supplied=supplied):
                arguments = self.render(supplied)
                overrides = {}
                for component, filename in COMPONENT_FILES.items():
                    argument = 'scanner' if component == 'scanners' else component
                    selected = Path(arguments[argument + '_parameters_file'])
                    self.assertEqual(selected.parent, Path('/opt/moboterra/share/mobotic_config/config'))
                    self.assertEqual(selected.name, filename)
                    overrides[component] = ROOT / 'src/mobotic_config/config' / selected.name
                default = Configuration()
                explicit = Configuration(overrides=overrides)
                for component in COMPONENT_FILES:
                    self.assertEqual(default.parameters(component), explicit.parameters(component))

    def test_custom_overlay_path_reaches_ros_unchanged(self):
        arguments = self.render({'DRIVER_PARAMETERS_FILE': '/etc/moboterra/custom-driver.yaml'})
        self.assertEqual(arguments['driver_parameters_file'], '/etc/moboterra/custom-driver.yaml')


if __name__ == '__main__':
    unittest.main()
