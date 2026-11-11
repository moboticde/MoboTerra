"""Keep real ROS imports separate from the ROS-free controller unit doubles."""
import os
from pathlib import Path
import subprocess
import sys
import unittest


@unittest.skipUnless(os.environ.get('AMENT_PREFIX_PATH'), 'requires a sourced ROS installation')
class RombIntegrationTest(unittest.TestCase):
    def test_actual_dds_bursts_and_input_inhibition(self):
        env = dict(os.environ, ROS_DOMAIN_ID='169', ROS_LOCALHOST_ONLY='1')
        result = subprocess.run([sys.executable, str(Path(__file__).with_name('remdevice_romb_ros.py'))],
                                env=env, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout)


if __name__ == '__main__':
    unittest.main()
