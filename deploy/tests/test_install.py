"""Run the software installer with simulated host/Docker commands in a temp root."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


DEPLOY = Path(__file__).resolve().parents[1]


class SoftwareInstallationTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.deploy = self.root / 'repo' / 'deploy'
        shutil.copytree(DEPLOY, self.deploy)
        text = (self.deploy / 'install.sh').read_text()
        # Only the test copy bypasses root authentication; every filesystem write
        # is redirected to this temporary root and external operations are fake.
        text = text.replace('if [[ "${EUID}" -ne 0 ]]; then', 'if false; then')
        for prefix in ('/etc/', '/opt/', '/usr/local/', '/var/lib/', '/run/'):
            text = text.replace(prefix, str(self.root) + prefix)
        (self.deploy / 'install.sh').write_text(text)
        (self.root / 'etc').mkdir()
        (self.root / 'etc/os-release').write_text('ID=ubuntu\nVERSION_ID=24.04\n')
        self.bin = self.root / 'bin'
        self.bin.mkdir()
        self.calls = self.root / 'calls.jsonl'
        stub = '''#!/usr/bin/python3
import json, os, pathlib, sys
name=pathlib.Path(sys.argv[0]).name
args=sys.argv[1:]
with open(os.environ['TEST_CALLS'], 'a') as f:
    f.write(json.dumps([name, *args])+'\\n')
if name == 'dpkg': print('arm64')
if name == 'dpkg-query':
    if args[-1] == os.environ.get('DOCKER_PACKAGE'): print('install ok installed')
    else: sys.exit(1)
if name == 'docker' and args[:2] == ['compose', 'version']:
    if os.environ.get('DOCKER_PACKAGE') and not pathlib.Path(os.environ['PLUGIN_MARKER']).exists():
        sys.exit(1)
if name == 'apt-get' and 'install' in args:
    pathlib.Path(os.environ['PLUGIN_MARKER']).touch()
if name == 'docker' and args[:2] == ['image', 'inspect']:
    print('ros@sha256:'+'a'*64)
if name == 'docker' and args[:1] == ['pull'] and os.environ.get('FAIL_PULL'):
    sys.exit(1)
if name == 'docker' and args[:1] == ['build'] and os.environ.get('FAIL_BUILD'):
    sys.exit(1)
'''
        for name in ('docker', 'dpkg', 'dpkg-query', 'systemctl', 'ip', 'udevadm',
                     'candump', 'apt-get', 'modprobe', 'sleep'):
            file = self.bin / name
            file.write_text(stub)
            file.chmod(0o755)
        self.env = dict(os.environ, PATH=f'{self.bin}:/usr/bin:/bin',
                        TEST_CALLS=str(self.calls), PLUGIN_MARKER=str(self.root / 'plugin-installed'))

    def run_installer(self, *args, **env):
        return subprocess.run(['bash', str(self.deploy / 'install.sh'), *args],
                              env=dict(self.env, **env), capture_output=True,
                              text=True, timeout=15)

    def recorded_calls(self):
        return [json.loads(line) for line in self.calls.read_text().splitlines()]

    def test_no_adapters_installs_all_images_without_robot_service(self):
        result = self.run_installer('--software-only', '--ros-domain-id', '43')
        self.assertEqual(result.returncode, 0, result.stderr)
        calls = self.recorded_calls()
        builds = [call for call in calls if call[:2] == ['docker', 'build']]
        self.assertEqual([call[call.index('--target') + 1] for call in builds],
                         ['runtime', 'gui', 'tools'])
        self.assertFalse(any(call[0] == 'modprobe' for call in calls))
        self.assertFalse(any('moboterra-supervisor.service' in call for call in calls))
        self.assertFalse((self.root / 'etc/moboterra/moboterra.env').exists())
        self.assertFalse((self.root / 'opt/moboterra/current').exists())
        config = (self.root / 'etc/moboterra/software.env').read_text()
        self.assertIn('ROS_DOMAIN_ID=43', config)
        self.assertIn('MOBOTERRA_TOOLS_IMAGE=', config)
        release = self.root / 'opt/moboterra/software'
        self.assertTrue(release.is_symlink())
        self.assertIn('tools_image', json.loads((release / 'release.json').read_text()))
        self.assertTrue((self.root / 'usr/local/bin/moboterra-tools').is_symlink())
        self.assertTrue((self.root / 'usr/local/bin/moboterra-ctl').is_symlink())

    def test_failed_build_does_not_publish_configuration_or_launchers(self):
        result = self.run_installer('--software-only', FAIL_BUILD='1')
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse((self.root / 'etc/moboterra/software.env').exists())
        self.assertFalse((self.root / 'usr/local/bin/moboterra-gui').exists())

    def test_software_install_preserves_existing_hardware_configuration(self):
        hardware = self.root / 'etc/moboterra/moboterra.env'
        hardware.parent.mkdir()
        hardware.write_text('PLATFORM_CAN_SERIAL=real-platform\nBATTERY_CAN_SERIAL=real-battery\n')
        current = self.root / 'opt/moboterra/current'
        current.parent.mkdir(parents=True)
        current.symlink_to('releases/existing-hardware')
        result = self.run_installer('--software-only')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(hardware.read_text(),
                         'PLATFORM_CAN_SERIAL=real-platform\nBATTERY_CAN_SERIAL=real-battery\n')
        self.assertEqual(os.readlink(current), 'releases/existing-hardware')

    def test_missing_compose_installs_matching_package(self):
        for installed, expected in (('docker-ce', 'docker-compose-plugin'),
                                    ('docker.io', 'docker-compose-v2')):
            with self.subTest(installed=installed):
                marker = self.root / 'plugin-installed'
                marker.unlink(missing_ok=True)
                result = self.run_installer('--software-only', DOCKER_PACKAGE=installed)
                self.assertEqual(result.returncode, 0, result.stderr)
                installs = [call for call in self.recorded_calls()
                            if call[0] == 'apt-get' and 'install' in call]
                self.assertIn(expected, installs[-1])

    def test_download_failure_is_bounded(self):
        result = self.run_installer('--software-only', FAIL_PULL='1')
        self.assertNotEqual(result.returncode, 0)
        pulls = [call for call in self.recorded_calls() if call[:2] == ['docker', 'pull']]
        self.assertEqual(len(pulls), 3)
        self.assertFalse((self.root / 'etc/moboterra/software.env').exists())

    def test_reinstall_creates_distinct_releases(self):
        for _ in range(2):
            result = self.run_installer('--software-only')
            self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(len(list((self.root / 'opt/moboterra/releases').iterdir())), 2)

    def test_reinstall_preserves_domain_and_accepts_decimal_leading_zero(self):
        first = self.run_installer('--software-only', '--ros-domain-id', '08')
        self.assertEqual(first.returncode, 0, first.stderr)
        second = self.run_installer('--software-only')
        self.assertEqual(second.returncode, 0, second.stderr)
        self.assertIn('ROS_DOMAIN_ID=8\n',
                      (self.root / 'etc/moboterra/software.env').read_text())

    def test_invalid_domain_stops_before_external_operations(self):
        result = self.run_installer('--software-only', '--ros-domain-id', '233')
        self.assertNotEqual(result.returncode, 0)
        self.assertTrue(all(call[0] == 'dpkg' for call in self.recorded_calls()))


if __name__ == '__main__':
    unittest.main()
