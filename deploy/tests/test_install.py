"""Exercise the actual shell/Python installer using fake host and Docker commands."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

DEPLOY = Path(__file__).resolve().parents[1]

STUB = '''#!/usr/bin/python3
import json,os,pathlib,sys
name=pathlib.Path(sys.argv[0]).name
args=sys.argv[1:]
root=pathlib.Path(os.environ['TEST_ROOT'])
with (root/'calls.jsonl').open('a') as f:f.write(json.dumps([name,*args])+'\\n')
kind=os.environ.get('DOCKER_KIND','complete')
if name=='dpkg':print('amd64')
if name=='dpkg-query':
 expected='docker.io' if kind=='distro' else 'docker-ce'
 if kind!='missing' and args[-1]==expected:print('install ok installed')
 else:sys.exit(1)
if name=='apt-get' and 'install' in args:
 if any(p in args for p in ['docker-ce','docker.io']):
  target=root/'bin/docker';target.write_text(pathlib.Path(__file__).read_text());target.chmod(0o755)
 for part,packages in [('compose',['docker-compose-plugin','docker-compose-v2']),('buildx',['docker-buildx-plugin','docker-buildx'])]:
  if any(p in args for p in packages):(root/part).touch()
if name=='curl':pathlib.Path(args[args.index('-o')+1]).write_text('fake signing key')
if name=='docker':
 if args[:2]==['compose','version'] and kind!='complete' and not (root/'compose').exists():sys.exit(1)
 if args[:2]==['buildx','version'] and kind!='complete' and not (root/'buildx').exists():sys.exit(1)
 if args[:2]==['buildx','build'] and os.environ.get('FAIL_BUILD'):sys.exit(1)
 if args[:2]==['image','load'] and os.environ.get('FAIL_LOAD'):sys.exit(1)
 if args[:1]==['run'] and os.environ.get('FAIL_VERIFY'):sys.exit(1)
 if args[:2]==['compose','--env-file'] and os.environ.get('FAIL_CONFIG'):sys.exit(1)
 if args[:2]==['image','inspect']:
  print(json.dumps([{'Id':'sha256:'+'a'*64,'Os':'linux','Architecture':os.environ.get('IMAGE_ARCH','amd64'),
    'Config':{'Labels':{'com.mobotic.stack':'four-micontrol-bundled-romb-v1'}}}]))
 if args[:2]==['info','--format']:print('linux/x86_64')
if name=='systemctl':
 if args[:1]==['is-active']:sys.exit(0 if os.environ.get('ACTIVE_STACK') and args[-1]=='moboterra-current-stack.service' else 3)
 if args[:1]==['show']:print('not-found')
'''


@unittest.skipUnless(sys.platform == 'linux', 'Installer integration tests require Linux shell semantics')
class InstallationTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.repo = self.root / 'repo'
        self.deploy = self.repo / 'deploy'
        self.deploy.mkdir(parents=True)
        for filename in ('install.sh','bootstrap_host.sh','current_stack.py','compose.current-stack.yaml'):
            shutil.copy2(DEPLOY / filename, self.deploy / filename)
        shutil.copytree(DEPLOY / 'vendor', self.deploy / 'vendor')
        (self.deploy / 'host').mkdir()
        for filename in ('moboterra_ctl.py', 'joystick_restart.py'):
            shutil.copy2(DEPLOY / 'host' / filename, self.deploy / 'host' / filename)
        shutil.copytree(DEPLOY.parent / 'src/mobotic_config', self.repo / 'src/mobotic_config',
                        ignore=shutil.ignore_patterns('__pycache__', '.pytest_cache'))
        # The optional WSL test runner has Python 3.8; the actual target is
        # Ubuntu 24.04/Python 3.12. Preserve this fixed max_ derivation in its
        # temporary fixture without changing the target's configuration module.
        config_module = self.repo / 'src/mobotic_config/mobotic_config/configuration.py'
        config_module.write_text(config_module.read_text()
                                 .replace("key.removeprefix('max_')", 'key[4:]')
                                 .replace("kind.removesuffix('_list')", 'kind[:-5]'))
        (self.repo / 'config/raspberry-pi').mkdir(parents=True)
        (self.repo / 'config/raspberry-pi/ros.env').write_text('export ROS_DOMAIN_ID=42\n')
        (self.root / 'etc').mkdir()
        (self.root / 'etc/os-release').write_text('ID=ubuntu\nVERSION_ID=24.04\nVERSION_CODENAME=noble\n')
        (self.root / 'etc/apt/sources.list.d').mkdir(parents=True)
        certificate = self.root / 'etc/ssl/certs/ca-certificates.crt'
        certificate.parent.mkdir(parents=True)
        certificate.touch()
        for filename in ('install.sh','bootstrap_host.sh','current_stack.py'):
            path = self.deploy / filename
            text = path.read_text()
            text = text.replace('[[ "$EUID" -eq 0 ]]', 'true')
            text = text.replace('[[ "$EUID" -ne 0 ]]', 'false')
            text = text.replace('if os.geteuid() != 0:', 'if False:')
            if filename == 'bootstrap_host.sh':
                text = text.replace('/etc/', str(self.root / 'etc') + '/')
            if filename == 'current_stack.py':
                text = text.replace("Path('/etc/systemd/system')", repr(str(self.root / 'etc/systemd/system')))
                text = text.replace(repr(str(self.root / 'etc/systemd/system')), "Path("+repr(str(self.root / 'etc/systemd/system'))+")")
                text = text.replace("Path('/usr/local/bin/moboterra-ctl')", "Path("+repr(str(self.root / 'usr/local/bin/moboterra-ctl'))+")")
                text = 'def test_chown(*args): pass\n' + text.replace('os.chown(', 'test_chown(')
            path.write_text(text)
        self.bin = self.root / 'bin'
        self.bin.mkdir()
        (self.bin / 'python3').symlink_to(sys.executable)
        for name in ('bash', 'cat', 'chmod', 'dirname', 'grep', 'install', 'uname'):
            (self.bin / name).symlink_to(shutil.which(name))
        for name in ('dpkg', 'dpkg-query', 'docker', 'systemctl', 'systemd-analyze', 'ip',
                     'candump', 'udevadm', 'modprobe', 'curl', 'apt-get'):
            path = self.bin / name
            path.write_text(STUB)
            path.chmod(0o755)
        self.env = dict(os.environ, PATH=str(self.bin), TEST_ROOT=str(self.root))
        self.env.pop('SUDO_USER', None)
        self.env.pop('MOBOTERRA_ROMB_RUNTIME', None)

    def invoke(self, *arguments, **environment):
        if environment.get('DOCKER_KIND') == 'missing':
            (self.bin / 'docker').unlink()
        return subprocess.run(['bash', str(self.deploy / 'install.sh'), *arguments],
                              env=dict(self.env, **environment), capture_output=True, text=True, timeout=40)

    def calls(self):
        path = self.root / 'calls.jsonl'
        return [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []

    def test_default_builds_bundled_stack_without_host_romb_or_ros(self):
        result = self.invoke()
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        calls = self.calls()
        build = next(call for call in calls if call[:3] == ['docker','buildx','build'])
        self.assertEqual(build[build.index('--target')+1], 'current-stack')
        self.assertNotIn('--build-context', build)
        env = (self.repo / 'runtime/docker-stack.env').read_text()
        self.assertIn("MOBOTERRA_CURRENT_IMAGE='sha256:"+'a'*64+"'", env)
        self.assertTrue((self.root / 'usr/local/bin/moboterra-ctl').is_symlink())
        units = self.root / 'etc/systemd/system'
        self.assertIn('Requires=docker.service moboterra-joystick-restart.socket',
                      (units / 'moboterra-current-stack.service').read_text())
        self.assertIn(f"MOBOTERRA_RESTART_SOCKET_DIR='{self.repo / 'runtime'}'", env)
        self.assertIn("MOBOTERRA_HOST_RESTART='true'", env)
        socket_unit = (units / 'moboterra-joystick-restart.socket').read_text()
        self.assertIn('ListenStream=/run/moboterra-restart/restart.sock', socket_unit)
        self.assertIn('SocketMode=0600', socket_unit)
        self.assertIn(' restart --if-running', (units / 'moboterra-joystick-restart@.service').read_text())
        listener_unit = (units / 'moboterra-joystick-listener.service').read_text()
        self.assertIn('Requires=moboterra-joystick-restart.socket', listener_unit)
        self.assertIn('deploy/host/joystick_restart.py', listener_unit)
        self.assertNotIn('PartOf=moboterra-current-stack.service', listener_unit)
        self.assertIn(['systemctl','enable','moboterra-current-stack.service'], calls)
        self.assertNotIn(['systemctl','start','moboterra-current-stack.service'], calls)
        self.assertFalse(any(call[0] == 'apt-get' for call in calls))
        for call in calls:
            if call[:2] == ['docker','run']:
                self.assertEqual(call[call.index('--network')+1], 'none')

    def test_fresh_host_installs_engine_and_plugins_automatically(self):
        result = self.invoke(DOCKER_KIND='missing')
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        installs=[call for call in self.calls() if call[0]=='apt-get' and 'install' in call]
        self.assertFalse(any('usbutils' in call for call in installs))
        self.assertTrue(any('docker-ce' in call and 'docker-buildx-plugin' in call and 'docker-compose-plugin' in call for call in installs))
        source=(self.root/'etc/apt/sources.list.d/docker.sources').read_text()
        self.assertIn('https://download.docker.com/linux/ubuntu',source)
        self.assertIn('Suites: noble',source)
        self.assertFalse(any('remove' in call or 'purge' in call for call in self.calls()))

    def test_existing_distro_engine_gets_matching_plugins(self):
        result=self.invoke(DOCKER_KIND='distro')
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)
        installs=[call for call in self.calls() if call[0]=='apt-get' and 'install' in call]
        self.assertTrue(any('docker-compose-v2' in call and 'docker-buildx' in call for call in installs))
        self.assertFalse(any('docker-ce' in call for call in installs))

    def test_failed_build_or_validation_does_not_publish_service_or_cli(self):
        for fault in ('FAIL_BUILD','FAIL_VERIFY','FAIL_CONFIG'):
            with self.subTest(fault=fault):
                result=self.invoke(**{fault:'1'})
                self.assertNotEqual(result.returncode,0)
                self.assertFalse((self.repo/'runtime/docker-stack.env').exists())
                self.assertFalse((self.root/'usr/local/bin/moboterra-ctl').exists())
                self.assertFalse((self.root/'etc/systemd/system/moboterra-current-stack.service').exists())

    def test_invalid_alternate_platform_fails_before_build(self):
        profile = self.repo / 'src/mobotic_config/config/platform-four-micontrol.yaml'
        profile.write_text(profile.read_text().replace('listen_only: false', 'listen_only: true'))
        result = self.invoke()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('platform-four-micontrol.yaml:', result.stderr)
        self.assertIn('shared drive and battery CAN interface cannot be listen-only', result.stderr)
        self.assertFalse(any(call[:3] == ['docker', 'buildx', 'build'] for call in self.calls()))

    def test_corrupt_vendor_package_fails_before_build(self):
        for name in ('agv_kinematics.deb', 'agv_kinematics_arm64.deb'):
            with self.subTest(package=name):
                package = self.deploy / 'vendor' / name
                original = package.read_bytes()
                try:
                    package.write_bytes(b'corrupted')
                    result = self.invoke()
                    self.assertNotEqual(result.returncode, 0)
                    self.assertIn('checksum', result.stderr)
                    self.assertIn(name, result.stderr)
                    self.assertFalse(any(call[:3] == ['docker','buildx','build'] for call in self.calls()))
                finally:
                    package.write_bytes(original)

    def test_software_only_registers_cli_but_no_robot_service(self):
        result=self.invoke('--software-only')
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)
        self.assertTrue((self.root/'usr/local/bin/moboterra-ctl').is_symlink())
        self.assertFalse((self.root/'etc/systemd/system/moboterra-current-stack.service').exists())
        self.assertFalse(any(call[:2]==['systemctl','enable'] and 'moboterra-current-stack.service' in call for call in self.calls()))
        self.assertFalse(any(call[0]=='modprobe' for call in self.calls()))

    def test_software_only_preserves_an_existing_deployment_selection(self):
        first=self.invoke()
        self.assertEqual(first.returncode,0,first.stdout+first.stderr)
        environment=self.repo/'runtime/docker-stack.env'
        previous=environment.read_bytes()
        result=self.invoke('--software-only','--ros-domain-id','8')
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)
        self.assertEqual(environment.read_bytes(),previous)
        self.assertEqual((self.repo/'config/raspberry-pi/ros.env').read_text(),'export ROS_DOMAIN_ID=42\n')
        self.assertIn('Existing image selection and ROS domain retained',result.stdout)

    def test_no_start_disables_boot_while_start_is_explicit(self):
        result=self.invoke('--no-start')
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)
        self.assertIn(['systemctl','disable','moboterra-current-stack.service'],self.calls())
        result=self.invoke('--start')
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)
        self.assertIn(['systemctl','start','moboterra-current-stack.service'],self.calls())
        self.assertTrue((self.repo/'runtime/docker-stack.previous.env').is_file())

    def test_prepared_image_skips_source_build(self):
        result=self.invoke('--image','supplier/complete:tested')
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)
        self.assertFalse(any(call[:3]==['docker','buildx','build'] for call in self.calls()))

    def test_archive_install_loads_and_checks_the_image_without_building_or_pulling(self):
        archive = self.root / 'validated image.tar'
        archive.touch()
        result = self.invoke('--no-start', '--image-archive', str(archive),
                             '--image', 'moboterra-current:release', DOCKER_KIND='missing')
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        calls = self.calls()
        load = calls.index(['docker', 'image', 'load', '--input', str(archive)])
        verify = next(index for index, call in enumerate(calls) if call[:2] == ['docker', 'run'])
        self.assertLess(load, verify)
        self.assertFalse(any(call[:3] == ['docker', 'buildx', 'build'] or call[:2] == ['docker', 'pull']
                             for call in calls))
        self.assertIn(['systemctl', 'disable', 'moboterra-current-stack.service'], calls)
        self.assertNotIn(['systemctl', 'start', 'moboterra-current-stack.service'], calls)

    def test_failed_archive_load_does_not_publish_the_service(self):
        archive = self.root / 'bad-image.tar'
        archive.touch()
        result = self.invoke('--no-start', '--image-archive', str(archive),
                             '--image', 'moboterra-current:release', FAIL_LOAD='1')
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse((self.repo / 'runtime/docker-stack.env').exists())
        self.assertFalse(any(call[:2] == ['docker', 'pull'] for call in self.calls()))

    def test_wrong_architecture_and_active_service_are_rejected(self):
        for options in ({'IMAGE_ARCH':'arm64'},{'ACTIVE_STACK':'1'}):
            with self.subTest(options=options):
                result=self.invoke(**options)
                self.assertNotEqual(result.returncode,0)
                self.assertFalse((self.repo/'runtime/docker-stack.env').exists())

    def test_domain_and_options_are_validated(self):
        for arguments in [('--ros-domain-id','233'),('--no-start','--start'),('--romb-runtime','/old/native/path'),
                          ('--image-archive', 'missing.tar'),
                          ('--image', 'moboterra-current:release', '--image-archive', 'missing.tar')]:
            result=self.invoke(*arguments)
            self.assertNotEqual(result.returncode,0)
        self.assertEqual(self.calls(),[])
        result=self.invoke('--ros-domain-id','8')
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)
        self.assertEqual((self.repo/'config/raspberry-pi/ros.env').read_text(),'export ROS_DOMAIN_ID=8\n')


if __name__ == '__main__':
    unittest.main()
