"""Customer CLI routing, offline checks, GUI isolation and stopped rollback."""
import argparse
import importlib.util
import os
import shutil
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('current_ctl', ROOT / 'deploy/host/moboterra_ctl.py')
ctl = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ctl)


class ControlTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root/'runtime').mkdir()
        self.values = dict(MOBOTERRA_PROJECT_ROOT=str(self.root), MOBOTERRA_CURRENT_IMAGE='sha256:current',
                           MOBOTERRA_UID='1000', MOBOTERRA_GID='1000', MOBOTERRA_ROBOT_NAME='moboterra')
        self.env = self.root/'runtime/docker-stack.env'
        self.env.write_text(ctl.deployment.env_text(self.values))
        for name, value in [('ROOT',self.root),('ENV_FILE',self.env)]:
            monkey=patch.object(ctl.deployment,name,value)
            monkey.start();self.addCleanup(monkey.stop)

    def test_service_commands_target_the_current_service(self):
        for action,now in [('start',False),('stop',False),('enable',True),('disable',True)]:
            with self.subTest(action=action),patch.object(ctl,'require_root'),patch.object(ctl,'run') as execute:
                ctl.service_action(argparse.Namespace(action=action,now=now))
                expected=['systemctl',action]+(['--now'] if now else [])+['moboterra-current-stack.service']
                execute.assert_called_once_with(expected,check=True)

    def test_joystick_restart_leaves_stopped_stack_stopped(self):
        with patch.object(ctl, 'require_root'), patch.object(ctl, 'run') as execute:
            self.assertEqual(ctl.main(['restart', '--if-running']), 0)
            execute.assert_called_once_with(['systemctl', 'try-restart', ctl.SERVICE], check=True)

    def test_logs_distinguish_ros_and_host_journal(self):
        with patch.object(ctl,'run',return_value=subprocess.CompletedProcess([],0)) as execute:
            ctl.show_logs(argparse.Namespace(host=False,follow=True,lines=200))
            self.assertEqual(execute.call_args.args[0],['docker','logs','--tail','200','--follow','moboterra-current-stack'])
            ctl.show_logs(argparse.Namespace(host=True,follow=False,lines=30))
            self.assertEqual(execute.call_args.args[0],['journalctl','-u',ctl.SERVICE,'--no-pager','-n','30'])

    def test_ros_arguments_are_passed_as_arguments_not_shell_text(self):
        arguments=['topic','echo','/test;echo injected']
        with patch.object(ctl,'run',return_value=subprocess.CompletedProcess([],0)) as execute:
            ctl.ros_command(argparse.Namespace(arguments=arguments))
        command=execute.call_args.args[0]
        self.assertIn('source scripts/setup_ros.bash && exec ros2 "$@"',command)
        self.assertEqual(command[-3:],arguments)
        self.assertEqual(command[2],'moboterra-current-stack')

    def test_check_has_no_hardware_network_or_host_runtime_mount(self):
        with patch.object(ctl,'run',return_value=subprocess.CompletedProcess([],0)) as execute:
            ctl.check(argparse.Namespace())
        command=execute.call_args.args[0]
        self.assertEqual(command[command.index('--network')+1],'none')
        self.assertIn('--check',command)
        self.assertNotIn('--privileged',command)
        self.assertFalse(any('native-stack.lock' in value for value in command))
        self.assertTrue(any('runtime:rw,uid=1000,gid=1000' in value for value in command))

    def test_rviz_uses_container_ros_and_desktop_credentials(self):
        account=types.SimpleNamespace(pw_dir='/home/operator',pw_gid=1000)
        fake_pwd=types.SimpleNamespace(getpwuid=lambda uid:account)
        with patch.dict(sys.modules,{'pwd':fake_pwd}), \
                patch.dict(os.environ,{'DISPLAY':':0','XAUTHORITY':'/home/operator/.Xauthority','SUDO_UID':'1000'}), \
                patch.object(ctl.os,'getuid',return_value=1000,create=True), \
                patch.object(ctl.os,'geteuid',return_value=0,create=True), \
                patch.object(Path,'is_file',return_value=True),patch.object(Path,'is_dir',return_value=True), \
                patch.object(ctl,'run',return_value=subprocess.CompletedProcess([],0)) as execute:
            ctl.rviz(argparse.Namespace())
        command=execute.call_args.args[0]
        self.assertIn('sha256:current',command)
        self.assertIn('__ns:=/moboterra',command)
        self.assertIn('/tf:=/moboterra/tf',command)
        self.assertIn('1000:1000',command)
        self.assertNotIn('--privileged',command)
        self.assertTrue(any('XAUTHORITY=' in value for value in command))
        self.assertTrue(any('exec rviz2' in value for value in command))

    def previous(self):
        path=self.root/'runtime/docker-stack.previous.env'
        path.write_text(ctl.deployment.env_text(dict(self.values,MOBOTERRA_CURRENT_IMAGE='sha256:previous')))
        return path

    def test_rollback_validates_current_config_and_leaves_service_stopped(self):
        previous=self.previous()
        with patch.object(ctl,'require_root'),patch.object(ctl.deployment,'inspect_image',return_value='sha256:previous'), \
                patch.object(ctl,'run',return_value=subprocess.CompletedProcess([],0)) as execute:
            ctl.rollback(argparse.Namespace(start=False))
        self.assertEqual(ctl.deployment.read_environment()['MOBOTERRA_CURRENT_IMAGE'],'sha256:previous')
        self.assertEqual(ctl.deployment.read_environment(previous)['MOBOTERRA_CURRENT_IMAGE'],'sha256:current')
        calls=[call.args[0] for call in execute.call_args_list]
        self.assertEqual(calls[0],['systemctl','stop',ctl.SERVICE])
        self.assertEqual(calls[1][calls[1].index('--network')+1],'none')
        self.assertNotIn(['systemctl','start',ctl.SERVICE],calls)

    def test_failed_rollback_check_preserves_current_image(self):
        self.previous()
        original=self.env.read_bytes()
        def execute(command,**kwargs):
            if command[:2]==['docker','run']:
                raise subprocess.CalledProcessError(1,command)
            return subprocess.CompletedProcess(command,0)
        with patch.object(ctl,'require_root'),patch.object(ctl.deployment,'inspect_image',return_value='sha256:previous'), \
                patch.object(ctl,'run',side_effect=execute):
            with self.assertRaises(subprocess.CalledProcessError):
                ctl.rollback(argparse.Namespace(start=False))
        self.assertEqual(self.env.read_bytes(),original)

    def test_rollback_preflight_accepts_legacy_and_renamed_image_launchers(self):
        bash = shutil.which('bash')
        if sys.platform == 'win32':
            git = shutil.which('git')
            candidate = Path(git).resolve().parents[1] / 'bin/bash.exe' if git else Path('missing')
            bash = str(candidate) if candidate.is_file() else None
        if not bash:
            self.skipTest('Bash is required to exercise image launcher compatibility')
        self.previous()
        with patch.object(ctl, 'require_root'), \
                patch.object(ctl.deployment, 'inspect_image', return_value='sha256:previous'), \
                patch.object(ctl, 'run', return_value=subprocess.CompletedProcess([], 0)) as execute:
            ctl.rollback(argparse.Namespace(start=False))
        docker = execute.call_args_list[1].args[0]
        preflight = docker[docker.index('sha256:previous') + 1:]
        self.assertEqual(preflight[:2], ['bash', '-c'])
        scripts = self.root / 'scripts'
        scripts.mkdir()
        new = scripts / 'start_MoboTerra'
        old = scripts / 'start_ros2_four_micontrol.sh'
        for names, expected in (([old], 'legacy'), ([new], 'current'), ([old, new], 'current'), ([], None)):
            with self.subTest(names=names):
                for path in (old, new):
                    path.unlink(missing_ok=True)
                for path in names:
                    marker = 'current' if path == new else 'legacy'
                    path.write_text('printf "%s\\n" ' + marker + ' "$@"\n', encoding='utf-8')
                result = subprocess.run([bash, *preflight[1:]], cwd=self.root,
                                        capture_output=True, text=True)
                if expected is None:
                    self.assertNotEqual(result.returncode, 0)
                else:
                    self.assertEqual(result.returncode, 0, result.stderr)
                    self.assertEqual(result.stdout.splitlines(), [expected, '--check', 'start_rviz:=false'])


if __name__ == '__main__':
    unittest.main()
