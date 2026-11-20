"""Boot preflight must preserve the current topology and never cycle live CAN."""
from contextlib import nullcontext
import copy
import importlib.util
import json
from pathlib import Path, PurePosixPath
import subprocess
import unittest
from unittest.mock import patch

import yaml

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('current_stack', ROOT / 'deploy/current_stack.py')
deployment = importlib.util.module_from_spec(spec)
spec.loader.exec_module(deployment)


def link(*, up=True, bitrate=500000, state='ERROR-ACTIVE', listen_only=False, kind='can'):
    return dict(flags=['UP'] if up else [], linkinfo=dict(info_kind=kind, info_data=dict(
        bittiming=dict(bitrate=bitrate), state=state,
        ctrlmode=['LISTEN-ONLY'] if listen_only else [])))


class CurrentStackTest(unittest.TestCase):
    def test_current_profile_is_one_transmitting_bus(self):
        self.assertEqual(deployment.can_plan(deployment.configuration().platform),
                         {'can0': (500000, False)})

    def test_conflicting_shared_bus_is_rejected(self):
        platform = copy.deepcopy(deployment.configuration().platform)
        for field, value in [('bitrate', 250000), ('listen_only', True)]:
            with self.subTest(field=field):
                altered = copy.deepcopy(platform)
                altered['can']['battery'][field] = value
                with self.assertRaisesRegex(ValueError, 'Conflicting'):
                    deployment.can_plan(altered)

    def test_separate_battery_bus_retains_listen_only(self):
        platform = copy.deepcopy(deployment.configuration().platform)
        platform['can']['battery'].update(interface='can1', listen_only=True)
        self.assertEqual(deployment.can_plan(platform),
                         {'can0': (500000, False), 'can1': (500000, True)})

    def prepare(self, responses, *, wait_seconds=0):
        calls = []
        responses = iter(responses)

        def execute(command, **kwargs):
            calls.append(command)
            if command[:4] == ['ip', '-json', '-details', 'link']:
                response = next(responses)
                if response is None:
                    return subprocess.CompletedProcess(command, 1, '', 'not found')
                return subprocess.CompletedProcess(command, 0, json.dumps([response]), '')
            return subprocess.CompletedProcess(command, 0, '', '')

        with patch.object(deployment, 'legacy_stopped'), \
                patch.object(deployment, 'stack_stopped', return_value=nullcontext()), \
                patch.object(deployment.subprocess, 'run', side_effect=execute):
            try:
                deployment.prepare_can(wait_seconds=wait_seconds)
            finally:
                self.calls = calls

    def test_up_shared_can_is_only_inspected_once(self):
        self.prepare([link()])
        self.assertEqual(self.calls, [['ip', '-json', '-details', 'link', 'show', 'dev', 'can0']])

    def test_down_link_configured_once_then_verified(self):
        self.prepare([link(up=False), link()])
        self.assertEqual(self.calls[1:3], [
            ['ip', 'link', 'set', 'dev', 'can0', 'type', 'can', 'bitrate', '500000',
             'listen-only', 'off', 'restart-ms', '0'],
            ['ip', 'link', 'set', 'dev', 'can0', 'up']])
        self.assertEqual(len(self.calls), 4)

    def test_bad_live_links_fail_without_mutating_them(self):
        for response in (link(bitrate=250000), link(state='BUS-OFF'),
                         link(listen_only=True), link(kind='vcan')):
            with self.subTest(response=response):
                with self.assertRaises(ValueError):
                    self.prepare([response])
                self.assertEqual(len(self.calls), 1)

    def test_missing_adapter_fails_without_creating_an_interface(self):
        with self.assertRaisesRegex(RuntimeError, 'Missing CAN interfaces: can0'):
            self.prepare([None])
        self.assertEqual(len(self.calls), 1)

    def test_newly_configured_link_must_pass_readback(self):
        with self.assertRaisesRegex(ValueError, 'not UP'):
            self.prepare([link(up=False), link(up=False)])

    def test_service_runs_foreground_compose_with_bounded_shutdown(self):
        unit = deployment.service_text(PurePosixPath('/home/robot/Mobo Terra'), '/usr/bin/docker')
        self.assertIn('WorkingDirectory="/home/robot/Mobo Terra"', unit)
        self.assertIn('"/home/robot/Mobo Terra/deploy/current_stack.py" prepare', unit)
        self.assertIn('up --no-build --abort-on-container-exit --exit-code-from robot', unit)
        self.assertIn('stop --timeout 45', unit)
        self.assertIn('Restart=no', unit)
        self.assertIn('Conflicts=moboterra-supervisor.service', unit)

    def test_systemd_escapes_paths_without_a_shell(self):
        self.assertEqual(deployment.systemd_quote('/home/100%/$robot'), '"/home/100%%/$robot"')
        self.assertEqual(deployment.systemd_quote('/home/100%/$robot', command=True),
                         '"/home/100%%/$$robot"')
        with self.assertRaises(ValueError):
            deployment.systemd_quote('/home/robot\nExecStart=bad')
        with self.assertRaises(ValueError):
            deployment.env_text({'ROOT': '/home/robot\nOTHER=bad'})

    def test_container_keeps_native_checks_and_current_configuration(self):
        service = yaml.safe_load((ROOT / 'deploy/compose.current-stack.yaml').read_text())['services']['robot']
        self.assertEqual(service['command'],
                         ['bash', 'scripts/start_MoboTerra', 'start_rviz:=false'])
        self.assertEqual(service['restart'], 'no')
        self.assertEqual(service['network_mode'], 'host')
        self.assertEqual(service['ipc'], 'host')
        self.assertEqual(service['pid'], 'host')
        mounts = {mount['target']: mount for mount in service['volumes']}
        self.assertTrue(mounts['/opt/moboterra/share/mobotic_config/config']['read_only'])
        self.assertTrue(mounts['/opt/moboterra-project/config/raspberry-pi/ros.env']['read_only'])
        self.assertFalse(mounts['/opt/moboterra-project/runtime'].get('read_only', False))


if __name__ == '__main__':
    unittest.main()
