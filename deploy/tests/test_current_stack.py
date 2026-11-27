"""Boot preflight cycles can0 while respecting the configured CAN topology."""
from contextlib import nullcontext
import copy
import importlib.util
import json
from pathlib import Path, PurePosixPath
import subprocess
import tempfile
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

    def prepare(self, responses, *, interface='can0', listen_only=False, wait_seconds=0):
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

        with patch.object(deployment.subprocess, 'run', side_effect=execute):
            try:
                deployment.prepare_can({interface: (500000, listen_only)},
                                       deployment.time.monotonic() + wait_seconds)
            finally:
                self.calls = calls

    def test_can0_is_always_cycled_and_verified(self):
        show = ['ip', '-json', '-details', 'link', 'show', 'dev', 'can0']
        expected = [
            show,
            ['ip', 'link', 'set', 'dev', 'can0', 'down'],
            ['ip', 'link', 'set', 'dev', 'can0', 'type', 'can', 'bitrate', '500000'],
            ['ip', 'link', 'set', 'dev', 'can0', 'up'],
            ['ip', '-details', '-statistics', 'link', 'show', 'can0'],
            show,
        ]
        for response in (link(), link(up=False), link(bitrate=250000, listen_only=True),
                         link(state='BUS-OFF')):
            with self.subTest(response=response):
                self.prepare([response, link()])
                self.assertEqual(self.calls, expected)

    def test_up_separate_battery_link_is_validated_without_cycling(self):
        show = ['ip', '-json', '-details', 'link', 'show', 'dev', 'can1']
        self.prepare([link(listen_only=True)], interface='can1', listen_only=True)
        self.assertEqual(self.calls, [show])
        for response in (link(bitrate=250000, listen_only=True),
                         link(state='BUS-OFF', listen_only=True), link()):
            with self.subTest(response=response):
                with self.assertRaises(ValueError):
                    self.prepare([response], interface='can1', listen_only=True)
                self.assertEqual(self.calls, [show])

    def test_virtual_can0_is_rejected_without_any_link_changes(self):
        with self.assertRaisesRegex(ValueError, 'not a physical SocketCAN interface'):
            self.prepare([link(kind='vcan')])
        self.assertEqual(self.calls, [['ip', '-json', '-details', 'link', 'show', 'dev', 'can0']])

    def test_missing_adapter_fails_without_creating_an_interface(self):
        with self.assertRaisesRegex(TimeoutError, 'Missing CAN interface: can1'):
            self.prepare([None], interface='can1')
        self.assertEqual(len(self.calls), 1)

    def test_newly_configured_link_must_pass_readback(self):
        with self.assertRaisesRegex(ValueError, 'not UP'):
            self.prepare([link(up=False), link(up=False)], interface='can1')

    def test_can0_setup_failure_propagates_without_retry(self):
        for error in (TimeoutError('missing'), deployment.CanLinkError('can0 is BUS-OFF'),
                      subprocess.CalledProcessError(2, ['ip', 'link', 'set', 'dev', 'can0', 'up'])):
            with self.subTest(error=error), \
                    patch.object(deployment, 'prepare_can_link', side_effect=error) as prepare, \
                    patch.object(deployment.time, 'sleep') as sleep, \
                    self.assertRaises(type(error)):
                try:
                    deployment.prepare_can({'can0': (500000, False)}, 0)
                finally:
                    prepare.assert_called_once_with('can0', 500000, False, 0)
                    sleep.assert_not_called()

    def test_can0_must_pass_readback_after_simple_reset(self):
        with self.assertRaisesRegex(deployment.CanLinkError, 'BUS-OFF'):
            self.prepare([link(state='BUS-OFF'), link(state='BUS-OFF')])
        self.assertEqual(len(self.calls), 6)

    def test_robot_ip_selection_ignores_wifi_disconnected_and_virtual_interfaces(self):
        def interface(name, host_ip, connected=True):
            return dict(ifname=name, link_type='ether', flags=['LOWER_UP'] if connected else [],
                        addr_info=[{'local': host_ip}])

        interfaces = [interface('eth0', '192.168.2.199'), interface('eth1', '192.168.42.17', False),
                      interface('wlan0', '192.168.1.250'), interface('docker0', '192.168.3.250')]
        interfaces[0]['addr_info'].extend({'local': address} for address in (
            '10.60.20.250', '192.168.2.0', '192.168.2.5', '192.168.2.6', '192.168.2.255'))
        devices = {Path('/sys/class/net') / path for path in (
            'eth0/device', 'eth1/device', 'wlan0/device', 'wlan0/phy80211')}
        with patch.object(deployment.Path, 'exists', autospec=True, side_effect=devices.__contains__), \
                patch.object(deployment, 'run') as run:
            run.return_value.stdout = json.dumps(interfaces)
            self.assertEqual(deployment.robot_ethernet_address(), ('eth0', '192.168.2.199'))
            interfaces[1]['flags'].append('LOWER_UP')
            run.return_value.stdout = json.dumps(interfaces)
            with self.assertRaisesRegex(RuntimeError, 'Ambiguous.*eth0=.*eth1='):
                deployment.robot_ethernet_address()
            self.assertEqual(deployment.robot_ethernet_address('192.168.42.17'), ('eth1', '192.168.42.17'))
            self.assertIsNone(deployment.robot_ethernet_address('192.168.42.18'))

    def test_prepare_selects_scanner_network_for_default_and_custom_hosts(self):
        from mobotic_config.configuration import Configuration

        for subnet, suffix, use_override in ((1, 250, False), (2, 199, False), (42, 17, True)):
            host_ip = f'192.168.{subnet}.{suffix}'
            override = host_ip if use_override else ''
            with self.subTest(host_ip=host_ip), tempfile.TemporaryDirectory() as directory:
                cfg = deployment.configuration()
                runtime = Path(directory) / 'runtime'
                with patch.dict(deployment.os.environ, MOBOTERRA_ROBOT_HOST_IP=override), \
                        patch.object(deployment, 'legacy_stopped'), \
                        patch.object(deployment, 'stack_stopped', return_value=nullcontext()), \
                        patch.object(deployment, 'robot_ethernet_address', return_value=('eth0', host_ip)) as ethernet, \
                        patch.object(deployment, 'prepare_can') as can, \
                        patch.object(deployment, 'configuration', return_value=cfg), \
                        patch.object(deployment, 'ROOT', Path(directory)):
                    deployment.prepare_stack(wait_seconds=0)
                ethernet.assert_called_with(override or None)
                selected = Configuration(runtime / 'platform.yaml')
                self.assertEqual(selected.scanner_parameters('front_left')['host_ip'], host_ip)
                self.assertEqual(selected.scanner_parameters('front_left')['sensor_ip'], f'192.168.{subnet}.5')
                self.assertEqual(selected.scanner_parameters('rear_right')['sensor_ip'], f'192.168.{subnet}.6')
                self.assertEqual(can.call_args.args[0], {'can0': (500000, False)})
                self.assertEqual(selected.parameters('driver'), deployment.configuration().parameters('driver'))

    def test_prepare_rejects_missing_or_changed_address(self):
        for addresses, message, checks_can in (
            ([None], 'No robot address', False),
            ([('eth0', '192.168.1.250'), ('eth0', '192.168.2.250')], 'address changed', True),
        ):
            with self.subTest(addresses=addresses), patch.object(deployment, 'legacy_stopped'), \
                    patch.object(deployment, 'stack_stopped', return_value=nullcontext()), \
                    patch.object(deployment, 'robot_ethernet_address', side_effect=addresses), \
                    patch.object(deployment, 'prepare_can') as can, \
                    patch.object(deployment, 'atomic_text') as write:
                with self.assertRaisesRegex(RuntimeError, message):
                    deployment.prepare_stack(wait_seconds=0)
                self.assertEqual(can.called, checks_can)
                write.assert_not_called()

    def test_service_runs_foreground_compose_with_bounded_shutdown(self):
        unit = deployment.service_text(PurePosixPath('/home/robot/Mobo Terra'), '/usr/bin/docker')
        self.assertIn('"/home/robot/Mobo Terra/deploy/current_stack.py" prepare', unit)
        self.assertIn('"--env-file" "/home/robot/Mobo Terra/runtime/docker-stack.env"', unit)
        self.assertIn('"--file" "/home/robot/Mobo Terra/deploy/compose.current-stack.yaml"', unit)
        self.assertIn('up --no-build --abort-on-container-exit --exit-code-from robot', unit)
        self.assertIn('stop --timeout 45', unit)
        self.assertIn('Restart=no', unit)
        self.assertIn('TimeoutStartSec=300', unit)
        self.assertIn('Conflicts=moboterra-supervisor.service', unit)
        self.assertIn('Wants=network-online.target moboterra-joystick-listener.service', unit)

    def test_systemd_escapes_paths_without_a_shell(self):
        self.assertEqual(deployment.systemd_quote('/home/100%/$robot'),
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
        self.assertEqual(service['ipc'], 'private')
        self.assertEqual(service['environment']['FASTDDS_BUILTIN_TRANSPORTS'], 'UDPv4')
        self.assertEqual(service['pid'], 'host')
        mounts = {mount['target']: mount for mount in service['volumes']}
        self.assertTrue(mounts['/opt/moboterra/share/mobotic_config/config']['read_only'])
        platform_mount = mounts['/opt/moboterra/share/mobotic_config/config/platform.yaml']
        self.assertTrue(platform_mount['source'].endswith('/runtime/platform.yaml'))
        self.assertTrue(platform_mount['read_only'])
        self.assertFalse(platform_mount['bind']['create_host_path'])
        self.assertTrue(mounts['/opt/moboterra-project/config/raspberry-pi/ros.env']['read_only'])
        self.assertFalse(mounts['/opt/moboterra-project/runtime'].get('read_only', False))


if __name__ == '__main__':
    unittest.main()
