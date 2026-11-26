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
            ['ip', 'link', 'set', 'dev', 'can0', 'type', 'can', 'bitrate', '500000',
             'listen-only', 'off', 'restart-ms', '0'],
            ['ip', 'link', 'set', 'dev', 'can0', 'up'],
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

    def test_can0_recovery_retries_once_after_one_minute(self):
        failed_up = subprocess.CalledProcessError(2, ['ip', 'link', 'set', 'dev', 'can0', 'up'])
        outcomes = [failed_up, deployment.CanLinkError('can0 is BUS-OFF'), None]
        events = []

        def prepare_link(interface, bitrate, listen_only, deadline):
            events.append(('configure', interface, bitrate, listen_only))
            result = outcomes.pop(0)
            if result:
                raise result

        with patch.dict(deployment.os.environ, MOBOTERRA_CAN_USB_PORT=''), \
                patch.object(deployment, 'prepare_can_link', side_effect=prepare_link), \
                patch.object(deployment, 'restart_can_adapter', side_effect=lambda port: events.append(('reset', port))), \
                patch.object(deployment.time, 'sleep', side_effect=lambda delay: events.append(('wait', delay))):
            deployment.prepare_can({'can0': (500000, False)}, 0)

        configure = ('configure', 'can0', 500000, False)
        self.assertEqual(events, [configure, ('reset', ''), configure, ('wait', 60), ('reset', ''), configure])

    def test_can0_recovery_stops_with_reconnect_message_when_adapter_stays_missing(self):
        with patch.dict(deployment.os.environ, MOBOTERRA_CAN_USB_PORT=''), \
                patch.object(deployment, 'prepare_can_link', side_effect=TimeoutError('missing')) as prepare, \
                patch.object(deployment, 'restart_can_adapter', side_effect=RuntimeError('not visible')) as restart, \
                patch.object(deployment.time, 'sleep') as sleep, \
                self.assertRaisesRegex(RuntimeError, '^Reconnect USB-to-CAN Adapter$'):
            deployment.prepare_can({'can0': (500000, False)}, 0)

        self.assertEqual(prepare.call_count, 3)
        self.assertEqual(restart.call_count, 2)
        sleep.assert_called_once_with(60)

    def test_first_successful_recovery_skips_the_delayed_retry(self):
        with patch.dict(deployment.os.environ, MOBOTERRA_CAN_USB_PORT=''), \
                patch.object(deployment, 'prepare_can_link', side_effect=[TimeoutError('missing'), None]), \
                patch.object(deployment, 'restart_can_adapter') as restart, \
                patch.object(deployment.time, 'sleep') as sleep:
            deployment.prepare_can({'can0': (500000, False)}, 0)

        restart.assert_called_once_with('')
        sleep.assert_not_called()

    def usb_adapter(self, devices, port, identity, *, bus='1', number='9'):
        adapter = devices / port
        adapter.mkdir()
        for attribute, value in dict(idVendor=identity[0], idProduct=identity[1],
                                     busnum=bus, devnum=number).items():
            (adapter / attribute).write_text(value + '\n')
        (adapter / (port + ':1.0')).mkdir()
        return adapter

    def test_usb_reset_targets_the_current_device_address(self):
        for identity in (('1d50', '606f'), ('0c72', '000c')):
            with self.subTest(identity=identity), tempfile.TemporaryDirectory() as directory:
                devices = Path(directory)
                self.usb_adapter(devices, '1-2', identity)
                with patch.object(deployment, 'USB_DEVICES', devices), \
                        patch.object(deployment, 'CAN0_DEVICE', devices / 'missing-can0'), \
                        patch.object(deployment, 'run') as run:
                    deployment.restart_can_adapter('')
                    run.assert_called_once_with(['usbreset', '001/009'], timeout=15)

    def test_missing_can0_rejects_missing_ambiguous_and_unknown_adapters(self):
        for identities in ([], [('1234', '5678')],
                           [('1d50', '606f'), ('0c72', '000c')],
                           [('0c72', '000c'), ('0c72', '000c')]):
            with self.subTest(identities=identities), tempfile.TemporaryDirectory() as directory:
                devices = Path(directory)
                for index, identity in enumerate(identities, 2):
                    self.usb_adapter(devices, f'1-{index}', identity)
                with patch.object(deployment, 'USB_DEVICES', devices), \
                        patch.object(deployment, 'CAN0_DEVICE', devices / 'missing-can0'), \
                        patch.object(deployment, 'run') as run:
                    with self.assertRaisesRegex(RuntimeError, 'Expected one visible supported CAN adapter'):
                        deployment.restart_can_adapter('')
                    run.assert_not_called()

    def test_can0_selects_its_adapter_even_when_another_supported_adapter_is_visible(self):
        for identity in (('1d50', '606f'), ('0c72', '000c')):
            with self.subTest(identity=identity), tempfile.TemporaryDirectory() as directory:
                devices = Path(directory)
                adapter = self.usb_adapter(devices, '4-1', identity, bus='4', number='3')
                self.usb_adapter(devices, '1-2', ('0c72', '000c'))
                can0 = devices / 'can0-device'
                can0.symlink_to(adapter / '4-1:1.0', target_is_directory=True)
                with patch.object(deployment, 'USB_DEVICES', devices), \
                        patch.object(deployment, 'CAN0_DEVICE', can0), \
                        patch.object(deployment, 'run') as run:
                    deployment.restart_can_adapter('')
                    run.assert_called_once_with(['usbreset', '004/003'], timeout=15)
                    run.reset_mock()
                    with self.assertRaisesRegex(RuntimeError, 'does not match'):
                        deployment.restart_can_adapter('1-2')
                    run.assert_not_called()

    def test_unsupported_can0_adapter_is_not_reset_or_replaced_by_another(self):
        with tempfile.TemporaryDirectory() as directory:
            devices = Path(directory)
            adapter = self.usb_adapter(devices, '1-2', ('1234', '5678'))
            self.usb_adapter(devices, '4-1', ('0c72', '000c'))
            can0 = devices / 'can0-device'
            can0.symlink_to(adapter / '1-2:1.0', target_is_directory=True)
            with patch.object(deployment, 'USB_DEVICES', devices), \
                    patch.object(deployment, 'CAN0_DEVICE', can0), \
                    patch.object(deployment, 'run') as run:
                with self.assertRaisesRegex(RuntimeError, 'not a supported CAN adapter'):
                    deployment.restart_can_adapter('')
                run.assert_not_called()

    def test_power_cycle_supports_peak_and_candlelight_on_can0(self):
        for identity in (('1d50', '606f'), ('0c72', '000c')):
            with self.subTest(identity=identity), tempfile.TemporaryDirectory() as directory:
                devices = Path(directory)
                adapter = self.usb_adapter(devices, '1-2.3', identity)
                can0 = devices / 'can0-device'
                can0.symlink_to(adapter / '1-2.3:1.0', target_is_directory=True)
                for current_device in (can0, devices / 'missing-can0'):
                    with self.subTest(current_device=current_device), \
                            patch.object(deployment, 'USB_DEVICES', devices), \
                            patch.object(deployment, 'CAN0_DEVICE', current_device), \
                            patch.object(deployment, 'run') as run:
                        deployment.restart_can_adapter('1-2.3')
                        run.assert_called_once_with(
                            ['uhubctl', '-e', '-l', '1-2', '-p', '3', '-a', 'cycle', '-d', '2'], timeout=15)

    def test_power_cycle_targets_only_the_configured_port(self):
        with tempfile.TemporaryDirectory() as directory:
            devices = Path(directory)
            with patch.object(deployment, 'USB_DEVICES', devices), \
                    patch.object(deployment, 'CAN0_DEVICE', devices / 'missing-can0'), \
                    patch.object(deployment, 'run') as run:
                deployment.restart_can_adapter('1-2.3')
                run.assert_called_once_with(['uhubctl', '-e', '-l', '1-2', '-p', '3', '-a', 'cycle', '-d', '2'],
                                            timeout=15)
                run.reset_mock()
                occupied_port = devices / '1-2.3'
                occupied_port.mkdir()
                (occupied_port / 'idVendor').write_text('1234\n')
                (occupied_port / 'idProduct').write_text('5678\n')
                with self.assertRaisesRegex(RuntimeError, 'does not match'):
                    deployment.restart_can_adapter('1-2.3')
                run.assert_not_called()

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
