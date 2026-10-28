import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src/mobotic_config'))
from mobotic_config.configuration import Configuration


SOURCE = Path(__file__).resolve().parents[1] / "host" / "moboterra_supervisor.py"
SPEC = importlib.util.spec_from_file_location("moboterra_host_supervisor", SOURCE)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class ConfigurationTest(unittest.TestCase):
    def test_host_reads_per_bus_bitrates_and_topology_from_shared_profile(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            env = root / 'moboterra.env'
            env.write_text('PLATFORM_CAN_SERIAL=p\nBATTERY_CAN_SERIAL=b\nROS_DOMAIN_ID=42\n')
            profile = Configuration().platform
            profile['can']['drives']['bitrate'] = 250000
            backend = MODULE.HostBackend(env, root / 'compose.yaml', FakeRunner())
            digests = []
            for names in (['front_left', 'front_right', 'rear_left', 'rear_right'],
                          ['front_right', 'rear_left']):
                selected = dict(profile, modules=[m for m in profile['modules'] if m['name'] in names])
                (root / 'platform.yaml').write_text(json.dumps(selected))
                config = backend.read_config()
                self.assertEqual(config.bitrate, 250000)
                self.assertEqual(config.effective_battery_bitrate, 500000)
                digests.append(config.platform_digest)
            self.assertNotEqual(*digests)
            env.write_text(env.read_text() + 'CAN_BITRATE=125000\n')
            self.assertEqual(backend.read_config().bitrate, 250000)

    def test_host_reads_battery_ack_mode_from_shared_profile(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            env = root / 'moboterra.env'
            env.write_text('PLATFORM_CAN_SERIAL=p\nBATTERY_CAN_SERIAL=b\nROS_DOMAIN_ID=42\n')
            profile = Configuration().platform
            profile['can']['battery']['listen_only'] = False
            (root / 'platform.yaml').write_text(json.dumps(profile))
            backend = MODULE.HostBackend(env, root / 'compose.yaml', FakeRunner())
            self.assertFalse(backend.read_config().battery_listen_only)

    def test_host_rejects_missing_invalid_or_duplicate_platform_configuration(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            env = root / 'moboterra.env'
            env.write_text('PLATFORM_CAN_SERIAL=p\nBATTERY_CAN_SERIAL=b\nROS_DOMAIN_ID=42\n')
            backend = MODULE.HostBackend(env, root / 'compose.yaml', FakeRunner())
            for content in (None, '{}', 'can: {}\ncan: {}\n'):
                if content is not None: (root / 'platform.yaml').write_text(content)
                with self.subTest(content=content), self.assertRaises(MODULE.ConfigurationError):
                    backend.read_config()

    def test_parse_and_validate(self):
        values = MODULE.parse_env(
            "PLATFORM_CAN_SERIAL=platform-1\n"
            "BATTERY_CAN_SERIAL=battery-2\n"
            "CAN_BITRATE=500000\n"
            "ROS_DOMAIN_ID=42\n"
        )
        config = MODULE.DeploymentConfig.from_values(values)
        self.assertEqual(config.bitrate, 500000)
        self.assertEqual(config.ros_domain_id, 42)

    def test_rejects_duplicate_serial(self):
        with self.assertRaises(MODULE.ConfigurationError):
            MODULE.DeploymentConfig.from_values({
                "PLATFORM_CAN_SERIAL": "same",
                "BATTERY_CAN_SERIAL": "same",
                "CAN_BITRATE": "500000",
                "ROS_DOMAIN_ID": "42",
            })

    def test_rejects_shell_syntax_in_serial(self):
        with self.assertRaises(MODULE.ConfigurationError):
            MODULE.DeploymentConfig.from_values({
                "PLATFORM_CAN_SERIAL": "$(bad)",
                "BATTERY_CAN_SERIAL": "battery serial",
                "CAN_BITRATE": "500000",
                "ROS_DOMAIN_ID": "42",
            })

    def test_domain_range(self):
        with self.assertRaises(MODULE.ConfigurationError):
            MODULE.DeploymentConfig.from_values({
                "PLATFORM_CAN_SERIAL": "p",
                "BATTERY_CAN_SERIAL": "b",
                "CAN_BITRATE": "500000",
                "ROS_DOMAIN_ID": "233",
            })


class InventoryTest(unittest.TestCase):
    def setUp(self):
        self.config = MODULE.DeploymentConfig("platform", "battery", 500000, 42)

    def test_selects_roles_independent_of_kernel_order(self):
        mapping = MODULE.select_devices([
            MODULE.CanDevice("can0", "battery", "usb-b"),
            MODULE.CanDevice("can1", "platform", "usb-p"),
        ], self.config)
        self.assertEqual(mapping.platform.name, "can1")
        self.assertEqual(mapping.battery.name, "can0")

    def test_missing_adapter_fails_closed(self):
        with self.assertRaises(MODULE.InventoryError):
            MODULE.select_devices([MODULE.CanDevice("can0", "platform")], self.config)

    def test_duplicate_match_fails_closed(self):
        with self.assertRaises(MODULE.InventoryError):
            MODULE.select_devices([
                MODULE.CanDevice("can0", "platform"),
                MODULE.CanDevice("can1", "platform"),
                MODULE.CanDevice("can2", "battery"),
            ], self.config)


class FakeRunner:
    def __init__(self):
        self.commands = []

    def run(self, command, **_kwargs):
        self.commands.append(list(command))
        return type("Result", (), {"returncode": 0, "stdout": "", "stderr": ""})()


class BackendTest(unittest.TestCase):
    def setUp(self):
        self.config = MODULE.DeploymentConfig("platform", "battery", 500000, 42)
        self.mapping = MODULE.CanMapping(
            MODULE.CanDevice("can0", "platform"),
            MODULE.CanDevice("can1", "battery"),
        )
        self.backend = MODULE.HostBackend(Path("config"), Path("compose"), FakeRunner())
        self.backend._interface_up = lambda _name: True

    def test_link_verification_requires_battery_listen_only(self):
        details = {
            "can0": "can state ERROR-ACTIVE bitrate 500000 listen-only off",
            "can1": "can state ERROR-ACTIVE bitrate 500000 listen-only on",
        }
        self.backend.link_details = lambda name: details[name]
        self.assertTrue(self.backend.links_ready(self.mapping, self.config))
        details["can1"] = "can state ERROR-ACTIVE bitrate 500000 listen-only off"
        self.assertFalse(self.backend.links_ready(self.mapping, self.config))

    def test_battery_ack_mode_is_configured_and_verified_without_changing_drive_mode(self):
        config = MODULE.dataclasses.replace(self.config, battery_listen_only=False)
        details = {'can0': 'can state ERROR-ACTIVE bitrate 500000',
                   'can1': 'can state ERROR-ACTIVE bitrate 500000'}
        self.backend.link_details = lambda name: details[name]
        self.assertTrue(self.backend.links_ready(self.mapping, config))
        details['can1'] += ' <LISTEN-ONLY>'
        self.assertFalse(self.backend.links_ready(self.mapping, config))
        self.backend._link_exists = lambda name: name in {'can0', 'can1'}
        self.backend.configure_links(self.mapping, config)
        self.assertIn(['ip', 'link', 'set', 'dev', 'can1', 'type', 'can', 'bitrate',
                       '500000', 'listen-only', 'off'], self.backend.runner.commands)

    def test_independent_platform_and_battery_bitrates_are_verified_and_configured(self):
        config = MODULE.dataclasses.replace(self.config, bitrate=250000, battery_bitrate=500000)
        details = {'can0': 'can state ERROR-ACTIVE bitrate 250000 listen-only off',
                   'can1': 'can state ERROR-ACTIVE bitrate 500000 listen-only on'}
        self.backend.link_details = lambda name: details[name]
        self.assertTrue(self.backend.links_ready(self.mapping, config))
        details['can1'] = 'can state ERROR-ACTIVE bitrate 250000 listen-only on'
        self.assertFalse(self.backend.links_ready(self.mapping, config))
        self.backend._link_exists = lambda name: name in {'can0', 'can1'}
        self.backend.configure_links(self.mapping, config)
        self.assertIn(['ip', 'link', 'set', 'dev', 'can0', 'type', 'can', 'bitrate',
                       '250000', 'restart-ms', '100', 'listen-only', 'off'], self.backend.runner.commands)
        self.assertIn(['ip', 'link', 'set', 'dev', 'can1', 'type', 'can', 'bitrate',
                       '500000', 'listen-only', 'on'], self.backend.runner.commands)

    def test_link_verification_rejects_listen_only_platform(self):
        details = {
            "can0": "can state ERROR-ACTIVE bitrate 500000 <LISTEN-ONLY>",
            "can1": "can state ERROR-ACTIVE bitrate 500000 <LISTEN-ONLY>",
        }
        self.backend.link_details = lambda name: details[name]
        self.assertFalse(self.backend.links_ready(self.mapping, self.config))

    def test_adapter_without_busoff_restart_still_configures_both_buses(self):
        self.backend._link_exists = lambda name: name in {'can0', 'can1'}
        original_run = self.backend.runner.run

        def run(command, **kwargs):
            result = original_run(command, **kwargs)
            if 'restart-ms' in command and command[command.index('restart-ms') + 1] == '100':
                raise MODULE.CommandError("Error: Device doesn't support restart from Bus Off.")
            return result

        config = MODULE.dataclasses.replace(self.config, bitrate=250000, battery_bitrate=500000)
        with patch.object(self.backend.runner, 'run', side_effect=run):
            self.backend.configure_links(self.mapping, config)
        commands = self.backend.runner.commands
        self.assertIn(['ip', 'link', 'set', 'dev', 'can0', 'type', 'can', 'bitrate',
                       '250000', 'restart-ms', '0', 'listen-only', 'off'], commands)
        self.assertIn(['ip', 'link', 'set', 'dev', 'can1', 'type', 'can', 'bitrate',
                       '500000', 'listen-only', 'on'], commands)
        self.assertEqual(commands[-2:], [
            ['ip', 'link', 'set', 'dev', 'can0', 'up'],
            ['ip', 'link', 'set', 'dev', 'can1', 'up'],
        ])

    def test_other_can_configuration_errors_do_not_trigger_fallback(self):
        self.backend._link_exists = lambda name: name in {'can0', 'can1'}
        original_run = self.backend.runner.run

        def run(command, **kwargs):
            result = original_run(command, **kwargs)
            if 'restart-ms' in command:
                raise MODULE.CommandError('RTNETLINK answers: Invalid argument')
            return result

        with patch.object(self.backend.runner, 'run', side_effect=run):
            with self.assertRaisesRegex(MODULE.CommandError, 'Invalid argument'):
                self.backend.configure_links(self.mapping, self.config)
        self.assertEqual(sum('restart-ms' in cmd for cmd in self.backend.runner.commands), 1)
        self.assertNotIn(['ip', 'link', 'set', 'dev', 'can0', 'up'], self.backend.runner.commands)

    def test_failed_busoff_restart_fallback_does_not_bring_interfaces_up(self):
        self.backend._link_exists = lambda name: name in {'can0', 'can1'}
        original_run = self.backend.runner.run

        def run(command, **kwargs):
            result = original_run(command, **kwargs)
            if 'restart-ms' in command:
                if command[command.index('restart-ms') + 1] == '100':
                    raise MODULE.CommandError("Error: Device doesn't support restart from Bus Off.")
                raise MODULE.CommandError('Fallback configuration failed')
            return result

        with patch.object(self.backend.runner, 'run', side_effect=run):
            with self.assertRaisesRegex(MODULE.CommandError, 'Fallback configuration failed'):
                self.backend.configure_links(self.mapping, self.config)
        self.assertNotIn(['ip', 'link', 'set', 'dev', 'can0', 'up'], self.backend.runner.commands)

    def test_rename_uses_temporary_names_for_reversed_kernel_order(self):
        mapping = MODULE.CanMapping(
            MODULE.CanDevice("can1", "platform"),
            MODULE.CanDevice("can0", "battery"),
        )
        existing = {"can0", "can1"}
        self.backend._link_exists = lambda name: name in existing
        self.backend.configure_links(mapping, self.config)
        commands = self.backend.runner.commands
        self.assertIn(["ip", "link", "set", "dev", "can1", "name", "mtcan_platform"], commands)
        self.assertIn(["ip", "link", "set", "dev", "can0", "name", "mtcan_battery"], commands)
        self.assertLess(
            commands.index(["ip", "link", "set", "dev", "can1", "name", "mtcan_platform"]),
            commands.index(["ip", "link", "set", "dev", "mtcan_platform", "name", "can0"]),
        )

    def test_partial_rename_is_recoverable(self):
        mapping = MODULE.CanMapping(
            MODULE.CanDevice("mtcan_platform", "platform"),
            MODULE.CanDevice("can0", "battery"),
        )
        existing = {"mtcan_platform", "can0"}
        self.backend._link_exists = lambda name: name in existing
        self.backend.configure_links(mapping, self.config)
        self.assertNotIn(
            ["ip", "link", "set", "dev", "mtcan_platform", "name", "mtcan_platform"],
            self.backend.runner.commands,
        )

    def test_bus_off_counter_is_parsed(self):
        self.backend.link_details = lambda _name: "re-started bus-errors bus-off 7 error-active 9"
        self.assertEqual(self.backend.platform_bus_off_count(), 7)


class MonitoringBackend:
    def __init__(self):
        self.config = MODULE.DeploymentConfig('platform', 'battery', 500000, 42, stable_seconds=0)
        self.devices = [MODULE.CanDevice('can0', 'platform'), MODULE.CanDevice('can1', 'battery')]
        self.status = ('running', 'healthy')
        self.ready = True
        self.bus_off = False
        self.bus_off_count = 0
        self.calls = []
        self.invalid_config = False
    def read_config(self):
        if self.invalid_config: raise MODULE.ConfigurationError('bad config')
        return self.config
    def list_can_devices(self): return self.devices
    def links_ready(self, *args): return self.ready
    def configure_links(self, *args): self.calls.append('configure'); self.ready = True
    def platform_bus_off(self): return self.bus_off
    def platform_bus_off_count(self): return self.bus_off_count
    def stack_status(self): return self.status
    def start_stack(self): self.calls.append('start'); self.status = ('running', 'starting')
    def stop_stack(self): self.calls.append('stop')
    def deactivate_links(self): self.calls.append('deactivate')


class ErrorModeLifecycleTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.backend = MonitoringBackend()
        self.supervisor = MODULE.Supervisor(self.backend, Path(self.temp.name) / 'state.json')

    def test_adapter_loss_keeps_running_container_and_recovers_health(self):
        self.supervisor.step(100)
        self.backend.devices = self.backend.devices[:1]
        self.supervisor.step(102)
        self.assertEqual(self.supervisor.state, 'ERROR')
        self.assertEqual(self.backend.calls, [])
        self.backend.devices.append(MODULE.CanDevice('can1', 'battery'))
        self.supervisor.step(104)
        self.assertEqual(self.supervisor.state, 'RUNNING')
        self.assertEqual(self.backend.calls, [])

    def test_busoff_and_repeated_busoff_preserve_container_and_interfaces(self):
        self.supervisor.step(100)
        self.backend.bus_off = True
        self.backend.bus_off_count = 3
        self.supervisor.step(102)
        self.supervisor.step(110)
        self.assertEqual(self.supervisor.state, 'ERROR')
        self.assertNotIn('stop', self.backend.calls)
        self.assertNotIn('deactivate', self.backend.calls)

    def test_unhealthy_and_timed_out_container_is_not_killed(self):
        for status in [('running', 'unhealthy'), ('running', 'starting'), ('running', 'none')]:
            self.backend.status = status
            self.supervisor.step(100)
            self.supervisor.step(300)
            self.assertEqual(self.supervisor.state, 'ERROR')
        self.assertEqual(self.backend.calls, [])

    def test_can_reconfiguration_is_announced_and_keeps_ros_alive(self):
        self.backend.ready = False
        self.supervisor.step(100)
        self.assertEqual(self.supervisor.state, 'ERROR')
        self.assertEqual(self.backend.calls, ['configure'])
        self.assertIn('reset required', self.supervisor.detail)
        self.supervisor.step(102)
        self.assertEqual(self.supervisor.state, 'RUNNING')

    def test_bad_config_does_not_stop_live_stack(self):
        self.backend.invalid_config = True
        self.supervisor.step(100)
        self.assertEqual(self.supervisor.state, 'ERROR')
        self.assertEqual(self.backend.calls, [])

    def test_profile_or_battery_bitrate_change_requires_explicit_restart(self):
        for change in ({'platform_digest': 'changed-topology'}, {'battery_bitrate': 250000}):
            with self.subTest(change=change):
                self.setUp()
                self.supervisor.step(100)
                self.backend.config = MODULE.dataclasses.replace(self.backend.config, **change)
                self.backend.ready = False
                self.supervisor.step(102)
                self.assertEqual(self.supervisor.state, 'ERROR')
                self.assertIn('explicit service restart', self.supervisor.detail)
                self.assertEqual(self.backend.calls, [])

    def test_start_once_without_adapters_but_do_not_recreate_crashed_container(self):
        self.backend.devices = []
        self.backend.status = ('absent', 'none')
        self.supervisor.step(100)
        self.assertEqual(self.backend.calls, ['start'])
        self.assertEqual(self.supervisor.state, 'ERROR')
        self.backend.status = ('exited', 'unhealthy')
        self.supervisor.step(102)
        self.assertEqual(self.backend.calls, ['start'])
        self.assertIn('explicit service restart', self.supervisor.detail)

    def test_explicit_service_stop_still_stops_container(self):
        self.supervisor.request_stop()
        self.assertEqual(self.supervisor.run(), 0)
        self.assertEqual(self.backend.calls, ['stop'])
        self.assertEqual(self.supervisor.state, 'STOPPED')


if __name__ == "__main__":
    unittest.main()
