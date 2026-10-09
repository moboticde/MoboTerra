import importlib.util
from pathlib import Path
import sys
import unittest


SOURCE = Path(__file__).resolve().parents[1] / "host" / "moboterra_supervisor.py"
SPEC = importlib.util.spec_from_file_location("moboterra_host_supervisor", SOURCE)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class ConfigurationTest(unittest.TestCase):
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

    def test_link_verification_rejects_listen_only_platform(self):
        details = {
            "can0": "can state ERROR-ACTIVE bitrate 500000 <LISTEN-ONLY>",
            "can1": "can state ERROR-ACTIVE bitrate 500000 <LISTEN-ONLY>",
        }
        self.backend.link_details = lambda name: details[name]
        self.assertFalse(self.backend.links_ready(self.mapping, self.config))

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


if __name__ == "__main__":
    unittest.main()
