#!/usr/bin/env python3
"""Fail-closed host supervisor for MoboTerra CAN and Docker lifecycle."""

from __future__ import annotations

import argparse
import dataclasses
import json
import logging
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
from typing import Iterable, Mapping, Sequence

try:
    import fcntl
except ImportError:  # Allows pure helper tests to run on non-Linux hosts.
    fcntl = None


LOG = logging.getLogger("moboterra-supervisor")
CAN_ARPHRD = "280"
SERIAL_RE = re.compile(r"^[A-Za-z0-9._:+-]+$")


class ConfigurationError(RuntimeError):
    pass


class InventoryError(RuntimeError):
    pass


class CommandError(RuntimeError):
    pass


@dataclasses.dataclass(frozen=True)
class DeploymentConfig:
    platform_serial: str
    battery_serial: str
    bitrate: int
    ros_domain_id: int
    stable_seconds: float = 3.0
    poll_seconds: float = 2.0
    start_timeout: float = 120.0

    @classmethod
    def from_values(cls, values: Mapping[str, str]) -> "DeploymentConfig":
        required = ("PLATFORM_CAN_SERIAL", "BATTERY_CAN_SERIAL", "CAN_BITRATE", "ROS_DOMAIN_ID")
        missing = [key for key in required if not values.get(key)]
        if missing:
            raise ConfigurationError(f"Missing required configuration: {', '.join(missing)}")

        platform = values["PLATFORM_CAN_SERIAL"]
        battery = values["BATTERY_CAN_SERIAL"]
        if not SERIAL_RE.fullmatch(platform) or not SERIAL_RE.fullmatch(battery):
            raise ConfigurationError("CAN serials may contain only letters, numbers, '.', '_', ':', '+', and '-'")
        if platform == battery:
            raise ConfigurationError("Platform and battery CAN serials must be different")

        try:
            bitrate = int(values["CAN_BITRATE"])
            domain = int(values["ROS_DOMAIN_ID"])
            stable = float(values.get("CAN_STABLE_SECONDS", "3"))
            poll = float(values.get("CAN_POLL_SECONDS", "2"))
            timeout = float(values.get("ROS_START_TIMEOUT", "120"))
        except ValueError as exc:
            raise ConfigurationError(f"Invalid numeric configuration: {exc}") from exc
        if bitrate <= 0:
            raise ConfigurationError("CAN_BITRATE must be positive")
        if not 0 <= domain <= 232:
            raise ConfigurationError("ROS_DOMAIN_ID must be between 0 and 232")
        if stable < 0 or poll < 0.2 or timeout < 10:
            raise ConfigurationError("Invalid supervisor timing values")

        return cls(platform, battery, bitrate, domain, stable, poll, timeout)


@dataclasses.dataclass(frozen=True)
class CanDevice:
    name: str
    serial: str | None
    path: str | None = None


@dataclasses.dataclass(frozen=True)
class CanMapping:
    platform: CanDevice
    battery: CanDevice

    @property
    def identity(self) -> tuple[str, str]:
        return self.platform.serial or "", self.battery.serial or ""


def parse_env(text: str) -> dict[str, str]:
    """Parse a strict KEY=VALUE file without evaluating shell syntax."""
    values: dict[str, str] = {}
    for line_number, raw in enumerate(text.splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if "=" not in line:
            raise ConfigurationError(f"Line {line_number} is not KEY=VALUE")
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip()
        if not re.fullmatch(r"[A-Z][A-Z0-9_]*", key):
            raise ConfigurationError(f"Invalid key on line {line_number}: {key!r}")
        if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
            value = value[1:-1]
        values[key] = value
    return values


def select_devices(devices: Iterable[CanDevice], config: DeploymentConfig) -> CanMapping:
    device_list = list(devices)
    platform = [item for item in device_list if item.serial == config.platform_serial]
    battery = [item for item in device_list if item.serial == config.battery_serial]
    if len(platform) != 1 or len(battery) != 1:
        found = ", ".join(f"{item.name}={item.serial or '<none>'}" for item in device_list) or "none"
        raise InventoryError(
            "Expected exactly one adapter for each configured serial; "
            f"platform matches={len(platform)}, battery matches={len(battery)}, found: {found}"
        )
    if platform[0].name == battery[0].name:
        raise InventoryError("One interface matched both CAN roles")
    return CanMapping(platform[0], battery[0])


class CommandRunner:
    def run(
        self,
        command: Sequence[str],
        *,
        check: bool = True,
        timeout: float = 30,
    ) -> subprocess.CompletedProcess[str]:
        result = subprocess.run(
            list(command),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout,
            check=False,
        )
        if check and result.returncode != 0:
            detail = result.stderr.strip() or result.stdout.strip() or f"exit {result.returncode}"
            raise CommandError(f"{' '.join(command)}: {detail}")
        return result


class HostBackend:
    def __init__(self, config_path: Path, compose_path: Path, runner: CommandRunner | None = None):
        self.config_path = config_path
        self.compose_path = compose_path
        self.runner = runner or CommandRunner()

    def read_config(self) -> DeploymentConfig:
        return DeploymentConfig.from_values(parse_env(self.config_path.read_text(encoding="utf-8")))

    def list_can_devices(self) -> list[CanDevice]:
        devices: list[CanDevice] = []
        for interface in sorted(Path("/sys/class/net").iterdir()):
            try:
                if (interface / "type").read_text(encoding="ascii").strip() != CAN_ARPHRD:
                    continue
            except (FileNotFoundError, PermissionError):
                continue
            result = self.runner.run(
                ["udevadm", "info", "--query=property", f"--path={interface}"], check=False
            )
            properties = parse_properties(result.stdout)
            devices.append(
                CanDevice(
                    name=interface.name,
                    serial=properties.get("ID_SERIAL_SHORT"),
                    path=properties.get("ID_PATH") or properties.get("DEVPATH"),
                )
            )
        return devices

    def _compose(self, *arguments: str, check: bool = True, timeout: float = 60):
        return self.runner.run(
            [
                "docker", "compose",
                "--project-directory", str(self.compose_path.parent),
                "--env-file", str(self.config_path),
                "--file", str(self.compose_path),
                *arguments,
            ],
            check=check,
            timeout=timeout,
        )

    def stop_stack(self) -> None:
        self._compose("down", "--remove-orphans", "--timeout", "30", check=False, timeout=45)

    def start_stack(self) -> None:
        self._compose("up", "--detach", "--no-build", check=True, timeout=90)

    def stack_status(self) -> tuple[str, str]:
        result = self.runner.run(
            ["docker", "inspect", "--format", "{{json .State}}", "moboterra-robot"],
            check=False,
        )
        if result.returncode != 0:
            return "absent", "none"
        try:
            state = json.loads(result.stdout)
        except json.JSONDecodeError:
            return "unknown", "unknown"
        health = state.get("Health", {}).get("Status", "none")
        return str(state.get("Status", "unknown")), str(health)

    def _link_exists(self, name: str) -> bool:
        return Path("/sys/class/net", name).exists()

    def _interface_up(self, name: str) -> bool:
        result = self.runner.run(["ip", "-json", "link", "show", "dev", name], check=False)
        if result.returncode != 0:
            return False
        try:
            data = json.loads(result.stdout)
            return bool(data) and "UP" in data[0].get("flags", [])
        except (json.JSONDecodeError, KeyError, TypeError):
            return False

    def link_details(self, name: str) -> str:
        return self.runner.run(["ip", "-details", "link", "show", "dev", name], check=False).stdout

    def links_ready(self, mapping: CanMapping, config: DeploymentConfig) -> bool:
        if mapping.platform.name != "can0" or mapping.battery.name != "can1":
            return False
        if not self._interface_up("can0") or not self._interface_up("can1"):
            return False
        platform_details = self.link_details("can0")
        battery_details = self.link_details("can1")
        bitrate = re.compile(rf"\bbitrate\s+{config.bitrate}\b")
        battery_listen_only = "listen-only on" in battery_details.lower() or "LISTEN-ONLY" in battery_details
        platform_listen_only = "listen-only on" in platform_details.lower() or "LISTEN-ONLY" in platform_details
        return bool(
            bitrate.search(platform_details)
            and bitrate.search(battery_details)
            and battery_listen_only
            and not platform_listen_only
        )

    def platform_bus_off(self) -> bool:
        return bool(re.search(r"\bstate\s+BUS-OFF\b", self.link_details("can0"), re.IGNORECASE))

    def platform_bus_off_count(self) -> int | None:
        match = re.search(r"\bbus-off\s+(\d+)\b", self.link_details("can0"), re.IGNORECASE)
        return int(match.group(1)) if match else None

    def deactivate_links(self) -> None:
        self.runner.run(["ip", "link", "set", "dev", "can0", "down"], check=False)
        self.runner.run(["ip", "link", "set", "dev", "can1", "down"], check=False)

    def configure_links(self, mapping: CanMapping, config: DeploymentConfig) -> None:
        role_names = {mapping.platform.name, mapping.battery.name}
        for target in ("can0", "can1"):
            if self._link_exists(target) and target not in role_names:
                raise InventoryError(f"Refusing to replace unrelated interface {target}")

        temporary = {"platform": "mtcan_platform", "battery": "mtcan_battery"}
        for name in temporary.values():
            if self._link_exists(name) and name not in role_names:
                raise InventoryError(f"Temporary interface name is already occupied: {name}")

        self.runner.run(["ip", "link", "set", "dev", mapping.platform.name, "down"], check=False)
        self.runner.run(["ip", "link", "set", "dev", mapping.battery.name, "down"], check=False)
        if mapping.platform.name != temporary["platform"]:
            self.runner.run(["ip", "link", "set", "dev", mapping.platform.name, "name", temporary["platform"]])
        if mapping.battery.name != temporary["battery"]:
            self.runner.run(["ip", "link", "set", "dev", mapping.battery.name, "name", temporary["battery"]])
        self.runner.run(["ip", "link", "set", "dev", temporary["platform"], "name", "can0"])
        self.runner.run(["ip", "link", "set", "dev", temporary["battery"], "name", "can1"])
        self.runner.run(["udevadm", "settle"], check=False)

        self.runner.run([
            "ip", "link", "set", "dev", "can0", "type", "can",
            "bitrate", str(config.bitrate), "restart-ms", "100", "listen-only", "off",
        ])
        self.runner.run([
            "ip", "link", "set", "dev", "can1", "type", "can",
            "bitrate", str(config.bitrate), "listen-only", "on",
        ])
        self.runner.run(["ip", "link", "set", "dev", "can0", "up"])
        self.runner.run(["ip", "link", "set", "dev", "can1", "up"])


def parse_properties(text: str) -> dict[str, str]:
    properties: dict[str, str] = {}
    for line in text.splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            properties[key] = value
    return properties


class Supervisor:
    def __init__(self, backend: HostBackend, state_path: Path):
        self.backend = backend
        self.state_path = state_path
        self.stop_requested = False
        self.state = "INITIALIZING"
        self.detail = "Starting"
        self.stable_identity: tuple[str, str] | None = None
        self.config_identity: tuple[str, str, int, int] | None = None
        self.stable_since = 0.0
        self.start_deadline: float | None = None
        self.running_since: float | None = None
        self.bus_off_since: float | None = None
        self.last_bus_off_count: int | None = None
        self.bus_off_events: list[float] = []
        self.failures = 0
        self.backoff_until = 0.0

    def request_stop(self, *_args) -> None:
        self.stop_requested = True

    def publish_state(self, state: str, detail: str, **extra) -> None:
        changed = state != self.state or detail != self.detail
        self.state, self.detail = state, detail
        if changed:
            LOG.info("state=%s detail=%s", state, detail)
        payload = {
            "state": state,
            "detail": detail,
            "updated_at": time.time(),
            "failures": self.failures,
            **extra,
        }
        self.state_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.state_path.with_suffix(".tmp")
        temporary.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        os.replace(temporary, self.state_path)

    def stop_stack(self, reason: str) -> None:
        status, _ = self.backend.stack_status()
        if status != "absent":
            LOG.warning("Stopping MoboTerra: %s", reason)
            self.backend.stop_stack()
        self.start_deadline = None
        self.running_since = None

    def schedule_backoff(self, now: float, reason: str, *, deactivate_can: bool = False) -> None:
        self.stop_stack(reason)
        if deactivate_can:
            self.backend.deactivate_links()
            self.bus_off_since = None
            self.last_bus_off_count = None
            self.bus_off_events.clear()
        self.failures += 1
        delay = min(60.0, float(2 ** min(self.failures, 6)))
        self.backoff_until = now + delay
        self.publish_state("BACKOFF", f"{reason}; retrying in {delay:.0f}s")

    def run(self) -> int:
        while not self.stop_requested:
            now = time.monotonic()
            try:
                config = self.backend.read_config()
                devices = self.backend.list_can_devices()
                mapping = select_devices(devices, config)
            except (OSError, ConfigurationError, InventoryError, CommandError) as exc:
                self.stable_identity = None
                self.stop_stack(str(exc))
                self.publish_state("WAITING_FOR_CAN", str(exc))
                time.sleep(2.0)
                continue

            config_identity = (
                config.platform_serial,
                config.battery_serial,
                config.bitrate,
                config.ros_domain_id,
            )
            if config_identity != self.config_identity:
                if self.config_identity is not None:
                    self.stop_stack("Deployment configuration changed")
                self.config_identity = config_identity
                self.stable_identity = None

            if mapping.identity != self.stable_identity:
                if self.stable_identity is not None:
                    self.stop_stack("CAN adapter identity changed")
                self.stable_identity = mapping.identity
                self.stable_since = now
                self.publish_state("WAITING_FOR_CAN", "Both adapters detected; debouncing")
                time.sleep(config.poll_seconds)
                continue
            if now - self.stable_since < config.stable_seconds:
                self.publish_state("WAITING_FOR_CAN", "Both adapters detected; debouncing")
                time.sleep(config.poll_seconds)
                continue

            if now < self.backoff_until:
                self.publish_state("BACKOFF", f"Waiting {self.backoff_until - now:.0f}s before retry")
                time.sleep(config.poll_seconds)
                continue

            try:
                if not self.backend.links_ready(mapping, config):
                    self.stop_stack("CAN topology or configuration changed")
                    self.publish_state("CONFIGURING_CAN", "Assigning and configuring can0/can1")
                    self.backend.configure_links(mapping, config)
                    remapped = select_devices(self.backend.list_can_devices(), config)
                    if not self.backend.links_ready(remapped, config):
                        raise CommandError("CAN verification failed after configuration")
                    mapping = remapped

                bus_off_count = self.backend.platform_bus_off_count()
                if bus_off_count is not None:
                    if self.last_bus_off_count is not None and bus_off_count > self.last_bus_off_count:
                        self.bus_off_events.append(now)
                    elif self.last_bus_off_count is not None and bus_off_count < self.last_bus_off_count:
                        self.bus_off_events.clear()
                    self.last_bus_off_count = bus_off_count
                self.bus_off_events = [event for event in self.bus_off_events if now - event <= 60.0]

                if len(self.bus_off_events) >= 3:
                    self.schedule_backoff(
                        now,
                        "Platform CAN entered BUS-OFF at least three times in 60s",
                        deactivate_can=True,
                    )
                    time.sleep(config.poll_seconds)
                    continue
                if self.backend.platform_bus_off():
                    if self.bus_off_since is None:
                        self.bus_off_since = now
                    elif now - self.bus_off_since >= 5.0:
                        self.schedule_backoff(
                            now, "Platform CAN remained BUS-OFF", deactivate_can=True
                        )
                        time.sleep(config.poll_seconds)
                        continue
                else:
                    self.bus_off_since = None

                container_state, health = self.backend.stack_status()
                if container_state == "absent" or container_state in {"exited", "dead"}:
                    self.publish_state("STARTING_ROS", "Starting MoboTerra container")
                    self.backend.start_stack()
                    self.start_deadline = now + config.start_timeout
                    self.running_since = None
                elif container_state == "running" and health == "healthy":
                    if self.running_since is None:
                        self.running_since = now
                    if now - self.running_since >= 60.0:
                        self.failures = 0
                    self.start_deadline = None
                    self.publish_state(
                        "RUNNING",
                        "CAN is ready and required ROS processes are present",
                        platform_interface="can0",
                        battery_interface="can1",
                    )
                elif container_state == "running" and health in {"starting", "none"}:
                    if self.start_deadline is None:
                        self.start_deadline = now + config.start_timeout
                    if now >= self.start_deadline:
                        self.schedule_backoff(now, "ROS health-check timeout")
                    else:
                        self.publish_state("STARTING_ROS", f"Container health is {health}")
                else:
                    self.schedule_backoff(now, f"Container state={container_state}, health={health}")
            except (OSError, InventoryError, CommandError, subprocess.SubprocessError) as exc:
                LOG.exception("Supervisor operation failed")
                self.schedule_backoff(now, str(exc))

            time.sleep(config.poll_seconds)

        self.stop_stack("Supervisor stopping")
        self.publish_state("STOPPED", "Supervisor stopped")
        return 0


def acquire_lock(path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    handle = path.open("w", encoding="utf-8")
    if fcntl is None:
        raise RuntimeError("MoboTerra supervisor requires Linux fcntl locking")
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        raise RuntimeError("Another MoboTerra supervisor is already running") from exc
    handle.write(str(os.getpid()))
    handle.flush()
    return handle


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("/etc/moboterra/moboterra.env"))
    parser.add_argument("--compose", type=Path, default=Path("/opt/moboterra/current/compose.yaml"))
    parser.add_argument("--state", type=Path, default=Path("/run/moboterra-supervisor/state.json"))
    parser.add_argument("--lock", type=Path, default=Path("/run/moboterra-supervisor/supervisor.lock"))
    args = parser.parse_args(argv)

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    lock_handle = acquire_lock(args.lock)

    supervisor = Supervisor(HostBackend(args.config, args.compose), args.state)
    signal.signal(signal.SIGTERM, supervisor.request_stop)
    signal.signal(signal.SIGINT, supervisor.request_stop)
    try:
        return supervisor.run()
    finally:
        lock_handle.close()


if __name__ == "__main__":
    sys.exit(main())

