#!/usr/bin/env python3
"""Operator interface for the installed MoboTerra deployment."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys


INSTALL_ROOT = Path("/opt/moboterra")
RELEASES = INSTALL_ROOT / "releases"
CURRENT = INSTALL_ROOT / "current"
PREVIOUS = INSTALL_ROOT / "previous"
CONFIG = Path("/etc/moboterra/moboterra.env")
STATE = Path("/run/moboterra-supervisor/state.json")
SERVICE = "moboterra-supervisor.service"


def run(command, *, check=False):
    return subprocess.run(command, text=True, check=check)


def require_root():
    if os.geteuid() != 0:
        raise SystemExit("This command changes system state; run it with sudo")


def show_status(_args):
    if STATE.exists():
        try:
            print(json.dumps(json.loads(STATE.read_text(encoding="utf-8")), indent=2))
        except (OSError, json.JSONDecodeError) as exc:
            print(f"State file error: {exc}", file=sys.stderr)
    else:
        print("Supervisor state: unavailable")
    run(["systemctl", "--no-pager", "--full", "status", SERVICE])
    run(["docker", "inspect", "--format", "container={{.State.Status}} health={{if .State.Health}}{{.State.Health.Status}}{{else}}none{{end}} image={{.Config.Image}}", "moboterra-robot"])
    for interface in ("can0", "can1"):
        run(["ip", "-details", "link", "show", "dev", interface])


def show_can(_args):
    for interface in sorted(Path("/sys/class/net").glob("*")):
        try:
            if (interface / "type").read_text(encoding="ascii").strip() != "280":
                continue
        except OSError:
            continue
        print(f"\n[{interface.name}]")
        run(["udevadm", "info", "--query=property", f"--path={interface}"])
        run(["ip", "-details", "-statistics", "link", "show", "dev", interface.name])


def service_action(args):
    require_root()
    run(["systemctl", args.action, SERVICE], check=True)


def show_logs(args):
    command = ["journalctl", "-u", SERVICE, "--no-pager", "-n", str(args.lines)]
    if args.follow:
        command.extend(["--follow"])
    run(command)
    if not args.follow:
        run(["docker", "logs", "--tail", str(args.lines), "moboterra-robot"])


def _resolved_release(link: Path) -> Path:
    target = link.resolve(strict=True)
    if not target.is_relative_to(RELEASES.resolve()):
        raise RuntimeError(f"Refusing release outside {RELEASES}: {target}")
    return target


def _replace_env_values(path: Path, updates: dict[str, str]):
    lines = path.read_text(encoding="utf-8").splitlines()
    seen = set()
    output = []
    for line in lines:
        if "=" in line and not line.lstrip().startswith("#"):
            key = line.split("=", 1)[0].strip()
            if key in updates:
                output.append(f"{key}={updates[key]}")
                seen.add(key)
                continue
        output.append(line)
    output.extend(f"{key}={value}" for key, value in updates.items() if key not in seen)
    temporary = path.with_suffix(".tmp")
    temporary.write_text("\n".join(output) + "\n", encoding="utf-8")
    os.chmod(temporary, 0o644)
    os.replace(temporary, path)


def rollback(_args):
    require_root()
    current = _resolved_release(CURRENT)
    previous = _resolved_release(PREVIOUS)
    manifest = json.loads((previous / "release.json").read_text(encoding="utf-8"))
    updates = {
        "MOBOTERRA_IMAGE": manifest["runtime_image"],
        "MOBOTERRA_GUI_IMAGE": manifest["gui_image"],
        "MOBOTERRA_RELEASE": manifest["version"],
    }
    config_backup = CONFIG.read_text(encoding="utf-8")
    swapped = False
    try:
        run(["systemctl", "stop", SERVICE], check=True)
        temp_current = INSTALL_ROOT / ".current.new"
        temp_previous = INSTALL_ROOT / ".previous.new"
        for path in (temp_current, temp_previous):
            path.unlink(missing_ok=True)
        temp_current.symlink_to(previous)
        temp_previous.symlink_to(current)
        os.replace(temp_current, CURRENT)
        os.replace(temp_previous, PREVIOUS)
        swapped = True
        _replace_env_values(CONFIG, updates)
        run(["systemctl", "start", SERVICE], check=True)
    except Exception:
        CONFIG.write_text(config_backup, encoding="utf-8")
        os.chmod(CONFIG, 0o644)
        if swapped:
            repair_current = INSTALL_ROOT / ".current.repair"
            repair_previous = INSTALL_ROOT / ".previous.repair"
            for path in (repair_current, repair_previous):
                path.unlink(missing_ok=True)
            repair_current.symlink_to(current)
            repair_previous.symlink_to(previous)
            os.replace(repair_current, CURRENT)
            os.replace(repair_previous, PREVIOUS)
            run(["systemctl", "start", SERVICE])
        raise
    print(f"Rolled back from {current.name} to {previous.name}")


def main():
    parser = argparse.ArgumentParser(prog="moboterra-ctl")
    subparsers = parser.add_subparsers(dest="command")
    subparsers.add_parser("status").set_defaults(func=show_status)
    subparsers.add_parser("can").set_defaults(func=show_can)
    for action in ("start", "stop", "restart"):
        subparsers.add_parser(action).set_defaults(func=service_action, action=action)
    logs = subparsers.add_parser("logs")
    logs.add_argument("--follow", action="store_true")
    logs.add_argument("--lines", type=int, default=100)
    logs.set_defaults(func=show_logs)
    subparsers.add_parser("rollback").set_defaults(func=rollback)
    args = parser.parse_args()
    if not hasattr(args, "func"):
        args = parser.parse_args(["status"])
    args.func(args)


if __name__ == "__main__":
    main()

