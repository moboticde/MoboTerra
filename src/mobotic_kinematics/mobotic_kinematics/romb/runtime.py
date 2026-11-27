"""Launch the verified native ROMB solver, retaining legacy guest-runtime support."""
import hashlib
import json
import os
from pathlib import Path
import sys


def runtime_command(arguments, runtime=None):
    runtime = Path(runtime or os.environ.get('MOBOTERRA_ROMB_RUNTIME',
                   '~/.local/share/moboterra/romb-amd64')).expanduser().resolve()
    root = runtime / 'rootfs'
    qemu = runtime / 'qemu-x86_64-static'
    loader = root / 'usr/lib/x86_64-linux-gnu/ld-linux-x86-64.so.2'
    binary = root / 'opt/ros/jazzy/lib/agv_kinematics/agv_kinematics'
    manifest = runtime / 'manifest.json'
    profiles = runtime / 'udp-only.xml'
    if not manifest.is_file():
        raise RuntimeError(f'Romb runtime is incomplete: {manifest}')
    record = json.loads(manifest.read_text())
    kind = record.get('runtime_kind', 'legacy')
    if kind not in ('native', 'legacy'):
        raise RuntimeError(f'Unsupported ROMB runtime kind: {kind}')
    native = kind == 'native'
    required = (binary, profiles) if native else (qemu, loader, binary, profiles)
    for path in required:
        if not path.is_file():
            raise RuntimeError(f'Romb runtime is incomplete: {path}')
    verified = [(binary, 'kinematics_binary_sha256')]
    if not native:
        verified.append((qemu, 'qemu_binary_sha256'))
    for path, key in verified:
        if hashlib.sha256(path.read_bytes()).hexdigest() != record[key]:
            raise RuntimeError(f'Romb runtime checksum mismatch: {path}')
    host = os.uname().machine
    architecture = {'x86_64': 'amd64', 'aarch64': 'arm64'}.get(host)
    if native and (architecture is None or record.get('architecture') != architecture):
        raise RuntimeError(f'ROMB architecture mismatch: package={record.get("architecture")}, host={host}')
    libraries = ':'.join(str(root / p) for p in ('opt/ros/jazzy/lib', 'usr/lib/x86_64-linux-gnu'))
    env = dict(os.environ)
    env.pop('LD_PRELOAD', None)
    env.pop('ROS_LOCALHOST_ONLY', None)
    env.pop('ROS_STATIC_PEERS', None)
    env.update(RMW_IMPLEMENTATION='rmw_fastrtps_cpp', ROS_AUTOMATIC_DISCOVERY_RANGE='SYSTEM_DEFAULT',
               FASTRTPS_DEFAULT_PROFILES_FILE=str(profiles),
               FASTDDS_DEFAULT_PROFILES_FILE=str(profiles))
    prefix = str(root / 'opt/ros/jazzy')
    if native:
        for name, value in (('LD_LIBRARY_PATH', prefix + '/lib'), ('AMENT_PREFIX_PATH', prefix)):
            env[name] = value + (':' + env[name] if env.get(name) else '')
        command = [str(binary), *arguments]
    else:
        env.update(LD_LIBRARY_PATH=libraries, AMENT_PREFIX_PATH=prefix)
        command = [str(loader), '--library-path', libraries, str(binary), *arguments]
        if host != 'x86_64':
            command = [str(qemu), '-L', str(root), *command]
    return command, env


def main():
    try:
        command, environment = runtime_command(sys.argv[1:])
    except (OSError, ValueError, KeyError, RuntimeError) as error:
        raise SystemExit(f'Cannot start original romb kinematics: {error}') from error
    mode = 'emulated amd64' if command[0].endswith('qemu-x86_64-static') else 'native ' + os.uname().machine
    print(f'Starting original customer-romb agv_kinematics (verified {mode} binary)', flush=True)
    os.execve(command[0], command, environment)


if __name__ == '__main__':
    main()
