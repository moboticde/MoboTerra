"""Launch the unchanged customer-romb amd64 kinematics on this ARM64 host."""
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
    for path in (qemu, loader, binary, manifest, profiles):
        if not path.is_file():
            raise RuntimeError(f'Romb runtime is incomplete: {path}')
    record = json.loads(manifest.read_text())
    for path, key in ((binary, 'kinematics_binary_sha256'), (qemu, 'qemu_binary_sha256')):
        if hashlib.sha256(path.read_bytes()).hexdigest() != record[key]:
            raise RuntimeError(f'Romb runtime checksum mismatch: {path}')
    libraries = ':'.join(str(root / p) for p in ('opt/ros/jazzy/lib', 'usr/lib/x86_64-linux-gnu'))
    env = dict(os.environ)
    env.pop('LD_PRELOAD', None)
    env.pop('ROS_LOCALHOST_ONLY', None)
    env.pop('ROS_STATIC_PEERS', None)
    env.update(LD_LIBRARY_PATH=libraries, AMENT_PREFIX_PATH=str(root / 'opt/ros/jazzy'),
               RMW_IMPLEMENTATION='rmw_fastrtps_cpp', ROS_AUTOMATIC_DISCOVERY_RANGE='SYSTEM_DEFAULT',
               FASTRTPS_DEFAULT_PROFILES_FILE=str(profiles),
               FASTDDS_DEFAULT_PROFILES_FILE=str(profiles))
    command = [str(qemu), '-L', str(root), str(loader), '--library-path', libraries,
               str(binary), *arguments]
    return command, env


def main():
    try:
        command, environment = runtime_command(sys.argv[1:])
    except (OSError, ValueError, KeyError, RuntimeError) as error:
        raise SystemExit(f'Cannot start original romb kinematics: {error}') from error
    print('Starting original customer-romb agv_kinematics (verified amd64 binary under QEMU)',
          flush=True)
    os.execve(command[0], command, environment)


if __name__ == '__main__':
    main()
