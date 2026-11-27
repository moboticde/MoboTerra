"""Verify the native vendor executable and record the selected package integrity."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import struct


def prepare_runtime(runtime, architecture, package_sha256):
    binary = runtime / 'rootfs/opt/ros/jazzy/lib/agv_kinematics/agv_kinematics'
    for file in (binary, runtime / 'udp-only.xml'):
        if not file.is_file():
            raise ValueError(f'Incomplete bundled ROMB runtime: {file}')
    data = binary.read_bytes()
    machines = {'amd64': 62, 'arm64': 183}
    if (len(data) < 20 or data[:6] != b'\x7fELF\x02\x01'
            or struct.unpack_from('<H', data, 18)[0] != machines[architecture]):
        raise ValueError(f'ROMB executable is not a native {architecture} ELF64 binary')
    if not re.fullmatch(r'[0-9a-f]{64}', package_sha256):
        raise ValueError('Invalid ROMB source package SHA-256')
    record = {
        'runtime_kind': 'native',
        'architecture': architecture,
        'kinematics_binary_sha256': hashlib.sha256(data).hexdigest(),
        'source_package_sha256': package_sha256,
    }
    (runtime / 'manifest.json').write_text(json.dumps(record, indent=2) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime', type=Path, default=Path('/opt/moboterra-romb'))
    parser.add_argument('--architecture', choices=('amd64', 'arm64'), required=True)
    parser.add_argument('--package-sha256', required=True)
    args = parser.parse_args()
    try:
        prepare_runtime(args.runtime, args.architecture, args.package_sha256)
    except (OSError, ValueError) as error:
        raise SystemExit(str(error)) from error


if __name__ == '__main__':
    main()
