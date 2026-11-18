"""Record the integrity of the runtime assembled from the pinned vendor package."""
import hashlib
import json
from pathlib import Path

runtime = Path('/opt/moboterra-romb')
binary = runtime / 'rootfs/opt/ros/jazzy/lib/agv_kinematics/agv_kinematics'
qemu = runtime / 'qemu-x86_64-static'
loader = runtime / 'rootfs/usr/lib/x86_64-linux-gnu/ld-linux-x86-64.so.2'
for file in (binary, qemu, loader, runtime / 'udp-only.xml'):
    if not file.is_file():
        raise SystemExit(f'Incomplete bundled ROMB runtime: {file}')
record = {
    'kinematics_binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
    'qemu_binary_sha256': hashlib.sha256(qemu.read_bytes()).hexdigest(),
    'source_package_sha256': 'ebabbc3aec523e083568c675b3634fc156adcf39023a5611f0da2c4422c16c4c',
}
(runtime / 'manifest.json').write_text(json.dumps(record, indent=2) + '\n')
