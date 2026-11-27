"""Native ROMB packaging rejects foreign binaries and records vendor integrity."""
import hashlib
import importlib.util
import json
from pathlib import Path
import struct
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('prepare_romb', ROOT / 'deploy/container/prepare_romb.py')
preparation = importlib.util.module_from_spec(spec)
spec.loader.exec_module(preparation)


class NativeRombPackageTest(unittest.TestCase):
    def fixture(self, directory, machine):
        runtime = Path(directory)
        binary = runtime / 'rootfs/opt/ros/jazzy/lib/agv_kinematics/agv_kinematics'
        binary.parent.mkdir(parents=True)
        data = bytearray(64)
        data[:6] = b'\x7fELF\x02\x01'
        struct.pack_into('<H', data, 18, machine)
        binary.write_bytes(data)
        (runtime / 'udp-only.xml').write_text('<profiles/>')
        return runtime, binary

    def test_both_native_architectures_record_binary_and_package_hashes(self):
        for architecture, machine in (('arm64', 183), ('amd64', 62)):
            with self.subTest(architecture=architecture), tempfile.TemporaryDirectory() as directory:
                runtime, binary = self.fixture(directory, machine)
                preparation.prepare_runtime(runtime, architecture, 'a' * 64)
                record = json.loads((runtime / 'manifest.json').read_text())
                self.assertEqual(record, dict(runtime_kind='native', architecture=architecture,
                    kinematics_binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
                    source_package_sha256='a' * 64))

    def test_foreign_or_invalid_elf_cannot_be_labelled_native(self):
        for machine in (62, 0):
            with self.subTest(machine=machine), tempfile.TemporaryDirectory() as directory:
                runtime, _ = self.fixture(directory, machine)
                with self.assertRaisesRegex(ValueError, 'not a native arm64'):
                    preparation.prepare_runtime(runtime, 'arm64', 'a' * 64)
                self.assertFalse((runtime / 'manifest.json').exists())
        with tempfile.TemporaryDirectory() as directory:
            runtime, binary = self.fixture(directory, 183)
            binary.write_bytes(b'invalid')
            with self.assertRaisesRegex(ValueError, 'ELF64'):
                preparation.prepare_runtime(runtime, 'arm64', 'a' * 64)

    def test_missing_binary_and_invalid_source_hash_cannot_create_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime, binary = self.fixture(directory, 183)
            with self.assertRaisesRegex(ValueError, 'SHA-256'):
                preparation.prepare_runtime(runtime, 'arm64', 'invalid')
            binary.unlink()
            with self.assertRaisesRegex(ValueError, 'Incomplete'):
                preparation.prepare_runtime(runtime, 'arm64', 'a' * 64)
            self.assertFalse((runtime / 'manifest.json').exists())
