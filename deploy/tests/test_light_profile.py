"""Check that the reduced overlay is complete and has no excluded packages."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('prepare_light', ROOT / 'deploy/container/prepare_light.py')
light = importlib.util.module_from_spec(spec)
spec.loader.exec_module(light)
scanner_spec = importlib.util.spec_from_file_location('scanner_deb', ROOT / 'deploy/container/prepare_scanner_deb.py')
scanner = importlib.util.module_from_spec(scanner_spec)
scanner_spec.loader.exec_module(scanner)


class LightProfileTest(unittest.TestCase):
    def test_scanner_keeps_runtime_dependencies_without_demo_gui_dependencies(self):
        original = ('Package: ros-jazzy-sick-safetyscanners2\nVersion: 1.0.5\n'
                    'Depends: libc6 (>= 2.34), ros-jazzy-rviz2,\n'
                    ' ros-jazzy-robot-state-publisher, ros-jazzy-xacro, ros-jazzy-rclcpp,\n'
                    ' ros-jazzy-sick-safetyscanners-base\nArchitecture: arm64\n')
        filtered = scanner.filter_control(original)
        self.assertNotIn('ros-jazzy-rviz2', filtered)
        self.assertNotIn('ros-jazzy-robot-state-publisher', filtered)
        self.assertNotIn('ros-jazzy-xacro', filtered)
        self.assertIn('libc6 (>= 2.34)', filtered)
        self.assertIn('ros-jazzy-rclcpp', filtered)
        self.assertIn('ros-jazzy-sick-safetyscanners-base', filtered)
        self.assertIn('\nArchitecture: arm64\n', filtered)

    def test_dependency_filter_preserves_driver_and_safety(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name in ('mobotic_odometry', 'mobotic_description', 'mobotic_bringup'):
                (root / name).mkdir()
            manifest = root / 'mobotic_bringup/package.xml'
            manifest.write_text('<package><name>mobotic_bringup</name>' + ''.join(
                f'<exec_depend>{name}</exec_depend>' for name in
                ('rviz2', 'robot_state_publisher', 'mobotic_odometry', 'mobotic_description',
                 'mobotic_driver', 'mobotic_safety', 'mobotic_kinematics')) + '</package>')
            light.prepare_source(root)
            self.assertFalse((root / 'mobotic_odometry').exists())
            self.assertFalse((root / 'mobotic_description').exists())
            self.assertEqual({e.text for e in ET.parse(manifest).getroot().findall('exec_depend')},
                             {'mobotic_driver', 'mobotic_safety', 'mobotic_kinematics'})

    def test_overlay_removes_executables_python_and_discovery_resources(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name in ('mobotic_odometry', 'mobotic_description', 'mobotic_driver'):
                for prefix in ('lib', 'share', 'lib/python3.12/site-packages'):
                    path = root / prefix / name
                    path.mkdir(parents=True)
                    (path / 'marker').write_text('installed')
                for prefix in ('share/ament_index/resource_index/packages', 'share/colcon-core/packages'):
                    path = root / prefix / name
                    path.parent.mkdir(parents=True, exist_ok=True)
                    path.write_text('')
            light.prepare_overlay(root)
            for name in ('mobotic_odometry', 'mobotic_description'):
                self.assertFalse(any(root.rglob(name)))
            self.assertTrue((root / 'lib/mobotic_driver/marker').is_file())
            self.assertTrue((root / 'share/ament_index/resource_index/packages/mobotic_driver').exists())


if __name__ == '__main__':
    unittest.main()
