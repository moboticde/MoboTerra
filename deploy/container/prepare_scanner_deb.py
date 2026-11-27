"""Repackage the signed-repository scanner binary without demo GUI dependencies.

Only package metadata and the ROS dependency index change. Driver executables,
libraries, license files and safety processing remain the upstream binaries.
"""
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

OPTIONAL = {'ros-jazzy-rviz2', 'ros-jazzy-robot-state-publisher', 'ros-jazzy-xacro'}
ROS_OPTIONAL = {'rviz2', 'robot_state_publisher', 'xacro'}


def filter_control(text):
    def dependencies(match):
        items = match.group(1).replace('\n', ' ').split(',')
        kept = [item.strip() for item in items if item.strip().split()[0] not in OPTIONAL]
        return 'Depends: ' + ', '.join(kept)
    return re.sub(r'^Depends:\s*([^\n]*(?:\n[ \t][^\n]*)*)', dependencies, text, flags=re.M)


def repackage(source, output):
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        subprocess.run(['dpkg-deb', '--raw-extract', str(source), str(root)], check=True)
        control = root / 'DEBIAN/control'
        text = filter_control(control.read_text())
        text = re.sub(r'^(Version: .+)$', r'\1+moboterra1', text, flags=re.M)
        control.write_text(text)
        manifest = root / 'opt/ros/jazzy/share/sick_safetyscanners2/package.xml'
        tree = ET.parse(manifest)
        for dependency in list(tree.getroot()):
            if dependency.tag.endswith('depend') and dependency.text in ROS_OPTIONAL:
                tree.getroot().remove(dependency)
        tree.write(manifest, encoding='unicode')
        index = root / 'opt/ros/jazzy/share/ament_index/resource_index/package_run_dependencies/sick_safetyscanners2'
        if index.exists():
            index.write_text(';'.join(item for item in index.read_text().strip().split(';')
                                      if item not in ROS_OPTIONAL))
        # Modified ROS metadata has a new checksum; retain checksums for all binaries.
        checksum = root / 'DEBIAN/md5sums'
        if checksum.exists():
            import hashlib
            entries = []
            for line in checksum.read_text().splitlines():
                _, relative = line.split(None, 1)
                relative = relative.lstrip('./')
                entries.append(hashlib.md5((root / relative).read_bytes()).hexdigest() + '  ' + relative)
            checksum.write_text('\n'.join(entries) + '\n')
        subprocess.run(['dpkg-deb', '--build', str(root), str(output)], check=True)


if __name__ == '__main__':
    repackage(Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve())
