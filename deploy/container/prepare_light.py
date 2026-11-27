"""Prepare the reduced dependency manifest or remove optional overlay packages."""
import argparse
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

EXCLUDED = {'mobotic_odometry', 'mobotic_description', 'robot_state_publisher', 'rviz2'}


def prepare_source(root):
    for package in ('mobotic_odometry', 'mobotic_description'):
        shutil.rmtree(root / package)
    for manifest in root.glob('*/package.xml'):
        tree = ET.parse(manifest)
        for dependency in list(tree.getroot()):
            if dependency.tag.endswith('depend') and dependency.text in EXCLUDED:
                tree.getroot().remove(dependency)
        tree.write(manifest, encoding='unicode')


def prepare_overlay(root):
    for package in ('mobotic_odometry', 'mobotic_description'):
        for path in (root / 'share' / package, root / 'lib' / package):
            if path.exists():
                shutil.rmtree(path)
        for directory in (root / 'share/ament_index/resource_index').iterdir():
            if directory.is_dir():
                (directory / package).unlink(missing_ok=True)
        (root / 'share/colcon-core/packages' / package).unlink(missing_ok=True)
        for path in root.glob(f'lib/python*/site-packages/{package}*'):
            if path.is_dir():
                shutil.rmtree(path)
            else:
                path.unlink()
    for manifest in root.glob('share/*/package.xml'):
        tree = ET.parse(manifest)
        for dependency in list(tree.getroot()):
            if dependency.tag.endswith('depend') and dependency.text in EXCLUDED:
                tree.getroot().remove(dependency)
        tree.write(manifest, encoding='unicode')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('kind', choices=('source', 'overlay'))
    parser.add_argument('root', type=Path)
    args = parser.parse_args()
    (prepare_source if args.kind == 'source' else prepare_overlay)(args.root)
