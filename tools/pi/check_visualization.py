"""Validate native visualization dependencies without starting ROS nodes or CAN."""
import argparse
import os
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from mobotic_config.configuration import Configuration
from mobotic_config.description import render_description


def executable(package, name):
    path = Path(get_package_prefix(package)) / 'lib' / package / name
    if not path.is_file() or not os.access(path, os.X_OK):
        raise RuntimeError(f'Missing visualization executable: {path}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', required=True)
    parser.add_argument('--rviz-config', required=True)
    for name in ['start-rviz', 'start-description', 'start-odometry']:
        parser.add_argument('--' + name, choices=['true', 'false'], default='true')
    parser.add_argument('--require-display', action='store_true')
    args = parser.parse_args()
    cfg = Configuration(args.profile)
    if args.start_description == 'true':
        executable('robot_state_publisher', 'robot_state_publisher')
        model = ET.fromstring(render_description(cfg))
        for mesh in model.findall('.//mesh'):
            resource = mesh.attrib['filename']
            if resource.startswith('package://'):
                package, relative = resource[len('package://'):].split('/', 1)
                if not (Path(get_package_share_directory(package)) / relative).is_file():
                    raise RuntimeError(f'Missing robot mesh: {resource}')
    if args.start_odometry == 'true':
        executable('mobotic_odometry', 'mobotic_odometry')
    if args.start_rviz == 'true':
        executable('rviz2', 'rviz2')
        settings = yaml.safe_load(Path(args.rviz_config).read_text())
        if not isinstance(settings, dict) or 'Visualization Manager' not in settings:
            raise RuntimeError(f'Invalid RViz configuration: {args.rviz_config}')
        if args.require_display and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
            raise RuntimeError('RViz needs a desktop display. Use start_rviz:=false for a headless terminal.')
    print('Robot description, odometry and RViz selections validated; no nodes started.')


if __name__ == '__main__':
    main()
