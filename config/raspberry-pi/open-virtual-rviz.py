import json
import os
from pathlib import Path
import subprocess
import time

manifest = json.loads(Path('/opt/moboterra/software/release.json').read_text())
profile = Path(os.environ.get('MOBOTERRA_VIRTUAL_RVIZ_CONFIG', '/home/mobotic/Downloads/MoboTerra-main/config/raspberry-pi/virtual.rviz'))
authority = Path(os.environ.get('XAUTHORITY', '/run/user/1000/gdm/Xauthority'))
if not authority.is_file():
    raise SystemExit(f'Display authorization file is missing: {authority}')
gui_image = os.environ.get('MOBOTERRA_VIRTUAL_GUI_IMAGE', manifest['gui_image'])
display = os.environ.get('DISPLAY', ':1')
found = subprocess.run(['docker', 'inspect', '--format', '{{json .}}',
                        'moboterra-virtual-rviz'], capture_output=True, text=True)
if found.returncode == 0:
    existing = json.loads(found.stdout)
    command = existing['Config'].get('Cmd') or []
    if 'rviz2' not in command or '/tf:=/moboterra_test/tf' not in command:
        raise SystemExit('Existing viewer belongs to a different program; refusing to replace it.')
    environment = dict(value.split('=', 1) for value in existing['Config'].get('Env', []) if '=' in value)
    matches = (existing['Config']['Image'] == gui_image
               and environment.get('ROS_DOMAIN_ID') == '142'
               and environment.get('ROS_LOCALHOST_ONLY') == '0'
               and environment.get('ROS_AUTOMATIC_DISCOVERY_RANGE') == 'SYSTEM_DEFAULT'
               and environment.get('FASTDDS_BUILTIN_TRANSPORTS') == 'UDPv4'
               and environment.get('DISPLAY') == display
               and environment.get('FASTRTPS_DEFAULT_PROFILES_FILE') == '/tmp/virtual-fastdds.xml'
               and 'FASTDDS_DEFAULT_PROFILES_FILE' not in environment)
    if matches:
        if existing['State']['Running']:
            print('Reusing running virtual RViz.', flush=True)
        else:
            subprocess.run(['docker', 'start', 'moboterra-virtual-rviz'], check=True)
        raise SystemExit(0)
    logs = subprocess.run(['docker', 'logs', '--tail', '100', 'moboterra-virtual-rviz'], capture_output=True, text=True)
    log_path = profile.parent / ('rviz-before-update-' + time.strftime('%Y%m%d_%H%M%S') + '.log')
    log_path.write_text(logs.stdout + logs.stderr)
    if existing['State']['Running']:
        subprocess.run(['docker', 'stop', '--timeout', '10', 'moboterra-virtual-rviz'], check=True)
    subprocess.run(['docker', 'rm', 'moboterra-virtual-rviz'], check=True)
    print('Updating virtual RViz to use the same ROS discovery as the simulation.', flush=True)
render = Path('/dev/dri/renderD128')
graphics = ['--device', str(render), '--group-add', str(render.stat().st_gid)] if render.exists() else []
config = profile.read_text().replace('Frame Rate: 10', 'Frame Rate: 20') if graphics else profile.read_text().replace('Frame Rate: 20', 'Frame Rate: 10')
profile.write_text(config)
subprocess.run(['docker', 'run', '--detach', '--init', '--network', 'host',
 '--cap-drop', 'ALL', '--security-opt', 'no-new-privileges:true',
 '--name', 'moboterra-virtual-rviz', '--user', '1000:1000',
 *graphics,
 '--cpu-shares', '128',
 '--env', 'ROS_DOMAIN_ID=142', '--env', 'ROS_LOCALHOST_ONLY=0',
 '--env', 'ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT',
 '--env', 'FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/virtual-fastdds.xml',
 '--env', 'FASTDDS_BUILTIN_TRANSPORTS=UDPv4',
 '--env', f'DISPLAY={display}', '--env', 'XAUTHORITY=/tmp/.Xauthority',
 '--env', 'HOME=/tmp/moboterra-home', '--env', 'ROS_HOME=/tmp/moboterra-home/.ros',
 '--env', f'LIBGL_ALWAYS_SOFTWARE={0 if graphics else 1}', '--env', 'LP_NUM_THREADS=2',
 '--env', 'MESA_SHADER_CACHE_DIR=/tmp/moboterra-home/.cache/mesa_shader_cache',
 '--volume', '/tmp/.X11-unix:/tmp/.X11-unix:ro',
 '--volume', f'{authority}:/tmp/.Xauthority:ro',
 '--volume', f'{profile}:/tmp/virtual.rviz:ro',
 '--volume', '/home/mobotic/Downloads/MoboTerra-main/config/raspberry-pi/virtual-fastdds.xml:/tmp/virtual-fastdds.xml:ro',
 '--tmpfs', '/tmp/moboterra-home:rw,nosuid,nodev,mode=700,uid=1000,gid=1000',
 '--log-opt', 'max-size=10m', '--log-opt', 'max-file=2',
 gui_image, 'rviz2', '-d', '/tmp/virtual.rviz', '--ros-args',
 '-r', '/tf:=/moboterra_test/tf', '-r', '/tf_static:=/moboterra_test/tf_static'], check=True)
