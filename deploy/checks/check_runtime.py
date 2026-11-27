"""Verify the image's declared runtime profile, including excluded packages."""
import os
import subprocess
from ament_index_python.packages import get_packages_with_prefixes

if os.environ.get('MOBOTERRA_RUNTIME_PROFILE') == 'light':
    excluded = {'rviz2', 'robot_state_publisher', 'mobotic_odometry', 'mobotic_description'}
    present = excluded & get_packages_with_prefixes().keys()
    if present:
        raise RuntimeError(f'Excluded packages present in light image: {sorted(present)}')
else:
    subprocess.run(['rviz2', '--help'], check=True,
                   env={**os.environ, 'QT_QPA_PLATFORM': 'offscreen'})
