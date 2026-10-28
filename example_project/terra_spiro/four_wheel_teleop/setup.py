from setuptools import setup
# For launch files
import os
from glob import glob

package_name = 'four_wheel_teleop'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include all config files
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Damjan',
    maintainer_email='damjan@romb-technologies.hr',
    description='Teleoperation for four wheel steer-drive vehicles.',
    license='(c) Copyright 2025 RoMb Technologies d.o.o. All rights reserved.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_node = four_wheel_teleop.teleop_node:main'
        ],
    },
)
