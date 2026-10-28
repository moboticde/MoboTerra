import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'mobotic_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.yml'),
        ),
        (
            os.path.join('share', package_name, 'maps'),
            glob('maps/*.yaml') + glob('maps/*.pgm') + glob('maps/*.posegraph') + glob('maps/*.data'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='ianpetek2004@gmail.com',
    description='SLAM and localization for the Mobotic robot using slam_toolbox.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odom_to_tf = mobotic_localization.odom_to_tf:main',
            'laser_tf_calibrator = mobotic_localization.laser_tf_calibrator:main',
        ],
    },
)
