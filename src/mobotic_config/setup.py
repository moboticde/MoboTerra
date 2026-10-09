from glob import glob
from setuptools import find_packages, setup

setup(
    name='mobotic_config', version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/mobotic_config']),
        ('share/mobotic_config', ['package.xml', 'README.md']),
        ('share/mobotic_config/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'PyYAML'], zip_safe=True,
    maintainer='Mobotic Development Team', maintainer_email='noreply@mobotic.invalid',
    description='Authoritative MoboTerra configuration', license='Apache-2.0',
    entry_points={'console_scripts': ['check_config = mobotic_config.configuration:main']},
)
