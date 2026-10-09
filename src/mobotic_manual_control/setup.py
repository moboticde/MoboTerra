from setuptools import find_packages, setup


package_name = 'mobotic_manual_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/manual_control.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mobotic Development Team',
    maintainer_email='noreply@mobotic.invalid',
    description='Supervised joystick command source with optional deadman for MoboTerra.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'manual_control = mobotic_manual_control.manual_control:main',
        ],
    },
)
