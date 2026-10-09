from setuptools import find_packages, setup


package_name = 'mobotic_safety'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/safety.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mobotic Development Team',
    maintainer_email='noreply@mobotic.invalid',
    description='FlexiSoft monitoring bridge and fail-safe safety-state adapter.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'flexisoft_tcp_bridge = mobotic_safety.flexisoft_tcp_bridge:main',
            'safety_monitor = mobotic_safety.safety_monitor:main',
        ],
    },
)
