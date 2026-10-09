from setuptools import find_packages, setup


package_name = 'mobotic_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/moboterra.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mobotic Development Team',
    maintainer_email='noreply@mobotic.invalid',
    description='Integrated hardware and virtual bringup for MoboTerra.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mock_platform_state = mobotic_bringup.mock_platform_state:main',
            'platform_diagnostics = mobotic_bringup.platform_diagnostics:main',
        ],
    },
)
