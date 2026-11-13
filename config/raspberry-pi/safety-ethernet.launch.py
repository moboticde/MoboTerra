import json
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    settings = json.loads(Path('/home/mobotic/Downloads/MoboTerra-main/config/raspberry-pi/safety-ethernet-parameters.json').read_text())
    nodes = []
    for location, parameters in settings['scanners'].items():
        nodes.append(Node(package='sick_safetyscanners2', executable='sick_safetyscanners2_node',
            name=f'{location}_scanner', namespace='moboterra', output='screen', parameters=[parameters],
            remappings=[('scan', f'scanner/{location}/scan'), ('raw_data', f'scanner/{location}/raw_data'),
                ('output_paths', f'scanner/{location}/output_paths'), ('field_data', f'scanner/{location}/field_data'),
                ('extended_scan', f'scanner/{location}/extended_laser_scan')]))
    for executable in ('flexisoft_tcp_bridge', 'safety_monitor', 'scanner_safety_adapter'):
        parameters = dict(settings['safety'])
        if executable == 'scanner_safety_adapter':
            parameters.update(scanner_source='scanner', scanner_input_suffix='raw_data')
        nodes.append(Node(package='mobotic_safety', executable=executable, name=executable,
            namespace='moboterra', output='screen', parameters=[parameters]))
    return LaunchDescription(nodes)
