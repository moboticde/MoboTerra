from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teltonika_gps_server',
            executable='teltonika_gps_server',
            name='teltonika_gps_server',
            output='screen',
            emulate_tty=True, # Recommended for seeing print statements in the terminal
            parameters=[
                # Set "tcp" or "udp" here
                {'protocol': 'tcp'},
                # Set the port
                {'port': 10001},
                {'gps_frame_id': 'gps'},
                {'gps_fix_pub_rate': 0.25}
            ]
        )
    ])