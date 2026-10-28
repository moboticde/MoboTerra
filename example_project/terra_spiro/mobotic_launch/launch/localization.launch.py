import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import SetRemap
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Get the path to the official Nav2 localization launch file
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    localization_launch_path = os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')

    # 2. Define the path to your custom parameters file and maps
    # (Update these paths to match where you saved them on your system)
    my_params_file = os.path.join(get_package_share_directory('mobotic_launch'), 'config', 'localization.yaml')
    my_map_file = os.path.join(get_package_share_directory('mobotic_launch'), 'maps', 'new-area-maps.yaml')

    return LaunchDescription([
        # 3. Use SetRemap to change the scan topic globally for all nodes included below
        SetRemap(src='scan', dst='/mobotic/front_scanner/scan'),

        # 4. Include the Nav2 bringup, which handles the lifecycle activation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch_path),
            launch_arguments={
                'maps': my_map_file,
                'params_file': my_params_file,
                'use_sim_time': 'false'
            }.items()
        )
    ])