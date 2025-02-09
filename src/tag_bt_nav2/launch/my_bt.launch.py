import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    tag_bt_nav2_dir = get_package_share_directory('tag_bt_nav2')

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'tb3_simulation_launch.py')
                ),
                launch_arguments={
                    'params_file': os.path.join(tag_bt_nav2_dir, 'config', 'nav2_params.yaml'),
                    'headless': 'True',
                    'use_simulator': 'False'
                }.items(),
            )
        ]
    )
