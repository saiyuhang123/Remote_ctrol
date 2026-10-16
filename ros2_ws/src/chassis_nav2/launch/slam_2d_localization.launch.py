"""Compatibility entry: SLAM Toolbox maps now use AMCL for localization."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    entry = Path(get_package_share_directory('chassis_nav2'))/'launch/amcl_localization.launch.py'
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(entry)))
    ])
