from functools import partial
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from ament_index_python.packages import get_package_share_directory
import os
from pathlib import Path
from chassis_nav2.bringup_2d import configure_localization


def generate_launch_description():
    nav = Path(get_package_share_directory('chassis_nav2'))
    lidar = Path(get_package_share_directory('rslidar_sdk'))
    project = Path(os.environ.get('ROBOT_PROJECT_ROOT') or Path.home() / 'Remote_ctrol').expanduser()
    return LaunchDescription([
        DeclareLaunchArgument('hardware_file', default_value=str(nav/'config/hardware.yaml')),
        DeclareLaunchArgument('lidar_config', default_value=str(lidar/'config/config.yaml')),
        DeclareLaunchArgument('nav2_params', default_value=str(nav/'config/nav2_carto_localization.yaml')),
        DeclareLaunchArgument('map', default_value=str(project/'maps/cartographer/map.yaml'),
                              description='Nav2 2D 栅格地图（pbstream 转 pgm 产物，固定 maps/cartographer/map.yaml）'),
        DeclareLaunchArgument('map_file', default_value=str(project/'maps/cartographer/map'),
                              description='cartographer 序列化地图路径，不含扩展名（固定 maps/cartographer/map）'),
        DeclareLaunchArgument('start_sensors', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('start_chassis', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('enable_control', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=partial(configure_localization, backend='cartographer')),
    ])
