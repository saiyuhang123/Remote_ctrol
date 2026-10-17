"""AMCL + Nav2, using a 2D occupancy map saved by SLAM Toolbox."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os
from chassis_nav2.bringup_2d import configure_localization


def generate_launch_description():
    nav = Path(get_package_share_directory('chassis_nav2'))
    lidar = Path(get_package_share_directory('rslidar_sdk'))
    project = Path(os.environ.get('ROBOT_PROJECT_ROOT') or Path.home() / 'Remote_ctrol').expanduser()
    return LaunchDescription([
        DeclareLaunchArgument('hardware_file', default_value=str(nav/'config/hardware.yaml')),
        DeclareLaunchArgument('lidar_config', default_value=str(lidar/'config/config.yaml')),
        DeclareLaunchArgument('nav2_params', default_value=str(nav/'config/nav2_2d_localization.yaml')),
        DeclareLaunchArgument('amcl_params', default_value=str(nav/'config/amcl.yaml')),
        DeclareLaunchArgument('map', default_value=str(project/'maps/slam_toolbox/map.yaml'),
                              description='AMCL 与 Nav2 共用的二维栅格地图 YAML（配套 PGM/PNG）'),
        DeclareLaunchArgument('map_file', default_value=str(project/'maps/slam_toolbox/map'),
                              description='兼容旧命令；AMCL 不读取此参数及 posegraph/data'),
        DeclareLaunchArgument('start_sensors', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('start_chassis', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('enable_control', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('enable_patrol', default_value='false', choices=['true', 'false'],
                             description='启用 patrol twist_mux 失联锁，并同时启动 patrol_node 与 patrol_guard'),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=configure_localization),
    ])
