from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
from pathlib import Path


def generate_launch_description():
    nav = Path(get_package_share_directory('chassis_nav2'))
    lidar = Path(get_package_share_directory('rslidar_sdk'))
    project = Path(os.environ.get('ROBOT_PROJECT_ROOT') or Path.home() / 'Remote_ctrol').expanduser()
    bringup = os.path.join(get_package_share_directory('chassis_nav2'), 'launch', 'bringup.launch.py')
    forwarded = {
        'mode': 'navigation',
        'hardware_file': LaunchConfiguration('hardware_file'),
        'lidar_config': LaunchConfiguration('lidar_config'),
        'pcd_map': LaunchConfiguration('pcd_map'),
        'map': LaunchConfiguration('map'),
        'nav2_params': LaunchConfiguration('nav2_params'),
        'start_sensors': LaunchConfiguration('start_sensors'),
        'start_chassis': LaunchConfiguration('start_chassis'),
        'enable_control': LaunchConfiguration('enable_control'),
        'enable_patrol': LaunchConfiguration('enable_patrol'),
        'odom_source': LaunchConfiguration('odom_source'),
        'compare_wheel_odometry': LaunchConfiguration('compare_wheel_odometry'),
        'rviz': LaunchConfiguration('rviz'),
    }
    return LaunchDescription([
        DeclareLaunchArgument('hardware_file', default_value=str(nav/'config/hardware.yaml')),
        DeclareLaunchArgument('lidar_config', default_value=str(lidar/'config/config.yaml')),
        DeclareLaunchArgument('pcd_map', default_value=str(project/'maps/fastlio/map.pcd')),
        DeclareLaunchArgument('map', default_value=str(project/'maps/fastlio/map.yaml')),
        DeclareLaunchArgument('nav2_params', default_value=str(nav/'config/nav2.yaml')),
        DeclareLaunchArgument('start_sensors', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('start_chassis', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('enable_control', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('enable_patrol', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('odom_source', default_value='lio', choices=['lio', 'wheel', 'wheel_imu']),
        DeclareLaunchArgument('compare_wheel_odometry', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup),
            launch_arguments=forwarded.items()),
    ])
