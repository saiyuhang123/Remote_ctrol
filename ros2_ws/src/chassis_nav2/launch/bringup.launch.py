from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from ament_index_python.packages import get_package_share_directory
import os
from pathlib import Path
from chassis_nav2.bringup import configure


def generate_launch_description():
    nav = Path(get_package_share_directory('chassis_nav2'))
    lidar = Path(get_package_share_directory('rslidar_sdk'))
    project = Path(os.environ.get('ROBOT_PROJECT_ROOT') or Path.home() / 'Remote_ctrol').expanduser()
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='drivers', choices=['drivers', 'mapping', 'localization', 'navigation']),
        DeclareLaunchArgument('hardware_file', default_value=str(nav/'config/hardware.yaml')),
        DeclareLaunchArgument('lidar_config', default_value=str(lidar/'config/config.yaml')),
        DeclareLaunchArgument('pcd_map', default_value=str(project/'maps/fastlio/map.pcd'),
                              description='建图输出文件 / 定位所用 PCD 地图（固定 maps/fastlio/map.pcd）'),
        DeclareLaunchArgument('map', default_value=str(project/'maps/fastlio/map.yaml'),
                              description='与 PCD 坐标一致的 Nav2 YAML 地图（固定 maps/fastlio/map.yaml）'),
        DeclareLaunchArgument('nav2_params', default_value=str(nav/'config/nav2.yaml')),
        DeclareLaunchArgument('start_sensors', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false'],
                              description='仅在 rosbag --clock 回放时设为 true'),
        DeclareLaunchArgument('start_chassis', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('enable_control', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('enable_recorded_routes', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('enable_patrol', default_value='false', choices=['true', 'false'],
                             description='启用 patrol twist_mux 失联锁，并同时启动 patrol_node 与 patrol_guard'),
        DeclareLaunchArgument('odom_source', default_value='lio', choices=['lio', 'wheel', 'wheel_imu']),
        DeclareLaunchArgument('compare_wheel_odometry', default_value='false', choices=['true', 'false'],
                             description='使用 LIO 时额外发布 /odom_wheel，需实测底盘反馈和轮速参数'),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=configure),
    ])
