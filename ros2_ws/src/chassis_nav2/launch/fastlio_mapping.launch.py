"""Start the Wit IMU, RoboSense LiDAR and FAST-LIO mapping together."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def configure_mapping(context):
    args = context.launch_configurations
    if args['pcd_map'].strip():
        output = Path(args['pcd_map']).expanduser().resolve()
    else:
        directory = Path(args['map_dir']).expanduser().resolve()
        output = directory / 'map.pcd'

    bringup = Path(get_package_share_directory('chassis_nav2')) / 'launch/bringup.launch.py'
    return [
        LogInfo(msg=f'FAST-LIO PCD 保存路径：{output}（若已存在旧图会自动备份为 名称_时间戳.pcd）'),
        LogInfo(msg="建图结束后优化并保存：ros2 service call /loop_closure/save_map std_srvs/srv/Trigger '{}'"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup)),
            launch_arguments={
                'mode': 'mapping',
                'hardware_file': args['hardware_file'],
                'lidar_config': args['lidar_config'],
                'pcd_map': str(output),
                'start_sensors': args['start_sensors'],
                'use_sim_time': args['use_sim_time'],
                'start_chassis': 'false',
                'enable_control': 'false',
                'odom_source': 'lio',
                'compare_wheel_odometry': 'false',
                'rviz': args['rviz'],
            }.items(),
        ),
    ]


def generate_launch_description():
    nav = Path(get_package_share_directory('chassis_nav2'))
    lidar = Path(get_package_share_directory('rslidar_sdk'))
    project = Path(os.environ.get('ROBOT_PROJECT_ROOT') or Path.home() / 'Remote_ctrol').expanduser()
    return LaunchDescription([
        DeclareLaunchArgument('hardware_file', default_value=str(nav / 'config/hardware.yaml'),
                              description='雷达/IMU 实测外参及维特串口配置'),
        DeclareLaunchArgument('lidar_config', default_value=str(lidar / 'config/config.yaml'),
                              description='速腾雷达驱动配置'),
        DeclareLaunchArgument('map_dir', default_value=str(project / 'maps/fastlio'),
                              description='默认 PCD 保存目录（固定文件名 map.pcd）'),
        DeclareLaunchArgument('pcd_map', default_value='',
                              description='指定 PCD 保存路径；留空时保存为 map_dir/map.pcd'),
        DeclareLaunchArgument('start_sensors', default_value='true', choices=['true', 'false'],
                              description='同时启动雷达和 IMU；驱动已运行时设为 false'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false'],
                              description='打开 FAST-LIO 建图界面'),
        OpaqueFunction(function=configure_mapping),
    ])
