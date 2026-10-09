from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from chassis_nav2.bringup import configure_chassis_odometry


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware_file', default_value=str(
            Path(get_package_share_directory('chassis_nav2'))/'config/hardware.yaml')),
        DeclareLaunchArgument('start_chassis', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('use_as_odom', default_value='false', choices=['true', 'false'],
                             description='true: 发布 /odom 和 odom->base_link；false: 只发布 /odom_wheel 做对比'),
        OpaqueFunction(function=configure_chassis_odometry),
    ])
