from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from ament_index_python.packages import get_package_share_directory
from chassis_nav2.bringup import navigation_nodes, existing_file


def start(context):
    args = context.launch_configurations
    hw = yaml.safe_load(Path(args['hardware_file']).expanduser().read_text())
    return navigation_nodes(existing_file(args['map'], 'Nav2 栅格地图'), hw,
                            existing_file(args['params_file'], 'Nav2 参数'))


def generate_launch_description():
    pkg = Path(get_package_share_directory('chassis_nav2'))
    return LaunchDescription([
        DeclareLaunchArgument('map', description='实测导航地图 YAML'),
        DeclareLaunchArgument('params_file', default_value=str(pkg/'config/nav2.yaml')),
        DeclareLaunchArgument('hardware_file', default_value=str(pkg/'config/hardware.yaml')),
        OpaqueFunction(function=start),
    ])
