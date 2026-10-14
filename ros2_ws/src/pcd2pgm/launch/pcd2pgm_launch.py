from pathlib import Path
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = Path(get_package_share_directory('pcd2pgm'))
    project = Path(os.environ.get('ROBOT_PROJECT_ROOT') or Path.home() / 'Remote_ctrol').expanduser()
    return LaunchDescription([
        DeclareLaunchArgument('pcd', default_value=str(project/'maps/map.pcd'),
                              description='用于 FAST-LIO 定位的同一份 PCD 地图（固定 maps/map.pcd）'),
        DeclareLaunchArgument('params_file', default_value=str(pkg/'config/pcd2pgm.yaml')),
        DeclareLaunchArgument('z_min', default_value='0.1'),
        DeclareLaunchArgument('z_max', default_value='2.0'),
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(package='pcd2pgm', executable='pcd2pgm_node', name='pcd2pgm',
             parameters=[LaunchConfiguration('params_file'), {
                 'pcd_file': LaunchConfiguration('pcd'), 'use_sim_time': False,
                 'thre_z_min': ParameterValue(LaunchConfiguration('z_min'), value_type=float),
                 'thre_z_max': ParameterValue(LaunchConfiguration('z_max'), value_type=float)}], output='screen'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(pkg/'rviz/pcd2pgm.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz')),
             parameters=[{'use_sim_time': False}]),
    ])
