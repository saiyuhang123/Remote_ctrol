import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('chassis_nav2')
    nav2_params = os.path.join(pkg, 'config', 'nav2_sim.yaml')
    map_file = os.path.join(pkg, 'maps', 'patrol.yaml')

    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='false')

    # 仿真（Gazebo + 车模 + 雷达）
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('chassis_gazebo'), 'launch', 'sim.launch.py')),
        launch_arguments={'gui': LaunchConfiguration('gui')}.items())

    # Nav2 全家桶：map_server + AMCL + planner/controller/behavior/bt_navigator/
    # waypoint_follower/velocity_smoother + lifecycle 自动拉起
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'true',
            'params_file': nav2_params,
            'autostart': 'true',
        }.items())

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': True}])

    return LaunchDescription([
        gui_arg,
        rviz_arg,
        sim,
        nav2,
        rviz,
    ])
