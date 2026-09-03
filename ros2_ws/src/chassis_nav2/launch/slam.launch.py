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
    slam_config = os.path.join(pkg, 'config', 'slam_sim.yaml')

    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')

    # 仿真（Gazebo + 车模 + 雷达）
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('chassis_gazebo'), 'launch', 'sim.launch.py')),
        launch_arguments={'gui': LaunchConfiguration('gui')}.items())

    # slam_toolbox 在线建图：订阅 /scan，发布 map->odom
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        parameters=[slam_config],
        output='screen')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg, 'config', 'slam.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': True}])

    return LaunchDescription([
        gui_arg,
        rviz_arg,
        sim,
        slam,
        rviz,
    ])
