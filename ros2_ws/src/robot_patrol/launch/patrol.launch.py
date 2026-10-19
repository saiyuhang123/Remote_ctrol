from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    share = Path(get_package_share_directory("robot_patrol"))
    parameters = str(share / "config" / "patrol.yaml")
    enable_guard = LaunchConfiguration("enable_guard")

    patrol = Node(
        package="robot_patrol",
        executable="patrol_node",
        name="robot_patrol",
        parameters=[parameters, {
            'enable_recorded_routes': ParameterValue(LaunchConfiguration('enable_recorded_routes'), value_type=bool),
            'recorded_min_turning_radius': ParameterValue(LaunchConfiguration('recorded_min_turning_radius'), value_type=float),
            'recorded_footprint': ParameterValue(LaunchConfiguration('recorded_footprint'), value_type=str),
        }],
        output="screen",
    )
    recorder = Node(
        package="robot_patrol",
        executable="inspection_recorder",
        name="inspection_recorder",
        parameters=[parameters],
        output="screen",
    )
    guard = Node(
        package="robot_patrol",
        executable="patrol_guard",
        name="patrol_guard",
        parameters=[parameters],
        output="screen",
        condition=IfCondition(enable_guard),
    )
    return LaunchDescription([
        DeclareLaunchArgument('enable_recorded_routes', default_value='false', choices=['true','false']),
        DeclareLaunchArgument('recorded_min_turning_radius', default_value='0.69'),
        DeclareLaunchArgument('recorded_footprint', default_value='[[0.4475,0.28],[0.4475,-0.28],[-0.4475,-0.28],[-0.4475,0.28]]'),
        DeclareLaunchArgument(
            "enable_guard", default_value="true", choices=["true", "false"],
            description="启动独立 patrol_guard；实车巡航必须保持 true"),
        patrol,
        recorder,
        guard,
    ])
