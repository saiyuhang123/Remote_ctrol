from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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
        DeclareLaunchArgument(
            "enable_guard", default_value="true", choices=["true", "false"],
            description="启动独立 patrol_guard；实车巡航必须保持 true"),
        patrol,
        guard,
    ])
