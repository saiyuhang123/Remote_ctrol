from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    share = Path(get_package_share_directory("robot_patrol"))
    parameters = str(share / "config" / "patrol.yaml")
    return LaunchDescription([
        Node(
            package="robot_patrol",
            executable="patrol_node",
            name="robot_patrol",
            parameters=[parameters],
            output="screen",
        )
    ])
