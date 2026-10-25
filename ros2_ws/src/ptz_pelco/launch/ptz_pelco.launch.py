from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ptz_pelco',
            executable='ptz_pelco_node',
            name='ptz_pelco',
            output='screen',
            parameters=['/home/jkbot/Remote_ctrol/ros2_ws/src/ptz_pelco/config/ptz_pelco.yaml'],
        ),
    ])
