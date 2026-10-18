"""Independent stationary duty: starts no navigation, chassis, gateway or patrol."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('start_detector', default_value='false'),
        Node(package='vision_events', executable='recorder', output='screen'),
        Node(package='detector_node', executable='detector', output='screen',
             condition=IfCondition(LaunchConfiguration('start_detector')),
             parameters=[{'input_mode': 'ros', 'enable_person': True, 'enable_plate': True}]),
    ])
