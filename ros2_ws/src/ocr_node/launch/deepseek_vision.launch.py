"""Launch the online DeepSeek image-understanding service."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path', default_value='/home/jkbot/dshapi',
            description='Local YAML/text file containing the DeepSeek API key'),
        DeclareLaunchArgument(
            'service_name', default_value='/vision/recognize_image'),
        Node(
            package='ocr_node',
            executable='deepseek_vision',
            name='deepseek_vision',
            output='screen',
            parameters=[{
                'config_path': LaunchConfiguration('config_path'),
                'service_name': LaunchConfiguration('service_name'),
            }],
        ),
    ])
