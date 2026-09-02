"""
Launch file for ocr_node.

Typical usage (service-only mode, default):
  ros2 launch ocr_node ocr_node.launch.py

Streaming mode (continuous + service):
  ros2 launch ocr_node ocr_node.launch.py enable_streaming:=true

English-only, service mode:
  ros2 launch ocr_node ocr_node.launch.py lang:=en
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ---- Declare launch arguments ----
    lang_arg = DeclareLaunchArgument(
        'lang',
        default_value='ch',
        description='OCR language: ch (Chinese+English) or en (English only)',
    )
    use_gpu_arg = DeclareLaunchArgument(
        'use_gpu',
        default_value='true',
        description='Use GPU acceleration (true/false)',
    )
    conf_threshold_arg = DeclareLaunchArgument(
        'conf_threshold',
        default_value='0.5',
        description='Minimum confidence threshold (0.0 - 1.0)',
    )
    enable_streaming_arg = DeclareLaunchArgument(
        'enable_streaming',
        default_value='false',
        description='Enable continuous streaming mode (true/false)',
    )
    processing_interval_arg = DeclareLaunchArgument(
        'processing_interval',
        default_value='0.5',
        description='Minimum seconds between streaming OCR runs',
    )
    enable_annotated_arg = DeclareLaunchArgument(
        'enable_annotated_img',
        default_value='true',
        description='Publish annotated preview image in streaming mode (true/false)',
    )

    # ---- Node ----
    ocr_node = Node(
        package='ocr_node',
        executable='ocr_node',
        name='ocr_node',
        output='screen',
        parameters=[{
            'lang': LaunchConfiguration('lang'),
            'use_gpu': LaunchConfiguration('use_gpu'),
            'conf_threshold': LaunchConfiguration('conf_threshold'),
            'enable_streaming': LaunchConfiguration('enable_streaming'),
            'processing_interval': LaunchConfiguration('processing_interval'),
            'enable_annotated_img': LaunchConfiguration('enable_annotated_img'),
        }],
    )

    info = LogInfo(
        msg='🚀 OCR Node ready — /ocr/recognize service available',
    )

    return LaunchDescription([
        lang_arg,
        use_gpu_arg,
        conf_threshold_arg,
        enable_streaming_arg,
        processing_interval_arg,
        enable_annotated_arg,
        info,
        ocr_node,
    ])
