"""
海康相机 ROS2 Launch 文件

用法:
  # 只启动 ROS2 节点 (headless)
  ros2 launch hk_camera hk_camera.launch.py

  # 自定义参数
  ros2 launch hk_camera hk_camera.launch.py ip:=192.168.1.100 channel:=2
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from pathlib import Path


def generate_launch_description():
    # 参数声明
    ip_arg = DeclareLaunchArgument(
        'ip', default_value='192.168.1.64',
        description='海康相机 IP 地址'
    )
    port_arg = DeclareLaunchArgument(
        'port', default_value='8000',
        description='海康相机端口'
    )
    username_arg = DeclareLaunchArgument(
        'username', default_value='admin',
        description='设备用户名'
    )
    password_arg = DeclareLaunchArgument(
        'password', default_value='zmdl@603',
        description='设备密码'
    )
    channel_arg = DeclareLaunchArgument(
        'channel', default_value='1',
        description='通道号'
    )
    auto_login_arg = DeclareLaunchArgument(
        'auto_login', default_value='true',
        description='启动时自动登录'
    )
    auto_stream_arg = DeclareLaunchArgument(
        'auto_stream', default_value='true',
        description='自动登录成功后开始取流'
    )
    enable_face_capture_arg = DeclareLaunchArgument(
        'enable_face_capture', default_value='true',
        description='登录后接收相机原生人脸抓拍报警并保存图片'
    )
    face_capture_dir_arg = DeclareLaunchArgument(
        'face_capture_dir', default_value=str(Path.home() / 'Remote_ctrol/captures/hk_faces'),
        description='人脸小图、背景图、带框图和坐标 JSON 的保存目录'
    )
    raw_publish_fps_arg = DeclareLaunchArgument(
        'raw_publish_fps', default_value='8.0',
        description='原图发布上限 0..30 fps；0 表示不限帧'
    )
    enable_gui_arg = DeclareLaunchArgument(
        'enable_gui', default_value='false',
        description='是否启动 Qt GUI (需编译时启用 BUILD_HK_GUI)'
    )

    # 节点配置
    hk_node = Node(
        package='hk_camera',
        executable='hk_camera_node',
        name='hk_camera_node',
        output='screen',
        parameters=[{
            'ip': LaunchConfiguration('ip'),
            'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
            'username': LaunchConfiguration('username'),
            'password': LaunchConfiguration('password'),
            'channel': ParameterValue(LaunchConfiguration('channel'), value_type=int),
            'auto_login': ParameterValue(LaunchConfiguration('auto_login'), value_type=bool),
            'auto_stream': ParameterValue(LaunchConfiguration('auto_stream'), value_type=bool),
            'enable_face_capture': ParameterValue(LaunchConfiguration('enable_face_capture'), value_type=bool),
            'face_capture_dir': LaunchConfiguration('face_capture_dir'),
            'raw_publish_fps': ParameterValue(LaunchConfiguration('raw_publish_fps'), value_type=float),
        }],
    )

    return LaunchDescription([
        ip_arg,
        port_arg,
        username_arg,
        password_arg,
        channel_arg,
        auto_login_arg,
        auto_stream_arg,
        enable_face_capture_arg,
        face_capture_dir_arg,
        raw_publish_fps_arg,
        enable_gui_arg,
        LogInfo(msg=['Starting HKCamera node...']),
        hk_node,
    ])
