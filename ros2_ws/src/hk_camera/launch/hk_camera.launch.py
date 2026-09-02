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
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


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
        'password', default_value='a1234567',
        description='设备密码'
    )
    channel_arg = DeclareLaunchArgument(
        'channel', default_value='1',
        description='通道号'
    )
    auto_login_arg = DeclareLaunchArgument(
        'auto_login', default_value='true',
        description='启动时自动登录并取流'
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
            'port': LaunchConfiguration('port'),
            'username': LaunchConfiguration('username'),
            'password': LaunchConfiguration('password'),
            'channel': LaunchConfiguration('channel'),
        }],
    )

    return LaunchDescription([
        ip_arg,
        port_arg,
        username_arg,
        password_arg,
        channel_arg,
        auto_login_arg,
        enable_gui_arg,
        LogInfo(msg=['Starting HKCamera node...']),
        hk_node,
    ])
