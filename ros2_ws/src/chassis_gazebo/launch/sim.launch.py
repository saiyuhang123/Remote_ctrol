import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('chassis_gazebo')
    urdf_file = os.path.join(pkg, 'urdf', 'chassis_gazebo.urdf')
    world_file = os.path.join(pkg, 'worlds', 'patrol.world')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # gui:=false 可无头运行（仅 gzserver，便于服务器/自动化验证）
    gui_arg = DeclareLaunchArgument('gui', default_value='true')

    # Gazebo 把 URDF 的 package:// 转成 model:// 后按 GAZEBO_MODEL_PATH 解析，
    # 必须把 chassis_description 的 share 父目录加进去，否则找不到 STL 网格
    # （找不到会进一步去 Fuel/在线模型库下载，国内网络直接卡死）。
    # 注意：设置 GAZEBO_MODEL_PATH 会覆盖内置默认路径，/usr/share/gazebo-11/models
    # （sun/ground_plane 所在地）必须显式补回，否则没有地面，模型会一直下落
    desc_share_parent = os.path.dirname(get_package_share_directory('chassis_description'))
    model_path = ':'.join(filter(None, [
        desc_share_parent,
        '/usr/share/gazebo-11/models',
        os.environ.get('GAZEBO_MODEL_PATH', ''),
    ]))

    # Gazebo Classic（gzserver + gzclient），加载巡逻测试场地
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_file,
                          'gui': LaunchConfiguration('gui')}.items())

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}])

    # 关节状态由仿真内 libgazebo_ros_joint_state_publisher 插件发布（真实转向/轮速），
    # 不再需要本地 joint_state_publisher 补零

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'chassis',
                   '-z', '0.02'],
        output='screen')

    return LaunchDescription([
        # 国内网络下 gzserver 访问 models.gazebosim.org 会卡死（spawn 时触发拉取模型库清单），
        # 指向一个秒拒地址让查询立即失败回退本地模型（sun/ground_plane 本地自带）
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', 'http://127.0.0.1:9/'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        gui_arg,
        gazebo,
        robot_state_publisher,
        spawn,
    ])
