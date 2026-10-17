"""2D 管线：SLAM Toolbox 建图、AMCL 定位，或 Cartographer 建图/定位。

map->odom 由所选定位/建图节点提供；odom->base_link 固定由 EKF（轮速+IMU）发布。

与 3D 管线（bringup.py，FAST-LIO）互不混用；slam_toolbox 与 cartographer 两个 2D 后端
配置、地图文件、Nav2 参数各自独立，运行时三选一，不可同时启动。
"""
from pathlib import Path
import yaml
from launch_ros.actions import Node
from chassis_nav2 import bringup


def cartographer_nodes(nav_share, args, mapping):
    remaps = [('scan', '/scan'), ('odom', '/odom'), ('imu', '/imu/data')]
    arguments = ['-configuration_directory', str(nav_share/'config/cartographer')]
    if mapping:
        arguments += ['-configuration_basename', 'mapping_2d.lua']
    else:
        pbstream = bringup.existing_file(args['map_file'] + '.pbstream', 'cartographer 序列化地图')
        arguments += ['-configuration_basename', 'localization_2d.lua',
                      '-load_state_filename', pbstream]
    nodes = [Node(package='cartographer_ros', executable='cartographer_node', name='cartographer_node',
                  parameters=[{'use_sim_time': False}], arguments=arguments,
                  remappings=remaps, output='screen')]
    if mapping:
        # 建图时由 occupancy_grid_node 实时发布 /map；定位时 Nav2 静态层用 map_server 的 pgm
        nodes.append(Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node',
                          name='cartographer_occupancy_grid_node',
                          parameters=[{'use_sim_time': False}],
                          arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
                          output='screen'))
    return nodes


def slam_nodes(nav_share, args, mapping):
    if mapping:
        return [Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
                     parameters=[str(nav_share/'config/slam_toolbox_mapping.yaml'),
                                 {'use_sim_time': False}], output='screen')]
    params = bringup.existing_file(args.get('amcl_params', str(nav_share/'config/amcl.yaml')),
                                  'AMCL 参数')
    return [Node(package='nav2_amcl', executable='amcl', name='amcl',
                 parameters=[params, {'use_sim_time': False}],
                 output='screen')]


BACKENDS = {'slam_toolbox': slam_nodes, 'cartographer': cartographer_nodes}


def assemble(args, mapping, backend):
    if backend not in BACKENDS:
        raise ValueError(f'未知 2D 后端 {backend!r}，可选：{sorted(BACKENDS)}')
    hw = yaml.safe_load(Path(args['hardware_file']).expanduser().read_text())
    nav_share = bringup.share('chassis_nav2')
    nodes = []
    if mapping:
        # 建图结束手动存图到 maps/<backend>/map.*，提前建好目录避免保存失败
        import os
        project = Path(os.environ.get('ROBOT_PROJECT_ROOT') or Path.home() / 'Remote_ctrol').expanduser()
        (project/'maps'/backend).mkdir(parents=True, exist_ok=True)
    base_lidar, base_body = bringup.calibrations(hw)
    nodes += [bringup.static_tf(base_lidar, 'base_link', 'velodyne'),
              bringup.static_tf(base_body, 'base_link', 'imu_link')]
    if args['start_sensors'].lower() == 'true':
        nodes += bringup.sensor_nodes(hw, args['lidar_config'])
    if args['start_chassis'].lower() == 'true':
        nodes.append(bringup.chassis_node(hw))
    if args['enable_control'].lower() == 'true':
        nodes.append(bringup.bridge_node(hw))
    # 轮速 + IMU 经 EKF 发 odom->base_link 与 /odom，定位节点与 Nav2 共用
    nodes.append(bringup.wheel_node(hw, '/odom_raw', 'wheel_odom', False, '/odom_wheel'))
    nodes.append(bringup.ekf_node(hw))
    enable_patrol = args.get('enable_patrol', 'false').lower() == 'true'
    mux_config = 'twist_mux_patrol.yaml' if enable_patrol else 'twist_mux.yaml'
    nodes.append(Node(package='twist_mux', executable='twist_mux',
                      parameters=[str(nav_share/'config'/mux_config)],
                      remappings=[('cmd_vel_out', '/cmd_vel')], output='screen'))
    if enable_patrol:
        # The guard owns the positive-timeout mux lock. Never select the
        # patrol mux configuration without starting its executor and guard.
        nodes.append(bringup.patrol_launch())
    description = (bringup.share('chassis_description')/'urdf/chassis.urdf').read_text()
    nodes.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                      parameters=[{'robot_description': description, 'use_sim_time': False}], output='screen'))
    nodes += bringup.scan_nodes(hw)
    nodes += BACKENDS[backend](nav_share, args, mapping)
    if mapping:
        # 建图默认遥控，不起 Nav2；需要边建图边发目标点时 start_nav2:=true，
        # 此时没有先验栅格地图：不开 map_server，全局代价地图不含静态层
        if args.get('start_nav2', 'false').lower() == 'true':
            nodes.extend(bringup.navigation_nodes(None, hw, bringup.existing_file(
                args['nav2_params'], 'Nav2 参数'), use_map_server=False))
        rviz_config = str(nav_share/'config/slam.rviz')
    else:
        nav2_params = bringup.existing_file(args['nav2_params'], 'Nav2 参数')
        nodes.extend(bringup.navigation_nodes(bringup.existing_file(args['map'], 'Nav2 栅格地图'),
                                              hw, nav2_params, use_map_server=True,
                                              localization_names=('amcl',) if backend == 'slam_toolbox' else ()))
        rviz_config = str(nav_share/('config/amcl_nav.rviz' if backend == 'slam_toolbox' else 'config/localization_nav.rviz'))
    if args['rviz'].lower() == 'true':
        nodes.append(Node(package='rviz2', executable='rviz2',
                          arguments=['-d', rviz_config],
                          parameters=[{'use_sim_time': False}], output='screen'))
    return nodes


def configure_mapping(context, backend='slam_toolbox'):
    return assemble(context.launch_configurations, mapping=True, backend=backend)


def configure_localization(context, backend='slam_toolbox'):
    return assemble(context.launch_configurations, mapping=False, backend=backend)
