"""NDT + CSV fixed routes on existing RoboSense/Wit/JD03 devices; no mapper."""
import json
import math
import os
from pathlib import Path
import yaml
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from . import bringup


def arguments(fixed=False):
    nav, lidar = bringup.share('chassis_nav2'), bringup.share('rslidar_sdk')
    project = Path(os.environ.get('ROBOT_PROJECT_ROOT') or Path.home()/'Remote_ctrol').expanduser()
    actions = [
        DeclareLaunchArgument('hardware_file', default_value=str(nav/'config/hardware.yaml')),
        DeclareLaunchArgument('lidar_config', default_value=str(lidar/'config/config.yaml')),
        DeclareLaunchArgument('pcd_map', default_value=str(project/'maps/fastlio/map.pcd')),
        DeclareLaunchArgument('pcd_dir', default_value='', description='旧系统多 PCD 地图目录，指定后优先于 pcd_map'),
        DeclareLaunchArgument('map', default_value='', description='可选，同坐标的栅格 YAML，仅供地图显示'),
        DeclareLaunchArgument('ndt_params', default_value=str(bringup.share('msf_localizer')/'config/ndt.yaml')),
        DeclareLaunchArgument('start_sensors', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('start_scan', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('start_chassis', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('enable_control', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('odom_source', default_value='wheel_imu', choices=['wheel_imu', 'wheel', 'external'],
                              description='external 使用外部已有 /odom 及 odom->base_link TF'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
    ]
    if fixed:
        actions += [
            DeclareLaunchArgument('start_localization', default_value='true', choices=['true', 'false']),
            DeclareLaunchArgument('route_file', default_value=str(project/'data/fixed_routes/route.csv')),
            DeclareLaunchArgument('route_params', default_value=str(bringup.share('fixed_route_navigation')/'config/fixed_route.yaml')),
            DeclareLaunchArgument('allow_reverse', default_value='false', choices=['true', 'false']),
        ]
    return actions


def stop_with(node, label):
    return RegisterEventHandler(OnProcessExit(target_action=node, on_exit=[
        EmitEvent(event=Shutdown(reason=label+' exited'))]))


def configure(context, fixed=False):
    args = context.launch_configurations
    yes = lambda key: args.get(key, 'false').lower() == 'true'
    sim = yes('use_sim_time')
    hw = yaml.safe_load(Path(args['hardware_file']).expanduser().read_text())
    nav = bringup.share('chassis_nav2')
    nodes = []
    if not fixed or yes('start_localization'):
        base_lidar, base_imu = bringup.calibrations(hw)
        nodes += [bringup.static_tf(base_lidar, 'base_link', 'velodyne'),
                  bringup.static_tf(base_imu, 'base_link', 'imu_link')]
        if yes('start_sensors'):
            nodes += [
                Node(package='rslidar_sdk', executable='rslidar_sdk_node', name='rslidar_sdk_node',
                     parameters=[{'config_path': bringup.existing_file(args['lidar_config'], '速腾配置'),
                                  'use_sim_time': sim}], output='screen'),
                Node(package='wit_ros2_imu', executable='wit_ros2_imu', name='imuDriverNode',
                     parameters=[hw['imu'], {'use_sim_time': sim}], output='screen'),
            ]
        if yes('start_chassis'):
            nodes.append(bringup.chassis_node(hw))
        if yes('enable_control'):
            if sim:
                raise ValueError('回放使用 use_sim_time:=true 时不能接管真实底盘')
            nodes.append(bringup.bridge_node(hw))
        source = args['odom_source']
        if source not in ('external', 'wheel', 'wheel_imu'):
            raise ValueError('NDT odom_source 仅支持 wheel_imu / wheel / external')
        if source != 'external':
            wheel = bringup.wheel_parameters(hw)
            wheel.update(use_sim_time=sim, odom_topic='/odom_raw' if source == 'wheel_imu' else '/odom',
                         odom_frame='wheel_odom' if source == 'wheel_imu' else 'odom',
                         publish_tf=source == 'wheel', comparison_topic='/odom_wheel')
            nodes.append(Node(package='chassis_nav2', executable='jd03_odometry.py', name='jd03_odometry',
                              parameters=[wheel], additional_env={'OPENBLAS_NUM_THREADS': '1'}, output='screen'))
        if source == 'wheel_imu':
            nodes.append(Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
                              parameters=[str(nav/'config/ekf.yaml'), {'use_sim_time': sim}],
                              remappings=[('odometry/filtered', '/odom')], output='screen'))
        # This entry owns the mux; start_localization:=false reuses the existing one.
        nodes.append(Node(package='twist_mux', executable='twist_mux',
                          parameters=[str(nav/'config/twist_mux.yaml'), {'use_sim_time': sim}],
                          remappings=[('cmd_vel_out', '/cmd_vel')], output='screen'))
        description = (bringup.share('chassis_description')/'urdf/chassis.urdf').read_text()
        nodes.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                          parameters=[{'robot_description': description, 'use_sim_time': sim}], output='screen'))
        map_params = {'use_sim_time': sim}
        if args.get('pcd_dir'):
            directory = Path(args['pcd_dir']).expanduser().resolve()
            if not directory.is_dir() or not any(directory.glob('*.pcd')):
                raise ValueError(f'PCD 地图目录无有效 .pcd：{directory}')
            map_params.update(pcd_map='', pcd_dir=str(directory))
        else:
            map_params.update(pcd_map=bringup.existing_file(args['pcd_map'], 'NDT 地图'), pcd_dir='')
        ndt = Node(package='msf_localizer', executable='multi_sensor_fusion', name='msf_localizer',
                   parameters=[bringup.existing_file(args['ndt_params'], 'NDT 参数'), map_params],
                   additional_env={'OMP_NUM_THREADS': '2'}, output='screen')
        nodes += [ndt, stop_with(ndt, 'NDT localization')]
        if yes('start_scan'):
            scan = dict(hw['scan'], use_sim_time=sim, target_frame='base_link',
                        transform_tolerance=.1, use_inf=True, inf_epsilon=1.)
            scan.setdefault('angle_min', -math.pi)
            scan.setdefault('angle_max', math.pi)
            nodes += [
                Node(package='chassis_nav2', executable='cloud_crop_box.py', name='cloud_crop_box',
                     parameters=[{'use_sim_time': sim}], output='screen'),
                Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
                     name='pointcloud_to_scan', parameters=[scan],
                     remappings=[('cloud_in', '/velodyne_points_cropped'), ('scan', '/scan')], output='screen'),
            ]
        if args.get('map'):
            nodes += [Node(package='nav2_map_server', executable='map_server', name='map_server',
                           parameters=[{'yaml_filename': bringup.existing_file(args['map'], '栅格地图'),
                                        'use_sim_time': sim}], output='screen'),
                      Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                           name='lifecycle_manager_map', parameters=[{'use_sim_time': sim, 'autostart': True,
                                                                     'node_names': ['map_server']}], output='screen')]
    if fixed:
        bringup.geometry(hw)
        radius = float(hw['chassis']['min_turning_radius'])
        if not math.isfinite(radius) or radius <= 0:
            raise ValueError('固定路线需要有效的实测 min_turning_radius')
        route = Node(package='fixed_route_navigation', executable='fixed_route_node', name='fixed_route',
                     parameters=[bringup.existing_file(args['route_params'], '路线参数'),
                                 {'use_sim_time': sim, 'route_file': str(Path(args['route_file']).expanduser().resolve()),
                                  'footprint': ParameterValue(json.dumps(hw['robot_geometry']['footprint']), value_type=str),
                                  'min_turning_radius': radius, 'allow_reverse': yes('allow_reverse')}], output='screen')
        nodes += [route, stop_with(route, 'Fixed route controller')]
    if yes('rviz'):
        nodes.append(Node(package='rviz2', executable='rviz2',
                          arguments=['-d', str(nav/'config/ndt_fixed_route.rviz')],
                          parameters=[{'use_sim_time': sim}], output='screen'))
    return nodes
