"""Assemble the real RoboSense/Wit/JD03 pipeline from one hardware file."""
import json
import math
from datetime import datetime
from pathlib import Path
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from chassis_nav2.geometry import transform, inverse, quaternion


def share(package):
    return Path(get_package_share_directory(package))


def vector(value, size, label):
    if not isinstance(value, list) or len(value) != size or not all(
            isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in value):
        raise ValueError(f'{label}: 请填写 {size} 个有限数值，新车外参不能沿用旧车或零值占位')
    return [float(x) for x in value]


def calibrations(hw):
    cfg = hw['sensor_calibration']
    if cfg.get('confirmed') is not True:
        raise ValueError('请先填写 hardware.yaml 的新车雷达/IMU 安装外参并设置 sensor_calibration.confirmed: true；目前可先启动 mode:=drivers 检查原始数据')
    matrices = []
    for sensor in ('lidar', 'imu'):
        pose = cfg[f'base_from_{sensor}']
        matrices.append(transform(vector(pose['translation'], 3, sensor+' translation'),
                                  vector(pose['rpy'], 3, sensor+' rpy')))
    return matrices


def geometry(hw):
    cfg = hw['robot_geometry']
    if cfg.get('confirmed') is not True:
        raise ValueError('请先填写新 JD03 的轴距、有效转角和 footprint，再设置 robot_geometry.confirmed: true')
    base = cfg['wheel_base']
    angle = cfg['max_steering_deg']
    if (not isinstance(base, (int, float)) or isinstance(base, bool) or not math.isfinite(base) or base <= 0
            or not isinstance(angle, (int, float)) or isinstance(angle, bool) or not math.isfinite(angle) or not 0 < angle <= 27):
        raise ValueError('wheel_base 必须为正数；max_steering_deg 必须在 (0,27] 内，与 JD03 编码限制一致')
    footprint = cfg['footprint']
    if not isinstance(footprint, list) or len(footprint) < 3:
        raise ValueError('footprint 至少需要三个顶点')
    for point in footprint:
        vector(point, 2, 'footprint')
    return float(base), float(angle), footprint


def wheel_parameters(hw):
    cfg = hw['wheel_odometry']
    if cfg.get('confirmed') is not True:
        raise ValueError('请先填写轴距、后轴相对 base_link 的位置和轮速/转角比例，设置 wheel_odometry.confirmed: true')
    base = hw['robot_geometry']['wheel_base']
    if isinstance(base, bool) or not isinstance(base, (int, float)) or not math.isfinite(base) or base <= 0:
        raise ValueError('轮速里程计需要实测 robot_geometry.wheel_base，单位 m')
    result = {k: cfg[k] for k in ('speed_scale', 'steering_scale', 'steering_bias_deg',
                                  'max_feedback_gap', 'speed_stddev', 'steering_stddev_deg',
                                  'lateral_velocity_stddev', 'stamp_source')}
    result.update(wheel_base=float(base), rear_axle_translation=vector(
        cfg['rear_axle_translation'], 2, '后轴中心相对 base_link 的 x/y'),
        feedback_steering_sign=hw['chassis']['feedback_steering_sign'],
        four_wheel_steering=bool(hw['chassis'].get('four_wheel_steering', False)),
        four_wheel_yaw_gain=float(hw['chassis'].get('four_wheel_curvature_gain', 2.)),
        use_sim_time=False)
    for key in ('speed_scale', 'steering_scale', 'max_feedback_gap', 'speed_stddev',
                'steering_stddev_deg', 'lateral_velocity_stddev'):
        result[key] = vector([result[key]], 1, key)[0]
        if result[key] <= 0:
            raise ValueError(f'{key} 必须为正数')
    result['steering_bias_deg'] = vector([result['steering_bias_deg']], 1, 'steering_bias_deg')[0]
    if result['stamp_source'] not in ('receipt', 'header') or result['feedback_steering_sign'] not in (-1., 1.):
        raise ValueError('stamp_source 必须为 receipt/header，feedback_steering_sign 必须为 -1/1')
    return result


def chassis_node(hw):
    if hw['chassis'].get('confirmed') is not True:
        raise ValueError('请先核对 JD03 CAN 通信配置，再设置 chassis.confirmed: true')
    return Node(package='chassis_driver', executable='chassis_driver_node', name='chassis_driver',
                parameters=[hw['chassis']['driver']], output='screen')


def wheel_node(hw, topic='/odom_wheel', frame='wheel_odom', publish_tf=False, comparison=''):
    params = wheel_parameters(hw)
    params.update(odom_topic=topic, odom_frame=frame, publish_tf=publish_tf, comparison_topic=comparison)
    return Node(package='chassis_nav2', executable='jd03_odometry.py', name='jd03_odometry',
                parameters=[params], additional_env={'OPENBLAS_NUM_THREADS': '1'}, output='screen')


def configure_chassis_odometry(context):
    args = context.launch_configurations
    hw = yaml.safe_load(Path(args['hardware_file']).expanduser().read_text())
    primary = args['use_as_odom'].lower() == 'true'
    # This entry uses only chassis geometry/feedback; no LiDAR or IMU calibration.
    nodes = [wheel_node(hw, '/odom' if primary else '/odom_wheel',
                        'odom' if primary else 'wheel_odom', primary,
                        '/odom_wheel' if primary else '')]
    if args['start_chassis'].lower() == 'true':
        nodes.insert(0, chassis_node(hw))
    return nodes


def static_tf(matrix, parent, child):
    x, y, z = matrix[:3, 3]
    qx, qy, qz, qw = quaternion(matrix)
    args = []
    for key, value in zip(('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'), (x, y, z, qx, qy, qz, qw)):
        args += ['--'+key, str(float(value))]
    args += ['--frame-id', parent, '--child-frame-id', child]
    return Node(package='tf2_ros', executable='static_transform_publisher',
                name=f'tf_{parent}_to_{child}', arguments=args, output='screen')


def existing_file(value, label):
    path = Path(value).expanduser()
    if not value or not path.is_file():
        raise ValueError(f'{label} 文件不存在：{value!r}；需要实测地图')
    return str(path.resolve())


def navigation_nodes(map_yaml, hw, params_file):
    from nav2_common.launch import ReplaceString, RewrittenYaml
    base, angle, footprint = geometry(hw)
    # 优先使用实测最小转弯半径（hardware.yaml chassis.min_turning_radius）。
    measured = hw['chassis'].get('min_turning_radius')
    if measured is not None:
        turning_radius = float(measured)
    else:
        halves = 2. if hw['chassis'].get('four_wheel_steering', False) else 1.
        turning_radius = base / (halves * math.tan(math.radians(angle)))
    params = RewrittenYaml(
        source_file=ReplaceString(source_file=params_file,
                                  replacements={'@CHASSIS_NAV2_SHARE@': str(share('chassis_nav2'))}),
        root_key='', param_rewrites={'footprint': json.dumps(footprint),
                                     'minimum_turning_radius': str(turning_radius),
                                     'use_sim_time': 'false'}, convert_types=True)
    nodes = [Node(package='nav2_map_server', executable='map_server', name='map_server',
                  parameters=[params, {'yaml_filename': map_yaml, 'use_sim_time': False}], output='screen')]
    names = []
    for package, executable in [('nav2_controller', 'controller_server'),
                                ('nav2_smoother', 'smoother_server'),
                                ('nav2_planner', 'planner_server'),
                                ('nav2_behaviors', 'behavior_server'),
                                ('nav2_bt_navigator', 'bt_navigator'),
                                ('nav2_waypoint_follower', 'waypoint_follower'),
                                ('nav2_velocity_smoother', 'velocity_smoother')]:
        remaps = []
        if executable in ('controller_server', 'behavior_server'):
            remaps = [('cmd_vel', '/cmd_vel_nav')]
        if executable == 'velocity_smoother':
            remaps = [('cmd_vel', '/cmd_vel_nav'), ('cmd_vel_smoothed', '/cmd_vel_auto')]
        nodes.append(Node(package=package, executable=executable, name=executable,
                          parameters=[params], remappings=remaps, output='screen'))
        names.append(executable)
    nodes += [Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                   name='lifecycle_manager_map', parameters=[{'use_sim_time': False,
                   'autostart': True, 'node_names': ['map_server']}], output='screen'),
              Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                   name='lifecycle_manager_navigation', parameters=[{'use_sim_time': False,
                   'autostart': True, 'node_names': names}], output='screen')]
    return nodes


def configure(context):
    args = context.launch_configurations
    mode = args['mode']
    hw = yaml.safe_load(Path(args['hardware_file']).expanduser().read_text())
    nodes = []
    active = mode != 'drivers'
    source = args['odom_source']
    wheel_imu = source == 'wheel_imu'
    compare_wheel = args.get('compare_wheel_odometry', 'false').lower() == 'true'
    control = args['enable_control'].lower() == 'true'
    chassis = args['start_chassis'].lower() == 'true'
    if mode == 'mapping' and source != 'lio':
        raise ValueError('建图使用 odom_source:=lio；轮式融合用于定位/导航')
    if not active and source != 'lio':
        raise ValueError('drivers 模式只检查驱动；单独轮速里程计请使用 chassis_odometry.launch.py')
    if active:
        base_lidar, base_body = calibrations(hw)
        body_lidar = inverse(base_body) @ base_lidar
        nodes += [static_tf(base_lidar, 'base_link', 'velodyne'),
                  static_tf(base_body, 'base_link', 'imu_link')]
    if args['start_sensors'].lower() == 'true':
        nodes += [Node(package='rslidar_sdk', executable='rslidar_sdk_node', name='rslidar_sdk_node',
                       parameters=[{'config_path': existing_file(args['lidar_config'], '速腾配置')}], output='screen'),
                  Node(package='wit_ros2_imu', executable='wit_ros2_imu', name='imuDriverNode',
                       parameters=[hw['imu'], {'use_sim_time': False}], output='screen')]
    if chassis:
        nodes.append(chassis_node(hw))
    if control:
        base, angle, _ = geometry(hw)
        if hw['chassis'].get('confirmed') is not True:
            raise ValueError('启用 JD03 控制/轮式融合前需要确认 chassis 配置')
        chassis_cfg = hw['chassis']
        params = {k: chassis_cfg[k] for k in ('command_steering_sign', 'feedback_steering_sign', 'max_speed')}
        params.update(wheel_base=base, max_steering_deg=angle, cmd_timeout=chassis_cfg['cmd_timeout'],
                      enable_control=True,
                      four_wheel_steering=bool(chassis_cfg.get('four_wheel_steering', False)),
                      rear_steering_sign=float(chassis_cfg.get('rear_steering_sign', -1.)),
                      four_wheel_curvature_gain=float(chassis_cfg.get('four_wheel_curvature_gain', 2.)))
        nodes.append(Node(package='chassis_nav2', executable='jd03_bridge.py', parameters=[params], output='screen'))
    if source == 'wheel':
        nodes.append(wheel_node(hw, '/odom', 'odom', True, '/odom_wheel'))
    elif wheel_imu:
        nodes.append(wheel_node(hw, '/odom_raw', 'wheel_odom', False, '/odom_wheel'))
    elif compare_wheel:
        nodes.append(wheel_node(hw))
    # Arbitration is also available during manual mapping; the gateway publishes /cmd_vel_teleop.
    if active or control:
        nodes.append(Node(package='twist_mux', executable='twist_mux',
                          parameters=[str(share('chassis_nav2')/'config/twist_mux.yaml')],
                          remappings=[('cmd_vel_out', '/cmd_vel')], output='screen'))
    if not active:
        return nodes
    description = (share('chassis_description')/'urdf/chassis.urdf').read_text()
    nodes.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                      parameters=[{'robot_description': description, 'use_sim_time': False}], output='screen'))
    profile = 'velodyne.yaml' if mode == 'mapping' else 'velodyne_test.yaml'
    profile_path = str(share('fast_lio_localization')/'config'/profile)
    fast = {'use_sim_time': False, 'lio_odom_frame': 'lio_odom', 'lio_body_frame': 'imu_link',
            'mapping.extrinsic_T': body_lidar[:3, 3].tolist(),
            'mapping.extrinsic_R': body_lidar[:3, :3].reshape(-1).tolist(),
            'common.time_offset_lidar_to_imu': float(hw['sensor_calibration']['time_offset_lidar_to_imu']),
            'base_from_body_translation': base_body[:3, 3].tolist(),
            'base_from_body_quaternion': quaternion(base_body),
            'broadcast_lio_frame': source != 'lio',
            'initial_base_z': float(-base_body[2, 3])}
    if mode == 'mapping':
        if not args['pcd_map']:
            raise ValueError('建图需要 pcd_map:=/绝对路径/site.pcd 作为保存路径')
        output = Path(args['pcd_map']).expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        if output.exists():
            backup = output.with_name(f"{output.stem}_{datetime.now():%Y%m%d_%H%M%S}{output.suffix}")
            output.rename(backup)
            print(f'[bringup] 旧地图已备份为 {backup.name}，本次建图保存到 {output.name}')
        fast['map_file_path'] = str(output)
    else:
        fast['map_file_path'] = existing_file(args['pcd_map'], '定位 PCD 地图')
    nodes.append(Node(package='fast_lio_localization', executable='fastlio_mapping', name='fast_lio_mapping',
                      parameters=[profile_path, fast], output='screen'))
    nodes.append(Node(package='chassis_nav2', executable='lio_odometry.py',
        parameters=[{'base_from_body_matrix': base_body.reshape(-1).tolist(),
                     'odom_topic': '/odom' if source == 'lio' else '/odom_lio',
                     'odom_frame': 'odom' if source == 'lio' else 'lio_odom',
                     'comparison_topic': '/odom_lio' if source == 'lio' else '',
                     'publish_tf': source == 'lio'}],
        additional_env={'OPENBLAS_NUM_THREADS': '1'}, output='screen'))
    if wheel_imu:
        nodes.append(Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
                          parameters=[str(share('chassis_nav2')/'config/ekf.yaml')],
                          remappings=[('odometry/filtered', '/odom')], output='screen'))
    elif source == 'lio':
        nodes.append(static_tf(np.eye(4), 'odom', 'lio_odom'))
    if mode in ('localization', 'navigation'):
        for executable, name in [('global_map_publisher.py', 'global_map_publisher'),
                                 ('global_localization.py', 'global_localization'),
                                 ('transform_fusion.py', 'transform_fusion')]:
            nodes.append(Node(package='fast_lio_localization', executable=executable, name=name,
                              parameters=[profile_path, fast],
                              additional_env={'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '2'}, output='screen'))
    # 摄像头支柱等车身固定物裁剪：/scan 链路不经过 LIO 的 blind，需在入口抠掉
    nodes.append(Node(package='chassis_nav2', executable='cloud_crop_box.py', name='cloud_crop_box',
                      parameters=[{'use_sim_time': False}], output='screen'))
    scan_params = dict(hw['scan'], use_sim_time=False, target_frame='base_link',
                       transform_tolerance=.1, angle_min=-math.pi, angle_max=math.pi,
                       use_inf=True, inf_epsilon=1.)
    nodes.append(Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
                      name='pointcloud_to_scan', parameters=[scan_params],
                      remappings=[('cloud_in', '/velodyne_points_cropped'), ('scan', '/scan')], output='screen'))
    if mode == 'localization' and args['map']:
        # 定位模式可选加载 Nav2 栅格地图（pcd2pgm 切片产物），供 RViz 叠加显示
        map_yaml = existing_file(args['map'], 'Nav2 栅格地图')
        nodes += [Node(package='nav2_map_server', executable='map_server', name='map_server',
                       parameters=[{'yaml_filename': map_yaml, 'use_sim_time': False}], output='screen'),
                  Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                       name='lifecycle_manager_map', parameters=[{'use_sim_time': False,
                       'autostart': True, 'node_names': ['map_server']}], output='screen')]
    if mode == 'navigation':
        nodes.extend(navigation_nodes(existing_file(args['map'], 'Nav2 栅格地图'), hw,
                                      existing_file(args['nav2_params'], 'Nav2 参数')))
    if args['rviz'].lower() == 'true':
        if mode == 'mapping':
            rviz_config = str(share('fast_lio_localization')/'rviz/fastlio.rviz')
        else:
            rviz_config = str(share('chassis_nav2')/'config/localization_nav.rviz')
        nodes.append(Node(package='rviz2', executable='rviz2',
                          arguments=['-d', rviz_config],
                          parameters=[{'use_sim_time': False}], output='screen'))
    return nodes
