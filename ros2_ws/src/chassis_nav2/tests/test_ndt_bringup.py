"""Hardware routing/TF ownership checks without starting any ROS process."""
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import pytest
from launch import LaunchContext
from chassis_nav2 import bringup, ndt_bringup

SRC = Path(__file__).resolve().parents[2]


@pytest.fixture
def assembly(tmp_path):
    pcd = tmp_path/'site.pcd'
    pcd.write_text('test-only placeholder; no processes run')
    args = dict(hardware_file=str(SRC/'chassis_nav2/config/hardware.yaml'),
                lidar_config=str(SRC/'rslidar_sdk-v1.5.19/config/config.yaml'),
                ndt_params=str(SRC/'msf_localizer/config/ndt.yaml'),
                route_params=str(SRC/'fixed_route_navigation/config/fixed_route.yaml'),
                pcd_map=str(pcd), pcd_dir='', map='', route_file=str(tmp_path/'route.csv'),
                start_localization='true', start_sensors='true', start_scan='true',
                start_chassis='false', enable_control='false', odom_source='wheel_imu',
                use_sim_time='false', allow_reverse='false', rviz='false')
    paths = {name: SRC/name for name in ('chassis_nav2', 'chassis_description', 'msf_localizer', 'fixed_route_navigation')}
    paths['rslidar_sdk'] = SRC/'rslidar_sdk-v1.5.19'
    node = lambda **kwargs: kwargs
    with patch.object(bringup, 'share', side_effect=lambda name: paths[name]), \
            patch.object(bringup, 'Node', side_effect=node), \
            patch.object(ndt_bringup, 'Node', side_effect=node), \
            patch.object(ndt_bringup, 'stop_with', side_effect=lambda *unused: {'handler': 'exit'}):
        def assemble(fixed=False):
            return [n for n in ndt_bringup.configure(SimpleNamespace(launch_configurations=args), fixed) if 'package' in n]
        yield args, assemble


def test_default_ndt_pipeline_reuses_jd03_wit_and_robosense_without_a_mapper(assembly):
    args, assemble = assembly
    nodes = assemble(fixed=True)
    packages = [n['package'] for n in nodes]
    assert packages.count('msf_localizer') == packages.count('fixed_route_navigation') == 1
    assert packages.count('robot_localization') == 1
    assert 'fast_lio_localization' not in packages
    assert 'nav2_controller' not in packages and 'nav2_amcl' not in packages
    assert 'chassis_driver' not in packages  # Existing driver is an explicit option.
    imu = next(n for n in nodes if n['package'] == 'wit_ros2_imu')
    assert imu['parameters'][0]['port'] == '/dev/imu_usb'
    assert imu['parameters'][0]['baudrate'] == 230400
    wheel = next(n for n in nodes if n['executable'] == 'jd03_odometry.py')['parameters'][0]
    assert wheel['odom_topic'] == '/odom_raw' and not wheel['publish_tf']
    assert wheel['speed_scale'] == .29 and wheel['feedback_steering_sign'] == -1.
    assert wheel['wheel_base'] == .5 and wheel['four_wheel_steering']
    assert not any(n['executable'] == 'jd03_bridge.py' for n in nodes)


def test_external_odometry_never_starts_a_second_odom_tf_owner(assembly):
    args, assemble = assembly
    args['odom_source'] = 'external'
    nodes = assemble()
    assert not any(n['package'] == 'robot_localization' or n['executable'] == 'jd03_odometry.py' for n in nodes)


def test_wheel_only_odometry_owns_odom_tf_without_ekf(assembly):
    args, assemble = assembly
    args['odom_source'] = 'wheel'
    nodes = assemble()
    wheel = next(n for n in nodes if n['executable'] == 'jd03_odometry.py')['parameters'][0]
    assert wheel['odom_topic'] == '/odom' and wheel['publish_tf']
    assert not any(n['package'] == 'robot_localization' for n in nodes)


def test_reuse_existing_localization_only_adds_the_csv_controller(assembly):
    args, assemble = assembly
    args['start_localization'] = 'false'
    nodes = assemble(fixed=True)
    assert len(nodes) == 1 and nodes[0]['package'] == 'fixed_route_navigation'
    params = nodes[0]['parameters'][1]
    assert params['min_turning_radius'] == .69
    polygon = json.loads(params['footprint'].evaluate(LaunchContext()))
    assert polygon[0] == [.4475, .28]


def test_bag_clock_is_forwarded_to_odometry_ndt_and_sensor_conversion(assembly):
    args, assemble = assembly
    args['use_sim_time'] = 'true'
    nodes = assemble(fixed=True)
    for executable in ('jd03_odometry.py', 'ekf_node', 'multi_sensor_fusion',
                       'pointcloud_to_laserscan_node', 'cloud_crop_box.py', 'fixed_route_node'):
        node = next(n for n in nodes if n['executable'] == executable)
        assert any(isinstance(p, dict) and p.get('use_sim_time') is True for p in node['parameters'])


def test_replay_cannot_enable_real_chassis_control(assembly):
    args, assemble = assembly
    args.update(use_sim_time='true', enable_control='true')
    with pytest.raises(ValueError, match='真实底盘'):
        assemble()


def test_legacy_pcd_directory_overrides_the_default_single_map(assembly, tmp_path):
    args, assemble = assembly
    (tmp_path/'part.pcd').write_text('test-only placeholder')
    args.update(pcd_dir=str(tmp_path), pcd_map='/nonexistent/default.pcd')
    ndt = next(n for n in assemble() if n['package'] == 'msf_localizer')
    assert ndt['parameters'][1]['pcd_dir'] == str(tmp_path)
    assert ndt['parameters'][1]['pcd_map'] == ''
