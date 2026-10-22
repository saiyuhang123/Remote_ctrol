#!/usr/bin/env python3
"""Check deployed FAST-LIO localization against a PCD using a private bag replay.

No sensor drivers, chassis interface, controllers or navigation goals are started.
The saved-map trajectory supplies only the initial pose and consistency reference,
not independent ground truth.
"""
import argparse
from collections import deque
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import time

os.environ.update(ROS_LOCALHOST_ONLY='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='3')

import numpy as np
from scipy.spatial.transform import Rotation
import yaml
import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Imu, PointCloud2

from fastlio_loop_closure.live_loop_closure import valid_body_up
from fastlio_loop_closure.geometry import level_rotation
from validate_fastlio_mapping import stop, wait_future


def stamp(message):
    return message.header.stamp.sec+message.header.stamp.nanosec*1e-9


def pose_row(message):
    p, q = message.pose.pose.position, message.pose.pose.orientation
    return [stamp(message), p.x, p.y, p.z, q.x, q.y, q.z, q.w]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--bag', required=True, type=Path)
    ap.add_argument('--output', required=True, type=Path)
    ap.add_argument('--pcd', required=True, type=Path)
    ap.add_argument('--map-yaml', required=True, type=Path)
    ap.add_argument('--initial-poses', required=True, type=Path)
    ap.add_argument('--rate', type=float, default=1.)
    ap.add_argument('--domain', type=int, default=218)
    args = ap.parse_args()
    if args.rate <= 0 or not 1 <= args.domain <= 232:
        ap.error('invalid rate or private ROS domain')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_LOG_DIR'] = str(out/'ros_logs')
    root = Path(__file__).resolve().parents[1]
    calibration = yaml.safe_load((root/'ros2_ws/src/chassis_nav2/config/hardware.yaml').read_text())['sensor_calibration']['base_from_imu']
    base_from_imu = np.eye(4)
    base_from_imu[:3, :3] = Rotation.from_euler('xyz', calibration['rpy']).as_matrix()
    base_from_imu[:3, 3] = calibration['translation']
    seed = np.load(args.initial_poses)[0] @ np.linalg.inv(base_from_imu)
    qseed = Rotation.from_matrix(seed[:3, :3]).as_quat()
    map_yaml = yaml.safe_load(args.map_yaml.read_text())
    rows, imus, fixes, localized, grids = [], [], [], [], []
    counts = {'lidar': 0, 'imu': 0}
    latest_input = [0.]
    rclpy.init()
    node = rclpy.create_node('deployed_localization_replay_check')
    qos = QoSProfile(depth=500, reliability=ReliabilityPolicy.BEST_EFFORT)

    def lidar(msg):
        counts['lidar'] += 1
        latest_input[0] = stamp(msg)

    def imu(msg):
        counts['imu'] += 1
        up = valid_body_up(msg)
        if up is not None:
            imus.append([stamp(msg), *up])

    def grid(msg):
        grids[:] = [{'frame': msg.header.frame_id, 'width': msg.info.width, 'height': msg.info.height,
                     'resolution': msg.info.resolution,
                     'origin': [msg.info.origin.position.x, msg.info.origin.position.y],
                     'cells': len(msg.data)}]

    node.create_subscription(PointCloud2, '/velodyne_points', lidar, qos)
    node.create_subscription(Imu, '/imu/data', imu, qos)
    node.create_subscription(Odometry, '/Odometry', lambda m: rows.append(pose_row(m)), qos)
    node.create_subscription(Odometry, '/map_to_odom', lambda m: fixes.append(pose_row(m)), qos)
    node.create_subscription(Odometry, '/localization', lambda m: localized.append(pose_row(m)), qos)
    node.create_subscription(OccupancyGrid, '/map', grid,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    initial = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
    params = node.create_client(GetParameters, '/fast_lio_mapping/get_parameters')
    mapper = player = None
    try:
        with (out/'launch.log').open('w') as launch_log, (out/'playback.log').open('w') as play_log:
            mapper = subprocess.Popen(['ros2', 'launch', 'chassis_nav2', 'bringup.launch.py',
                'mode:=localization', 'start_sensors:=false', 'start_chassis:=false', 'enable_control:=false',
                'rviz:=false', 'use_sim_time:=true', 'odom_source:=lio',
                'pcd_map:='+str(args.pcd.resolve()), 'map:='+str(args.map_yaml.resolve())],
                stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic()+30.
            while not params.service_is_ready():
                rclpy.spin_once(node, timeout_sec=.1)
                if mapper.poll() is not None or time.monotonic() >= deadline:
                    raise RuntimeError('localization frontend not ready; see launch.log')
            names = ['gravity_aid.enable', 'gravity_aid.sigma_deg', 'preprocess.scan_line',
                     'mapping.extrinsic_R', 'mapping.extrinsic_T', 'use_sim_time']
            response = wait_future(node, params.call_async(GetParameters.Request(names=names)), 5.)
            runtime = {name: parameter_value_to_python(v) for name, v in zip(names, response.values)}
            if not runtime['gravity_aid.enable'] or runtime['preprocess.scan_line'] != 16:
                raise RuntimeError('wrong localization profile')
            (out/'runtime_parameters.json').write_text(json.dumps(runtime, indent=2)+'\n')
            print(json.dumps({'runtime': runtime, 'initial_base_xyz': seed[:3, 3].tolist()}), flush=True)
            player = subprocess.Popen(['ros2', 'bag', 'play', str(args.bag.resolve()),
                '--clock', '100', '--rate', str(args.rate), '--delay', '2', '--read-ahead-queue-size', '2000',
                '--topics', '/velodyne_points', '/imu/data'],
                stdout=play_log, stderr=subprocess.STDOUT, start_new_session=True)
            progress_at, initial_at = time.monotonic()+30., time.monotonic()
            while player.poll() is None:
                rclpy.spin_once(node, timeout_sec=.05)
                now = time.monotonic()
                if mapper.poll() is not None:
                    raise RuntimeError('localization launch exited')
                if rows and not fixes and now >= initial_at:
                    pose = PoseWithCovarianceStamped()
                    pose.header.frame_id = 'map'
                    pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z = map(float, seed[:3, 3])
                    pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z, pose.pose.pose.orientation.w = map(float, qseed)
                    pose.pose.covariance[0] = pose.pose.covariance[7] = .25
                    pose.pose.covariance[35] = .07
                    initial.publish(pose)
                    initial_at = now+1.
                if now >= progress_at:
                    report = {'counts': counts.copy(), 'odometry': len(rows), 'map_fixes': len(fixes),
                              'localized': len(localized), 'map_loaded': bool(grids)}
                    if rows:
                        report.update(elapsed_sensor_s=rows[-1][0]-rows[0][0], xyz=rows[-1][1:4],
                                      lag_s=latest_input[0]-rows[-1][0])
                    (out/'progress.json').write_text(json.dumps(report, indent=2)+'\n')
                    print(json.dumps(report), flush=True)
                    progress_at += 30.
            if player.returncode:
                raise RuntimeError('bag playback failed')
            deadline = time.monotonic()+5.
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
        O, U, F, L = map(np.asarray, (rows, imus, fixes, localized))
        for name, values in [('odometry', O), ('imu_up', U), ('map_fixes', F), ('localization', L)]:
            np.save(out/(name+'.npy'), values)
        if len(O) < 100 or len(U) < 100 or len(F) < 5 or len(L) < 100:
            raise RuntimeError('insufficient frontend or localization data')
        idx = np.clip(np.searchsorted(U[:, 0], O[:, 0]), 0, len(U)-1)
        prior = np.maximum(idx-1, 0)
        idx = np.where(abs(U[prior, 0]-O[:, 0]) < abs(U[idx, 0]-O[:, 0]), prior, idx)
        good = abs(U[idx, 0]-O[:, 0]) < .04
        R = Rotation.from_quat(O[:, 4:8])
        reference_up = R[0].apply(U[idx[0], 1:4])
        predicted = R.inv().apply(np.tile(reference_up, (len(O), 1)))
        tilt = np.rad2deg(np.arccos(np.clip(np.sum(predicted*U[idx, 1:4], axis=1), -1., 1.)))[good]
        level = level_rotation(reference_up)
        height = (O[:, 1:4] @ level.T)[:, 2]
        log = (out/'launch.log').read_text()
        fitness = np.array([float(v) for v in re.findall(r'ICP fitness: ([0-9.]+)', log)])
        success_rate = float(np.mean(fitness > .5)) if len(fitness) else 0.
        duration = float(O[-1, 0]-O[0, 0])
        coverage = float((F[-1, 0]-F[0, 0])/duration)
        max_gap = float(np.diff(F[:, 0]).max())
        map_ok = bool(grids and grids[0]['frame'] == 'map' and grids[0]['cells'] > 0
                      and abs(grids[0]['resolution']-map_yaml['resolution']) < 1e-7
                      and np.allclose(grids[0]['origin'], map_yaml['origin'][:2], atol=1e-5))
        report = {'checks_passed': bool(np.isfinite(O).all() and np.isfinite(L).all()
            and len(O) >= .97*counts['lidar'] and np.percentile(tilt, 95) < 1.
            and success_rate >= .95 and coverage >= .95 and max_gap < 5. and map_ok),
            'runtime_parameters': runtime, 'counts': counts, 'odometry': len(O), 'map_fixes': len(F),
            'localized': len(L), 'duration_s': duration, 'tilt_error_p95_deg': float(np.percentile(tilt, 95)),
            'tilt_error_max_deg': float(tilt.max()), 'frontend_height_range_m': [float(height.min()), float(height.max())],
            'icp_fitness_quantiles': np.quantile(fitness, [0., .05, .5, 1.]).tolist() if len(fitness) else [],
            'icp_success_fraction': success_rate, 'map_fix_time_coverage': coverage, 'max_map_fix_gap_s': max_gap,
            'map_grid': grids[0] if grids else None, 'map_grid_metadata_matches': map_ok,
            'pcd': str(args.pcd.resolve()), 'pcd_sha256': hashlib.sha256(args.pcd.read_bytes()).hexdigest(),
            'rate': args.rate, 'domain': args.domain,
            'limits': 'Same-bag/map internal consistency check; initial pose from map trajectory. No independent truth or physical navigation test.'}
        (out/'localization_validation.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        print(json.dumps(report, ensure_ascii=False), flush=True)
        if not report['checks_passed']:
            raise RuntimeError('localization acceptance checks failed; see localization_validation.json')
    finally:
        stop(player)
        stop(mapper)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
