#!/usr/bin/env python3
"""Replay a bag through the deployed mapping launch in an isolated ROS domain.

Source Humble and ros2_ws/install/setup.bash first. Never starts sensor drivers,
chassis communication, motion control, RViz, or the desktop application.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['OPENBLAS_NUM_THREADS'] = '1'
os.environ['OMP_NUM_THREADS'] = '3'

import numpy as np
import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger


def stop(process):
    if process is None or process.poll() is not None:
        return
    for sig, timeout in ((signal.SIGINT, 12), (signal.SIGTERM, 3), (signal.SIGKILL, 3)):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            pass


def wait_future(node, future, timeout):
    deadline = time.monotonic()+timeout
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    if not future.done():
        raise TimeoutError('ROS service response timed out')
    return future.result()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--bag', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--rate', type=float, default=1.)
    ap.add_argument('--domain', type=int, default=217)
    args = ap.parse_args()
    if args.rate <= 0 or not 1 <= args.domain <= 232:
        ap.error('invalid replay rate or private domain')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_LOG_DIR'] = str(out/'ros_logs')
    launch = player = None
    counters = {'lidar': 0, 'imu': 0, 'odom': 0, 'body_cloud': 0}
    statuses, odometry = [], []
    latest_input = [0.]
    rclpy.init()
    node = rclpy.create_node('deployed_mapping_replay_check')
    qos = QoSProfile(depth=500, reliability=ReliabilityPolicy.BEST_EFFORT)

    def lidar(msg):
        counters['lidar'] += 1
        latest_input[0] = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

    def imu(msg):
        counters['imu'] += 1

    def odom(msg):
        counters['odom'] += 1
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        odometry.append([msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                         p.x, p.y, p.z, q.x, q.y, q.z, q.w])

    def body(msg):
        counters['body_cloud'] += 1

    node.create_subscription(PointCloud2, '/velodyne_points', lidar, qos)
    node.create_subscription(Imu, '/imu/data', imu, qos)
    node.create_subscription(Odometry, '/Odometry', odom, qos)
    node.create_subscription(PointCloud2, '/cloud_registered_body', body, qos)
    node.create_subscription(String, '/loop_closure/status', lambda m: statuses.append(json.loads(m.data)),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    client = node.create_client(Trigger, '/map_save')
    params = node.create_client(GetParameters, '/fast_lio_mapping/get_parameters')
    try:
        with (out/'launch.log').open('w') as launch_log, (out/'playback.log').open('w') as play_log:
            launch = subprocess.Popen(['ros2', 'launch', 'chassis_nav2', 'bringup.launch.py',
                'mode:=mapping', 'start_sensors:=false', 'start_chassis:=false', 'enable_control:=false',
                'rviz:=false', 'use_sim_time:=true', 'pcd_map:='+str(out/'map.pcd')],
                stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic()+30.
            while not client.service_is_ready() or not params.service_is_ready():
                rclpy.spin_once(node, timeout_sec=.1)
                if launch.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError('mapping launch did not become ready; see launch.log')
            names = ['gravity_aid.enable', 'gravity_aid.sigma_deg', 'preprocess.scan_line',
                     'mapping.extrinsic_R', 'mapping.extrinsic_T', 'use_sim_time']
            response = wait_future(node, params.call_async(GetParameters.Request(names=names)), 5.)
            runtime = {name: parameter_value_to_python(v) for name, v in zip(names, response.values)}
            if not runtime['gravity_aid.enable'] or runtime['preprocess.scan_line'] != 16:
                raise RuntimeError('deployed frontend profile is incorrect')
            (out/'runtime_parameters.json').write_text(json.dumps(runtime, indent=2)+'\n')
            print(json.dumps({'runtime_parameters': runtime, 'private_domain': args.domain}), flush=True)
            player = subprocess.Popen(['ros2', 'bag', 'play', str(args.bag.resolve()),
                '--clock', '100', '--rate', str(args.rate), '--delay', '2', '--read-ahead-queue-size', '2000',
                '--topics', '/velodyne_points', '/imu/data'],
                stdout=play_log, stderr=subprocess.STDOUT, start_new_session=True)
            next_progress = time.monotonic()+30.
            while player.poll() is None:
                rclpy.spin_once(node, timeout_sec=.1)
                if launch.poll() is not None:
                    raise RuntimeError('mapping launch exited during replay')
                if time.monotonic() >= next_progress:
                    report = {'counts': counters.copy(), 'status': statuses[-1] if statuses else None}
                    if odometry:
                        report.update(elapsed_sensor_s=odometry[-1][0]-odometry[0][0],
                                      xyz=odometry[-1][1:4], lag_sec=latest_input[0]-odometry[-1][0])
                    print(json.dumps(report, ensure_ascii=False), flush=True)
                    (out/'progress.json').write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
                    next_progress += 30.
            if player.returncode:
                raise RuntimeError('rosbag playback failed')
            drain = time.monotonic()+4.
            while time.monotonic() < drain:
                rclpy.spin_once(node, timeout_sec=.1)
            np.save(out/'odometry.npy', np.asarray(odometry))
            print('replay complete; calling deployed /map_save', flush=True)
            saved = wait_future(node, client.call_async(Trigger.Request()), 180.)
            report = {'save_success': saved.success, 'save_message': saved.message,
                      'counts': counters.copy(), 'status': statuses[-1] if statuses else None,
                      'runtime_parameters': runtime, 'rate': args.rate, 'domain': args.domain}
            if saved.success:
                import open3d as o3d
                pcd = out/'map.pcd'
                xyz = np.asarray(o3d.io.read_point_cloud(str(pcd)).points)
                validation = json.loads((out/'map.validation.json').read_text())
                report.update(points=len(xyz), points_finite=bool(np.isfinite(xyz).all()),
                    sha256=hashlib.sha256(pcd.read_bytes()).hexdigest(), validation=validation)
            (out/'replay_validation.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
            print(json.dumps(report, ensure_ascii=False), flush=True)
            if not saved.success:
                raise RuntimeError(saved.message)
            if (not report['points_finite'] or report['points'] != validation['points']
                    or report['sha256'] != validation['sha256'] or not validation['geometry_checks_passed']
                    or counters['odom'] < .97*counters['lidar']):
                raise RuntimeError('replay acceptance checks failed')
    finally:
        stop(player)
        stop(launch)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
