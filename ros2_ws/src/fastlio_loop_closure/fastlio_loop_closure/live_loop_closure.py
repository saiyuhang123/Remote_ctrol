#!/usr/bin/env python3
"""Capture live FAST-LIO keyframes; validate and refine measured overlaps on save."""
from __future__ import annotations

from collections import OrderedDict, deque
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import threading
import time

os.environ.setdefault('OMP_NUM_THREADS', '3')
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import Imu, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger

from .geometry import (Keyframe, atomic_json, build_map, level_rotation,
                       overlap_is_usable, refine, register_overlap, voxel_indices, write_pcd)


def stamp_ns(stamp):
    return int(stamp.sec)*1_000_000_000 + int(stamp.nanosec)


def valid_body_up(msg):
    q = msg.orientation
    quat = np.array([q.x, q.y, q.z, q.w])
    a = msg.linear_acceleration
    acceleration = np.array([a.x, a.y, a.z])
    if (msg.orientation_covariance[0] < 0 or not np.isfinite(quat).all()
            or not .95 <= np.linalg.norm(quat) <= 1.05
            or not np.isfinite(acceleration).all() or not 9. <= np.linalg.norm(acceleration) <= 10.6):
        return None
    up = Rotation.from_quat(quat).inv().apply([0., 0., 1.])
    if np.dot(up, acceleration/np.linalg.norm(acceleration)) < math.cos(math.radians(5.)):
        return None
    return up


def decode_cloud(msg):
    raw = point_cloud2.read_points(msg, field_names=('x', 'y', 'z', 'intensity'), skip_nans=True)
    xyz = np.column_stack((raw['x'], raw['y'], raw['z'])).astype(np.float32)
    intensity = np.asarray(raw['intensity'], dtype=np.float32)
    good = np.isfinite(xyz).all(axis=1) & np.isfinite(intensity)
    xyz, intensity = xyz[good], intensity[good]
    selected = voxel_indices(xyz, .15)
    return xyz[selected], intensity[selected]


class LiveLoopClosure(Node):
    def __init__(self):
        super().__init__('fastlio_loop_closure')
        self.output_map_path = Path(self.declare_parameter('output_map_path', '/tmp/fastlio_map.pcd').value).expanduser()
        self.keyframe_distance = float(self.declare_parameter('keyframe_distance', .8).value)
        self.keyframe_angle = math.radians(float(self.declare_parameter('keyframe_angle_deg', 10.).value))
        self.keyframe_max_gap = float(self.declare_parameter('keyframe_max_gap_sec', 2.).value)
        self.stationary_distance = float(self.declare_parameter('stationary_distance', .12).value)
        self.stationary_angle = math.radians(float(self.declare_parameter('stationary_angle_deg', 2.).value))
        self.search_radius = float(self.declare_parameter('search_radius', 5.).value)
        self.min_time_diff = float(self.declare_parameter('min_time_diff_sec', 300.).value)
        self.min_index_diff = int(self.declare_parameter('min_index_diff', 30).value)
        self.submap_half = int(self.declare_parameter('submap_half_window', 3).value)
        self.overlap_spacing = float(self.declare_parameter('overlap_spacing_m', 2.).value)
        self.map_voxel = float(self.declare_parameter('map_voxel', .10).value)
        self.frame = str(self.declare_parameter('lio_frame', 'lio_odom').value)
        imu_topic = str(self.declare_parameter('imu_topic', '/imu/data').value)
        self.session = self.output_map_path.with_name(
            self.output_map_path.stem+'_session_'+datetime.now().strftime('%Y%m%d_%H%M%S')+f'_{os.getpid()}')
        (self.session/'keyframes').mkdir(parents=True)
        self.manifest = (self.session/'keyframes.jsonl').open('a', buffering=1)
        self.trajectory = (self.session/'odometry.jsonl').open('a', buffering=1)
        self.odoms = OrderedDict()
        self.pending_clouds = OrderedDict()
        self.imus = deque(maxlen=300)
        self.keyframes = []
        self.matches = []
        self.rejected_matches = []
        self.map_rotation = None
        self.reference_up = None
        self.tilt_errors = []
        self.unmatched_clouds = 0
        self.total_clouds = 0
        self.last_cloud_stamp = 0
        self.fatal_error = ''
        self.processed = 0
        self.last_overlap_position = None
        self.lock = threading.RLock()
        self.processing_lock = threading.Lock()
        self.save_lock = threading.Lock()
        self.work_event = threading.Event()
        self.running = True
        self.callback_group = MutuallyExclusiveCallbackGroup()
        sensor_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/Odometry', self.on_odom, 200)
        self.create_subscription(PointCloud2, '/cloud_registered_body', self.on_cloud, sensor_qos)
        self.create_subscription(Imu, imu_topic, self.on_imu, sensor_qos)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                            reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub = self.create_publisher(PointCloud2, '/loop_closure/corrected_map', latched)
        self.path_pub = self.create_publisher(RosPath, '/loop_closure/corrected_path', latched)
        self.status_pub = self.create_publisher(String, '/loop_closure/status', latched)
        self.create_service(Trigger, '/loop_closure/save_map', self.save_callback,
                            callback_group=self.callback_group)
        self.create_service(Trigger, '/map_save', self.save_callback,
                            callback_group=self.callback_group)
        self.create_timer(5., self.publish_status)
        self.worker = threading.Thread(target=self.worker_loop, daemon=True)
        self.worker.start()
        atomic_json(self.session/'session.json', {'output': str(self.output_map_path),
            'workflow': 'gravity_aided_frontend; measured_structural_overlaps; xyz_yaw_refinement',
            'created': datetime.now().isoformat(), 'min_time_diff_sec': self.min_time_diff,
            'search_radius_m': self.search_radius, 'overlap_spacing_m': self.overlap_spacing,
            'no_start_end_identity_constraint': True})
        self.get_logger().info(f'gravity-preserving mapping ready; session={self.session}')

    def on_imu(self, msg):
        up = valid_body_up(msg)
        if up is not None:
            self.imus.append((stamp_ns(msg.header.stamp), up))

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        quat = np.array([q.x, q.y, q.z, q.w])
        if not np.isfinite(quat).all() or np.linalg.norm(quat) < .5:
            return
        pose = np.eye(4)
        pose[:3, :3] = Rotation.from_quat(quat).as_matrix()
        pose[:3, 3] = [p.x, p.y, p.z]
        if not np.isfinite(pose).all():
            self.fatal_error = '前端出现无效位姿，请停止并检查数据'
            return
        stamp = stamp_ns(msg.header.stamp)
        self.odoms[stamp] = pose
        while len(self.odoms) > 300:
            self.odoms.popitem(last=False)
        self.trajectory.write(json.dumps({'stamp_ns': stamp, 'pose': pose.tolist()})+'\n')
        pending = self.pending_clouds.pop(stamp, None)
        if pending is not None:
            self.record_keyframe(pending, pose)

    def on_cloud(self, msg):
        stamp = stamp_ns(msg.header.stamp)
        if stamp <= self.last_cloud_stamp:
            self.fatal_error = '检测到点云时间回退或重复；请新开建图会话'
            return
        self.last_cloud_stamp = stamp
        self.total_clouds += 1
        pose = self.odoms.get(stamp)
        if pose is None:
            self.pending_clouds[stamp] = msg
            while len(self.pending_clouds) > 30:
                self.pending_clouds.popitem(last=False)
                self.unmatched_clouds += 1
        else:
            self.record_keyframe(msg, pose)

    def record_keyframe(self, msg, pose):
        stamp = stamp_ns(msg.header.stamp)
        if self.imus:
            imu_stamp, up = min(self.imus, key=lambda item: abs(item[0]-stamp))
            if abs(imu_stamp-stamp) < 40_000_000:
                if self.map_rotation is None:
                    self.reference_up = pose[:3, :3] @ up
                    self.map_rotation = level_rotation(self.reference_up)
                    atomic_json(self.session/'gravity_reference.json',
                        {'stamp_ns': stamp, 'map_from_lio_rotation': self.map_rotation.tolist()})
                predicted = pose[:3, :3].T @ self.reference_up
                self.tilt_errors.append(float(np.rad2deg(np.arccos(np.clip(np.dot(up, predicted), -1., 1.)))))
        if self.keyframes:
            last = self.keyframes[-1]
            if stamp <= last.stamp_ns:
                return
            distance = np.linalg.norm(pose[:3, 3]-last.pose[:3, 3])
            angle = Rotation.from_matrix(last.pose[:3, :3].T @ pose[:3, :3]).magnitude()
            if distance < self.keyframe_distance and angle < self.keyframe_angle:
                if (stamp-last.stamp_ns)*1e-9 < self.keyframe_max_gap:
                    return
                if distance < self.stationary_distance and angle < self.stationary_angle:
                    return
        try:
            xyz, intensity = decode_cloud(msg)
            if len(xyz) < 100:
                return
            with self.lock:
                key = Keyframe(len(self.keyframes), stamp, pose.copy(), xyz, intensity)
                filename = f'{key.index:05d}.npz'
                np.savez(self.session/'keyframes'/filename, xyz=xyz, intensity=intensity,
                         pose=pose, stamp_ns=stamp)
                self.manifest.write(json.dumps({'index': key.index, 'stamp_ns': stamp,
                    'file': filename, 'points': len(xyz)})+'\n')
                self.keyframes.append(key)
                self.work_event.set()
            if key.index % 50 == 0:
                self.get_logger().info(f'keyframes={key.index+1}, xyz={np.round(pose[:3, 3], 2)}')
        except Exception as exc:
            self.fatal_error = f'关键帧保存失败：{exc}'
            self.get_logger().error(self.fatal_error)

    def process_keyframe(self, kfs, i):
        if i < self.min_index_diff:
            return
        current = kfs[i]
        stamps = np.array([k.stamp_ns for k in kfs[:i]])
        eligible_end = min(i-self.min_index_diff,
                           int(np.searchsorted(stamps, current.stamp_ns-int(self.min_time_diff*1e9), side='right')))
        if eligible_end <= 0:
            return
        positions = np.array([k.pose[:2, 3] for k in kfs[:eligible_end]])
        distances = np.linalg.norm(positions-current.pose[:2, 3], axis=1)
        target = int(np.argmin(distances))
        if distances[target] > self.search_radius:
            return
        if self.last_overlap_position is not None and np.linalg.norm(
                current.pose[:2, 3]-self.last_overlap_position) < self.overlap_spacing:
            return
        self.last_overlap_position = current.pose[:2, 3].copy()
        match = register_overlap(kfs, i, target, eligible_end, self.submap_half)
        if overlap_is_usable(match):
            self.matches.append(match)
            self.get_logger().info(f'overlap {i}->{target}: fitness={match["fitness"]:.3f}, rmse={match["rmse_m"]:.3f} m')
        elif match is not None:
            self.rejected_matches.append(match)

    def process_through(self, kfs):
        while self.processed < len(kfs):
            self.process_keyframe(kfs, self.processed)
            self.processed += 1

    def worker_loop(self):
        while self.running:
            self.work_event.wait(timeout=.5)
            self.work_event.clear()
            try:
                with self.processing_lock:
                    with self.lock:
                        kfs = list(self.keyframes)
                    self.process_through(kfs)
            except Exception as exc:
                self.fatal_error = f'重叠匹配失败：{exc}'
                self.get_logger().error(self.fatal_error)

    def publish_status(self):
        with self.lock:
            status = {'workflow': 'gravity_preserving_measured_overlaps', 'keyframes': len(self.keyframes),
                'qualified_overlaps': len(self.matches), 'processed_keyframes': self.processed,
                'gravity_reference_ready': self.map_rotation is not None,
                'unmatched_clouds': self.unmatched_clouds, 'error': self.fatal_error,
                'session': str(self.session)}
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))

    def save_callback(self, request, response):
        if not self.save_lock.acquire(blocking=False):
            response.success, response.message = False, '地图正在优化保存，请等待当前任务完成'
            return response
        try:
            response.success, response.message = self.save()
        except Exception as exc:
            response.success, response.message = False, f'地图未通过检查或保存失败：{exc}；关键帧保留于 {self.session}'
            self.get_logger().error(response.message)
        finally:
            self.save_lock.release()
        return response

    def save(self):
        begin = time.monotonic()
        with self.processing_lock:
            with self.lock:
                kfs = list(self.keyframes)
                error = self.fatal_error
                rotation = None if self.map_rotation is None else self.map_rotation.copy()
                tilt_errors = list(self.tilt_errors)
                unmatched = self.unmatched_clouds
                total_clouds = self.total_clouds
            if len(kfs) < 2:
                raise ValueError('尚无足够关键帧')
            if error:
                raise ValueError(error)
            if rotation is None or len(tilt_errors) < 10:
                raise ValueError('没有足够的有效 IMU 倾角观测')
            if np.percentile(tilt_errors, 95) > 3.:
                raise ValueError('前端倾角与 IMU 不一致，请检查倾角约束是否启用')
            if unmatched > max(5, total_clouds*.01):
                raise ValueError('点云与里程计时间匹配缺失过多')
            self.get_logger().info(f'finishing overlaps and refining {len(kfs)} keyframes')
            self.process_through(kfs)
            matches = list(self.matches)
            rejected = list(self.rejected_matches)
        poses = np.array([k.pose for k in kfs])
        poses[:, :3, :3] = rotation @ poses[:, :3, :3]
        poses[:, :3, 3] = poses[:, :3, 3] @ rotation.T
        report = {'status': 'REJECTED', 'keyframes': len(kfs), 'qualified_overlaps': len(matches),
            'overlap_measurements': matches, 'rejected_overlap_measurements': rejected,
            'map_from_lio_rotation': rotation.tolist(), 'last_keyframe_stamp_ns': kfs[-1].stamp_ns,
            'tilt_error_p95_deg': float(np.percentile(tilt_errors, 95)),
            'session': str(self.session), 'voxel_m': self.map_voxel}
        try:
            corrected, validation = refine(poses, matches)
            report.update(validation)
            if not validation['geometry_checks_passed']:
                raise ValueError('优化变形或留出重叠残差超过阈值')
        except Exception as exc:
            report['reason'] = str(exc)
            atomic_json(self.session/'validation.json', report)
            candidate = self.output_map_path.with_name(self.output_map_path.stem+'_UNVERIFIED.pcd')
            write_pcd(candidate, build_map(kfs, poses, self.map_voxel))
            return False, f'地图未通过几何检查：{exc}。候选图保留于 {candidate}；会话 {self.session}'
        points = build_map(kfs, corrected, self.map_voxel)
        report.update(status='GEOMETRY_CHECKS_PASSED_NEEDS_REAL_VEHICLE_LOCALIZATION_CHECK',
            points=len(points), pcd=str(self.output_map_path), elapsed_save_sec=time.monotonic()-begin,
            frame='gravity-aligned local map; origin at initial IMU position')
        np.save(self.session/'poses.npy', corrected)
        atomic_json(self.session/'validation.json', report)
        # The final filename is replaced only after optimization and validation succeed.
        write_pcd(self.output_map_path, points)
        report['sha256'] = hashlib.sha256(self.output_map_path.read_bytes()).hexdigest()
        atomic_json(self.output_map_path.with_suffix('.validation.json'), report)
        atomic_json(self.session/'validation.json', report)
        self.publish_result(kfs, corrected, points, rotation)
        return True, (f'已保存几何检查通过的 PCD：{self.output_map_path}（{len(points)} 点，'
                      f'{len(matches)} 处重叠）；上车后仍需验证定位。报告：{self.output_map_path.with_suffix(".validation.json")}')

    def publish_result(self, kfs, poses, points, rotation):
        # Keep the live visualization in the frontend frame. The PCD is levelled.
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.frame)
        display = points.copy()
        display[:, :3] = display[:, :3] @ rotation
        fields = [PointField(name=name, offset=k*4, datatype=PointField.FLOAT32, count=1)
                  for k, name in enumerate(('x', 'y', 'z', 'intensity'))]
        message = PointCloud2(header=header, height=1, width=len(display), fields=fields,
            is_bigendian=False, point_step=16, row_step=len(display)*16,
            data=np.ascontiguousarray(display, dtype='<f4').tobytes(), is_dense=True)
        self.map_pub.publish(message)
        path = RosPath(header=header)
        for key, pose in zip(kfs, poses):
            ps = PoseStamped(header=Header(stamp=rclpy.time.Time(nanoseconds=key.stamp_ns).to_msg(), frame_id=self.frame))
            pos = rotation.T @ pose[:3, 3]
            q = Rotation.from_matrix(rotation.T @ pose[:3, :3]).as_quat()
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, pos)
            ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = map(float, q)
            path.poses.append(ps)
        self.path_pub.publish(path)

    def shutdown(self):
        self.running = False
        self.work_event.set()
        self.worker.join(timeout=5.)
        self.manifest.close()
        self.trajectory.close()


def main():
    rclpy.init()
    node = LiveLoopClosure()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
