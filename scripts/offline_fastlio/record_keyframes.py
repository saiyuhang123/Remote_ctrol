#!/usr/bin/env python3
"""Record LiDAR keyframes and FAST-LIO poses from a replay.

Subscribes to FAST-LIO outputs:
  /Odometry              nav_msgs/Odometry       (lio_odom -> imu_link)
  /cloud_registered_body sensor_msgs/PointCloud2 (scan in imu_link)

Each accepted keyframe is stored as an uncompressed .npz containing xyz,
intensity, stamp and the 4x4 pose in lio_odom. A JSONL manifest is written
alongside it so the loop-closure processor does not need ROS.

This is an offline data-capture tool. It does not modify FAST-LIO.
"""
from __future__ import annotations

import argparse
from collections import deque
import json
import math
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = x*x + y*y + z*z + w*w
    if not math.isfinite(n) or n < 1e-12:
        raise ValueError('invalid quaternion')
    s = 2.0 / n
    return np.array([
        [1-s*(y*y+z*z), s*(x*y-z*w),   s*(x*z+y*w)],
        [s*(x*y+z*w),   1-s*(x*x+z*z), s*(y*z-x*w)],
        [s*(x*z-y*w),   s*(y*z+x*w),   1-s*(x*x+y*y)],
    ], dtype=np.float64)


def pose_matrix(msg: Odometry) -> np.ndarray:
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = quat_to_matrix(q.x, q.y, q.z, q.w)
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def yaw_of(T: np.ndarray) -> float:
    return math.atan2(T[1, 0], T[0, 0])


def relative_yaw(a: np.ndarray, b: np.ndarray) -> float:
    return math.atan2(math.sin(yaw_of(b)-yaw_of(a)), math.cos(yaw_of(b)-yaw_of(a)))


class KeyframeRecorder(Node):
    def __init__(self, out: Path, distance: float, angle_deg: float,
                 max_gap: float, odom_tolerance: float,
                 stationary_distance: float = 0.15,
                 stationary_angle_deg: float = 2.0):
        super().__init__('fastlio_keyframe_recorder')
        self.out = out
        self.out.mkdir(parents=True, exist_ok=True)
        self.distance = float(distance)
        self.angle = math.radians(float(angle_deg))
        self.max_gap = float(max_gap)
        self.odom_tolerance = float(odom_tolerance)
        self.stationary_distance = float(stationary_distance)
        self.stationary_angle = math.radians(float(stationary_angle_deg))
        self.odoms: deque[tuple[int, np.ndarray]] = deque(maxlen=300)
        self.latest_cloud = None
        self.latest_cloud_stamp_ns = 0
        self.last_attempt_cloud_stamp_ns = 0
        self.last_pose = None
        self.last_stamp_ns = 0
        self.index = 0
        self.manifest = (out / 'keyframes.jsonl').open('w', encoding='utf-8')
        qos = QoSProfile(depth=20, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/Odometry', self.on_odom, 50)
        self.create_subscription(PointCloud2, '/cloud_registered_body', self.on_cloud, qos)
        self.create_timer(0.2, self.try_record)
        self.get_logger().info(
            f'keyframes -> {out}; trigger distance={self.distance:.2f}m '
            f'angle={angle_deg:.1f}deg max_gap={self.max_gap:.1f}s')

    def on_odom(self, msg: Odometry) -> None:
        try:
            self.odoms.append((int(msg.header.stamp.sec)*1_000_000_000 +
                               int(msg.header.stamp.nanosec), pose_matrix(msg)))
        except Exception:
            return

    def on_cloud(self, msg: PointCloud2) -> None:
        self.latest_cloud = msg
        self.latest_cloud_stamp_ns = (int(msg.header.stamp.sec)*1_000_000_000 +
                                      int(msg.header.stamp.nanosec))

    def find_pose(self, stamp_ns: int):
        best = None
        for s, T in reversed(self.odoms):
            dt = abs(s - stamp_ns) * 1e-9
            if best is None or dt < best[0]:
                best = (dt, T)
            if s < stamp_ns - int(self.odom_tolerance*1e9):
                break
        if best is None or best[0] > self.odom_tolerance:
            return None
        return best[1]

    def try_record(self) -> None:
        if self.latest_cloud is None:
            return
        # Avoid re-reading the same scan, including after a decode failure.
        if self.latest_cloud_stamp_ns == self.last_attempt_cloud_stamp_ns:
            return
        self.last_attempt_cloud_stamp_ns = self.latest_cloud_stamp_ns
        T = self.find_pose(self.latest_cloud_stamp_ns)
        if T is None:
            return
        if self.last_pose is not None:
            moved = float(np.linalg.norm(T[:3, 3] - self.last_pose[:3, 3]))
            turned = abs(relative_yaw(self.last_pose, T))
            elapsed = (self.latest_cloud_stamp_ns - self.last_stamp_ns) * 1e-9
            if moved < self.distance and turned < self.angle:
                if elapsed < self.max_gap:
                    return
                # Do not create endless duplicate keyframes while parked.
                if moved < self.stationary_distance and turned < self.stationary_angle:
                    return
            if elapsed <= 0:
                return
        # Decode only accepted scans.
        try:
            raw = np.asarray(list(point_cloud2.read_points(
                self.latest_cloud, field_names=('x', 'y', 'z', 'intensity'),
                skip_nans=True)))
            if len(raw) < 100:
                return
            xyz = np.column_stack((raw['x'], raw['y'], raw['z'])).astype(np.float32)
            intensity = np.asarray(raw['intensity'], dtype=np.float32)
        except Exception as e:
            self.get_logger().warn(f'point decode failed: {e}')
            return
        if xyz.ndim != 2 or xyz.shape[1] != 3 or len(xyz) < 100:
            return
        file_name = f'kf_{self.index:06d}.npz'
        np.savez(self.out / file_name, xyz=xyz, intensity=intensity,
                 stamp_ns=np.int64(self.latest_cloud_stamp_ns), pose=T)
        item = {
            'index': self.index,
            'file': file_name,
            'stamp_ns': int(self.latest_cloud_stamp_ns),
            'pose': T.tolist(),
            'points': int(len(xyz)),
            'x': float(T[0, 3]), 'y': float(T[1, 3]), 'z': float(T[2, 3]),
            'yaw': float(yaw_of(T)),
        }
        self.manifest.write(json.dumps(item, ensure_ascii=False) + '\n')
        self.manifest.flush()
        if self.index % 20 == 0:
            self.get_logger().info(f'keyframe {self.index}: {len(xyz)} points '
                                   f'({T[0,3]:.1f}, {T[1,3]:.1f}, {T[2,3]:.1f})')
        self.index += 1
        self.last_pose = T
        self.last_stamp_ns = self.latest_cloud_stamp_ns

    def close(self) -> None:
        self.manifest.close()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', default='/tmp/fastlio_keyframes')
    parser.add_argument('--distance', type=float, default=0.8)
    parser.add_argument('--angle-deg', type=float, default=12.0)
    parser.add_argument('--max-gap', type=float, default=2.0)
    parser.add_argument('--stationary-distance', type=float, default=0.15)
    parser.add_argument('--stationary-angle-deg', type=float, default=2.0)
    parser.add_argument('--odom-tolerance', type=float, default=0.08)
    args = parser.parse_args()
    rclpy.init()
    node = KeyframeRecorder(Path(args.output_dir).expanduser(),
                            args.distance, args.angle_deg,
                            args.max_gap, args.odom_tolerance,
                            args.stationary_distance,
                            args.stationary_angle_deg)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
