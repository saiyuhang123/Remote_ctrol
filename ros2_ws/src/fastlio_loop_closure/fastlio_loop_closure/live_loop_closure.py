#!/usr/bin/env python3
"""Live FAST-LIO 6DoF loop-closure backend.

This node consumes the already-running FAST-LIO mapping outputs and never
blocks the FAST-LIO callback path:
  /Odometry              nav_msgs/Odometry
  /cloud_registered_body sensor_msgs/PointCloud2

It records spatial keyframes, runs ICP loop detection in a worker thread,
optimizes a full 6DoF pose graph with Open3D, and exposes:
  /loop_closure/save_map       std_srvs/Trigger  (write corrected PCD)
  /loop_closure/corrected_map  sensor_msgs/PointCloud2
  /loop_closure/corrected_path nav_msgs/Path
  /loop_closure/status         std_msgs/String (JSON status)

The corrected PCD is written to `output_map_path`. The normal FAST-LIO raw
map can be configured to a different path so the UI save action can prefer
the corrected map.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
import math
from pathlib import Path
import threading

import numpy as np
import open3d as o3d
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger


def inv(T: np.ndarray) -> np.ndarray:
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ T[:3, 3]
    return out


def yaw_of(T: np.ndarray) -> float:
    return math.atan2(T[1, 0], T[0, 0])


def relative_yaw(a: np.ndarray, b: np.ndarray) -> float:
    return math.atan2(math.sin(yaw_of(b) - yaw_of(a)), math.cos(yaw_of(b) - yaw_of(a)))


def yaw_only_initial(T: np.ndarray) -> np.ndarray:
    """Useful for revisits outside: discard accumulated roll/pitch and z."""
    yaw = yaw_of(T)
    c, s = math.cos(yaw), math.sin(yaw)
    out = np.eye(4)
    out[:3, :3] = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
    out[:2, 3] = T[:2, 3]
    return out


def quat_xyzw(T: np.ndarray) -> tuple[float, float, float, float]:
    m = T[:3, :3]
    tr = float(np.trace(m))
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2.0
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
        w = 0.25 * s
    else:
        i = int(np.argmax(np.diag(m)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(max(0.0, 1.0 + m[i, i] - m[j, j] - m[k, k])) * 2.0
        q = [0.0, 0.0, 0.0, 0.0]
        q[i], q[j], q[k], q[3] = s / 4.0, (m[j, i] + m[i, j]) / s, (m[k, i] + m[i, k]) / s, (m[k, j] - m[j, k]) / s
        x, y, z, w = q
    n = math.sqrt(x*x + y*y + z*z + w*w)
    return x/n, y/n, z/n, w/n


def cloud_from_xyz(xyz: np.ndarray) -> o3d.geometry.PointCloud:
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(np.asarray(xyz, dtype=np.float64))
    return cloud


def try_icp(source: o3d.geometry.PointCloud, target: o3d.geometry.PointCloud,
            initial: np.ndarray, max_corr: float):
    target.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
        radius=max(0.5, max_corr * 2.0), max_nn=30))
    return o3d.pipelines.registration.registration_icp(
        source, target, max_corr, initial,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=60))


def write_pcd(path: Path, xyz: np.ndarray) -> None:
    """Write PCL-compatible x y z intensity normals curvature binary PCD."""
    n = len(xyz)
    out = np.zeros((n, 8), dtype=np.float32)
    out[:, :3] = xyz.astype(np.float32, copy=False)
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\n'
        'VERSION 0.7\n'
        'FIELDS x y z intensity normal_x normal_y normal_z curvature\n'
        'SIZE 4 4 4 4 4 4 4 4\n'
        'TYPE F F F F F F F F\n'
        'COUNT 1 1 1 1 1 1 1 1\n'
        f'WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {n}\nDATA binary\n'
    )
    tmp = path.with_name(path.name + '.tmp')
    with tmp.open('wb') as f:
        f.write(header.encode('ascii'))
        f.write(out.tobytes(order='C'))
    tmp.replace(path)


@dataclass
class Keyframe:
    index: int
    stamp_ns: int
    pose: np.ndarray
    xyz: np.ndarray
    intensity: np.ndarray
    down: o3d.geometry.PointCloud


class LiveLoopClosure(Node):
    def __init__(self):
        super().__init__('fastlio_loop_closure')
        self.output_map_path = Path(str(self.declare_parameter(
            'output_map_path', '/tmp/fastlio_loop_closed.pcd').value)).expanduser()
        self.keyframe_distance = float(self.declare_parameter('keyframe_distance', 0.8).value)
        self.keyframe_angle = math.radians(float(self.declare_parameter('keyframe_angle_deg', 12.0).value))
        self.keyframe_max_gap = float(self.declare_parameter('keyframe_max_gap_sec', 2.0).value)
        self.stationary_distance = float(self.declare_parameter('stationary_distance', 0.15).value)
        self.stationary_angle = math.radians(float(self.declare_parameter('stationary_angle_deg', 2.0).value))
        self.odom_tolerance = float(self.declare_parameter('odom_match_tolerance_sec', 0.08).value)
        self.icp_voxel = float(self.declare_parameter('icp_voxel', 0.25).value)
        self.target_voxel = float(self.declare_parameter('target_voxel', 0.20).value)
        self.map_voxel = float(self.declare_parameter('map_voxel', 0.10).value)
        self.search_radius = float(self.declare_parameter('search_radius', 12.0).value)
        self.min_time_diff = float(self.declare_parameter('min_time_diff_sec', 300.0).value)
        self.min_index_diff = int(self.declare_parameter('min_index_diff', 30).value)
        self.submap_half = int(self.declare_parameter('submap_half_window', 5).value)
        self.max_correspondence = float(self.declare_parameter('max_correspondence', 1.0).value)
        self.min_fitness = float(self.declare_parameter('min_fitness', 0.35).value)
        self.max_rmse = float(self.declare_parameter('max_rmse', 0.50).value)
        self.max_loops = int(self.declare_parameter('max_loops', 20).value)
        self.max_candidates = int(self.declare_parameter('max_candidates_per_key', 3).value)
        self.save_keyframe_voxel = float(self.declare_parameter('save_keyframe_voxel', 0.10).value)
        self.try_start_end_on_save = bool(self.declare_parameter('try_start_end_on_save', True).value)

        self.odoms: deque[tuple[int, np.ndarray]] = deque(maxlen=300)
        self.latest_cloud = None
        self.latest_cloud_stamp_ns = 0
        self.last_attempt_cloud_stamp_ns = 0
        self.last_keyframe_pose = None
        self.last_keyframe_stamp_ns = 0
        self.keyframes: list[Keyframe] = []
        self.loops: list[dict] = []
        self.corrected_poses: np.ndarray | None = None
        self.pending_index: int | None = None
        self.work_event = threading.Event()
        self.running = True
        self.lock = threading.RLock()
        self.save_lock = threading.Lock()
        self.worker = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker.start()

        cloud_qos = QoSProfile(depth=20, history=HistoryPolicy.KEEP_LAST,
                               reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Odometry, '/Odometry', self.on_odom, 50)
        self.create_subscription(PointCloud2, '/cloud_registered_body', self.on_cloud, cloud_qos)
        self.map_pub = self.create_publisher(PointCloud2, '/loop_closure/corrected_map', 1)
        self.path_pub = self.create_publisher(RosPath, '/loop_closure/corrected_path', 1)
        self.status_pub = self.create_publisher(String, '/loop_closure/status', 10)
        self.save_srv = self.create_service(Trigger, '/loop_closure/save_map', self.save_callback)
        self.get_logger().info(f'live 6DoF loop-closure ready; corrected map -> {self.output_map_path}')

    def on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        n = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        if not math.isfinite(n) or n < 1e-12:
            return
        s = 2.0 / n
        T = np.eye(4)
        T[:3, :3] = [
            [1-s*(q.y*q.y+q.z*q.z), s*(q.x*q.y-q.z*q.w), s*(q.x*q.z+q.y*q.w)],
            [s*(q.x*q.y+q.z*q.w), 1-s*(q.x*q.x+q.z*q.z), s*(q.y*q.z-q.x*q.w)],
            [s*(q.x*q.z-q.y*q.w), s*(q.y*q.z+q.x*q.w), 1-s*(q.x*q.x+q.y*q.y)],
        ]
        T[:3, 3] = [p.x, p.y, p.z]
        with self.lock:
            self.odoms.append((int(msg.header.stamp.sec)*1_000_000_000 + int(msg.header.stamp.nanosec), T))
        self._maybe_record_keyframe()

    def on_cloud(self, msg: PointCloud2) -> None:
        self.latest_cloud = msg
        self.latest_cloud_stamp_ns = int(msg.header.stamp.sec)*1_000_000_000 + int(msg.header.stamp.nanosec)
        self._maybe_record_keyframe()

    def _find_pose(self, stamp_ns: int) -> np.ndarray | None:
        with self.lock:
            odoms = list(self.odoms)
        best = None
        for s, T in reversed(odoms):
            dt = abs(s - stamp_ns) * 1e-9
            if best is None or dt < best[0]:
                best = (dt, T)
        if best is None or best[0] > self.odom_tolerance:
            return None
        return best[1]

    def _maybe_record_keyframe(self) -> None:
        msg = self.latest_cloud
        stamp = self.latest_cloud_stamp_ns
        if msg is None or stamp == self.last_attempt_cloud_stamp_ns:
            return
        self.last_attempt_cloud_stamp_ns = stamp
        T = self._find_pose(stamp)
        if T is None:
            return
        if self.last_keyframe_pose is not None:
            moved = float(np.linalg.norm(T[:3, 3] - self.last_keyframe_pose[:3, 3]))
            turned = abs(relative_yaw(self.last_keyframe_pose, T))
            elapsed = (stamp - self.last_keyframe_stamp_ns) * 1e-9
            if moved < self.keyframe_distance and turned < self.keyframe_angle:
                if elapsed < self.keyframe_max_gap:
                    return
                if moved < self.stationary_distance and turned < self.stationary_angle:
                    return
        try:
            raw = np.asarray(list(point_cloud2.read_points(
                msg, field_names=('x', 'y', 'z', 'intensity'), skip_nans=True)))
            if len(raw) < 100:
                return
            xyz = np.column_stack((raw['x'], raw['y'], raw['z'])).astype(np.float32)
            intensity = np.asarray(raw['intensity'], dtype=np.float32)
            down = cloud_from_xyz(xyz).voxel_down_sample(self.icp_voxel)
        except Exception as exc:
            self.get_logger().warn(f'keyframe decode failed: {exc}')
            return
        with self.lock:
            kf = Keyframe(len(self.keyframes), stamp, T.copy(), xyz, intensity, down)
            self.keyframes.append(kf)
            self.last_keyframe_pose = T.copy()
            self.last_keyframe_stamp_ns = stamp
            self.pending_index = kf.index
            self.work_event.set()
        if kf.index % 20 == 0:
            self.get_logger().info(f'keyframes={kf.index+1}, x={T[0,3]:.1f}, y={T[1,3]:.1f}, z={T[2,3]:.2f}')

    def _target_submap(self, kfs: list[Keyframe], center: int, half: int) -> o3d.geometry.PointCloud:
        lo, hi = max(0, center-half), min(len(kfs), center+half+1)
        center_inv = inv(kfs[center].pose)
        parts = []
        for k in range(lo, hi):
            xyz = kfs[k].xyz.astype(np.float64)
            T = center_inv @ kfs[k].pose
            parts.append(xyz @ T[:3, :3].T + T[:3, 3])
        return cloud_from_xyz(np.concatenate(parts, axis=0)).voxel_down_sample(self.target_voxel)

    def _candidate_loops(self, kfs: list[Keyframe], i: int) -> list[tuple[float, int]]:
        poses = np.asarray([k.pose for k in kfs])
        pos = poses[:, :3, 3]
        d = np.linalg.norm(pos[:i, :2] - pos[i, :2], axis=1)
        candidates = []
        for j in np.where(d <= self.search_radius)[0]:
            j = int(j)
            if j >= i - self.min_index_diff:
                continue
            if (kfs[i].stamp_ns - kfs[j].stamp_ns) * 1e-9 < self.min_time_diff:
                continue
            candidates.append((float(d[j]), j))
        candidates.sort()
        return candidates[:self.max_candidates]

    def _process_keyframe(self, i: int) -> None:
        with self.lock:
            kfs = list(self.keyframes)
            existing = {(int(loop['source']), int(loop['target'])) for loop in self.loops}
        if i >= len(kfs) or i < self.min_index_diff:
            return
        best = None
        source = kfs[i].down
        for _, j in self._candidate_loops(kfs, i):
            if (i, j) in existing:
                continue
            target = self._target_submap(kfs, j, self.submap_half)
            if len(target.points) < 200:
                continue
            initial_raw = inv(kfs[j].pose) @ kfs[i].pose
            for initial in (initial_raw, yaw_only_initial(initial_raw)):
                try:
                    result = try_icp(source, target, initial, self.max_correspondence)
                except Exception:
                    continue
                if result.fitness < self.min_fitness or result.inlier_rmse > self.max_rmse:
                    continue
                delta = inv(initial_raw) @ result.transformation
                dt_xy = float(np.linalg.norm(delta[:2, 3]))
                dt_z = abs(float(delta[2, 3]))
                da = abs(relative_yaw(np.eye(4), delta))
                # Long outdoor loops can have many metres of Z drift. Allow
                # large vertical correction, but keep a tight horizontal/yaw
                # gate so a nearby parallel corridor is not accepted.
                if dt_xy > 5.0 or dt_z > 20.0 or da > math.radians(30.0):
                    continue
                score = result.fitness - 0.1 * result.inlier_rmse
                if best is None or score > best[0]:
                    best = (score, j, result.transformation, result.fitness, result.inlier_rmse, dt, da)
        if best is None:
            return
        _, j, transform, fitness, rmse, dt, da = best
        loop = {'source': i, 'target': j, 'transform': transform.tolist(),
                'fitness': float(fitness), 'rmse': float(rmse),
                'correction_translation': dt, 'correction_yaw_rad': da}
        with self.lock:
            if len(self.loops) >= self.max_loops:
                return
            self.loops.append(loop)
            kfs = list(self.keyframes)
            loops = list(self.loops)
        self.get_logger().info(f'loop accepted {j}->{i}: fitness={fitness:.3f} rmse={rmse:.3f}')
        try:
            corrected = self._optimize(kfs, loops)
            self._publish_result(kfs, corrected, loops)
        except Exception as exc:
            self.get_logger().warn(f'pose-graph optimization failed: {exc}')

    def _optimize(self, kfs: list[Keyframe], loops: list[dict]) -> np.ndarray:
        graph = o3d.pipelines.registration.PoseGraph()
        for k in kfs:
            graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(k.pose))
        odom_info = np.diag([100., 100., 40., 100., 100., 40.])
        loop_info = np.diag([20., 20., 8., 20., 20., 8.])
        for i in range(len(kfs)-1):
            T_edge = inv(kfs[i+1].pose) @ kfs[i].pose
            graph.edges.append(o3d.pipelines.registration.PoseGraphEdge(
                i, i+1, T_edge, odom_info, uncertain=False))
        for loop in loops:
            graph.edges.append(o3d.pipelines.registration.PoseGraphEdge(
                int(loop['source']), int(loop['target']),
                np.asarray(loop['transform'], dtype=np.float64),
                loop_info, uncertain=True))
        method = o3d.pipelines.registration.GlobalOptimizationGaussNewton()
        criteria = o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria()
        option = o3d.pipelines.registration.GlobalOptimizationOption(
            max_correspondence_distance=0.35, edge_prune_threshold=0.25, reference_node=0)
        o3d.pipelines.registration.global_optimization(graph, method, criteria, option)
        return np.asarray([np.asarray(n.pose) for n in graph.nodes])

    def _build_map(self, kfs: list[Keyframe], poses: np.ndarray, voxel: float) -> np.ndarray:
        parts = []
        for k, T in zip(kfs, poses):
            xyz = k.xyz.astype(np.float64)
            parts.append(xyz @ T[:3, :3].T + T[:3, 3])
        merged = np.concatenate(parts, axis=0)
        cloud = cloud_from_xyz(merged).voxel_down_sample(voxel)
        return np.asarray(cloud.points)

    def _publish_result(self, kfs: list[Keyframe], poses: np.ndarray, loops: list[dict]) -> None:
        xyz = self._build_map(kfs, poses, self.map_voxel)
        stamp = self.get_clock().now().to_msg()
        header = Header()
        header.stamp = stamp
        header.frame_id = 'lio_odom'
        self.map_pub.publish(point_cloud2.create_cloud_xyz32(header, xyz))
        path = RosPath()
        path.header = header
        for k, T in zip(kfs, poses):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, T[:3, 3])
            x, y, z, w = quat_xyzw(T)
            ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = x, y, z, w
            path.poses.append(ps)
        self.path_pub.publish(path)
        status = {
            'keyframes': len(kfs), 'loops': len(loops),
            'corrected_points': int(len(xyz)),
            'start_end_xy': float(np.linalg.norm(poses[-1][:2,3]-poses[0][:2,3])),
            'start_end_z': float(poses[-1][2,3]-poses[0][2,3]),
        }
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))
        with self.lock:
            self.corrected_poses = poses.copy()

    def _save(self) -> tuple[bool, str]:
        with self.lock:
            kfs = list(self.keyframes)
            loops = list(self.loops)
            poses = self.corrected_poses.copy() if self.corrected_poses is not None else None
        if len(kfs) < 2:
            return False, 'no keyframes recorded'
        if self.try_start_end_on_save and not loops:
            try:
                loop = self._manual_start_end(kfs)
            except Exception as exc:
                self.get_logger().warn(f'start/end attempt failed: {exc}')
                loop = None
            if loop is not None:
                with self.lock:
                    self.loops.append(loop)
                    loops = list(self.loops)
        # New keyframes may have arrived after the last online optimization.
        # Re-optimize at save time so the saved map contains the full session.
        if loops:
            poses = self._optimize(kfs, loops)
            self._publish_result(kfs, poses, loops)
        else:
            poses = np.asarray([k.pose for k in kfs])
        xyz = self._build_map(kfs, poses, self.save_keyframe_voxel)
        self.output_map_path.parent.mkdir(parents=True, exist_ok=True)
        write_pcd(self.output_map_path, xyz)
        return True, f'saved {len(xyz)} corrected points to {self.output_map_path}'

    def _manual_start_end(self, kfs: list[Keyframe]) -> dict | None:
        i, j = len(kfs)-1, 0
        target = self._target_submap(kfs, j, 10)
        initial_raw = inv(kfs[j].pose) @ kfs[i].pose
        initial = yaw_only_initial(initial_raw)
        result = try_icp(kfs[i].down, target, initial, max(1.0, self.max_correspondence))
        self.get_logger().info(
            f'manual start/end ICP fitness={result.fitness:.3f} rmse={result.inlier_rmse:.3f}')
        if result.fitness < 0.35 or result.inlier_rmse > 0.55:
            return None
        delta = inv(initial_raw) @ result.transformation
        return {'source': i, 'target': j, 'transform': result.transformation.tolist(),
                'fitness': float(result.fitness), 'rmse': float(result.inlier_rmse),
                'correction_translation': float(np.linalg.norm(delta[:3,3])),
                'correction_yaw_rad': abs(relative_yaw(np.eye(4), delta)),
                'manual_start_end': True}

    def save_callback(self, request, response):
        with self.save_lock:
            ok, message = self._save()
        response.success = ok
        response.message = message
        self.get_logger().info(message)
        return response

    def _worker_loop(self) -> None:
        while self.running:
            self.work_event.wait(timeout=0.5)
            self.work_event.clear()
            with self.lock:
                i = self.pending_index
                self.pending_index = None
            if i is not None:
                try:
                    self._process_keyframe(i)
                except Exception as exc:
                    self.get_logger().warn(f'loop worker error: {exc}')

    def shutdown(self) -> None:
        self.running = False
        self.work_event.set()
        self.worker.join(timeout=2.0)


def main() -> int:
    rclpy.init()
    node = LiveLoopClosure()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
