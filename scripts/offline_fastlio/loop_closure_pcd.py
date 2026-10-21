#!/usr/bin/env python3
"""Offline loop-closure post-processing for FAST-LIO keyframe recordings.

Input is the directory written by record_keyframes.py:
  keyframes/kf_*.npz  (xyz, intensity, stamp_ns, pose)
  keyframes/keyframes.jsonl

Pipeline:
  1. Build a local submap around spatially nearby historical keyframes.
  2. Run ICP with the FAST-LIO relative pose as the initial guess.
  3. Add accepted loop edges and optimize the Open3D pose graph.
  4. Rebuild a corrected PCD from the keyframe clouds.

This is deliberately an offline tool. It does not alter the live FAST-LIO node.
Note: Open3D pose graph edges use the transform from the target node to the
source node, i.e. inv(P_target) @ P_source for the poses used here.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Iterable

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree


def load_keyframes(directory: Path):
    manifest = directory / 'keyframes.jsonl'
    if not manifest.is_file():
        raise FileNotFoundError(manifest)
    rows = []
    with manifest.open(encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            item = json.loads(line)
            item['path'] = directory / item['file']
            item['pose_np'] = np.asarray(item['pose'], dtype=np.float64)
            if item['path'].is_file():
                rows.append(item)
    if len(rows) < 3:
        raise RuntimeError('need at least 3 keyframes')
    rows.sort(key=lambda x: x['stamp_ns'])
    for i, item in enumerate(rows):
        item['index'] = i
    return rows


def inv(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    result = np.eye(4)
    result[:3, :3] = R.T
    result[:3, 3] = -R.T @ T[:3, 3]
    return result


def yaw_of(T: np.ndarray) -> float:
    return math.atan2(T[1, 0], T[0, 0])


def relative_yaw(a: np.ndarray, b: np.ndarray) -> float:
    return math.atan2(math.sin(yaw_of(b)-yaw_of(a)), math.cos(yaw_of(b)-yaw_of(a)))


def yaw_only_initial(T: np.ndarray, reset_z: bool = True) -> np.ndarray:
    """Initial guess for a revisit, discarding accumulated roll/pitch and z."""
    yaw = yaw_of(T)
    c, s = math.cos(yaw), math.sin(yaw)
    out = np.eye(4)
    out[:3, :3] = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
    out[:2, 3] = T[:2, 3]
    out[2, 3] = 0.0 if reset_z else T[2, 3]
    return out


def load_xyz(item) -> np.ndarray:
    with np.load(item['path']) as data:
        xyz = np.asarray(data['xyz'], dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError(f'bad xyz in {item["path"]}')
    return xyz[np.isfinite(xyz).all(axis=1)]


def cloud_from_xyz(xyz: np.ndarray) -> o3d.geometry.PointCloud:
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(xyz)
    return cloud


class CloudCache:
    """Lazy, downsampled keyframe clouds for ICP."""
    def __init__(self, rows, voxel: float):
        self.rows = rows
        self.voxel = float(voxel)
        self.cache: dict[int, o3d.geometry.PointCloud] = {}

    def get(self, index: int) -> o3d.geometry.PointCloud:
        if index not in self.cache:
            xyz = load_xyz(self.rows[index])
            cloud = cloud_from_xyz(xyz).voxel_down_sample(self.voxel)
            if len(cloud.points) < 50:
                raise RuntimeError(f'keyframe {index} is too sparse after downsample')
            self.cache[index] = cloud
        return self.cache[index]


def make_target_submap(rows, cache: CloudCache, center: int,
                       half_window: int, voxel: float) -> o3d.geometry.PointCloud:
    points = []
    lo = max(0, center - half_window)
    hi = min(len(rows), center + half_window + 1)
    center_inv = inv(rows[center]['pose_np'])
    for k in range(lo, hi):
        xyz = load_xyz(rows[k])
        T = center_inv @ rows[k]['pose_np']
        points.append((xyz @ T[:3, :3].T) + T[:3, 3])
    merged = np.concatenate(points, axis=0)
    target = cloud_from_xyz(merged).voxel_down_sample(voxel)
    return target


def try_icp(source: o3d.geometry.PointCloud, target: o3d.geometry.PointCloud,
            initial: np.ndarray, max_correspondence: float):
    target.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
        radius=max(0.5, max_correspondence * 2.0), max_nn=30))
    # Point-to-plane generally converges better on wall/ground scans. Fall back
    # to point-to-point if the Open3D build rejects the normals.
    try:
        return o3d.pipelines.registration.registration_icp(
            source, target, max_correspondence, initial,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=60))
    except Exception:
        target.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
            radius=max(0.5, max_correspondence * 2.0), max_nn=30))
        return o3d.pipelines.registration.registration_icp(
            source, target, max_correspondence, initial,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=60))


def find_loops(rows, cache: CloudCache, search_radius: float,
               min_time_diff: float, min_index_diff: int,
               half_window: int, target_voxel: float,
               max_correspondence: float, min_fitness: float,
               max_rmse: float, max_loops: int,
               max_candidates_per_key: int, no_level_initial: bool = False,
               keep_initial_z: bool = False):
    poses = np.asarray([r['pose_np'] for r in rows])
    positions = poses[:, :3, 3]
    stamps = np.asarray([r['stamp_ns'] for r in rows], dtype=np.int64)
    tree = cKDTree(positions[:, :2])
    loops = []
    for i in range(min_index_diff, len(rows)):
        ids = tree.query_ball_point(positions[i, :2], search_radius)
        candidates = []
        for j in ids:
            if j >= i - min_index_diff:
                continue
            if (stamps[i] - stamps[j]) * 1e-9 < min_time_diff:
                continue
            xy_dist = float(np.linalg.norm(positions[i, :2] - positions[j, :2]))
            candidates.append((xy_dist, int(j)))
        candidates.sort()
        if not candidates:
            continue
        source = cache.get(i)
        best = None
        for _, j in candidates[:max_candidates_per_key]:
            target = make_target_submap(rows, cache, j, half_window, target_voxel)
            if len(target.points) < 200:
                continue
            initial_raw = inv(poses[j]) @ poses[i]
            inits = [initial_raw]
            if not no_level_initial:
                # A long outdoor loop often accumulates a large z/roll/pitch
                # error. Start ICP at the historical height and yaw-only
                # orientation; this lets a true loop candidate converge even
                # when FAST-LIO's current z is many metres away.
                level = yaw_only_initial(initial_raw,
                                         reset_z=not keep_initial_z)
                if not np.allclose(level, initial_raw):
                    inits.append(level)
            best_result = None
            best_init = None
            for initial in inits:
                try:
                    result = try_icp(source, target, initial, max_correspondence)
                except Exception:
                    continue
                if result.fitness < min_fitness or result.inlier_rmse > max_rmse:
                    continue
                delta = inv(initial) @ result.transformation
                dt = float(np.linalg.norm(delta[:3, 3]))
                da = abs(relative_yaw(np.eye(4), delta))
                # A loop correction should be a moderate local ICP correction,
                # not an arbitrary scan match to a different corridor.
                if dt > 5.0 or da > math.radians(30.0):
                    continue
                score = result.fitness - 0.1 * result.inlier_rmse
                if best_result is None or score > best_result[0]:
                    best_result = (score, result)
                    best_init = initial
            if best_result is None:
                continue
            _, result = best_result
            delta_original = inv(initial_raw) @ result.transformation
            dt = float(np.linalg.norm(delta_original[:3, 3]))
            da = abs(relative_yaw(np.eye(4), delta_original))
            score, result = best_result
            if best is None or score > best[0]:
                best = (score, j, result.transformation,
                        float(result.fitness), float(result.inlier_rmse),
                        xy_dist, dt, da)
        if best is not None:
            _, j, T_loop, fitness, rmse, xy_dist, dt, da = best
            loops.append({
                'source': int(i), 'target': int(j),
                'transform': T_loop.tolist(),
                'fitness': fitness, 'rmse': rmse,
                'xy_distance_initial': xy_dist,
                'correction_translation': dt,
                'correction_yaw_rad': da,
                'stamp_source': int(stamps[i]), 'stamp_target': int(stamps[j]),
            })
            print(f'loop {j:4d}->{i:4d} fitness={fitness:.3f} '
                  f'rmse={rmse:.3f} xy0={xy_dist:.2f}m '
                  f'corr={dt:.2f}m/{math.degrees(da):.1f}deg')
            if len(loops) >= max_loops:
                break
    return loops


def manual_start_end_loop(rows, cache: CloudCache, submap_half: int,
                          target_voxel: float, max_correspondence: float,
                          min_fitness: float, max_rmse: float):
    """Force one start/end 6DOF candidate for a bag that ends near its start.

    This is intentionally explicit: it is useful for diagnosing Z/roll/pitch
    drift when automatic loop search rejects a partially overlapping endpoint.
    It does not force XY to zero; the ICP result and the remaining endpoint
    residual are written to diagnostics for inspection.
    """
    if len(rows) < 8:
        return []
    i = len(rows) - 1
    j = 0
    source = cache.get(i)
    target = make_target_submap(rows, cache, j, submap_half, target_voxel)
    if len(target.points) < 200:
        return []
    initial_raw = inv(rows[j]['pose_np']) @ rows[i]['pose_np']
    initial = yaw_only_initial(initial_raw, reset_z=True)
    try:
        result = try_icp(source, target, initial, max_correspondence)
    except Exception as exc:
        print(f'manual start/end ICP failed: {exc}')
        return []
    print(f'manual end->start ICP fitness={result.fitness:.3f} '
          f'rmse={result.inlier_rmse:.3f}')
    if result.fitness < min_fitness or result.inlier_rmse > max_rmse:
        print('manual start/end candidate rejected by thresholds')
        return []
    delta = inv(initial_raw) @ result.transformation
    dt = float(np.linalg.norm(delta[:3, 3]))
    da = abs(relative_yaw(np.eye(4), delta))
    return [{
        'source': int(i), 'target': int(j),
        'transform': result.transformation.tolist(),
        'fitness': float(result.fitness),
        'rmse': float(result.inlier_rmse),
        'xy_distance_initial': float(np.linalg.norm(
            rows[i]['pose_np'][:2, 3] - rows[j]['pose_np'][:2, 3])),
        'correction_translation': dt,
        'correction_yaw_rad': da,
        'stamp_source': int(rows[i]['stamp_ns']),
        'stamp_target': int(rows[j]['stamp_ns']),
        'manual_start_end': True,
    }]


def optimized_poses(rows, loops):
    pg = o3d.pipelines.registration.PoseGraph()
    for row in rows:
        pg.nodes.append(o3d.pipelines.registration.PoseGraphNode(row['pose_np']))
    # Open3D's edge transform is target <- source: inv(P_target) @ P_source.
    odom_info = np.diag([100., 100., 40., 100., 100., 40.])
    loop_info = np.diag([20., 20., 8., 20., 20., 8.])
    for i in range(len(rows) - 1):
        # source i, target i+1
        T_edge = inv(rows[i+1]['pose_np']) @ rows[i]['pose_np']
        pg.edges.append(o3d.pipelines.registration.PoseGraphEdge(
            i, i + 1, T_edge, odom_info, uncertain=False))
    for loop in loops:
        pg.edges.append(o3d.pipelines.registration.PoseGraphEdge(
            loop['source'], loop['target'],
            np.asarray(loop['transform'], dtype=np.float64),
            loop_info, uncertain=True))
    method = o3d.pipelines.registration.GlobalOptimizationGaussNewton()
    criteria = o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria()
    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=0.35,
        edge_prune_threshold=0.25,
        reference_node=0)
    o3d.pipelines.registration.global_optimization(pg, method, criteria, option)
    return [np.asarray(n.pose, dtype=np.float64) for n in pg.nodes]


def write_binary_pcd(path: Path, xyz: np.ndarray, intensity: np.ndarray | None = None):
    """Write x y z intensity normals curvature, compatible with PCL PointXYZINormal."""
    n = len(xyz)
    out = np.zeros((n, 8), dtype=np.float32)
    out[:, :3] = xyz.astype(np.float32, copy=False)
    if intensity is not None:
        out[:, 3] = intensity.astype(np.float32, copy=False)
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\n'
        'VERSION 0.7\n'
        'FIELDS x y z intensity normal_x normal_y normal_z curvature\n'
        'SIZE 4 4 4 4 4 4 4 4\n'
        'TYPE F F F F F F F F\n'
        'COUNT 1 1 1 1 1 1 1 1\n'
        f'WIDTH {n}\n'
        'HEIGHT 1\n'
        'VIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {n}\n'
        'DATA binary\n'
    )
    with path.open('wb') as f:
        f.write(header.encode('ascii'))
        f.write(out.tobytes(order='C'))


def main() -> int:
    ap = argparse.ArgumentParser(description='Offline FAST-LIO loop closure and PCD rebuild')
    ap.add_argument('--keyframes', required=True,
                    help='directory containing keyframes.jsonl and kf_*.npz')
    ap.add_argument('--output-dir', default='',
                    help='default: <keyframes>/loop_closure')
    ap.add_argument('--search-radius', type=float, default=8.0)
    ap.add_argument('--min-time-diff', type=float, default=60.0)
    ap.add_argument('--min-index-diff', type=int, default=30)
    ap.add_argument('--submap-half-window', type=int, default=5)
    ap.add_argument('--icp-voxel', type=float, default=0.25)
    ap.add_argument('--target-voxel', type=float, default=0.20)
    ap.add_argument('--map-voxel', type=float, default=0.10)
    ap.add_argument('--max-correspondence', type=float, default=1.0)
    ap.add_argument('--min-fitness', type=float, default=0.30)
    ap.add_argument('--max-rmse', type=float, default=0.30)
    ap.add_argument('--max-loops', type=int, default=20)
    ap.add_argument('--max-candidates-per-key', type=int, default=3)
    ap.add_argument('--manual-start-end', action='store_true',
                    help='force one last-keyframe -> first-keyframe 6DOF candidate')
    ap.add_argument('--manual-submap-half', type=int, default=10)
    ap.add_argument('--manual-min-fitness', type=float, default=0.35)
    ap.add_argument('--manual-max-rmse', type=float, default=0.55)
    ap.add_argument('--no-level-initial', action='store_true',
                    help='use full FAST-LIO relative orientation for ICP initial guess')
    ap.add_argument('--keep-initial-z', action='store_true',
                    help='do not reset the loop initial z to the historical keyframe height')
    args = ap.parse_args()

    source_dir = Path(args.keyframes).expanduser().resolve()
    out = Path(args.output_dir).expanduser().resolve() if args.output_dir else source_dir / 'loop_closure'
    out.mkdir(parents=True, exist_ok=True)
    rows = load_keyframes(source_dir)
    print(f'loaded {len(rows)} keyframes from {source_dir}')
    cache = CloudCache(rows, args.icp_voxel)
    if args.manual_start_end:
        loops = manual_start_end_loop(
            rows, cache, args.manual_submap_half, args.target_voxel,
            args.max_correspondence, args.manual_min_fitness,
            args.manual_max_rmse)
    else:
        loops = find_loops(rows, cache, args.search_radius, args.min_time_diff,
                           args.min_index_diff, args.submap_half_window,
                           args.target_voxel, args.max_correspondence,
                           args.min_fitness, args.max_rmse, args.max_loops,
                           args.max_candidates_per_key, args.no_level_initial,
                           args.keep_initial_z)
    (out / 'loop_closures.json').write_text(
        json.dumps(loops, ensure_ascii=False, indent=2), encoding='utf-8')
    print(f'accepted loops: {len(loops)}')
    if not loops:
        print('no loop accepted; no pose graph optimization performed')
        return 0

    corrected = optimized_poses(rows, loops)
    start_pose = corrected[0]
    end_pose = corrected[-1]
    residual = inv(start_pose) @ end_pose
    diagnostics = {
        'method': 'manual_start_end' if args.manual_start_end else 'automatic',
        'accepted_loops': len(loops),
        'start_pose': start_pose.tolist(),
        'end_pose': end_pose.tolist(),
        'start_end_residual': {
            'translation_xyz': residual[:3, 3].tolist(),
            'translation_norm_xy': float(np.linalg.norm(residual[:2, 3])),
            'translation_z': float(residual[2, 3]),
            'yaw_rad': float(yaw_of(residual)),
        },
        'raw_start_end_distance_xy': float(np.linalg.norm(
            rows[-1]['pose_np'][:2, 3] - rows[0]['pose_np'][:2, 3])),
        'raw_start_end_z': float(rows[-1]['pose_np'][2, 3] -
                                 rows[0]['pose_np'][2, 3]),
    }
    (out / 'diagnostics.json').write_text(
        json.dumps(diagnostics, ensure_ascii=False, indent=2), encoding='utf-8')
    print('start/end residual after optimization:', diagnostics['start_end_residual'])
    with (out / 'trajectory_optimized.tum').open('w', encoding='utf-8') as f:
        for row, T in zip(rows, corrected):
            x, y, z = T[:3, 3]
            yaw = yaw_of(T)
            f.write(f'{row["stamp_ns"]*1e-9:.9f} {x:.9f} {y:.9f} {z:.9f} '
                    f'0 0 {math.sin(yaw/2):.9f} {math.cos(yaw/2):.9f}\n')

    # Rebuild global map from corrected keyframe poses.
    all_xyz = []
    for row, T in zip(rows, corrected):
        xyz = load_xyz(row)
        all_xyz.append((xyz @ T[:3, :3].T) + T[:3, 3])
    merged = np.concatenate(all_xyz, axis=0)
    print(f'merged points: {len(merged)}; voxel downsampling {args.map_voxel}m')
    down = cloud_from_xyz(merged).voxel_down_sample(args.map_voxel)
    xyz_out = np.asarray(down.points, dtype=np.float64)
    pcd = out / 'loop_closed_map.pcd'
    write_binary_pcd(pcd, xyz_out)
    print(f'wrote {pcd} ({len(xyz_out)} points)')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
