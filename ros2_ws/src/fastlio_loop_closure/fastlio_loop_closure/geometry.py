"""Measured overlaps and gravity-preserving map refinement, independent of ROS."""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy.optimize import least_squares
from scipy.sparse import coo_matrix, lil_matrix
from scipy.sparse.linalg import lsmr
from scipy.spatial.transform import Rotation


@dataclass
class Keyframe:
    index: int
    stamp_ns: int
    pose: np.ndarray
    xyz: np.ndarray
    intensity: np.ndarray


def inverse(pose):
    out = np.eye(4)
    out[:3, :3] = pose[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ pose[:3, 3]
    return out


def level_rotation(up):
    up = np.asarray(up, dtype=float)
    if not np.isfinite(up).all() or np.linalg.norm(up) < 0.5:
        raise ValueError('invalid gravity reference')
    up = up / np.linalg.norm(up)
    if up[2] < 0.5:
        raise ValueError('initial IMU tilt exceeds the JD03 mapping profile')
    axis = np.cross(up, [0., 0., 1.])
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + skew + skew @ skew / (1. + up[2])


def voxel_indices(xyz, size):
    if size <= 0:
        raise ValueError('voxel size must be positive')
    return np.unique(np.floor(xyz / size).astype(np.int64), axis=0, return_index=True)[1]


def cloud(xyz, voxel=0.2):
    out = o3d.geometry.PointCloud()
    out.points = o3d.utility.Vector3dVector(np.asarray(xyz, dtype=float))
    return out.voxel_down_sample(voxel)


def structures(xyz):
    """Exclude the ground plane from overlap acceptance on this upright vehicle."""
    radius = np.linalg.norm(xyz[:, :2], axis=1)
    return xyz[(radius > 2.) & (radius < 35.) & (xyz[:, 2] > .1) & (xyz[:, 2] < 6.)]


def register_overlap(kfs, source, target, eligible_end, half_window=3):
    src = cloud(structures(kfs[source].xyz))
    center_inv = inverse(kfs[target].pose)
    parts = []
    for k in range(max(0, target-half_window), min(eligible_end, target+half_window+1)):
        transform = center_inv @ kfs[k].pose
        parts.append(kfs[k].xyz @ transform[:3, :3].T + transform[:3, 3])
    dst = cloud(structures(np.concatenate(parts)))
    if len(src.points) < 150 or len(dst.points) < 300:
        return None
    reg = o3d.pipelines.registration
    guess = center_inv @ kfs[source].pose
    coarse = reg.registration_icp(src, dst, 1., guess,
        reg.TransformationEstimationPointToPoint(), reg.ICPConvergenceCriteria(max_iteration=60))
    dst.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=1., max_nn=30))
    fine = reg.registration_icp(src, dst, .6, coarse.transformation,
        reg.TransformationEstimationPointToPlane(reg.HuberLoss(.15)),
        reg.ICPConvergenceCriteria(max_iteration=70))
    measured = np.array(fine.transformation)
    ev = reg.evaluate_registration(src, dst, .4, measured)
    fitted_pose = kfs[target].pose @ measured
    correction = fitted_pose @ inverse(kfs[source].pose)
    rpy = Rotation.from_matrix(correction[:3, :3].copy()).as_euler('xyz', degrees=True)
    # Measure displacement at the sensor, not at the map origin (lever-arm invariant).
    shift = fitted_pose[:3, 3] - kfs[source].pose[:3, 3]
    return {'source': source, 'target': target,
            'fitness': float(ev.fitness), 'rmse_m': float(ev.inlier_rmse),
            'position_correction_m': shift.tolist(), 'rotation_correction_deg': rpy.tolist(),
            'measurement_target_from_source': measured.tolist()}


def overlap_is_usable(match):
    if match is None:
        return False
    shift = np.asarray(match['position_correction_m'])
    angles = np.asarray(match['rotation_correction_deg'])
    measured = np.asarray(match['measurement_target_from_source'])
    return bool(np.isfinite(measured).all() and np.isfinite(shift).all()
                and np.isfinite(angles).all() and match['fitness'] >= .85
                and match['rmse_m'] <= .20 and np.linalg.norm(shift[:2]) < 2.
                and abs(shift[2]) < .4 and np.linalg.norm(angles[:2]) < 1.5
                and abs(angles[2]) < 1.)


def refine(poses, matches):
    """Optimize xyz/yaw only; fix the first pose and retain measured roll/pitch."""
    poses = np.asarray(poses, dtype=float)
    n = len(poses)
    if len(matches) < 6:
        raise ValueError('至少需要 6 处合格重叠匹配；请重复经过一段有建筑或立柱的路段')
    training = matches[::3]
    heldout = [m for k, m in enumerate(matches) if k % 3]
    angles = Rotation.from_matrix(poses[:, :3, :3]).as_euler('xyz')
    angles[:, 2] = np.unwrap(angles[:, 2])
    ii, jj = list(range(n-1)), list(range(1, n))
    measurements = [inverse(poses[i]) @ poses[i+1] for i in range(n-1)]
    sigmas = [[.03, .03, .02, math.radians(.05)] for _ in range(n-1)]
    for m in training:
        ii.append(m['target'])
        jj.append(m['source'])
        measurements.append(np.asarray(m['measurement_target_from_source']))
        sigmas.append([.12, .12, .12, math.radians(.5)])
    ii, jj, measurement, sigma = map(np.asarray, (ii, jj, measurements, sigmas))
    rotation, position = poses[:, :3, :3], poses[:, :3, 3]
    rt = rotation[ii].transpose(0, 2, 1)
    b = ((measurement[:, :3, 3] - np.einsum('nij,nj->ni', rt, position[jj]-position[ii]))
         / sigma[:, :3]).ravel()
    ar, ac, av = [], [], []
    for k, (i, j) in enumerate(zip(ii, jj)):
        block = rt[k] / sigma[k, :3, None]
        for node, sign in ((i, -1), (j, 1)):
            if node:
                for r in range(3):
                    for c in range(3):
                        ar.append(k*3+r)
                        ac.append((node-1)*3+c)
                        av.append(sign*block[r, c])
    A = coo_matrix((av, (ar, ac)), shape=(3*len(ii), 3*(n-1))).tocsr()
    shift = lsmr(A, b, atol=1e-10, btol=1e-10, maxiter=5000)[0].reshape(n-1, 3)
    x0 = np.column_stack((position[1:]+shift, angles[1:, 2])).ravel()

    def unpack(x):
        values = x.reshape(n-1, 4)
        p = np.vstack((position[0], values[:, :3]))
        e = angles.copy()
        e[1:, 2] = values[:, 3]
        return p, Rotation.from_euler('xyz', e).as_matrix()

    def residual(x):
        p, R = unpack(x)
        rit = R[ii].transpose(0, 2, 1)
        ep = np.einsum('nij,nj->ni', rit, p[jj]-p[ii]) - measurement[:, :3, 3]
        er = Rotation.from_matrix(measurement[:, :3, :3].transpose(0, 2, 1) @ (rit @ R[jj])).as_rotvec()
        body_up = R[jj].transpose(0, 2, 1) @ np.array([0., 0., 1.])
        ey = np.sum(er * body_up, axis=1)
        return (np.column_stack((ep, ey)) / sigma).ravel()

    sparsity = lil_matrix((len(ii)*4, (n-1)*4), dtype=np.int8)
    for k, (i, j) in enumerate(zip(ii, jj)):
        for node in (i, j):
            if node:
                sparsity[k*4:(k+1)*4, (node-1)*4:node*4] = 1
    fit = least_squares(residual, x0, jac_sparsity=sparsity.tocsr(), x_scale='jac',
        loss='soft_l1', f_scale=2., ftol=1e-8, xtol=1e-7, gtol=1e-7,
        max_nfev=70, tr_options={'maxiter': 1500, 'atol': 1e-9, 'btol': 1e-9})
    p, R = unpack(fit.x)
    result = np.tile(np.eye(4), (n, 1, 1))
    result[:, :3, :3], result[:, :3, 3] = R, p
    validation = []
    for m in heldout:
        predicted = inverse(result[m['target']]) @ result[m['source']]
        error = inverse(np.asarray(m['measurement_target_from_source'])) @ predicted
        validation.append({'source': m['source'], 'target': m['target'],
            'translation_error_m': float(np.linalg.norm(error[:3, 3])),
            'yaw_error_deg': float(Rotation.from_matrix(error[:3, :3].copy()).as_euler('xyz', degrees=True)[2])})
    max_shift = float(np.linalg.norm(p-position, axis=1).max())
    max_angle = float(np.rad2deg(np.linalg.norm(Rotation.from_matrix(
        rotation.transpose(0, 2, 1) @ R).as_rotvec(), axis=1)).max())
    max_error = max(v['translation_error_m'] for v in validation)
    max_yaw = max(abs(v['yaw_error_deg']) for v in validation)
    passed = bool(fit.success and np.isfinite(result).all() and max_shift < 2.
                  and max_angle < 1. and max_error < .4 and max_yaw < .6)
    report = {'geometry_checks_passed': passed, 'solver_success': bool(fit.success),
        'solver_message': fit.message, 'training_loops': training, 'heldout_validation': validation,
        'max_pose_translation_change_m': max_shift, 'max_pose_rotation_change_deg': max_angle,
        'max_heldout_translation_error_m': max_error, 'max_heldout_yaw_error_deg': max_yaw,
        'keyframe_height_range_m': [float(p[:, 2].min()), float(p[:, 2].max())],
        'endpoint_xyz_m': p[-1].tolist(), 'endpoint_is_not_constrained_to_origin': True,
        'roll_pitch_preserved': True,
        'validation_limits': 'Internal geometric consistency; shared structures, no survey ground truth or vehicle acceptance.'}
    return result, report


def build_map(kfs, poses, voxel=.1):
    # Reduce each chunk before the final union to bound peak memory on long routes.
    chunks, batch = [], []
    for k, T in zip(kfs, poses):
        world = k.xyz @ T[:3, :3].T + T[:3, 3]
        batch.append(np.column_stack((world, k.intensity)).astype(np.float32))
        if len(batch) >= 30:
            merged = np.concatenate(batch)
            chunks.append(merged[voxel_indices(merged[:, :3], voxel)])
            batch.clear()
    if batch:
        merged = np.concatenate(batch)
        chunks.append(merged[voxel_indices(merged[:, :3], voxel)])
    merged = np.concatenate(chunks)
    merged = merged[np.isfinite(merged).all(axis=1)]
    return np.ascontiguousarray(merged[voxel_indices(merged[:, :3], voxel)], dtype='<f4')


def atomic_json(path, value):
    path = Path(path)
    tmp = path.with_name(path.name+'.tmp')
    tmp.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    os.replace(tmp, path)


def write_pcd(path, points):
    path = Path(path)
    points = np.ascontiguousarray(points, dtype='<f4')
    if points.ndim != 2 or points.shape[1] != 4 or not len(points) or not np.isfinite(points).all():
        raise ValueError('invalid XYZI map')
    path.parent.mkdir(parents=True, exist_ok=True)
    header = ('# PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n'
        'TYPE F F F F\nCOUNT 1 1 1 1\n'
        f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n')
    tmp = path.with_name(path.name+'.tmp')
    try:
        with tmp.open('wb') as stream:
            stream.write(header.encode('ascii'))
            stream.write(points.tobytes())
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(tmp, path)
    finally:
        tmp.unlink(missing_ok=True)
