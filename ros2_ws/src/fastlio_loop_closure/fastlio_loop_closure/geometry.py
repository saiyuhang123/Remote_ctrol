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


def ground_surface(xyz):
    """Ground support, excluded from structural ICP, for seeding/verification."""
    radius = np.linalg.norm(xyz[:, :2], axis=1)
    points = xyz[(radius > 2.5) & (radius < 12.) & (xyz[:, 2] > -1.2) & (xyz[:, 2] < .1)]
    if len(points) < 150:
        return None
    ground = cloud(points, .15)
    if len(ground.points) < 100:
        return None
    plane, indices = ground.segment_plane(.07, 3, 200)
    plane = np.asarray(plane)
    if plane[2] < 0:
        plane *= -1
    if plane[2] < math.cos(math.radians(10.)) or len(indices) < 100 or len(indices) < .5*len(ground.points):
        return None
    return plane, np.asarray(ground.points)[indices].mean(axis=0)


def register_overlap(kfs, source, target, eligible_end, half_window=3):
    src = cloud(structures(kfs[source].xyz))
    center_inv = inverse(kfs[target].pose)
    parts = []
    for k in range(max(0, target-half_window), min(eligible_end, target+half_window+1)):
        transform = center_inv @ kfs[k].pose
        parts.append(kfs[k].xyz @ transform[:3, :3].T + transform[:3, 3])
    target_xyz = np.concatenate(parts)
    dst = cloud(structures(target_xyz))
    if len(src.points) < 150 or len(dst.points) < 300:
        return None
    reg = o3d.pipelines.registration
    guess = center_inv @ kfs[source].pose
    dst.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=1., max_nn=30))
    source_ground = ground_surface(kfs[source].xyz)
    target_ground = ground_surface(target_xyz)
    seeds = [(guess, 1.)]
    if source_ground is not None and target_ground is not None:
        normal, offset = target_ground[0][:3], target_ground[0][3]
        centroid = source_ground[1]
        for dx, dy in ((0., 0.), (-2.5, 0.), (2.5, 0.), (0., -2.5), (0., 2.5)):
            seed = guess.copy()
            seed[:2, 3] += [dx, dy]
            dz = -(normal @ (seed[:3, :3] @ centroid + seed[:3, 3])+offset)/normal[2]
            if abs(dz) <= 2.5:
                seed[2, 3] += dz
                seeds.append((seed, 2.5))
    solutions = []
    for seed, correspondence in seeds:
        coarse = reg.registration_icp(src, dst, correspondence, seed,
            reg.TransformationEstimationPointToPoint(), reg.ICPConvergenceCriteria(max_iteration=60))
        fine = reg.registration_icp(src, dst, .6, coarse.transformation,
            reg.TransformationEstimationPointToPlane(reg.HuberLoss(.15)),
            reg.ICPConvergenceCriteria(max_iteration=70))
        measured = np.array(fine.transformation)
        ev = reg.evaluate_registration(src, dst, .4, measured)
        solutions.append((ev.fitness-.1*ev.inlier_rmse, measured, ev))
    _, measured, ev = max(solutions, key=lambda item: item[0])
    fitted_pose = kfs[target].pose @ measured
    correction = fitted_pose @ inverse(kfs[source].pose)
    rpy = Rotation.from_matrix(correction[:3, :3].copy()).as_euler('xyz', degrees=True)
    # Measure displacement at the sensor, not at the map origin (lever-arm invariant).
    shift = fitted_pose[:3, 3] - kfs[source].pose[:3, 3]
    agreements = int(sum(np.linalg.norm(other[:3, 3]-measured[:3, 3]) < .15
        and Rotation.from_matrix(other[:3, :3].T @ measured[:3, :3]).magnitude() < math.radians(.4)
        for _, other, _ in solutions))
    ground_residual = None
    if source_ground is not None and target_ground is not None:
        ground_residual = abs(float(target_ground[0][:3] @
            (measured[:3, :3] @ source_ground[1]+measured[:3, 3])+target_ground[0][3]))
    reciprocal_translation = reciprocal_rotation = None
    if ev.fitness >= .65 and ev.inlier_rmse <= .22:
        src.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=1., max_nn=30))
        backward = reg.registration_icp(dst, src, .6, inverse(measured),
            reg.TransformationEstimationPointToPlane(reg.HuberLoss(.15)),
            reg.ICPConvergenceCriteria(max_iteration=60))
        closure = measured @ backward.transformation
        reciprocal_translation = float(np.linalg.norm(closure[:3, 3]))
        reciprocal_rotation = float(np.rad2deg(Rotation.from_matrix(closure[:3, :3].copy()).magnitude()))
    return {'source': source, 'target': target,
            'fitness': float(ev.fitness), 'rmse_m': float(ev.inlier_rmse),
            'position_correction_m': shift.tolist(), 'rotation_correction_deg': rpy.tolist(),
            'inlier_count': len(ev.correspondence_set), 'initialization_agreements': agreements,
            'ground_residual_m': ground_residual, 'reciprocal_translation_m': reciprocal_translation,
            'reciprocal_rotation_deg': reciprocal_rotation,
            'measurement_target_from_source': measured.tolist()}


def registration_checks_pass(match):
    values = [match.get('ground_residual_m'), match.get('reciprocal_translation_m'),
              match.get('reciprocal_rotation_deg')]
    return bool(all(v is not None and math.isfinite(v) for v in values)
                and values[0] < .10 and values[1] < .15 and values[2] < .5
                and match.get('inlier_count', 0) >= 500
                and match.get('initialization_agreements', 0) >= 2)


def overlap_is_usable(match, z_bias=0.0):
    """Judge one loop match.

    ``z_bias`` is a common vertical offset shared by the loop's matches. A
    frontend height drift shifts every match by the same dz; the gate must then
    look at the residual ``dz - z_bias``, otherwise a perfectly good loop is
    rejected as a whole. The absolute dz is still capped so a bogus match cannot
    ride an accepted bias arbitrarily far.
    """
    if match is None:
        return False
    shift = np.asarray(match['position_correction_m'])
    angles = np.asarray(match['rotation_correction_deg'])
    measured = np.asarray(match['measurement_target_from_source'])
    if not (np.isfinite(measured).all() and np.isfinite(shift).all() and np.isfinite(angles).all()
            and np.linalg.norm(angles[:2]) < 1.5 and abs(angles[2]) < 1.):
        return False
    if abs(shift[2]) > 10.:
        return False
    vertical = abs(float(shift[2]) - float(z_bias))
    strong = match['fitness'] >= .85 and match['rmse_m'] <= .20
    small = np.linalg.norm(shift[:2]) < 2. and vertical < .4
    if strong and small:
        return True
    return bool(match['fitness'] >= .65 and match['rmse_m'] <= .22
                and np.linalg.norm(shift[:2]) < 3. and vertical < 2.
                and registration_checks_pass(match))


def common_vertical_bias(matches, window=8, spread=.30, min_matches=3, max_abs=10.):
    """Return the dz shared by a run of matches, or None if they do not agree.

    A false loop closure produces a random dz; a real frontend height drift
    produces the same dz on every match. Requiring the recent matches to agree
    within ``spread`` is what makes accepting a large |dz| safe.
    """
    if matches is None or len(matches) < min_matches:
        return None
    recent = list(matches)[-window:]
    if len(recent) < min_matches:
        return None
    dz = np.array([float(np.asarray(m.get('position_correction_m', [0., 0., 0.]))[2])
                   for m in recent])
    if not np.isfinite(dz).all() or float(np.ptp(dz)) > spread:
        return None
    bias = float(np.median(dz))
    return None if abs(bias) > max_abs else bias


def refine(poses, matches):
    """Optimize xyz/yaw only; fix the first pose and retain measured roll/pitch."""
    poses = np.asarray(poses, dtype=float)
    n = len(poses)
    if len(matches) < 6:
        raise ValueError('至少需要 6 处合格重叠匹配；请重复经过一段有建筑或立柱的路段')
    partial = any(m.get('fitness', 1.) < .85 for m in matches)
    source_positions = poses[[m['source'] for m in matches], :3, 3]
    source_span = float(np.linalg.norm(np.ptp(source_positions, axis=0)))
    if partial and source_span < 6.:
        raise ValueError(f'部分重叠仅覆盖 {source_span:.1f} 米，至少需要 6 米的不同位置观测')
    extended = any(np.linalg.norm(m.get('position_correction_m', [0., 0., 0.])) >= 2.
                   for m in matches)
    # A common vertical offset is a real frontend height drift, not a bad match:
    # the correction is allowed to scale with it (the held-out checks below still
    # reject a correction that does not actually close the loop).
    common_z_bias = float(np.median([float(np.asarray(m.get('position_correction_m', [0., 0., 0.]))[2])
                                     for m in matches]))
    base_limit = 3.5 if extended and all(registration_checks_pass(m) for m in matches) else 2.
    shift_limit = max(base_limit, 1.2*abs(common_z_bias))
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
    passed = bool(fit.success and np.isfinite(result).all() and max_shift < shift_limit
                  and max_angle < 1. and max_error < .4 and max_yaw < .6)
    report = {'geometry_checks_passed': passed, 'solver_success': bool(fit.success),
        'solver_message': fit.message, 'training_loops': training, 'heldout_validation': validation,
        'max_pose_translation_change_m': max_shift, 'max_pose_rotation_change_deg': max_angle,
        'pose_translation_limit_m': shift_limit, 'overlap_source_span_m': source_span,
        'common_z_bias_m': common_z_bias,
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
