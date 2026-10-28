#!/usr/bin/env python3
"""Calibrate the lidar-IMU mounting roll/pitch from static recordings.

Why: the mapping frontend assumes the lidar and the IMU are parallel
(``base_from_imu.rpy`` only carries the -90 deg yaw). A real mounting pitch of
even 0.3 deg makes the lidar ground look tilted in the body frame; the gravity
aid keeps the attitude on the IMU, so the map z drops by ``tan(pitch)`` per
metre travelled - a fixed height bias per loop (0.35 deg over 758 m = 4.6 m).

Method: stop several times with different headings on one flat patch (an
Ackermann car cannot turn in place: drive a small loop or a U-turn, then stop
facing A, B, ... within a few metres of each other). For each still segment fit
the ground plane in the lidar frame and compare it with the IMU gravity
direction (both mapped through the current extrinsic). The discrepancy is
``mounting tilt + ground slope``; the slope rotates with the heading while the
mounting tilt does not, so a least-squares fit over the segments separates them.
Yaw is not observable from gravity and stays as configured.

Usage:
    # on the robot, sensors running, on a flat open area:
    ros2 bag record -o /tmp/calib /velodyne_points /imu/data
    #   drive a small loop / U-turn and stop 2-4 times, different headings,
    #   each stop ~10 s, all within a few metres on the same flat patch; Ctrl-C
    python3 calibrate_lidar_imu.py /tmp/calib --hardware ~/Remote_ctrol/ros2_ws/src/chassis_nav2/config/hardware.yaml

The bag path is read-only; the script prints the suggested rpy, it does not
edit hardware.yaml.
"""
import argparse
import glob
import math
import sqlite3
import struct
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

# ---------------------------------------------------------------- bag reading


class CDR:
    def __init__(self, data):
        self.d = data
        self.o = 4

    def align(self, n):
        self.o += (-(self.o - 4)) % n

    def u8(self):
        v = self.d[self.o]
        self.o += 1
        return v

    def u32(self):
        self.align(4)
        v = struct.unpack_from('<I', self.d, self.o)[0]
        self.o += 4
        return v

    def i32(self):
        self.align(4)
        v = struct.unpack_from('<i', self.d, self.o)[0]
        self.o += 4
        return v

    def f64(self):
        self.align(8)
        v = struct.unpack_from('<d', self.d, self.o)[0]
        self.o += 8
        return v

    def string(self):
        n = self.u32()
        s = self.d[self.o:self.o + n - 1].decode('utf-8', 'replace')
        self.o += n
        return s


def parse_header(c):
    sec = c.i32()
    nsec = c.u32()
    return sec + nsec * 1e-9, c.string()


def parse_imu(d):
    c = CDR(d)
    stamp, frame = parse_header(c)
    c.align(8)
    q = [c.f64() for _ in range(4)]          # orientation x y z w
    c.o += 9 * 8                              # orientation covariance
    g = [c.f64() for _ in range(3)]
    c.o += 9 * 8
    a = [c.f64() for _ in range(3)]
    return stamp, q, np.array(g), np.array(a)


def parse_cloud(d):
    c = CDR(d)
    stamp, frame = parse_header(c)
    h = c.u32()
    w = c.u32()
    nf = c.u32()
    fields = {}
    for _ in range(nf):
        name = c.string()
        off = c.u32()
        dt = c.u8()
        cnt = c.u32()
        fields[name] = (off, dt, cnt)
    c.u8()
    ps = c.u32()
    c.u32()
    n = c.u32()
    buf = np.frombuffer(c.d[c.o:c.o + n], dtype=np.uint8).reshape(-1, ps)
    xyz = np.stack([buf[:, fields[k][0]:fields[k][0] + 4].copy().view('<f4').ravel()
                    for k in ('x', 'y', 'z')], 1)
    return stamp, frame, xyz


def read_bag(path):
    imu, clouds = [], []
    for db in sorted(glob.glob(str(path / '*.db3'))):
        con = sqlite3.connect('file:%s?mode=ro' % db, uri=True)
        topics = {name: tid for tid, name in con.execute('select id,name from topics')}
        for name, target in (('/imu/data', imu), ('/velodyne_points', clouds)):
            tid = topics.get(name)
            if tid is None:
                continue
            for _, data in con.execute(
                    'select timestamp,data from messages where topic_id=? order by timestamp', (tid,)):
                try:
                    if name == '/imu/data':
                        st, q, g, a = parse_imu(data)
                        target.append((st, np.array(q), g, a))
                    else:
                        st, fr, xyz = parse_cloud(data)
                        target.append((st, fr, xyz))
                except Exception:
                    continue
    imu.sort(key=lambda r: r[0])
    clouds.sort(key=lambda r: r[0])
    return imu, clouds


# ---------------------------------------------------------------- calibration


def ground_normal(xyz, r0=2.5, r1=9., threshold=.03, iterations=150):
    """Robust ground plane normal in the lidar frame.

    A plain least-squares fit over a 3-14 m band is biased by curbs, grass,
    parked cars and the lot's crown: on the .63 pad it reported a 0.23 m RMS
    "plane" and normals that swung 1.5 deg between stops. RANSAC on the near
    ground (2.5-9 m, where the 16-line lidar actually hits the floor) gives a
    ~8 mm RMS plane and repeatable normals.
    """
    r = np.hypot(xyz[:, 0], xyz[:, 1])
    z = xyz[:, 2]
    p = xyz[(r > r0) & (r < r1) & (z > -1.0) & (z < -.05)]
    if len(p) < 300:
        return None
    step = max(1, len(p) // 4000)
    q = p[::step]
    rng = np.random.default_rng(0)
    best = None
    for _ in range(iterations):
        i = rng.choice(len(q), 3, replace=False)
        n = np.cross(q[i[1]] - q[i[0]], q[i[2]] - q[i[0]])
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        if abs(n[2]) < .9:
            continue
        inliers = np.abs(q @ n - n @ q[i[0]]) < threshold
        count = int(inliers.sum())
        if best is None or count > best[0]:
            best = (count, n, q[i[0]], inliers)
    if best is None or best[0] < 300:
        return None
    _, n, origin, _ = best
    r2 = np.abs(p @ n - n @ origin) < threshold
    if r2.sum() < 300:
        return None
    A = np.c_[p[r2, 0], p[r2, 1], np.ones(int(r2.sum()))]
    coef, *_ = np.linalg.lstsq(A, p[r2, 2], rcond=None)
    n = np.array([-coef[0], -coef[1], 1.])
    n /= np.linalg.norm(n)
    return n if n[2] > 0 else -n


def rotvec_from_to(a, b):
    """Smallest rotation vector (in the frame of a,b) taking unit a to unit b."""
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    axis = np.cross(a, b)
    s = np.linalg.norm(axis)
    if s < 1e-12:
        return np.zeros(3)
    angle = math.atan2(s, float(np.dot(a, b)))
    return axis / s * angle


def transform(translation, rpy):
    roll, pitch, yaw = rpy
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    m = np.eye(4)
    m[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                 [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                 [-sp, cp * sr, cp * cr]]
    m[:3, 3] = translation
    return m


def inverse(m):
    out = np.eye(4)
    out[:3, :3] = m[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ m[:3, 3]
    return out


def load_extrinsic(hw_path):
    import yaml
    hw = yaml.safe_load(Path(hw_path).expanduser().read_text())
    cfg = hw['sensor_calibration']
    base_lidar = transform([float(v) for v in cfg['base_from_lidar']['translation']],
                           [float(v) for v in cfg['base_from_lidar']['rpy']])
    base_imu = transform([float(v) for v in cfg['base_from_imu']['translation']],
                         [float(v) for v in cfg['base_from_imu']['rpy']])
    return cfg, base_lidar, base_imu, inverse(base_imu) @ base_lidar


def find_static(imu, min_seconds):
    quiet = []
    for st, q, g, a in imu:
        an = np.linalg.norm(a)
        quiet.append(abs(an - 9.81) < .10 and np.linalg.norm(g) < .03)
    runs, start = [], None
    for k, (st, *_rest) in enumerate(imu):
        if quiet[k] and start is None:
            start = k
        if start is not None and (not quiet[k] or k == len(imu) - 1 or imu[k + 1][0] - st > .5):
            end = k
            if imu[end][0] - imu[start][0] >= min_seconds:
                runs.append((start, end))
            start = None
    return runs


def analyse(runs, imu, clouds, R):
    segments = []
    for start, end in runs:
        t0, t1 = imu[start][0], imu[end][0]
        scans = [c for c in clouds if t0 + .2 <= c[0] <= t1 - .2]
        normals = [n for n in (ground_normal(c[2]) for c in scans) if n is not None]
        if len(normals) < 5:
            continue
        n_l = np.median(np.array(normals), axis=0)
        n_l /= np.linalg.norm(n_l)
        acc = np.array([imu[k][3] for k in range(start, end + 1)]).mean(axis=0)
        u_i = acc / np.linalg.norm(acc)
        yaws = Rotation.from_quat(np.array([imu[k][1] for k in range(start, end + 1)])).as_euler('xyz')
        yaw = math.degrees(math.atan2(np.sin(yaws[:, 2]).mean(), np.cos(yaws[:, 2]).mean()))
        a = R[:3, :3] @ n_l                       # lidar ground in the IMU frame
        c = rotvec_from_to(a, u_i)                # correction in the IMU frame
        segments.append({'t': t0, 'duration': t1 - t0, 'scans': len(scans),
                         'n_l': n_l, 'u_i': u_i, 'yaw': yaw,
                         'tilt_before_deg': math.degrees(np.linalg.norm(c)),
                         'correction': c})
    return segments


def circular_span(degrees):
    vals = np.sort(np.mod(np.asarray(degrees), 360))
    gaps = np.diff(np.r_[vals, vals[0] + 360])
    return float(360 - gaps.max())


def fit_tilt(segments):
    """Separate the mounting tilt from the ground slope.

    c_i (heading-independent + Rz(-dtheta_i) * slope) is linear, so a
    least-squares fit over the stops yields the constant part ``a`` (the
    mounting tilt) and the slope ``b``. Two headings >=60 deg apart already
    determine both; 3-4 stops spread over 90-180 deg are more robust.
    """
    th0 = segments[0]['yaw']
    rows, targets = [], []
    for s in segments:
        th = math.radians(s['yaw'] - th0)
        rz = np.array([[math.cos(th), math.sin(th)], [-math.sin(th), math.cos(th)]])
        rows.append(np.c_[np.eye(2), rz])
        targets.append(np.asarray(s['correction'])[:2])
    A = np.vstack(rows)
    y = np.concatenate(targets)
    x, *_ = np.linalg.lstsq(A, y, rcond=None)
    residual = float(np.sqrt(np.mean((A @ x - y) ** 2)))
    return x[:2], x[2:], math.degrees(residual)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--hardware', default=str(Path.home() / 'Remote_ctrol/ros2_ws/src/chassis_nav2/config/hardware.yaml'))
    ap.add_argument('--min-static', type=float, default=5., help='最短静止段（秒）')
    ap.add_argument('--allow-single', action='store_true',
                    help='只有一段静止数据时也给出建议（结果含地面坡度，仅适合已知水平场地）')
    args = ap.parse_args()
    bag = Path(args.bag).expanduser()
    cfg, base_lidar, base_imu, R = load_extrinsic(args.hardware)
    imu, clouds = read_bag(bag)
    print(f'bag={bag}  imu={len(imu)}  lidar={len(clouds)}')
    if not imu or not clouds:
        raise SystemExit('包里没有 /imu/data 或 /velodyne_points')
    runs = find_static(imu, args.min_static)
    print(f'静止段: {[(round(imu[a][0], 1), round(imu[b][0] - imu[a][0], 1)) for a, b in runs]}')
    segments = analyse(runs, imu, clouds, R)
    if not segments:
        raise SystemExit('没有同时满足条件的静止段（IMU 静止 + 雷达地面点足够）')
    for k, s in enumerate(segments):
        print(f'  seg{k}: t={s["t"]:.1f}s  {s["duration"]:.0f}s  scans={s["scans"]}  '
              f'yaw={s["yaw"]:+.1f}°  tilt={s["tilt_before_deg"]:.3f}°')
    span = circular_span([s['yaw'] for s in segments])
    if len(segments) >= 2 and span >= 60:
        a, b, residual = fit_tilt(segments)
        c = np.array([a[0], a[1], 0.])
        print(f'\n用 {len(segments)} 个停点、朝向跨度 {span:.0f}° 联合拟合：'
              f'安装角 {math.degrees(np.linalg.norm(a)):.3f}°，'
              f'地面坡度 {math.degrees(np.linalg.norm(b)):.2f}°，残差 {residual:.3f}°')
        if span < 120:
            print('提示：朝向跨度不到 120°，外参和坡度会互相串；条件允许再多停 1~2 个朝向。')
        if residual > .2:
            print('警告：拟合残差偏大，可能是停点之间地面不平/不在同一块平地，建议重录。')
    elif len(segments) >= 2 and not args.allow_single:
        print(f'\n有 {len(segments)} 个停点，但朝向跨度只有 {span:.0f}°，分不开"安装角"和"地面坡度"。')
        print('Ackermann 车不用原地掉头：在平坦空地开个小环/掉头，在不同朝向各停 10 s，'
              '停点之间保持几米内，再跑一次。')
        return
    elif args.allow_single:
        longest = max(segments, key=lambda s: s['duration'])
        c = longest['correction']
        print(f'警告：按 --allow-single 用最长静止段（{longest["duration"]:.0f}s，'
              f'yaw={longest["yaw"]:+.1f}°）。结果含地面坡度，只在已知水平场地可信。')
    else:
        print('\n只有一个静止段，先不给 rpy 建议：单段结果里混着地面坡度。')
        print('Ackermann 车不用原地掉头：在平坦空地开个小环/掉头，在不同朝向各停 10 s，'
              '停点之间保持几米内，再跑一次。')
        print('（确定场地水平时可加 --allow-single 强制给出单段结果。）')
        return
    R_new = Rotation.from_rotvec(c).as_matrix() @ R[:3, :3]
    base_imu_new = inverse(np.block([[R_new, np.zeros((3, 1))], [np.zeros((1, 3)), np.ones((1, 1))]])) @ base_lidar
    rpy_new = Rotation.from_matrix(base_imu_new[:3, :3]).as_euler('xyz', degrees=True)
    rpy_old = Rotation.from_matrix(base_imu[:3, :3]).as_euler('xyz', degrees=True)
    yaw_ref = math.radians(rpy_old[2])
    # keep the same yaw representation as the configured value
    rpy_new[2] = math.degrees(math.remainder(math.radians(rpy_new[2]) - yaw_ref, 2 * math.pi) + yaw_ref)
    print('\n当前 base_from_imu.rpy = [%.2f, %.2f, %.2f]' % tuple(rpy_old))
    print('建议 base_from_imu.rpy = [%.2f, %.2f, %.2f]' % tuple(rpy_new))
    print('修正量: droll=%+.2f°  dpitch=%+.2f°  (yaw 不参与重力标定)'
          % (rpy_new[0] - rpy_old[0], rpy_new[1] - rpy_old[1]))
    grade = math.tan(math.radians(rpy_new[1] - rpy_old[1]))
    print('该俯仰误差对应高度漂移: %.3f m / 100 m  (%.2f m / 758 m 大圈)'
          % (grade * 100, abs(grade) * 758))
    print('\n只改 hardware.yaml 的 sensor_calibration.base_from_imu.rpy；'
          '改完先跑一圈看回环 Z 修正是否降到 0.5 m 以内。')


if __name__ == '__main__':
    main()
