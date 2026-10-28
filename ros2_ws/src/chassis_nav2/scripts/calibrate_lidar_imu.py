#!/usr/bin/env python3
"""Calibrate the lidar-IMU mounting roll/pitch from static recordings.

Why: the mapping frontend assumes the lidar and the IMU are parallel
(``base_from_imu.rpy`` only carries the -90 deg yaw). A real mounting pitch of
even 0.3 deg makes the lidar ground look tilted in the body frame; the gravity
aid keeps the attitude on the IMU, so the map z drops by ``tan(pitch)`` per
metre travelled - a fixed height bias per loop (0.35 deg over 758 m = 4.6 m).

Method: park still, record; turn the car ~180 deg in place, park again, record.
For each still segment fit the ground plane in the lidar frame and compare it
with the IMU gravity direction (both mapped through the current extrinsic).
The ground slope is identical in the world for both headings but flips in the
body frame, so averaging the two discrepancies cancels the slope and leaves the
mounting tilt. Yaw is not observable from gravity; it stays as configured.

Usage:
    # on the robot, with sensors running
    ros2 bag record -o /tmp/calib -o /tmp/calib /velodyne_points /imu/data
    #   ... park facing A for ~10 s, turn the car ~180 deg, face B for ~10 s, Ctrl-C
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


def ground_normal(xyz):
    r = np.hypot(xyz[:, 0], xyz[:, 1])
    z = xyz[:, 2]
    p = xyz[(r > 3) & (r < 14) & (z > -1.2) & (z < .05)]
    if len(p) < 300:
        return None
    A = np.c_[p[:, 0], p[:, 1], np.ones(len(p))]
    w = np.ones(len(p), bool)
    for _ in range(4):
        coef, *_ = np.linalg.lstsq(A[w], p[w, 2], rcond=None)
        res = p[:, 2] - A @ coef
        s = np.median(np.abs(res[w])) * 1.4826 + 1e-9
        w = np.abs(res) < max(.04, 2.5 * s)
    if w.sum() < 200:
        return None
    coef, *_ = np.linalg.lstsq(A[w], p[w, 2], rcond=None)
    n = np.array([-coef[0], -coef[1], 1.])
    return n / np.linalg.norm(n)


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
    pair = None
    best = 180.
    for i in range(len(segments)):
        for j in range(i + 1, len(segments)):
            diff = abs((segments[i]['yaw'] - segments[j]['yaw'] + 180) % 360 - 180)
            if abs(diff - 180) < best:
                best = abs(diff - 180)
                pair = (i, j)
    if pair is not None and best <= 30:
        i, j = pair
        heading_diff = abs((segments[i]['yaw'] - segments[j]['yaw'] + 180) % 360 - 180)
        c = (segments[i]['correction'] + segments[j]['correction']) / 2
        print(f'用 seg{i}+seg{j}（朝向差 {heading_diff:.0f}°，偏离 180° 仅 {best:.0f}°，地面坡度对消）')
    elif not args.allow_single:
        print('\n没有找到朝向差 ~180° 的两段静止数据，先不给 rpy 建议。')
        print('上面的 tilt 是“雷达地面 vs IMU 重力”的夹角，里面混着停车点的地面坡度；'
              '单段直接当外参会把坡度一起标进去。')
        print('请在同一个地点：车头朝 A 停 10 s → 原地掉头 180° → 朝 B 停 10 s，用这一个包再跑。')
        print('（如果确定场地水平，可加 --allow-single 强制给出单段结果。）')
        return
    else:
        longest = max(segments, key=lambda s: s['duration'])
        c = longest['correction']
        print(f'警告：没有 180° 成对数据，按 --allow-single 用最长静止段（{longest["duration"]:.0f}s，'
              f'yaw={longest["yaw"]:+.1f}°）。结果含地面坡度，只在已知水平场地可信。')
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
