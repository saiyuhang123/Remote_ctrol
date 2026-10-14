#!/usr/bin/env python3
"""测量地面点云在 base_link 系下的 z 分布，为障碍高度带定阈值。

用法（先启动 mode:=drivers，车停在干净平地上）：
    python3 scripts/measure_ground_noise.py
    python3 scripts/measure_ground_noise.py --frames 100 --rmax 4.0

输出：z 直方图（1cm  bins）、地面簇峰值/噪声σ、建议的 scan.min_height。
换不同地面各测一次：最平的、要通行的最差的、有真实马路牙子的位置。
"""
import argparse

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

_DT = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
       5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}


def cloud_to_xyz(msg):
    dtype = np.dtype({'names': [f.name for f in msg.fields],
                      'formats': [_DT[f.datatype] for f in msg.fields],
                      'offsets': [f.offset for f in msg.fields],
                      'itemsize': msg.point_step})
    rows = []
    for h in range(msg.height):
        start = h * msg.row_step
        rows.append(np.frombuffer(msg.data[start:start + msg.width * msg.point_step], dtype=dtype))
    pts = np.concatenate(rows) if rows else np.empty(0, dtype=dtype)
    return np.stack([pts['x'], pts['y'], pts['z']], axis=1)


class GroundProbe(Node):
    def __init__(self, args):
        super().__init__('ground_noise_probe')
        self.args = args
        self.chunks = []
        self.sub = self.create_subscription(PointCloud2, args.topic, self.cb, 10)

    def cb(self, msg):
        xyz = cloud_to_xyz(msg)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        r = np.hypot(xyz[:, 0], xyz[:, 1])
        z_base = xyz[:, 2] + self.args.lidar_height
        zmin, zmax = (float(v) for v in self.args.zwin.split(','))
        keep = (r >= self.args.rmin) & (r <= self.args.rmax) & (z_base >= zmin) & (z_base <= zmax)
        self.chunks.append(np.stack([r[keep], z_base[keep]], axis=1))
        if len(self.chunks) % 10 == 0:
            self.get_logger().info(f'已采集 {len(self.chunks)}/{self.args.frames} 帧...')
        if len(self.chunks) >= self.args.frames:
            self.report(np.concatenate(self.chunks))
            rclpy.shutdown()

    def report(self, pts):
        a = self.args
        r, z = pts[:, 0], pts[:, 1]
        print(f'\n===== 采样 {len(self.chunks)} 帧，窗口内 {len(z)} 点（base_link 系 z）=====')
        zmin, zmax = (float(v) for v in a.zwin.split(','))
        if len(z) == 0:
            print(f'\n!! 窗口（r {a.rmin}~{a.rmax}m，z {zmin}~{zmax}m）内 0 点。')
            print('!! 该雷达近处是盲区（实测 1.5m 内无点），请用 --rmin 1.5 --rmax 3.0 重试。')
            return
        edges = np.arange(zmin, zmax + a.bin, a.bin)
        hist, _ = np.histogram(z, bins=edges)
        peak_i = int(np.argmax(hist))
        scale = max(1, hist.max() // 60)
        print('\nz 直方图（每行 1cm，# 约 %d 点）：' % scale)
        for i, c in enumerate(hist):
            bar = '#' * (c // scale)
            mark = ' <== 峰值' if i == peak_i else ''
            if c or abs(edges[i]) < 1e-9:
                print(f'  {edges[i]:+.2f} ~ {edges[i + 1]:+.2f} | {bar}{mark}')
        peak = 0.5 * (edges[peak_i] + edges[peak_i + 1])
        g = z[np.abs(z - peak) <= 0.05]
        if len(g) < 30:
            print(f'\n!! 地面簇只有 {len(g)} 点，峰值 {peak:+.3f} 处不可信，请调整 --zwin 后重测。')
            return
        mu, sigma = float(g.mean()), float(g.std())
        hi = float(np.percentile(g, 99.9))
        upper = max(mu + 3 * sigma, hi)
        print(f'\n地面簇：中心 z = {mu:+.3f} m（即 base_link 离地约 {-mu:.2f} m），σ = {sigma * 100:.1f} cm')
        print(f'地面簇实际上沿（3σ 与 99.9% 取大者）: z = {upper:+.3f} m')
        print(f'建议 scan.min_height ≥ {upper + 0.01:+.3f}（上沿 +1cm 余量），'
              f'此时离地约 {upper + 0.01 - mu:.2f} m 以上的障碍才会被看见')
        print('\n按距离分段的地面簇 σ（噪声随距离变厚，重点看局部代价地图 2.5m 范围内）：')
        print(f'  {"距离段":>10}  {"点数":>8}  {"σ(cm)":>7}  {"上沿z":>8}')
        for lo, hi_r in [(a.rmin, 2.0), (2.0, 2.5), (2.5, 3.0), (3.0, a.rmax)]:
            if lo >= a.rmax:
                break
            seg = z[(r >= lo) & (r < min(hi_r, a.rmax)) & (np.abs(z - peak) <= 0.05)]
            if len(seg) < 30:
                print(f'  {lo:.1f}~{min(hi_r, a.rmax):.1f}m  {len(seg):>8}  点太少')
                continue
            s = float(seg.std())
            up = max(float(seg.mean()) + 3 * s, float(np.percentile(seg, 99.9)))
            print(f'  {lo:.1f}~{min(hi_r, a.rmax):.1f}m  {len(seg):>8}  {s * 100:>7.1f}  {up:>+8.3f}')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--topic', default='/velodyne_points')
    p.add_argument('--frames', type=int, default=50)
    p.add_argument('--lidar-height', type=float, default=0.46,
                   help='雷达相对 base_link 的安装高度（hardware.yaml base_from_lidar z）')
    p.add_argument('--rmin', type=float, default=1.5,
                   help='该雷达近处为盲区（实测 1.5m 内无点），默认从 1.5m 起')
    p.add_argument('--rmax', type=float, default=3.0)
    p.add_argument('--zwin', default='-0.8,0.3', help='base_link 系 z 采集窗口，逗号分隔')
    p.add_argument('--bin', type=float, default=0.01)
    args = p.parse_args()
    rclpy.init()
    node = GroundProbe(args)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
