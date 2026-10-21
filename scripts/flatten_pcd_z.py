#!/usr/bin/env python3
"""PCD 首尾高差拉平：消除 FAST-LIO 纯里程计累积的 Z 向漂移（整张图缓慢翘起）。

原理：长直平地场景下 roll/pitch 缓慢漂移，地图整体带一个小倾角，
回到起点就出现首尾高度差。脚本在点云里 RANSAC 拟合地面平面，然后：
  linear 模式（默认）：只做剪切式 Z 补偿 z' = z - (a*x + b*y)，x/y 完全不动，
                      不影响下游 pcd2pgm 切片和 2D 定位坐标系；
  rotate 模式：把整张图绕质心旋转到地面水平，修正更"物理"，但 x/y 会变。

处理的是建图产物 maps/fastlio/map.pcd，在切片（pcd2pgm）之前做。

用法：
    python3 scripts/flatten_pcd_z.py                          # 处理默认地图（原图自动备份）
    python3 scripts/flatten_pcd_z.py --input x.pcd --dry-run  # 只报告倾角/高差，不写文件
    python3 scripts/flatten_pcd_z.py --mode rotate            # 旋转拉平
"""
import argparse
import shutil
from datetime import datetime
from pathlib import Path

import numpy as np
import open3d as o3d

DEFAULT_MAP = Path.home() / 'Remote_ctrol/maps/fastlio/map.pcd'


def fit_ground_plane(cloud, quantile, distance, ransac_n, iters):
    """在最低 quantile 比例的点里 RANSAC 拟合地面，返回单位法向(nz>0)和内点掩码。"""
    pts = np.asarray(cloud.points)
    z_thresh = np.quantile(pts[:, 2], quantile)
    ground_idx = np.where(pts[:, 2] <= z_thresh)[0]
    ground = cloud.select_by_index(ground_idx.tolist())
    plane, inliers = ground.segment_plane(distance_threshold=distance,
                                          ransac_n=ransac_n,
                                          num_iterations=iters)
    normal = np.array(plane[:3])
    normal = normal / np.linalg.norm(normal)
    if normal[2] < 0:
        normal = -normal
    ratio = len(inliers) / max(len(ground_idx), 1)
    return normal, ratio, len(ground_idx)


def height_gap_along_path(pts):
    """沿路径主方向（XY 第一主成分）取首尾各 10% 的低位点，返回平均高差。"""
    low = pts[pts[:, 2] <= np.quantile(pts[:, 2], 0.2)]
    xy = low[:, :2] - low[:, :2].mean(axis=0)
    axis = np.linalg.eigh(xy.T @ xy)[1][:, -1]
    t = xy @ axis
    lo, hi = np.quantile(t, [0.1, 0.9])
    z_head = low[t <= lo, 2].mean()
    z_tail = low[t >= hi, 2].mean()
    return abs(z_tail - z_head)


def main():
    parser = argparse.ArgumentParser(description='PCD 首尾高差拉平（FAST-LIO Z 漂移后处理）')
    parser.add_argument('--input', default=str(DEFAULT_MAP), help='输入 pcd（默认 maps/fastlio/map.pcd）')
    parser.add_argument('--output', default='', help='输出路径；留空则覆盖输入（原图备份为 *_时间戳.pcd）')
    parser.add_argument('--mode', choices=['linear', 'rotate'], default='linear',
                        help='linear: 只补偿 Z，x/y 不变（默认）；rotate: 整图旋转拉平')
    parser.add_argument('--ground-quantile', type=float, default=0.2,
                        help='取 z 最低的比例作为地面候选点（默认 0.2）')
    parser.add_argument('--distance', type=float, default=0.15, help='RANSAC 平面距离阈值 m')
    parser.add_argument('--iters', type=int, default=2000, help='RANSAC 迭代次数')
    parser.add_argument('--dry-run', action='store_true', help='只报告不写文件')
    args = parser.parse_args()

    src = Path(args.input).expanduser()
    if not src.is_file():
        raise SystemExit(f'地图不存在：{src}')
    cloud = o3d.io.read_point_cloud(str(src))
    pts = np.asarray(cloud.points)
    total = len(pts)
    print(f'读入 {src}：{total} 点')

    gap_before = height_gap_along_path(pts)
    normal, ratio, n_ground = fit_ground_plane(cloud, args.ground_quantile,
                                               args.distance, 3, args.iters)
    tilt = np.degrees(np.arccos(np.clip(normal[2], -1, 1)))
    print(f'地面候选 {n_ground} 点，RANSAC 内点率 {ratio:.1%}')
    print(f'地面法向 ({normal[0]:.4f}, {normal[1]:.4f}, {normal[2]:.4f})，倾角 {tilt:.3f}°')
    print(f'拉平前路径首尾高差约 {gap_before:.3f} m')
    if ratio < 0.5:
        print('警告：地面内点率偏低，场景可能不是平地为主，结果需谨慎')

    if args.mode == 'linear':
        # 平面 nx*x+ny*y+nz*z+d=0 -> 地面坡度 dz/dx=-nx/nz, dz/dy=-ny/nz
        a, b = -normal[0] / normal[2], -normal[1] / normal[2]
        slope = a * pts[:, 0] + b * pts[:, 1]
        low = pts[:, 2] <= np.quantile(pts[:, 2], args.ground_quantile)
        pts[:, 2] -= slope - slope[low].mean()  # 减去坡度，保持地面原平均高度
    else:
        z_axis = np.array([0.0, 0.0, 1.0])
        axis = np.cross(normal, z_axis)
        norm = np.linalg.norm(axis)
        if norm < 1e-6:
            print('倾角接近 0，无需处理')
            return
        rot = o3d.geometry.get_rotation_matrix_from_axis_angle(
            axis / norm * np.radians(tilt))
        center = pts.mean(axis=0)
        pts[:] = (pts - center) @ rot.T + center

    gap_after = height_gap_along_path(pts)
    print(f'拉平后路径首尾高差约 {gap_after:.3f} m')

    if args.dry_run:
        print('dry-run，未写文件')
        return
    out = Path(args.output).expanduser() if args.output else src
    if out == src:
        backup = src.with_name(f'{src.stem}_backup_{datetime.now():%Y%m%d_%H%M%S}{src.suffix}')
        shutil.copy2(src, backup)
        print(f'原图已备份：{backup}')
    corrected = o3d.geometry.PointCloud()
    corrected.points = o3d.utility.Vector3dVector(pts)
    if cloud.has_colors():
        corrected.colors = cloud.colors
    o3d.io.write_point_cloud(str(out), corrected)
    print(f'已写出 {out}：{total} 点')


if __name__ == '__main__':
    main()
