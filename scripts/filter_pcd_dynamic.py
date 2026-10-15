#!/usr/bin/env python3
"""PCD 动态物体拖影过滤：删掉半径内邻居过少的稀疏点（走动人/车的残影）。

原理：FAST-LIO 不做动静态区分，移动物体在累积点云里是低密度条纹；
静态墙/地面是致密面。半径滤波（radius 内邻居 < neighbors 则删除）对拖影有效，
配合统计离群滤波清浮点。处理的是建图产物 maps/fastlio/map.pcd，在切片（pcd2pgm）之前做。

用法：
    python3 scripts/filter_pcd_dynamic.py                      # 处理默认地图（原图自动备份）
    python3 scripts/filter_pcd_dynamic.py --input x.pcd --radius 0.2 --neighbors 8
    python3 scripts/filter_pcd_dynamic.py --dry-run            # 只报告会删多少，不写文件
"""
import argparse
import shutil
from datetime import datetime
from pathlib import Path

import numpy as np
import open3d as o3d

DEFAULT_MAP = Path.home() / 'Remote_ctrol/maps/fastlio/map.pcd'


def main():
    parser = argparse.ArgumentParser(description='PCD 动态拖影过滤')
    parser.add_argument('--input', default=str(DEFAULT_MAP), help='输入 pcd（默认 maps/fastlio/map.pcd）')
    parser.add_argument('--output', default='', help='输出路径；留空则覆盖输入（原图备份为 *_时间戳.pcd）')
    parser.add_argument('--radius', type=float, default=0.15, help='邻域半径 m（拖影稀疏，墙地面致密）')
    parser.add_argument('--neighbors', type=int, default=6, help='半径内最少邻居数，不足则删点')
    parser.add_argument('--statistical', type=int, default=20, help='统计滤波邻居数；0 关闭')
    parser.add_argument('--std-ratio', type=float, default=2.0, help='统计滤波标准差倍率')
    parser.add_argument('--dry-run', action='store_true', help='只报告不写文件')
    args = parser.parse_args()

    src = Path(args.input).expanduser()
    if not src.is_file():
        raise SystemExit(f'地图不存在：{src}')
    cloud = o3d.io.read_point_cloud(str(src))
    total = len(cloud.points)
    print(f'读入 {src}：{total} 点')

    kept = cloud
    if args.statistical > 0:
        kept, _ = kept.remove_statistical_outlier(nb_neighbors=args.statistical,
                                                  std_ratio=args.std_ratio)
        print(f'统计滤波后：{len(kept.points)} 点（删 {total - len(kept.points)}）')
    filtered, _ = kept.remove_radius_outlier(nb_points=args.neighbors, radius=args.radius)
    removed = total - len(filtered.points)
    print(f'半径滤波(r={args.radius}, n>={args.neighbors})后：{len(filtered.points)} 点，'
          f'共删 {removed} 点（{removed / total:.1%}）')
    if args.dry_run:
        print('dry-run，未写文件')
        return

    out = Path(args.output).expanduser() if args.output else src
    if out == src:
        backup = src.with_name(f'{src.stem}_prefilter_{datetime.now():%Y%m%d_%H%M%S}{src.suffix}')
        shutil.copy2(src, backup)
        print(f'原图已备份：{backup.name}')
    o3d.io.write_point_cloud(str(out), filtered)
    print(f'已保存：{out}')


if __name__ == '__main__':
    main()
