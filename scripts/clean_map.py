#!/usr/bin/env python3
"""清理 nav2 栅格图(map.pgm)里的孤立噪点占用格。

pcd2pgm 切片后, 点云噪声会在栅格图里留下孤立/稀疏的占用像素;
膨胀层会把每个噪点放大成 inflation_radius 的圆斑, 连成一片就堵死可行走空间。
规则: 占用格 8 邻域内占用邻居 <=2 则置为空闲。直线墙面/椅腿是连续结构, 不受影响。

用法: python3 clean_map.py [pgm路径]   (默认 ~/Remote_ctrol/maps/map.pgm)
"""
import sys
from pathlib import Path

import numpy as np


def read_pgm(path):
    with open(path, 'rb') as f:
        if f.readline().strip() != b'P5':
            raise ValueError('只支持二进制 P5 pgm')
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        maxv = int(f.readline())
        if maxv != 255:
            raise ValueError(f'maxval={maxv}, 预期 255')
        return w, h, np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w).copy()


def main():
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path.home() / 'Remote_ctrol/maps/map.pgm'
    w, h, img = read_pgm(path)
    occ = img < 100
    pad = np.pad(occ.astype(np.int32), 1)
    n8 = sum(pad[dy:dy + h, dx:dx + w] for dy in range(3) for dx in range(3)) - occ
    noise = occ & (n8 <= 2)
    img[noise] = 254
    with open(path, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (w, h))
        f.write(img.tobytes())
    print(f'{path}: 清除噪点 {int(noise.sum())} 格, 剩余占用 {int((img < 100).sum())} 格')


if __name__ == '__main__':
    main()
