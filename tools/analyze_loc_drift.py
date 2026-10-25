#!/usr/bin/env python3
"""闭环定位漂移分析：开车绕一圈回到起点，比较起终点位姿差。

用法:
    python3 tools/analyze_loc_drift.py <bag目录 或 .db3 文件> [--plot out.png]

读的话题（缺哪个就跳过哪个）:
    /localization   导航实际使用的位姿（融合后，map 系）
    /odom_lio       FAST-LIO 原始里程计（odom 系，未校正）
    /map_to_odom    ICP 重定位输出的 map->odom 校正
    /rosout         ICP fitness 日志

输出:
    起点/终点位姿、闭环误差（Δx/Δy/Δyaw/距离）、行驶里程、
    LIO 自身漂移、ICP 校正量变化、fitness 统计、静止抖动。
"""
from __future__ import annotations

import math
import os
import re
import sqlite3
import subprocess
import sys
import tempfile

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

WANT = ('/localization', '/odom_lio', '/map_to_odom', '/rosout')
SETTLE = 2.0          # 首尾各取多少秒当作静止段
FITNESS_RE = re.compile(r'ICP fitness:\s*([0-9.]+)')


def bag_to_db3(path: str) -> str:
    if os.path.isfile(path) and path.endswith('.db3'):
        return path
    if not os.path.isdir(path):
        raise SystemExit(f'找不到 bag: {path}')
    db3 = [f for f in os.listdir(path) if f.endswith('.db3')]
    if db3:
        return os.path.join(path, db3[0])
    zst = [f for f in os.listdir(path) if f.endswith('.db3.zstd')]
    if not zst:
        raise SystemExit(f'{path} 里没有 .db3 / .db3.zstd')
    out = os.path.join(tempfile.mkdtemp(prefix='locbag_'), zst[0][:-5])
    with open(out, 'wb') as fh:
        subprocess.run(['zstd', '-d', '-c', os.path.join(path, zst[0])],
                       check=True, stdout=fh)
    return out


def load(db3: str, wanted) -> dict:
    conn = sqlite3.connect(db3)
    topics = {tid: (name, typ) for tid, name, typ in
              conn.execute('select id,name,type from topics')}
    keep = {tid: (name, typ) for tid, (name, typ) in topics.items() if name in wanted}
    cls = {tid: get_message(typ) for tid, (_, typ) in keep.items()}
    out = {name: [] for name, _ in keep.values()}
    for tid, ts, blob in conn.execute(
            'select topic_id,timestamp,data from messages order by timestamp'):
        if tid not in keep:
            continue
        out[keep[tid][0]].append((ts / 1e9, deserialize_message(blob, cls[tid])))
    conn.close()
    return out


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def norm(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def pose_series(msgs):
    """(t, x, y, yaw) 列表，兼容 PoseWithCovarianceStamped / Odometry。"""
    out = []
    for t, m in msgs:
        p = m.pose.pose if hasattr(m, 'pose') and hasattr(m.pose, 'pose') else None
        if p is None:
            continue
        out.append((t, p.position.x, p.position.y, yaw_of(p.orientation)))
    return out


def mean_pose(series, t_from, t_to):
    seg = [s for s in series if t_from <= s[0] <= t_to]
    if not seg:
        return None
    n = len(seg)
    x = sum(s[1] for s in seg) / n
    y = sum(s[2] for s in seg) / n
    sy = sum(math.sin(s[3]) for s in seg)
    cy = sum(math.cos(s[3]) for s in seg)
    return x, y, math.atan2(sy / n, cy / n)


def path_len(series):
    return sum(math.hypot(series[i][1] - series[i - 1][1], series[i][2] - series[i - 1][2])
               for i in range(1, len(series)))


def report(name, series, t0, t1):
    if len(series) < 10:
        print(f'  {name}: 数据不足（{len(series)} 帧），跳过')
        return
    a = mean_pose(series, t0, t0 + SETTLE)
    b = mean_pose(series, t1 - SETTLE, t1)
    if not a or not b:
        print(f'  {name}: 首尾静止段没有数据，跳过')
        return
    dx, dy, dyaw = b[0] - a[0], b[1] - a[1], norm(b[2] - a[2])
    seg_a = [s for s in series if t0 <= s[0] <= t0 + SETTLE]
    seg_b = [s for s in series if t1 - SETTLE <= s[0] <= t1]
    jit = max(max(abs(s[1] - a[0]), abs(s[2] - a[1])) for s in seg_a)
    jit_b = max(max(abs(s[1] - b[0]), abs(s[2] - b[1])) for s in seg_b)
    print(f'\n[{name}]  {len(series)} 帧，里程 {path_len(series):.2f} m')
    print(f'  起点 ({a[0]:+.3f}, {a[1]:+.3f}) yaw {math.degrees(a[2]):+7.2f}°')
    print(f'  终点 ({b[0]:+.3f}, {b[1]:+.3f}) yaw {math.degrees(b[2]):+7.2f}°')
    print(f'  闭环误差: Δx {dx * 100:+.1f} cm  Δy {dy * 100:+.1f} cm  '
          f'距离 {math.hypot(dx, dy) * 100:.1f} cm  Δyaw {math.degrees(dyaw):+.2f}°')
    print(f'  静止抖动: 起点 {jit * 100:.1f} cm / 终点 {jit_b * 100:.1f} cm'
          '  (>5 cm 说明定位本身在漂，起终点不可比)')
    return dx, dy, math.hypot(dx, dy), math.degrees(dyaw)


def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    bag = sys.argv[1]
    plot = None
    if '--plot' in sys.argv:
        plot = sys.argv[sys.argv.index('--plot') + 1]

    db3 = bag_to_db3(bag)
    data = load(db3, WANT)
    loc = pose_series(data.get('/localization', []))
    lio = pose_series(data.get('/odom_lio', []))
    if not loc:
        raise SystemExit('/localization 没有数据，无法分析')

    times = [s[0] for s in loc]
    t0, t1 = times[0], times[-1]
    print(f'bag 时长 {t1 - t0:.1f} s，/localization 帧数 {len(loc)}')

    report('导航位姿 /localization', loc, t0, t1)
    if lio:
        report('LIO 原始 /odom_lio', lio, lio[0][0], lio[-1][0])

    # ICP 校正量
    m2o = pose_series(data.get('/map_to_odom', []))
    if len(m2o) > 2:
        a, b = m2o[0], m2o[-1]
        print(f'\n[ICP /map_to_odom]  {len(m2o)} 帧')
        print(f'  校正位移: Δx {(b[1] - a[1]) * 100:+.1f} cm  Δy {(b[2] - a[2]) * 100:+.1f} cm  '
              f'Δyaw {math.degrees(norm(b[3] - a[3])):+.2f}°')
        jumps = [math.hypot(m2o[i][1] - m2o[i - 1][1], m2o[i][2] - m2o[i - 1][2])
                 for i in range(1, len(m2o))]
        if jumps:
            print(f'  相邻帧最大跳变 {max(jumps) * 100:.1f} cm（>20 cm 说明 ICP 在跳）')

    # ICP fitness
    fits = []
    for _, m in data.get('/rosout', []):
        hit = FITNESS_RE.search(getattr(m, 'msg', '') or '')
        if hit:
            fits.append(float(hit.group(1)))
    if fits:
        fits.sort()
        print(f'\n[ICP fitness]  {len(fits)} 次')
        print(f'  中位 {fits[len(fits) // 2]:.3f}  p95 {fits[int(len(fits) * 0.95)]:.3f}  '
              f'最大 {fits[-1]:.3f}')
        print('  (Open3D 重叠率，越高越好；节点阈值 0.9，低于它会打 warn 并保持旧校正)')

    if plot:
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(1, 2, figsize=(13, 6))
            for name, series in (('/localization', loc), ('/odom_lio', lio)):
                if series:
                    ax[0].plot([s[1] for s in series], [s[2] for s in series], lw=1, label=name)
            ax[0].set_aspect('equal')
            ax[0].grid(alpha=.3)
            ax[0].legend(fontsize=8)
            ax[0].set_title('轨迹（原点对齐）')
            if m2o:
                tt = [s[0] - m2o[0][0] for s in m2o]
                ax[1].plot(tt, [s[1] for s in m2o], lw=1, label='map->odom x')
                ax[1].plot(tt, [s[2] for s in m2o], lw=1, label='map->odom y')
                ax[1].legend(fontsize=8)
                ax[1].set_title('ICP 校正量随时间')
                ax[1].set_xlabel('t (s)')
            fig.tight_layout()
            fig.savefig(plot, dpi=120)
            print(f'\n图已保存: {plot}')
        except Exception as exc:                     # 画图失败不影响数值
            print(f'画图失败: {exc}')


if __name__ == '__main__':
    main()
