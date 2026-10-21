#!/usr/bin/env python3
"""Inspect FAST-LIO keyframe trajectory before running loop closure."""
from __future__ import annotations
import argparse, json
from pathlib import Path
import numpy as np


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('keyframes')
    ap.add_argument('--min-time-diff', type=float, default=60.0)
    ap.add_argument('--min-index-diff', type=int, default=30)
    ap.add_argument('--radius', type=float, default=8.0)
    ap.add_argument('--top', type=int, default=20)
    args = ap.parse_args()
    directory = Path(args.keyframes).expanduser().resolve()
    rows = [json.loads(line) for line in (directory / 'keyframes.jsonl').read_text().splitlines() if line.strip()]
    rows.sort(key=lambda x: x['stamp_ns'])
    P = np.asarray([r['pose'] for r in rows])
    xy = P[:, :3, 3][:, :2]
    stamps = np.asarray([r['stamp_ns'] for r in rows], dtype=np.int64)
    print(f'keyframes: {len(rows)}')
    print(f'duration: {(stamps[-1]-stamps[0])*1e-9:.1f}s')
    if len(rows) > 1:
        steps = np.linalg.norm(np.diff(xy, axis=0), axis=1)
        print(f'trajectory length: {steps.sum():.1f}m')
        print(f'start/end distance: {np.linalg.norm(xy[-1]-xy[0]):.2f}m')
    print(f'x range: {xy[:,0].min():.1f}..{xy[:,0].max():.1f}m')
    print(f'y range: {xy[:,1].min():.1f}..{xy[:,1].max():.1f}m')
    print('\nclosest older keyframes within radius:')
    out = []
    for i in range(len(rows)):
        best = None
        for j in range(max(0, i-args.min_index_diff), -1, -1):
            if (stamps[i] - stamps[j]) * 1e-9 < args.min_time_diff:
                continue
            dist = float(np.linalg.norm(xy[i]-xy[j]))
            if dist <= args.radius and (best is None or dist < best[0]):
                best = (dist, j)
        if best is not None:
            out.append((best[0], i, best[1]))
    for dist, i, j in sorted(out, key=lambda x: x[0])[:args.top]:
        print(f'  {i:4d} <- {j:4d}  xy_dist={dist:.2f}m  dt={(stamps[i]-stamps[j])*1e-9:.0f}s')
    if not out:
        print('  (none under current radius/time filters)')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
