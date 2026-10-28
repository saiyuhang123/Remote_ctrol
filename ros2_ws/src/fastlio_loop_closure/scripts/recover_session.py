#!/usr/bin/env python3
"""Recover a saved FAST-LIO session offline, including a common vertical bias.

A frontend height drift shifts every loop match by the same dz; the live gate
used to reject the whole loop because of it. This tool reuses the matches that
were already measured and stored in the session's export, accepts the ones that
agree on a common dz, refines the trajectory and writes a corrected PCD.

Usage:
    recover_session.py <session_dir> [--out PREFIX] [--export EXPORT_DIR]

Only reads the session; never touches the robot's active map.
"""
import argparse
import json
import sys
from datetime import datetime
from pathlib import Path

import numpy as np

from fastlio_loop_closure.geometry import (
    Keyframe, build_map, overlap_is_usable, refine, write_pcd)


def load_session(session, limit=None):
    manifest = [json.loads(l) for l in (session/'keyframes.jsonl').open() if l.strip()]
    if limit is not None:
        manifest = manifest[:limit]
    kfs = []
    for item in manifest:
        data = np.load(session/'keyframes'/item['file'])
        kfs.append(Keyframe(item['index'], item['stamp_ns'], data['pose'].copy(),
                            data['xyz'], data['intensity']))
    return kfs


def newest_export(session):
    exports = sorted((session/'exports').glob('*'), key=lambda p: p.stat().st_mtime)
    if not exports:
        raise SystemExit(f'no export under {session}/exports')
    return exports[-1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('session')
    ap.add_argument('--out', default=None)
    ap.add_argument('--export', default=None)
    ap.add_argument('--voxel', type=float, default=.10)
    args = ap.parse_args()
    session = Path(args.session).expanduser()
    export = Path(args.export).expanduser() if args.export else newest_export(session)
    validation = json.load((export/'map.validation.json').open())

    total = int(validation.get('keyframes') or len(validation.get('overlap_measurements', [])))
    kfs = load_session(session, total)
    poses = np.array([k.pose for k in kfs], dtype=float)
    rotation = np.array(validation['map_from_lio_rotation'])
    poses[:, :3, :3] = rotation @ poses[:, :3, :3]
    poses[:, :3, 3] = poses[:, :3, 3] @ rotation.T

    matches = list(validation.get('overlap_measurements', [])) + \
        list(validation.get('rejected_overlap_measurements', []))
    matches = [m for m in matches if m['source'] < total and m['target'] < total]
    if not matches:
        raise SystemExit('no measured overlaps in the export')
    dzs = np.array([m['position_correction_m'][2] for m in matches])
    bias = float(np.median(dzs))
    spread = float(np.ptp(dzs))
    usable = [m for m in matches if overlap_is_usable(m, bias)]
    print(f'keyframes={total} matches={len(matches)} common_dz={bias:+.2f} m spread={spread:.2f} m')
    print(f'usable after bias removal={len(usable)}')
    if len(usable) < 6:
        raise SystemExit('not enough consistent matches; nothing written')

    corrected, report = refine(poses, usable)
    report.update({'recovered_from': str(export), 'common_vertical_bias_m': bias,
                   'common_vertical_spread_m': spread,
                   'recovered_at': datetime.now().isoformat()})
    print(json.dumps({k: report[k] for k in (
        'geometry_checks_passed', 'max_pose_translation_change_m',
        'max_heldout_translation_error_m', 'max_heldout_yaw_error_deg',
        'keyframe_height_range_m', 'endpoint_xyz_m')}, indent=1, ensure_ascii=False))
    if not report['geometry_checks_passed']:
        raise SystemExit('refine failed the held-out checks; not writing a map')

    out = Path(args.out).expanduser() if args.out else \
        session/'exports'/f'recovered_{int(datetime.now().timestamp())}'/'map'
    out.parent.mkdir(parents=True, exist_ok=True)
    points = build_map(kfs, corrected, args.voxel)
    write_pcd(out.with_suffix('.pcd'), points)
    np.save(out.parent/'poses.npy', corrected)
    report.update(points=len(points), pcd=str(out.with_suffix('.pcd')), voxel_m=args.voxel)
    (out.with_suffix('.validation.json')).write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    (out.parent/'使用说明.txt').write_text(
        '离线恢复图（回环已闭合）\n'
        f'原会话：{session}\n'
        f'公共垂直偏置：{bias:+.2f} m（{len(usable)} 条一致回环匹配）\n'
        f'留出位置误差：{report["max_heldout_translation_error_m"]:.3f} m，'
        f'留出角度误差：{report["max_heldout_yaw_error_deg"]:.3f}°\n'
        f'关键帧高度范围：{report["keyframe_height_range_m"][0]:.2f} ~ '
        f'{report["keyframe_height_range_m"][1]:.2f} m\n'
        '仅用于核对；正式替换导航图前仍需实车定位检查。\n', encoding='utf-8')
    print('written', out.with_suffix('.pcd'))


if __name__ == '__main__':
    sys.exit(main())
