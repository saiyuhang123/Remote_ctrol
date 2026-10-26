#!/usr/bin/env python3
"""Reprocess captured body keyframes using the installed mapping backend.

Never starts sensor drivers or modifies the source session / UI map. A passing
result still needs visual review and vehicle localization validation.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path

os.environ.update(ROS_LOCALHOST_ONLY='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='3')

import numpy as np
import rclpy
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from fastlio_loop_closure.geometry import Keyframe, atomic_json, build_map, refine, write_pcd
from fastlio_loop_closure.live_loop_closure import LiveLoopClosure


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('session', type=Path)
    ap.add_argument('--output', required=True, type=Path)
    ap.add_argument('--domain', type=int, default=222)
    args = ap.parse_args()
    source, output = args.session.resolve(), args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_LOG_DIR'] = str(output/'ros_logs')
    records = [json.loads(row) for row in (source/'keyframes.jsonl').read_text().splitlines()]
    keyframes = []
    for record in records:
        with np.load(source/'keyframes'/record['file']) as data:
            keyframes.append(Keyframe(record['index'], record['stamp_ns'], data['pose'], data['xyz'], data['intensity']))
    gravity = json.loads((source/'gravity_reference.json').read_text())
    rotation = np.asarray(gravity['map_from_lio_rotation'])
    rclpy.init(args=['--ros-args', '-p', 'output_map_path:='+str(output/'map.pcd')])
    node = LiveLoopClosure()
    # Use the production matcher synchronously; no fabricated ROS sensor data.
    node.running = False
    node.work_event.set()
    node.worker.join(timeout=5.)
    try:
        node.finish_matching(keyframes)
        first_count = len(node.matches)
        node.finish_matching(keyframes)
        if len(node.matches) != first_count:
            raise RuntimeError('Repeated save finalization duplicated loop constraints')
        poses = np.asarray([k.pose for k in keyframes])
        poses[:, :3, :3] = rotation @ poses[:, :3, :3]
        poses[:, :3, 3] = poses[:, :3, 3] @ rotation.T
        report = {'source_session': str(source), 'keyframes': len(keyframes),
                  'qualified_overlaps': len(node.matches), 'overlap_measurements': node.matches,
                  'rejected_overlap_measurements': node.rejected_matches,
                  'map_from_lio_rotation': rotation.tolist(),
                  'processing': 'Production matcher and graph using captured online keyframes, not a new frontend replay.'}
        report['repeat_finalize_deduplicated'] = True
        original_report = json.loads((source/'validation.json').read_text()) if (source/'validation.json').is_file() else {}
        report['original_tilt_error_p95_deg'] = original_report.get('tilt_error_p95_deg')
        try:
            corrected, validation = refine(poses, node.matches)
            report.update(validation)
        except ValueError as error:
            report.update(geometry_checks_passed=False, reason=str(error))
            atomic_json(output/'validation.json', report)
            print(json.dumps({k: v for k, v in report.items() if k not in ('overlap_measurements', 'rejected_overlap_measurements')}, ensure_ascii=False), flush=True)
            return 2
        report['status'] = 'GEOMETRY_CHECKS_PASSED_REVIEW_AND_LOCALIZATION_REQUIRED' if validation['geometry_checks_passed'] else 'REJECTED'
        np.save(output/'poses.npy', corrected)
        atomic_json(output/'validation.json', report)
        if not validation['geometry_checks_passed']:
            return 2
        points = build_map(keyframes, corrected)
        write_pcd(output/'map.pcd', points)
        report.update(points=len(points), voxel_m=.1, sha256=hashlib.sha256((output/'map.pcd').read_bytes()).hexdigest())
        atomic_json(output/'validation.json', report)
        fig, axs = plt.subplots(1, 3, figsize=(18, 6), layout='constrained')
        sample = points[::max(1, len(points)//350000)]
        axs[0].scatter(sample[:, 0], sample[:, 1], c=sample[:, 2], s=.1, cmap='turbo', vmin=-1, vmax=5)
        axs[0].plot(corrected[:, 0, 3], corrected[:, 1, 3], 'k-', lw=.6)
        axs[0].axis('equal')
        axs[0].set(title='Reprocessed captured map', xlabel='X m', ylabel='Y m')
        axs[1].plot(poses[:, 2, 3], label='Before refinement')
        axs[1].plot(corrected[:, 2, 3], label='After refinement')
        axs[1].legend()
        axs[1].set(title='Height; endpoint not forced to origin', xlabel='Keyframe', ylabel='Z m')
        axs[2].scatter(sample[:, 0], sample[:, 2], s=.1)
        axs[2].set(title='Side view', xlabel='X m', ylabel='Z m', ylim=(-3, 10))
        fig.savefig(output/'map_overview.png', dpi=140)
        plt.close(fig)
        print(json.dumps({k: v for k, v in report.items() if k not in ('overlap_measurements', 'rejected_overlap_measurements', 'training_loops', 'heldout_validation')}, ensure_ascii=False), flush=True)
        return 0
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
