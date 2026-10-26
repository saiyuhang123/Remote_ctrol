"""Saving a useful artifact must not depend on finding a loop."""
import json
import os
from pathlib import Path

os.environ['ROS_DOMAIN_ID'] = '227'
os.environ['ROS_LOCALHOST_ONLY'] = '1'

import numpy as np
import pytest
import rclpy

from fastlio_loop_closure.geometry import Keyframe
from fastlio_loop_closure.live_loop_closure import LiveLoopClosure


@pytest.fixture
def mapping(tmp_path):
    os.environ['ROS_LOG_DIR'] = str(tmp_path/'ros_logs')
    active = tmp_path/'active.pcd'
    active.write_bytes(b'previous navigation map')
    rclpy.init(args=['--ros-args', '-p', f'output_map_path:={active}'])
    node = LiveLoopClosure()
    node.running = False
    node.work_event.set()
    node.worker.join(timeout=5.)
    xyz = np.random.default_rng(42).normal(size=(200, 3)).astype(np.float32)
    for i in range(40):
        pose = np.eye(4)
        pose[0, 3] = .8*i
        node.keyframes.append(Keyframe(i, int(i*1.5e9), pose, xyz, np.ones(len(xyz))))
    node.map_rotation = np.eye(3)
    node.tilt_errors = [.1]*40
    node.total_clouds = 40
    yield node, active
    node.shutdown()
    node.destroy_node()
    rclpy.try_shutdown()


def saved_result(node):
    success, message = node.save_bundle()
    assert success
    payload = json.loads(message)
    prefix = Path(payload['artifact_prefix'])
    report = json.loads(prefix.with_suffix('.validation.json').read_text())
    return payload, prefix, report


def test_short_one_way_map_saves_without_replacing_navigation_map(mapping):
    node, active = mapping
    payload, prefix, report = saved_result(node)
    assert payload['quality'] == 'no_loop'
    assert payload['localization_test_allowed']
    assert not report['geometry_checks_passed']
    assert report['qualified_overlaps'] == 0
    assert prefix.with_suffix('.pcd').stat().st_size > 100
    assert (prefix.parent/'使用说明.txt').is_file()
    assert active.read_bytes() == b'previous navigation map'
    assert not active.with_suffix('.validation.json').exists()


def test_repeated_saves_preserve_earlier_snapshots(mapping):
    node, _ = mapping
    _, first, _ = saved_result(node)
    contents = first.with_suffix('.pcd').read_bytes()
    node.keyframes[-1].pose[1, 3] += 1.
    _, second, _ = saved_result(node)
    assert second != first
    assert first.with_suffix('.pcd').read_bytes() == contents
    assert len(node.matches) == 0


def test_sensor_error_archives_data_with_action_and_disallows_loading(mapping):
    node, active = mapping
    node.fatal_error = '检测到点云时间回退或重复'
    payload, _, report = saved_result(node)
    assert payload['quality'] == 'sensor_error'
    assert not payload['localization_test_allowed']
    assert '时间回退' in report['reason']
    assert '重新建图' in report['next_action']
    assert active.read_bytes() == b'previous navigation map'


def test_rejected_optimizer_never_exports_its_distorted_poses(mapping, monkeypatch):
    node, active = mapping
    node.matches = [{'source': i, 'target': 0} for i in range(30, 36)]
    node.finish_matching = lambda _: None
    def distorted(poses, matches):
        result = poses.copy()
        result[-1, 2, 3] += 100.
        return result, {'geometry_checks_passed': False, 'max_pose_translation_change_m': 100.,
                        'max_pose_rotation_change_deg': 0., 'max_heldout_translation_error_m': 10.}
    monkeypatch.setattr('fastlio_loop_closure.live_loop_closure.refine', distorted)
    payload, prefix, report = saved_result(node)
    assert payload['quality'] == 'geometry_rejected'
    assert not payload['localization_test_allowed']
    assert report['exported_pose_source'] == 'frontend'
    np.testing.assert_allclose(np.load(prefix.parent/'poses.npy')[:, 2, 3], 0.)
    assert active.read_bytes() == b'previous navigation map'
