"""Exercise observable failure modes rather than the implementation structure."""
import json
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation
import pytest

from fastlio_loop_closure.geometry import (inverse, level_rotation, overlap_is_usable,
                                         refine, write_pcd)


def graph_fixture():
    # A rectangle with a real continuation beyond the starting point. The
    # endpoint must not be set to the origin, and road inclinations must survive.
    path = np.array([[0, 0], [8, 0], [8, 6], [0, 6], [0, 0], [0, -4]], dtype=float)
    xy = np.concatenate([np.linspace(a, b, 8, endpoint=False) for a, b in zip(path[:-1], path[1:])])
    xy = np.vstack((xy, path[-1]))
    n = len(xy)
    truth = np.tile(np.eye(4), (n, 1, 1))
    truth[:, :2, 3] = xy
    truth[:, 2, 3] = .1*np.sin(np.arange(n)*.1)
    euler = np.column_stack((.01*np.sin(np.arange(n)*.2),
                            .02*np.cos(np.arange(n)*.2), np.zeros(n)))
    truth[:, :3, :3] = Rotation.from_euler('xyz', euler).as_matrix()
    raw = truth.copy()
    raw[:, 0, 3] += np.linspace(0, .8, n)
    matches = []
    for source, target in zip(range(29, 39), range(10)):
        measurement = inverse(truth[target]) @ truth[source]
        matches.append({'source': source, 'target': target,
                        'measurement_target_from_source': measurement.tolist()})
    return raw, truth, matches


def test_measured_nonidentity_loops_preserve_slopes_and_true_endpoint():
    raw, truth, matches = graph_fixture()
    corrected, report = refine(raw, matches)
    assert report['geometry_checks_passed'], json.dumps(report)
    np.testing.assert_allclose(corrected[0], raw[0], atol=1e-9)
    before = Rotation.from_matrix(raw[:, :3, :3]).as_euler('xyz')
    after = Rotation.from_matrix(corrected[:, :3, :3]).as_euler('xyz')
    np.testing.assert_allclose(before[:, :2], after[:, :2], atol=1e-10)
    assert np.linalg.norm(corrected[-1, :2, 3]) > 3.5
    assert np.linalg.norm(corrected[-1, :3, 3]-truth[-1, :3, 3]) < .5*np.linalg.norm(raw[-1, :3, 3]-truth[-1, :3, 3])


def test_one_loop_cannot_be_exported_as_validated_map():
    raw, _, matches = graph_fixture()
    with pytest.raises(ValueError, match='6'):
        refine(raw, matches[:1])


def test_conflicting_holdout_prevents_export():
    raw, _, matches = graph_fixture()
    matches[1]['measurement_target_from_source'][0][3] += 3.
    _, report = refine(raw, matches)
    assert not report['geometry_checks_passed']


def test_ground_or_large_correction_cannot_pass_overlap_gate():
    match = {'position_correction_m': [0., 0., 0.], 'rotation_correction_deg': [0., 0., 0.],
             'measurement_target_from_source': np.eye(4).tolist(), 'fitness': .23, 'rmse_m': .1}
    assert not overlap_is_usable(match)
    match.update(fitness=.95, position_correction_m=[0., 0., 10.])
    assert not overlap_is_usable(match)


def test_gravity_alignment_does_not_remove_relative_grade():
    up = Rotation.from_euler('xy', [.1, -.05]).apply([0., 0., 1.])
    level = level_rotation(up)
    np.testing.assert_allclose(level @ up, [0., 0., 1.], atol=1e-10)
    np.testing.assert_allclose(level.T @ level, np.eye(3), atol=1e-10)
    with pytest.raises(ValueError):
        level_rotation([0., 0., -1.])


def test_failed_pcd_commit_retains_previous_map_and_intensity(tmp_path):
    filename = tmp_path/'map.pcd'
    points = np.array([[1., 2., 3., 17.], [4., 5., 6., 21.]], dtype=np.float32)
    write_pcd(filename, points)
    original = filename.read_bytes()
    np.testing.assert_array_equal(np.frombuffer(original.split(b'DATA binary\n')[1], '<f4').reshape(-1, 4), points)
    with patch('fastlio_loop_closure.geometry.os.replace', side_effect=OSError('disk error')):
        with pytest.raises(OSError):
            write_pcd(filename, points+1.)
    assert filename.read_bytes() == original
    assert not filename.with_name('map.pcd.tmp').exists()
