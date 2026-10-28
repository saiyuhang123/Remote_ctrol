"""Exercise observable failure modes rather than the implementation structure."""
import json
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation
import pytest

from fastlio_loop_closure.geometry import (Keyframe, common_vertical_bias, inverse, level_rotation,
                                         overlap_is_usable, refine, register_overlap, write_pcd)


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


def test_common_vertical_offset_unlocks_a_consistent_loop():
    # Every match carries the same frontend height drift; judged by |dz| alone the
    # whole loop is rejected, but the shared offset is a real correction to make.
    matches = [{'position_correction_m': [.1, -.2, 4.6 + .01*i],
                'rotation_correction_deg': [.1, -.1, .2],
                'measurement_target_from_source': np.eye(4).tolist(),
                'fitness': .94, 'rmse_m': .15} for i in range(4)]
    assert not any(overlap_is_usable(m) for m in matches)
    bias = common_vertical_bias(matches)
    assert bias == pytest.approx(4.615, abs=.02)
    assert all(overlap_is_usable(m, bias) for m in matches)


def test_random_vertical_offsets_are_not_a_common_bias():
    # A false loop closure produces scattered dz, not three matches in one cluster.
    matches = [{'position_correction_m': [0., 0., dz]} for dz in (4.6, .3, -2.0, 4.7)]
    assert common_vertical_bias(matches) is None
    assert common_vertical_bias([{'position_correction_m': [0., 0., 12.]}] * 3) is None


def test_common_vertical_bias_relaxes_the_refine_shift_limit():
    # 40 poses on a straight line; the last six are 4.6 m too low. The loop
    # matches say they are the same places, so the refine must be allowed to
    # shift by ~4.6 m (the old fixed 2.0/3.5 m limit rejected this).
    poses = np.tile(np.eye(4), (40, 1, 1))
    poses[:, 0, 3] = np.arange(40)*.5
    poses[30:, 2, 3] = -4.6
    matches = []
    for k in range(6):
        source, target = 30+k, k
        matches.append({'source': source, 'target': target, 'fitness': .94, 'rmse_m': .15,
                        'position_correction_m': [0., 0., 4.6],
                        'rotation_correction_deg': [0., 0., 0.],
                        'measurement_target_from_source': np.eye(4).tolist()})
    _, report = refine(poses, matches)
    assert report['pose_translation_limit_m'] > 4.6
    assert report['common_z_bias_m'] == pytest.approx(4.6)


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


def verified_match():
    return {'position_correction_m': [.1, -2.4, 1.5],
            'rotation_correction_deg': [0., 0., -.7],
            'measurement_target_from_source': np.eye(4).tolist(),
            'fitness': .72, 'rmse_m': .19, 'inlier_count': 800,
            'ground_residual_m': .03, 'reciprocal_translation_m': .08,
            'reciprocal_rotation_deg': .2, 'initialization_agreements': 3}


def test_partial_overlap_with_height_drift_requires_all_verification():
    match = verified_match()
    assert overlap_is_usable(match)
    for key, value in [('ground_residual_m', .3), ('reciprocal_translation_m', .4),
                       ('reciprocal_rotation_deg', 2.), ('initialization_agreements', 1),
                       ('inlier_count', 100), ('ground_residual_m', None)]:
        assert not overlap_is_usable(dict(match, **{key: value})), key
    assert not overlap_is_usable(dict(match, fitness=.4))
    assert not overlap_is_usable(dict(match, position_correction_m=[0., 0., 10.]))


def test_many_correlated_partial_matches_at_one_position_are_insufficient():
    raw, _, matches = graph_fixture()
    for m in matches:
        m['fitness'] = .72
        raw[m['source'], :3, 3] = [0., 0., 0.]
    with pytest.raises(ValueError, match='至少需要 6 米'):
        refine(raw, matches)


def test_ground_initialization_recovers_vertical_and_horizontal_drift():
    rng = np.random.default_rng(1729)
    x, y = np.meshgrid(np.linspace(-11, 11, 65), np.linspace(-11, 11, 65))
    parts = [np.column_stack((x.ravel(), y.ravel(), np.full(x.size, -.4)))]
    for cx, cy, width, height in [(4., 5., 2.2, 2.3), (-5., -4., 3.4, 3.1), (-6., 5., 2.8, 1.7)]:
        a, b = np.meshgrid(np.linspace(-width/2, width/2, 30), np.linspace(.15, height, 30))
        parts.append(np.column_stack((a.ravel()+cx, np.full(a.size, cy), b.ravel())))
        parts.append(np.column_stack((np.full(a.size, cx-width/2), a.ravel()+cy, b.ravel())))
        a, b = np.meshgrid(np.linspace(-width/2, width/2, 25), np.linspace(-width/2, width/2, 25))
        parts.append(np.column_stack((a.ravel()+cx, b.ravel()+cy, np.full(a.size, height))))
    xyz = np.concatenate(parts).astype(np.float32)
    intensity = np.ones(len(xyz), dtype=np.float32)
    kfs = [Keyframe(i, i*1_000_000_000, np.eye(4), xyz, intensity) for i in range(7)]
    drifted = np.eye(4)
    drifted[:3, 3] = [0., 2.4, -1.5]
    kfs[-1] = Keyframe(6, 600_000_000_000, drifted, xyz+rng.normal(0, .005, xyz.shape), intensity)
    match = register_overlap(kfs, 6, 0, 6)
    assert overlap_is_usable(match), json.dumps(match)
    recovered = np.asarray(match['measurement_target_from_source'])
    assert np.linalg.norm(recovered[:3, 3]) < .05
    assert np.linalg.norm(np.asarray(match['position_correction_m'])-[0., -2.4, 1.5]) < .05
