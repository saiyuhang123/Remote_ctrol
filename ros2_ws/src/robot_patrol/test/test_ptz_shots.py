import math
from dataclasses import replace
from types import SimpleNamespace as NS

import pytest

from robot_patrol.models import PatrolError, RouteSnapshot
from robot_patrol.ptz_shots import Shot, parse_shots, shots_json
from robot_patrol.store import PatrolStore
from robot_patrol.point_task import point_route
from test_store import map_data


def shot(identifier='wide', **values):
    result = Shot(identifier, identifier, 359., 0., 0., 0., -8., 2., -1, 1, .01)
    return replace(result, **values)


@pytest.mark.parametrize('sign,yaw,expected', [(-1, 2., 1.), (1, 2., 357.), (-1, -2., 357.), (1, -2., 1.)])
def test_yaw_compensation_wraps_in_both_encoder_directions(sign, yaw, expected):
    assert shot(pan_sign=sign).target_pan(math.radians(yaw)) == pytest.approx(expected)


def test_calibrated_yaw_is_used_and_serialization_round_trips():
    calibrated = shot(yaw_cal=math.radians(179.))
    assert calibrated.target_pan(math.radians(-179.)) == pytest.approx(1.)
    assert parse_shots(shots_json([calibrated])) == [calibrated]


@pytest.mark.parametrize('values', [dict(zoom=12.), dict(tilt=-91.), dict(pan_sign=0),
    dict(pan_cal=360.), dict(yaw_cal=90.), dict(x_cal=math.nan), dict(shot_id='../bad'),
    dict(map_revision=0), dict(settle_seconds=-1.)])
def test_invalid_calibration_is_rejected(values):
    with pytest.raises(PatrolError):
        parse_shots([shot(**values)])


def test_only_two_unique_views_with_one_encoder_direction():
    for views in ([shot(), shot()], [shot(), shot('b', pan_sign=1)],
                  [shot(), shot('b'), shot('c')]):
        with pytest.raises(PatrolError):
            parse_shots(views)


def test_point_shots_survive_legacy_edits_and_reach_both_task_paths(tmp_path):
    store = PatrolStore(tmp_path/'db.sqlite3')
    mapping = store.upsert_map(map_data())
    point = store.upsert_point('', mapping.map_id, 'A', 0., 0., 0., 'manual', 1)
    route = store.upsert_route('', mapping.map_id, 'route', [NS(point_id=point.point_id, dwell_seconds=0.)])
    configured = store.replace_point_shots(point.point_id, mapping.map_id, 1, point.revision, [shot(), shot('side')])
    assert configured.inspection_enabled and configured.ptz_preset == 0
    assert store.get_route(route.route_id).revision == route.revision + 1
    frozen = store.get_route(route.route_id).steps
    saved = store.upsert_point(point.point_id, mapping.map_id, 'A renamed', 0., 0., 0., 'manual', 2,
        configured.revision, True, 0)
    assert saved.shots == configured.shots
    request = NS(point_id=point.point_id, point_revision=saved.revision)
    assert point_route(store, request, mapping).steps[0].shots == configured.shots
    snapshot = RouteSnapshot(route.route_id, mapping.map_id, 'route', 1, frozen)
    assert snapshot.as_dict()['steps'][0]['shots'][0]['pan_sign'] == -1
    store.replace_point_shots(point.point_id, mapping.map_id, 1, saved.revision, [])
    assert len(frozen[0].shots) == 2  # Running snapshots cannot change with edits.
    assert not store.get_point(point.point_id).inspection_enabled
    store.close()


def test_shot_updates_require_point_and_map_revision_and_nearby_calibration(tmp_path):
    store = PatrolStore(tmp_path/'db.sqlite3')
    mapping = store.upsert_map(map_data())
    point = store.upsert_point('', mapping.map_id, 'A', 0., 0., 0., 'manual', 1)
    for point_revision, map_revision, view in [(0,1,shot()), (1,2,shot()),
            (1,1,shot(map_revision=2)), (1,1,shot(x_cal=1.))]:
        with pytest.raises(PatrolError):
            store.replace_point_shots(point.point_id, mapping.map_id, map_revision, point_revision, [view])
    assert store.get_point(point.point_id).revision == 1
    store.close()


def test_v6_database_is_backed_up_and_legacy_data_is_preserved(tmp_path):
    path = tmp_path/'db.sqlite3'
    store = PatrolStore(path)
    mapping = store.upsert_map(map_data())
    point = store.upsert_point('', mapping.map_id, 'legacy', 0., 0., 0., 'manual', 1,
                               inspection_enabled=True, ptz_preset=7)
    store._conn.execute('UPDATE schema_version SET version=6')
    store._conn.execute('ALTER TABLE points DROP COLUMN ptz_shots_json')
    for column in ('shot_id', 'shot_name', 'metadata_json'):
        store._conn.execute(f'ALTER TABLE inspection_results DROP COLUMN {column}')
    store.close()
    upgraded = PatrolStore(path)
    restored = upgraded.get_point(point.point_id)
    assert restored.name == 'legacy' and restored.ptz_preset == 7 and not restored.shots
    assert list(tmp_path.glob('*.pre_v6.*.bak'))
    upgraded.close()
