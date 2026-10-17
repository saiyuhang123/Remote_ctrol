from pathlib import Path

import pytest

from robot_patrol.models import PatrolError
from robot_patrol.store import PatrolStore


@pytest.fixture()
def store(tmp_path):
    instance = PatrolStore(tmp_path / "patrol.sqlite3")
    yield instance
    instance.close()


def map_data(navigation_yaml="/tmp/map.yaml", name="map"):
    return {
        "name": name,
        "backend": "amcl",
        "frame_id": "map",
        "navigation_yaml": navigation_yaml,
        "image_path": "/tmp/map.pgm",
        "localization_file": "",
        "resolution": 0.05,
        "origin_x": -1.0,
        "origin_y": -2.0,
        "origin_yaw": 0.0,
        "width": 100,
        "height": 80,
        "navigation_yaml_sha256": "a" * 64,
        "image_sha256": "b" * 64,
        "localization_sha256": "",
        "verified": True,
    }


def test_map_and_point_revision_round_trip(store):
    mapping = store.upsert_map(map_data())
    assert mapping.revision == 1
    point = store.upsert_point("", mapping.map_id, "A", 1.0, 2.0, 0.3, "manual", 123)
    assert point.revision == 1
    assert store.list_points(mapping.map_id)[0].name == "A"
    updated = store.upsert_point(point.point_id, mapping.map_id, "A2", 2.0, 3.0, 0.4,
                                 "manual", 456, expected_revision=1)
    assert updated.revision == 2
    with pytest.raises(PatrolError) as error:
        store.upsert_point(point.point_id, mapping.map_id, "bad", 0, 0, 0,
                           "manual", 789, expected_revision=1)
    assert error.value.code == "STALE_DATA"


def test_route_order_duplicate_points_and_reference_delete_guard(store):
    mapping = store.upsert_map(map_data())
    a = store.upsert_point("", mapping.map_id, "A", 0, 0, 0, "manual", 1)
    b = store.upsert_point("", mapping.map_id, "B", 1, 1, 0, "manual", 2)
    route = store.upsert_route("", mapping.map_id, "route",
                               [type("S", (), {"point_id": a.point_id, "dwell_seconds": 0})(),
                                type("S", (), {"point_id": b.point_id, "dwell_seconds": 2})(),
                                type("S", (), {"point_id": a.point_id, "dwell_seconds": 0})()])
    assert [step.point_id for step in route.steps] == [a.point_id, b.point_id, a.point_id]
    assert [step.step_order if hasattr(step, "step_order") else step.order for step in route.steps] == [0, 1, 2]
    with pytest.raises(PatrolError):
        store.delete_point(a.point_id)
    store.delete_route(route.route_id, expected_revision=1)
    store.delete_point(a.point_id, expected_revision=1)


def test_point_update_bumps_referencing_route_revision(store):
    mapping = store.upsert_map(map_data())
    point = store.upsert_point("", mapping.map_id, "A", 1.0, 2.0, 0.1, "manual", 1)
    route = store.upsert_route("", mapping.map_id, "route",
                               [type("S", (), {"point_id": point.point_id, "dwell_seconds": 0})()])
    assert route.revision == 1
    store.upsert_point(point.point_id, mapping.map_id, "A", 9.0, 2.0, 0.1,
                       "manual", 2, expected_revision=1)
    refreshed = store.get_route(route.route_id)
    assert refreshed.revision == 2
    assert refreshed.steps[0].x == 9.0


def test_map_geometry_change_invalidates_old_points_and_routes(store):
    data = map_data()
    mapping = store.upsert_map(data)
    point = store.upsert_point("", mapping.map_id, "A", 1.0, 2.0, 0.0, "manual", 1)
    store.upsert_route("", mapping.map_id, "route",
                       [type("S", (), {"point_id": point.point_id, "dwell_seconds": 0})()])
    changed = map_data()
    changed["origin_x"] = 100.0
    changed["origin_y"] = 200.0
    updated = store.upsert_map(changed)
    assert updated.map_id == mapping.map_id
    assert updated.revision == 2
    assert store.list_points(mapping.map_id) == []
    assert store.list_routes(mapping.map_id) == []


def test_cross_map_point_reference_rejected(store):
    first = store.upsert_map(map_data("/tmp/a.yaml", "a"))
    second = store.upsert_map(map_data("/tmp/b.yaml", "b"))
    point = store.upsert_point("", first.map_id, "A", 0, 0, 0, "manual", 1)
    with pytest.raises(PatrolError) as error:
        store.upsert_route("", second.map_id, "bad",
                           [type("S", (), {"point_id": point.point_id, "dwell_seconds": 0})()])
    assert error.value.code == "MAP_MISMATCH"
