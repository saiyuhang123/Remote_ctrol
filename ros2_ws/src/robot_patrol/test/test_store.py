from pathlib import Path

import pytest

from robot_patrol.models import PatrolError
from robot_patrol.store import PatrolStore, SCHEMA_VERSION


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


def test_schema_v2_home_point_round_trip_and_revision_invalidation(store):
    mapping = store.upsert_map(map_data())
    assert store.get_home_point(mapping.map_id) is None
    home = store.set_home_point(mapping.map_id, 1, 1.25, -2.5, 0.75)
    assert home.map_revision == 1
    loaded = store.get_home_point(mapping.map_id)
    assert loaded is not None
    assert (loaded.x, loaded.y, loaded.yaw) == (1.25, -2.5, 0.75)
    with store._conn:
        version = store._conn.execute("SELECT version FROM schema_version").fetchone()[0]
    assert version == SCHEMA_VERSION
    # Any map revision change invalidates coordinates from the old map.
    store.upsert_map(map_data())
    assert store.get_home_point(mapping.map_id) is None
    store.set_home_point(mapping.map_id, 2, 0.0, 0.0, 0.0)
    store.clear_home_point(mapping.map_id)
    assert store.get_home_point(mapping.map_id) is None
    store.set_active_map(mapping.map_id)
    assert store.get_active_map_id() == mapping.map_id
    store.upsert_map(map_data())
    assert store.get_active_map_id() == ""


def test_inspection_summary_deduplicates_and_counts_plates(store):
    summary = store.upsert_inspection_summary(
        "run_1", "2026-10-18", "map_1", "route_1", 3, 5,
        ["京A12345", {"plate": "京B67890", "count": 2}, "京A12345"])
    assert summary.face_capture_count == 3
    assert summary.person_count == 5
    assert summary.plates == [
        {"plate": "京A12345", "count": 2},
        {"plate": "京B67890", "count": 2},
    ]
    store.upsert_inspection_summary(
        "run_1", "2026-10-18", "map_1", "route_1", 4, 6,
        [{"plate": "京A12345", "count": 3}])
    items = store.list_inspection_summaries()
    assert len(items) == 1
    assert items[0].face_capture_count == 4
    assert items[0].plates == [{"plate": "京A12345", "count": 3}]


def test_v1_database_is_migrated_without_losing_maps(tmp_path):
    path = tmp_path / "old.sqlite3"
    old = PatrolStore(path)
    mapping = old.upsert_map(map_data())
    with old._conn:
        old._conn.execute("DROP TABLE inspection_summary")
        old._conn.execute("DROP TABLE home_points")
        old._conn.execute("DROP TABLE active_map_state")
        old._conn.execute("UPDATE schema_version SET version = 1")
    old.close()

    migrated = PatrolStore(path)
    try:
        assert migrated.get_map(mapping.map_id).name == "map"
        with migrated._conn:
            assert migrated._conn.execute(
                "SELECT version FROM schema_version").fetchone()[0] == SCHEMA_VERSION
            names = {row[0] for row in migrated._conn.execute(
                "SELECT name FROM sqlite_master WHERE type='table'")}
        assert {"home_points", "inspection_summary", "active_map_state"} <= names
    finally:
        migrated.close()
