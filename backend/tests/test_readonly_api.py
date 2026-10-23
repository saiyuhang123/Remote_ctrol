import json
import os
from pathlib import Path
import sys

from asgi_client import ASGIClient
from PIL import Image


def _build_fixture(tmp_path, monkeypatch):
    root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(root / "ros2_ws" / "src" / "robot_patrol"))
    from robot_patrol.store import PatrolStore

    database = tmp_path / "patrol.sqlite3"
    store = PatrolStore(database)
    mapping = store.upsert_map({
        "name": "test-map", "backend": "fastlio", "frame_id": "map",
        "navigation_yaml": str(tmp_path / "map.yaml"),
        "image_path": str(tmp_path / "map.pgm"), "localization_file": "",
        "resolution": 0.05, "origin_x": -1.0, "origin_y": -2.0,
        "origin_yaw": 0.3, "width": 20, "height": 10,
        "navigation_yaml_sha256": "a" * 64, "image_sha256": "b" * 64,
        "localization_sha256": "", "verified": True,
    })
    point = store.upsert_point("", mapping.map_id, "P", 0.5, 0.25, 0.1,
                               "manual", 1)
    route = store.upsert_route(
        "", mapping.map_id, "R",
        [type("Step", (), {"point_id": point.point_id, "dwell_seconds": 1.0})()])
    store.set_home_point(mapping.map_id, mapping.revision, 0.0, 0.0, 0.0)
    store.set_active_map(mapping.map_id)
    store.upsert_inspection_summary("run_1", "2026-10-18", mapping.map_id,
                                    route.route_id, 2, 3,
                                    [{"plate": "京A12345", "count": 2}])
    store.close()

    web_maps = tmp_path / "web_maps"
    web_maps.mkdir()
    Image.new("L", (20, 10), 205).save(web_maps / "active.png")
    import hashlib
    checksum = hashlib.sha256((web_maps / "active.png").read_bytes()).hexdigest()
    (web_maps / "meta.json").write_text(json.dumps({
        "schema_version": 1, "map_id": mapping.map_id,
        "revision": mapping.revision, "backend": "fastlio",
        "image": "active.png", "resolution": 0.05,
        "origin": [-1.0, -2.0, 0.3], "width": 20, "height": 10,
        "sha256": checksum, "updated_at": 1,
    }))
    monkeypatch.setenv("ROBOT_PATROL_DB", str(database))
    monkeypatch.setenv("ROBOT_WEB_MAP_DIR", str(web_maps))
    # backend.db resolves env at import time; load the modules once per fixture.
    for name in [name for name in sys.modules if name == "backend" or name.startswith("backend.")]:
        del sys.modules[name]
    from backend.main import app
    return ASGIClient(app), mapping, route


def test_read_only_local_state_and_old_write_routes_absent(tmp_path, monkeypatch):
    client, mapping, route = _build_fixture(tmp_path, monkeypatch)
    health = client.get("/api/health")
    assert health.status_code == 200
    assert health.json()["database_ok"] is True
    from robot_patrol.store import SCHEMA_VERSION
    assert health.json()["schema_version"] == SCHEMA_VERSION

    state = client.get("/api/local/state")
    assert state.status_code == 200
    payload = state.json()
    assert payload["active_map"]["map_id"] == mapping.map_id
    assert payload["map_sync_ok"] is True
    assert payload["home"]["x"] == 0.0
    assert payload["routes"][0]["route_id"] == route.route_id
    assert payload["routes"][0]["valid"] is True

    active_response = client.get("/api/local/maps/active")
    assert active_response.status_code == 200
    assert active_response.json()["map"]["map_id"] == mapping.map_id
    assert active_response.json()["map_sync_ok"] is True
    map_routes = client.get(f"/api/local/maps/{mapping.map_id}/routes").json()
    assert map_routes["map_sync_ok"] is True
    assert map_routes["items"][0]["route_id"] == route.route_id
    map_home = client.get(f"/api/local/maps/{mapping.map_id}/home")
    assert map_home.status_code == 200
    assert map_home.json()["map_revision"] == mapping.revision

    records = client.get("/api/inspection/records")
    assert records.status_code == 200
    assert records.json()["items"][0]["plates"] == [
        {"plate": "京A12345", "count": 2}]

    meta_response = client.get("/api/map/meta")
    assert meta_response.status_code == 200
    meta_data = meta_response.json()
    assert meta_data["origin"] == [-1.0, -2.0, 0.3]
    assert meta_data["sync_valid"] is True
    assert "no-store" in meta_response.headers["cache-control"]

    # A stale web revision must disable every route and, by revision mismatch,
    # the home point remains invalid to task callers.
    stale = dict(meta_data)
    stale["revision"] = mapping.revision + 1
    stale.pop("sync_valid", None)
    (tmp_path / "web_maps" / "meta.json").write_text(json.dumps(stale))
    stale_state = client.get("/api/local/state").json()
    assert stale_state["map_sync_ok"] is False
    assert stale_state["active_map"]["map_id"] == mapping.map_id
    assert stale_state["routes"][0]["valid"] is False
    assert "不一致" in stale_state["active_error"]

    # Old web CRUD endpoints are intentionally not mounted.
    assert client.post("/api/points", json={"name": "x", "x": 0, "y": 0}).status_code in (404, 405)
    assert client.post("/api/routes", json={"name": "x", "point_ids": []}).status_code in (404, 405)
    assert client.post("/api/plans", json={}).status_code in (404, 405)
    assert client.get("/api/audio").status_code in (404, 405)


def test_fixed_routes_are_selectable_and_require_a_fresh_preview(tmp_path, monkeypatch):
    client, mapping, _ = _build_fixture(tmp_path, monkeypatch)
    from robot_patrol.store import PatrolStore
    from robot_patrol.recorded_routes.geometry import motion_blocks
    from types import SimpleNamespace
    store=PatrolStore(tmp_path/'patrol.sqlite3')
    point=store.list_points(mapping.map_id)[0]
    a=dict(id='a',name='A',x=point.x,y=point.y,yaw=.1)
    b=dict(id='b',name='B',x=point.x+1.,y=point.y+.1,yaw=.1)
    samples=[dict(x=a['x']+i/50,y=a['y']+.1*i/50,yaw=.1) for i in range(51)]
    doc=dict(schema_version=1,nodes={'a':a,'b':b},edges={'e':dict(id='e',name='路',**{'from':'a','to':'b'},speed=.2,blocks=motion_blocks(samples))},bindings={point.point_id:['a']})
    net=store.save_network(mapping.map_id,'道路',doc,published=True)
    fixed=store.upsert_route('',mapping.map_id,'固定', [SimpleNamespace(point_id=point.point_id,dwell_seconds=0)],
        navigation_mode='RECORDED_NETWORK',network_id=net['network_id'],network_revision=1)
    store.close()
    items=client.get('/api/local/state').json()['routes']
    value=next(item for item in items if item['route_id']==fixed.route_id)
    assert value['navigation_mode']=='RECORDED_NETWORK'
    assert value['network_id']==net['network_id']
    assert value['valid'] is True and value['requires_preview'] is True
    assert not value['invalid_reason']
    with __import__('sqlite3').connect(tmp_path/'patrol.sqlite3') as connection:
        connection.execute('UPDATE road_networks SET published=0')
    value=next(item for item in client.get('/api/local/state').json()['routes'] if item['route_id']==fixed.route_id)
    assert not value['valid'] and '路网' in value['invalid_reason']
