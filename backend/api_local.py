"""Read-only local business APIs for the web MVP."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any

from fastapi import APIRouter, HTTPException, Query
from fastapi.responses import JSONResponse

from . import db

router = APIRouter(tags=["local"])


def _db_error(error: Exception) -> HTTPException:
    return HTTPException(status_code=503, detail=str(error))


def _read_meta() -> tuple[dict[str, Any] | None, str]:
    path = db.WEB_MAP_DIR / "meta.json"
    if not path.is_file():
        return None, "尚未导出网页地图"
    try:
        meta = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        return None, f"网页地图配置无法读取：{error}"
    if (not isinstance(meta, dict)
            or not isinstance(meta.get("image"), str)
            or not isinstance(meta.get("origin"), list)
            or len(meta.get("origin", [])) < 3
            or not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                       and math.isfinite(float(v)) for v in meta["origin"][:3])
            or not isinstance(meta.get("resolution"), (int, float))
            or isinstance(meta.get("resolution"), bool)
            or not math.isfinite(float(meta["resolution"]))
            or float(meta["resolution"]) <= 0
            or not isinstance(meta.get("width"), int)
            or isinstance(meta.get("width"), bool)
            or int(meta["width"]) <= 0
            or not isinstance(meta.get("height"), int)
            or isinstance(meta.get("height"), bool)
            or int(meta["height"]) <= 0
            or not isinstance(meta.get("map_id", ""), str)
            or not isinstance(meta.get("revision", 0), int)
            or isinstance(meta.get("revision", 0), bool)
            or int(meta.get("revision", 0)) < 0):
        return None, "网页地图配置字段无效"
    image_name = Path(meta["image"]).name
    if image_name != meta["image"] or not image_name:
        return None, "网页地图图片名无效"
    image_path = db.WEB_MAP_DIR / image_name
    if not image_path.is_file():
        return None, "网页地图图片不存在"
    expected = str(meta.get("sha256") or "")
    if expected:
        try:
            actual = hashlib.sha256(image_path.read_bytes()).hexdigest()
        except OSError as error:
            return None, f"网页地图图片无法读取：{error}"
        if actual != expected:
            return None, "网页地图图片与 meta.sha256 不一致"
    return meta, ""


def _db_active_map() -> tuple[dict[str, Any] | None, str]:
    state = db.query_one(
        "SELECT map_id FROM active_map_state WHERE singleton = 1")
    if state is None:
        return None, "本地尚未设置活跃地图"
    row = db.query_one("SELECT * FROM maps WHERE map_id = ?", (state["map_id"],))
    if row is None:
        return None, "活跃地图记录已不存在"
    return _map_out(row), ""


def _web_sync_state(active: dict[str, Any] | None,
                    meta: dict[str, Any] | None,
                    meta_error: str = "") -> tuple[bool, str]:
    if active is None:
        return False, meta_error or "本地尚未设置活跃地图"
    if meta is None:
        return False, meta_error or "尚未导出活跃网页地图"
    map_id = str(meta.get("map_id") or "")
    revision = int(meta.get("revision") or 0)
    if not map_id or revision <= 0:
        return False, "网页地图尚未关联 map_id/revision"
    if map_id != active["map_id"] or revision != int(active["revision"]):
        return False, (
            f"网页地图 {map_id} r{revision} 与活跃地图 "
            f"{active['map_id']} r{active['revision']} 不一致")
    return True, ""


def _map_out(row: dict[str, Any]) -> dict[str, Any]:
    return {
        "map_id": row["map_id"],
        "name": row["name"],
        "backend": row["backend"],
        "frame_id": row["frame_id"],
        "navigation_yaml": row["navigation_yaml"],
        "image_path": row["image_path"],
        "localization_file": row["localization_file"],
        "resolution": row["resolution"],
        "origin_x": row["origin_x"],
        "origin_y": row["origin_y"],
        "origin_yaw": row["origin_yaw"],
        "width": row["width"],
        "height": row["height"],
        "navigation_yaml_sha256": row["navigation_yaml_sha256"],
        "image_sha256": row["image_sha256"],
        "localization_sha256": row["localization_sha256"],
        "verified": bool(row["verified"]),
        "revision": row["revision"],
        "created_at": row["created_at"] / 1_000_000_000.0,
        "updated_at": row["updated_at"] / 1_000_000_000.0,
    }


def _home_out(row: dict[str, Any]) -> dict[str, Any]:
    return {
        "map_id": row["map_id"],
        "map_revision": row["map_revision"],
        "x": row["x"], "y": row["y"], "yaw": row["yaw"],
        "updated_at": row["updated_at"] / 1_000_000_000.0,
    }


def _valid_home(map_id: str, map_revision: int) -> dict[str, Any] | None:
    row = db.query_one("SELECT * FROM home_points WHERE map_id = ?", (map_id,))
    if row is None or int(row["map_revision"]) != int(map_revision):
        return None
    return _home_out(row)


def _routes_for(map_id: str, map_revision: int, map_sync_ok: bool) -> list[dict[str, Any]]:
    route_rows = db.query(
        "SELECT * FROM routes WHERE map_id = ? ORDER BY updated_at DESC", (map_id,))
    result: list[dict[str, Any]] = []
    for route in route_rows:
        steps = db.query(
            """SELECT s.step_id, s.step_order, s.point_id, s.dwell_seconds,
                      p.name AS point_name, p.x, p.y, p.yaw
               FROM route_steps s JOIN points p ON p.point_id = s.point_id
               WHERE s.route_id = ? ORDER BY s.step_order""", (route["route_id"],))
        mode = route.get('navigation_mode', 'FREE_NAV')
        fixed = mode == 'RECORDED_NETWORK'
        valid = map_sync_ok and bool(steps) and not fixed
        result.append({
            "route_id": route["route_id"],
            "map_id": route["map_id"],
            "name": route["name"],
            "navigation_mode": mode,
            "network_id": route.get('network_id', ''),
            "network_revision": route.get('network_revision', 0),
            "revision": route["revision"],
            "map_revision": map_revision,
            "updated_at": route["updated_at"] / 1_000_000_000.0,
            "steps": [{
                "step_id": row["step_id"],
                "step_order": row["step_order"],
                "point_id": row["point_id"],
                "point_name": row["point_name"],
                "x": row["x"], "y": row["y"], "yaw": row["yaw"],
                "dwell_seconds": row["dwell_seconds"],
            } for row in steps],
            "valid": valid,
            "invalid_reason": "固定路线请在本地客户端预览并启动；网页暂仅查看" if fixed else "" if valid else (
                "网页地图与本地地图版本不一致" if not map_sync_ok else "路线没有可执行步骤"),
        })
    return result


@router.get("/api/health")
def health() -> dict[str, Any]:
    result: dict[str, Any] = {"ok": True, "ts": __import__("time").time(),
                              "database": str(db.DB_PATH),
                              "web_map_dir": str(db.WEB_MAP_DIR)}
    try:
        result["schema_version"] = db.schema_version()
        result["database_ok"] = True
    except Exception as error:  # noqa: BLE001 - health must explain failure
        result["database_ok"] = False
        result["database_error"] = str(error)
    meta, meta_error = _read_meta()
    result["map_ok"] = meta is not None
    if meta is not None:
        result["map_id"] = meta.get("map_id", "")
        result["map_revision"] = meta.get("revision", 0)
    elif meta_error:
        result["map_error"] = meta_error
    try:
        active, active_error = _db_active_map()
        result["active_map_id"] = active["map_id"] if active else ""
        sync_ok, sync_error = _web_sync_state(active, meta, meta_error)
        result["map_sync_ok"] = sync_ok
        if sync_error:
            result["map_sync_error"] = sync_error
    except Exception as error:  # noqa: BLE001 - health is diagnostic
        result["active_map_error"] = str(error)
    return result


@router.get("/api/map/meta")
def map_meta():
    meta, error = _read_meta()
    if meta is None:
        raise HTTPException(status_code=404, detail=error)
    meta = dict(meta)
    try:
        db.schema_version()
        active, active_error = _db_active_map()
        sync_ok, sync_error = _web_sync_state(active, meta, error)
    except Exception as schema_error:  # noqa: BLE001 - expose a safe block reason
        sync_ok, sync_error = False, str(schema_error)
    meta["sync_valid"] = bool(sync_ok)
    if not sync_ok:
        meta["sync_error"] = sync_error
    return JSONResponse(meta, headers={"Cache-Control": "no-store, max-age=0"})


@router.get("/api/local/maps")
def list_maps() -> dict[str, Any]:
    try:
        db.schema_version()
        return {"items": [_map_out(row) for row in db.query(
            "SELECT * FROM maps ORDER BY updated_at DESC")]}
    except Exception as error:  # noqa: BLE001
        raise _db_error(error)


@router.get("/api/local/maps/active")
def active_map() -> dict[str, Any]:
    try:
        db.schema_version()
        active, active_error = _db_active_map()
        if active is None:
            raise HTTPException(status_code=404, detail=active_error)
        meta, meta_error = _read_meta()
        sync_ok, sync_error = _web_sync_state(active, meta, meta_error)
        return {"map": active, "web_meta": meta, "map_sync_ok": sync_ok,
                "sync_error": sync_error}
    except HTTPException:
        raise
    except Exception as error:  # noqa: BLE001
        raise _db_error(error)


@router.get("/api/local/state")
def local_state() -> dict[str, Any]:
    try:
        db.schema_version()
        maps = [_map_out(row) for row in db.query(
            "SELECT * FROM maps ORDER BY updated_at DESC")]
        meta, meta_error = _read_meta()
        active, active_error = _db_active_map()
        map_sync_ok, sync_error = _web_sync_state(active, meta, meta_error)
        home = _valid_home(active["map_id"], int(active["revision"])) if active else None
        routes = (_routes_for(active["map_id"], int(active["revision"]), map_sync_ok)
                  if active else [])
        return {
            "schema_version": db.SUPPORTED_SCHEMA_VERSION,
            "maps": maps,
            "active_map": active,
            "active_error": (active_error if active is None else sync_error),
            "map_sync_ok": map_sync_ok,
            "sync_error": sync_error,
            "map_meta": meta,
            "home": home,
            "routes": routes,
        }
    except Exception as error:  # noqa: BLE001
        raise _db_error(error)


@router.get("/api/local/maps/{map_id}/routes")
def map_routes(map_id: str) -> dict[str, Any]:
    try:
        db.schema_version()
        row = db.query_one("SELECT * FROM maps WHERE map_id = ?", (map_id,))
        if row is None:
            raise HTTPException(status_code=404, detail="地图不存在")
        meta, meta_error = _read_meta()
        active, _ = _db_active_map()
        web_ok, web_error = _web_sync_state(active, meta, meta_error)
        ok = bool(web_ok and active is not None and active["map_id"] == map_id and
                  int(active["revision"]) == int(row["revision"]))
        return {"map_id": map_id, "map_revision": row["revision"],
                "map_sync_ok": ok,
                "sync_error": "" if ok else (web_error or "网页地图与本地地图版本不一致"),
                "items": _routes_for(map_id, int(row["revision"]), ok)}
    except HTTPException:
        raise
    except Exception as error:  # noqa: BLE001
        raise _db_error(error)


@router.get("/api/local/maps/{map_id}/home")
def map_home(map_id: str) -> dict[str, Any]:
    try:
        db.schema_version()
        row = db.query_one("SELECT * FROM maps WHERE map_id = ?", (map_id,))
        if row is None:
            raise HTTPException(status_code=404, detail="地图不存在")
        home = _valid_home(map_id, int(row["revision"]))
        if home is None:
            raise HTTPException(status_code=404, detail="当前地图尚未设置有效返航点")
        return home
    except HTTPException:
        raise
    except Exception as error:  # noqa: BLE001
        raise _db_error(error)


@router.get("/api/inspection/records")
def inspection_records(limit: int = Query(100, ge=1, le=500)) -> dict[str, Any]:
    try:
        db.schema_version()
        rows = db.query(
            "SELECT * FROM inspection_summary "
            "ORDER BY inspection_date DESC, updated_at DESC LIMIT ?", (limit,))
        items = []
        for row in rows:
            try:
                plates = json.loads(row["plates_json"] or "[]")
            except ValueError:
                plates = []
            normalized = []
            for item in plates if isinstance(plates, list) else []:
                if isinstance(item, str):
                    normalized.append({"plate": item, "count": 1})
                elif isinstance(item, dict) and item.get("plate"):
                    try:
                        count = max(1, int(item.get("count", 1)))
                    except (TypeError, ValueError):
                        count = 1
                    normalized.append({"plate": str(item["plate"]), "count": count})
            items.append({
                "run_id": row["run_id"],
                "inspection_date": row["inspection_date"],
                "map_id": row["map_id"],
                "route_id": row["route_id"],
                "face_capture_count": row["face_capture_count"],
                "person_count": row["person_count"],
                "plates": normalized,
                "updated_at": row["updated_at"] / 1_000_000_000.0,
            })
        return {"items": items}
    except Exception as error:  # noqa: BLE001
        raise _db_error(error)
