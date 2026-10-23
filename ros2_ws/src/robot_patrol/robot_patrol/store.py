"""SQLite persistence for maps, points, routes and patrol run records."""
from __future__ import annotations

from contextlib import contextmanager
from pathlib import Path
from typing import Any, Iterable
import json
import sqlite3
import threading
import time

from .recorded_routes.storage import RoadStorage

from .models import (
    HomePoint,
    InspectionResult,
    InspectionSummary,
    MapBinding,
    PatrolError,
    Point,
    Route,
    RouteStep,
    RunRecord,
    new_id,
    now_ns,
    validate_inspection_settings,
    validate_name,
    validate_route_steps,
)

SCHEMA_VERSION = 6


class PatrolStore(RoadStorage):
    def __init__(self, database_path: str | Path):
        self.path = str(Path(database_path).expanduser())
        Path(self.path).parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._conn = sqlite3.connect(self.path, check_same_thread=False, isolation_level=None)
        self._conn.row_factory = sqlite3.Row
        self._conn.execute("PRAGMA foreign_keys = ON")
        self._conn.execute("PRAGMA busy_timeout = 5000")
        self._conn.execute("PRAGMA journal_mode = WAL")
        if self._conn.execute("SELECT 1 FROM sqlite_master WHERE name='schema_version'").fetchone():
            previous = self._conn.execute('SELECT version FROM schema_version LIMIT 1').fetchone()
            if previous and int(previous[0]) > SCHEMA_VERSION:
                self._conn.close()
                raise RuntimeError('Database schema is newer than supported')
            if previous and int(previous[0]) < SCHEMA_VERSION:
                backup = sqlite3.connect(self.path + f'.pre_v{int(previous[0])}.{time.time_ns()}.bak')
                try:
                    self._conn.backup(backup)
                finally:
                    backup.close()
        self._initialize()

    def close(self) -> None:
        with self._lock:
            self._conn.close()

    @contextmanager
    def _transaction(self):
        with self._lock:
            try:
                self._conn.execute("BEGIN IMMEDIATE")
                yield self._conn
                self._conn.execute("COMMIT")
            except Exception:
                try:
                    self._conn.execute("ROLLBACK")
                except sqlite3.Error:
                    pass
                raise

    def _initialize(self) -> None:
        with self._lock:
            self._conn.executescript(
                """
                CREATE TABLE IF NOT EXISTS schema_version (
                    version INTEGER NOT NULL
                );
                INSERT INTO schema_version(version)
                    SELECT 1 WHERE NOT EXISTS (SELECT 1 FROM schema_version);

                CREATE TABLE IF NOT EXISTS maps (
                    map_id TEXT PRIMARY KEY,
                    name TEXT NOT NULL,
                    backend TEXT NOT NULL,
                    frame_id TEXT NOT NULL,
                    navigation_yaml TEXT NOT NULL UNIQUE,
                    image_path TEXT NOT NULL,
                    localization_file TEXT NOT NULL DEFAULT '',
                    resolution REAL NOT NULL,
                    origin_x REAL NOT NULL,
                    origin_y REAL NOT NULL,
                    origin_yaw REAL NOT NULL,
                    width INTEGER NOT NULL,
                    height INTEGER NOT NULL,
                    navigation_yaml_sha256 TEXT NOT NULL,
                    image_sha256 TEXT NOT NULL,
                    localization_sha256 TEXT NOT NULL DEFAULT '',
                    verified INTEGER NOT NULL DEFAULT 1,
                    revision INTEGER NOT NULL DEFAULT 1,
                    created_at INTEGER NOT NULL,
                    updated_at INTEGER NOT NULL
                );

                CREATE TABLE IF NOT EXISTS points (
                    point_id TEXT PRIMARY KEY,
                    map_id TEXT NOT NULL REFERENCES maps(map_id) ON DELETE CASCADE,
                    name TEXT NOT NULL,
                    x REAL NOT NULL,
                    y REAL NOT NULL,
                    yaw REAL NOT NULL,
                    source TEXT NOT NULL DEFAULT '',
                    sampled_at INTEGER NOT NULL,
                    revision INTEGER NOT NULL DEFAULT 1,
                    created_at INTEGER NOT NULL,
                    updated_at INTEGER NOT NULL,
                    inspection_enabled INTEGER NOT NULL DEFAULT 0,
                    ptz_preset INTEGER NOT NULL DEFAULT 0,
                    recognition_prompt TEXT NOT NULL DEFAULT ''
                );
                CREATE INDEX IF NOT EXISTS idx_points_map ON points(map_id);

                CREATE TABLE IF NOT EXISTS routes (
                    route_id TEXT PRIMARY KEY,
                    map_id TEXT NOT NULL REFERENCES maps(map_id) ON DELETE CASCADE,
                    name TEXT NOT NULL,
                    revision INTEGER NOT NULL DEFAULT 1,
                    created_at INTEGER NOT NULL,
                    updated_at INTEGER NOT NULL
                );
                CREATE INDEX IF NOT EXISTS idx_routes_map ON routes(map_id);

                CREATE TABLE IF NOT EXISTS route_steps (
                    step_id TEXT PRIMARY KEY,
                    route_id TEXT NOT NULL REFERENCES routes(route_id) ON DELETE CASCADE,
                    step_order INTEGER NOT NULL,
                    point_id TEXT NOT NULL REFERENCES points(point_id),
                    dwell_seconds REAL NOT NULL,
                    UNIQUE(route_id, step_order)
                );
                CREATE INDEX IF NOT EXISTS idx_route_steps_route ON route_steps(route_id);

                CREATE TABLE IF NOT EXISTS runs (
                    run_id TEXT PRIMARY KEY,
                    route_id TEXT NOT NULL,
                    map_id TEXT NOT NULL,
                    route_revision INTEGER NOT NULL,
                    map_revision INTEGER NOT NULL,
                    route_snapshot_json TEXT NOT NULL,
                    loop_count INTEGER NOT NULL,
                    state INTEGER NOT NULL,
                    success INTEGER NOT NULL DEFAULT 0,
                    completed_loops INTEGER NOT NULL DEFAULT 0,
                    completed_steps INTEGER NOT NULL DEFAULT 0,
                    error_code TEXT NOT NULL DEFAULT '',
                    message TEXT NOT NULL DEFAULT '',
                    started_at INTEGER NOT NULL,
                    finished_at INTEGER NOT NULL DEFAULT 0
                );
                CREATE INDEX IF NOT EXISTS idx_runs_map_route ON runs(map_id, route_id);

                CREATE TABLE IF NOT EXISTS stop_attempts (
                    attempt_id TEXT PRIMARY KEY,
                    run_id TEXT NOT NULL REFERENCES runs(run_id) ON DELETE CASCADE,
                    loop_index INTEGER NOT NULL,
                    step_index INTEGER NOT NULL,
                    attempt_index INTEGER NOT NULL,
                    goal_uuid TEXT NOT NULL DEFAULT '',
                    target_x REAL NOT NULL,
                    target_y REAL NOT NULL,
                    target_yaw REAL NOT NULL,
                    actual_x REAL NOT NULL DEFAULT 0,
                    actual_y REAL NOT NULL DEFAULT 0,
                    actual_yaw REAL NOT NULL DEFAULT 0,
                    actual_valid INTEGER NOT NULL DEFAULT 0,
                    sampled_at INTEGER NOT NULL DEFAULT 0,
                    position_error REAL NOT NULL DEFAULT 0,
                    yaw_error REAL NOT NULL DEFAULT 0,
                    started_at INTEGER NOT NULL,
                    finished_at INTEGER NOT NULL DEFAULT 0,
                    result TEXT NOT NULL DEFAULT '',
                    error_code TEXT NOT NULL DEFAULT ''
                );
                CREATE INDEX IF NOT EXISTS idx_attempts_run ON stop_attempts(run_id);

                CREATE TABLE IF NOT EXISTS home_points (
                    map_id TEXT PRIMARY KEY REFERENCES maps(map_id) ON DELETE CASCADE,
                    map_revision INTEGER NOT NULL,
                    x REAL NOT NULL,
                    y REAL NOT NULL,
                    yaw REAL NOT NULL,
                    updated_at INTEGER NOT NULL
                );

                CREATE TABLE IF NOT EXISTS active_map_state (
                    singleton INTEGER PRIMARY KEY CHECK (singleton = 1),
                    map_id TEXT NOT NULL REFERENCES maps(map_id) ON DELETE CASCADE,
                    updated_at INTEGER NOT NULL
                );

                CREATE TABLE IF NOT EXISTS inspection_results (
                    result_id TEXT PRIMARY KEY,
                    run_id TEXT NOT NULL REFERENCES runs(run_id) ON DELETE CASCADE,
                    map_id TEXT NOT NULL,
                    point_id TEXT NOT NULL,
                    point_name TEXT NOT NULL DEFAULT '',
                    loop_index INTEGER NOT NULL,
                    step_index INTEGER NOT NULL,
                    started_at INTEGER NOT NULL,
                    finished_at INTEGER NOT NULL DEFAULT 0,
                    ptz_preset INTEGER NOT NULL DEFAULT 0,
                    prompt TEXT NOT NULL DEFAULT '',
                    image_path TEXT NOT NULL DEFAULT '',
                    text TEXT NOT NULL DEFAULT '',
                    model TEXT NOT NULL DEFAULT '',
                    success INTEGER NOT NULL DEFAULT 0,
                    error_code TEXT NOT NULL DEFAULT '',
                    message TEXT NOT NULL DEFAULT ''
                );
                CREATE INDEX IF NOT EXISTS idx_inspection_results_run
                    ON inspection_results(run_id, started_at DESC);
                CREATE INDEX IF NOT EXISTS idx_inspection_results_map
                    ON inspection_results(map_id, started_at DESC);

                CREATE TABLE IF NOT EXISTS inspection_summary (
                    run_id TEXT PRIMARY KEY,
                    inspection_date TEXT NOT NULL,
                    map_id TEXT NOT NULL,
                    route_id TEXT NOT NULL,
                    face_capture_count INTEGER NOT NULL DEFAULT 0,
                    person_count INTEGER NOT NULL DEFAULT 0,
                    plates_json TEXT NOT NULL DEFAULT '[]',
                    updated_at INTEGER NOT NULL
                );
                CREATE INDEX IF NOT EXISTS idx_inspection_date
                    ON inspection_summary(inspection_date DESC, updated_at DESC);
                CREATE TABLE IF NOT EXISTS inspection_reports (
                    run_id TEXT PRIMARY KEY REFERENCES runs(run_id) ON DELETE CASCADE,
                    generated_at INTEGER NOT NULL,
                    payload_json TEXT NOT NULL
                );
                """
            )
            self._migrate_columns()
            row = self._conn.execute(
                "SELECT version FROM schema_version ORDER BY rowid DESC LIMIT 1").fetchone()
            version = int(row["version"]) if row else 0
            if version > SCHEMA_VERSION:
                raise RuntimeError(
                    f"robot_patrol database schema {version} is newer than supported {SCHEMA_VERSION}")
            if version < SCHEMA_VERSION:
                self.initialize_roads()
                self._conn.execute("DELETE FROM schema_version")
                self._conn.execute("INSERT INTO schema_version(version) VALUES(?)",
                                   (SCHEMA_VERSION,))
                self._conn.execute(f"PRAGMA user_version = {SCHEMA_VERSION}")
            else:
                self.initialize_roads()
            # Backfill historical terminal tasks during the additive migration.
            for item in self._conn.execute(
                    'SELECT run_id FROM runs WHERE finished_at>0 AND run_id NOT IN '
                    '(SELECT run_id FROM inspection_reports)').fetchall():
                self._write_report(self._conn, item['run_id'])

    def _migrate_columns(self) -> None:
        """Additive migrations that preserve existing field data."""
        columns = {
            row["name"] for row in
            self._conn.execute("PRAGMA table_info(points)").fetchall()
        }
        additions = {
            "inspection_enabled": "INTEGER NOT NULL DEFAULT 0",
            "ptz_preset": "INTEGER NOT NULL DEFAULT 0",
            "recognition_prompt": "TEXT NOT NULL DEFAULT ''",
        }
        for name, declaration in additions.items():
            if name not in columns:
                self._conn.execute(
                    f"ALTER TABLE points ADD COLUMN {name} {declaration}")
        result_columns = {
            row["name"] for row in
            self._conn.execute("PRAGMA table_info(inspection_results)").fetchall()
        }
        if result_columns and "ptz_preset" not in result_columns:
            self._conn.execute(
                "ALTER TABLE inspection_results ADD COLUMN ptz_preset INTEGER "
                "NOT NULL DEFAULT 0")

    @staticmethod
    def _map_from_row(row: sqlite3.Row) -> MapBinding:
        return MapBinding(
            map_id=row["map_id"], name=row["name"], backend=row["backend"],
            frame_id=row["frame_id"], navigation_yaml=row["navigation_yaml"],
            image_path=row["image_path"], localization_file=row["localization_file"],
            resolution=row["resolution"], origin_x=row["origin_x"],
            origin_y=row["origin_y"], origin_yaw=row["origin_yaw"],
            width=row["width"], height=row["height"],
            navigation_yaml_sha256=row["navigation_yaml_sha256"],
            image_sha256=row["image_sha256"],
            localization_sha256=row["localization_sha256"],
            verified=bool(row["verified"]), revision=row["revision"],
            created_at_ns=row["created_at"], updated_at_ns=row["updated_at"],
        )

    @staticmethod
    def _point_from_row(row: sqlite3.Row) -> Point:
        return Point(
            point_id=row["point_id"], map_id=row["map_id"], name=row["name"],
            x=row["x"], y=row["y"], yaw=row["yaw"], source=row["source"],
            sampled_at_ns=row["sampled_at"], revision=row["revision"],
            created_at_ns=row["created_at"], updated_at_ns=row["updated_at"],
            inspection_enabled=bool(row["inspection_enabled"]),
            ptz_preset=int(row["ptz_preset"]),
            recognition_prompt=str(row["recognition_prompt"] or ""),
        )

    @staticmethod
    def _run_from_row(row: sqlite3.Row) -> RunRecord:
        return RunRecord(
            run_id=row["run_id"], route_id=row["route_id"], map_id=row["map_id"],
            state=row["state"], completed_loops=row["completed_loops"],
            completed_steps=row["completed_steps"], success=bool(row["success"]),
            error_code=row["error_code"], message=row["message"],
            route_revision=row["route_revision"], map_revision=row["map_revision"],
            started_at_ns=row["started_at"], finished_at_ns=row["finished_at"],
        )

    @staticmethod
    def _inspection_from_row(row: sqlite3.Row) -> InspectionResult:
        return InspectionResult(
            result_id=row["result_id"], run_id=row["run_id"], map_id=row["map_id"],
            point_id=row["point_id"], point_name=row["point_name"],
            loop_index=row["loop_index"], step_index=row["step_index"],
            started_at_ns=row["started_at"], finished_at_ns=row["finished_at"],
            ptz_preset=int(row["ptz_preset"]),
            prompt=row["prompt"], image_path=row["image_path"], text=row["text"],
            model=row["model"], success=bool(row["success"]),
            error_code=row["error_code"], message=row["message"],
        )

    @staticmethod
    def _home_from_row(row: sqlite3.Row) -> HomePoint:
        return HomePoint(
            map_id=row["map_id"], map_revision=row["map_revision"],
            x=row["x"], y=row["y"], yaw=row["yaw"],
            updated_at_ns=row["updated_at"],
        )

    @staticmethod
    def _summary_from_row(row: sqlite3.Row) -> InspectionSummary:
        try:
            plates = json.loads(row["plates_json"] or "[]")
        except (TypeError, ValueError):
            plates = []
        if not isinstance(plates, list):
            plates = []
        normalized = []
        for item in plates:
            if isinstance(item, str):
                normalized.append({"plate": item, "count": 1})
            elif isinstance(item, dict) and item.get("plate"):
                try:
                    count = max(1, int(item.get("count", 1)))
                except (TypeError, ValueError):
                    count = 1
                normalized.append({"plate": str(item["plate"]), "count": count})
        return InspectionSummary(
            run_id=row["run_id"], inspection_date=row["inspection_date"],
            map_id=row["map_id"], route_id=row["route_id"],
            face_capture_count=int(row["face_capture_count"]),
            person_count=int(row["person_count"]), plates=normalized,
            updated_at_ns=row["updated_at"],
        )

    # ------------------------------------------------------------------ maps
    def upsert_map(self, data: dict[str, Any], map_id: str = "") -> MapBinding:
        now = now_ns()
        navigation_yaml = str(data["navigation_yaml"])
        with self._transaction() as conn:
            existing = None
            if map_id:
                existing = conn.execute("SELECT * FROM maps WHERE map_id = ?", (map_id,)).fetchone()
            if existing is None:
                existing = conn.execute(
                    "SELECT * FROM maps WHERE navigation_yaml = ?", (navigation_yaml,)
                ).fetchone()
            if existing:
                chosen_id = existing["map_id"]
                geometry_changed = any((
                    float(existing["resolution"]) != float(data["resolution"]),
                    float(existing["origin_x"]) != float(data["origin_x"]),
                    float(existing["origin_y"]) != float(data["origin_y"]),
                    float(existing["origin_yaw"]) != float(data["origin_yaw"]),
                    int(existing["width"]) != int(data["width"]),
                    int(existing["height"]) != int(data["height"]),
                ))
                if geometry_changed:
                    # Coordinates from the previous map generation are no longer
                    # valid. Remove dependent routes first, then points.
                    conn.execute("DELETE FROM routes WHERE map_id = ?", (chosen_id,))
                    conn.execute("DELETE FROM points WHERE map_id = ?", (chosen_id,))
                revision = int(existing["revision"]) + 1
                # A home point is coordinates in one map generation. Any
                # revision change invalidates it; stale rows are removed so a
                # later route/return check cannot accidentally use old pose.
                conn.execute("DELETE FROM home_points WHERE map_id = ?", (chosen_id,))
                conn.execute("DELETE FROM active_map_state WHERE map_id = ?", (chosen_id,))
                conn.execute(
                    """UPDATE maps SET name=?, backend=?, frame_id=?, navigation_yaml=?,
                       image_path=?, localization_file=?, resolution=?, origin_x=?, origin_y=?,
                       origin_yaw=?, width=?, height=?, navigation_yaml_sha256=?, image_sha256=?,
                       localization_sha256=?, verified=?, revision=?, updated_at=? WHERE map_id=?""",
                    (validate_name(data.get("name", ""), "地图名称"), str(data["backend"]),
                     str(data.get("frame_id") or "map"), navigation_yaml, str(data["image_path"]),
                     str(data.get("localization_file") or ""), float(data["resolution"]),
                     float(data["origin_x"]), float(data["origin_y"]), float(data["origin_yaw"]),
                     int(data["width"]), int(data["height"]), str(data["navigation_yaml_sha256"]),
                     str(data["image_sha256"]), str(data.get("localization_sha256") or ""),
                     1 if data.get("verified", True) else 0, revision, now, chosen_id),
                )
            else:
                chosen_id = map_id or new_id("map")
                revision = 1
                conn.execute(
                    """INSERT INTO maps(map_id,name,backend,frame_id,navigation_yaml,image_path,
                       localization_file,resolution,origin_x,origin_y,origin_yaw,width,height,
                       navigation_yaml_sha256,image_sha256,localization_sha256,verified,revision,
                       created_at,updated_at)
                       VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)""",
                    (chosen_id, validate_name(data.get("name", ""), "地图名称"), str(data["backend"]),
                     str(data.get("frame_id") or "map"), navigation_yaml, str(data["image_path"]),
                     str(data.get("localization_file") or ""), float(data["resolution"]),
                     float(data["origin_x"]), float(data["origin_y"]), float(data["origin_yaw"]),
                     int(data["width"]), int(data["height"]), str(data["navigation_yaml_sha256"]),
                     str(data["image_sha256"]), str(data.get("localization_sha256") or ""),
                     1 if data.get("verified", True) else 0, revision, now, now),
                )
            row = conn.execute("SELECT * FROM maps WHERE map_id = ?", (chosen_id,)).fetchone()
            return self._map_from_row(row)

    def list_maps(self) -> list[MapBinding]:
        with self._lock:
            rows = self._conn.execute("SELECT * FROM maps ORDER BY updated_at DESC").fetchall()
        return [self._map_from_row(row) for row in rows]

    def get_map(self, map_id: str) -> MapBinding:
        with self._lock:
            row = self._conn.execute("SELECT * FROM maps WHERE map_id = ?", (map_id,)).fetchone()
        if not row:
            raise PatrolError("MAP_MISMATCH", f"地图不存在：{map_id}")
        return self._map_from_row(row)

    def get_map_by_yaml(self, navigation_yaml: str) -> MapBinding | None:
        with self._lock:
            row = self._conn.execute(
                "SELECT * FROM maps WHERE navigation_yaml = ?", (navigation_yaml,)
            ).fetchone()
        return self._map_from_row(row) if row else None

    # ---------------------------------------------------------------- points
    def list_points(self, map_id: str) -> list[Point]:
        if map_id:
            self.get_map(map_id)
        with self._lock:
            if map_id:
                rows = self._conn.execute(
                    "SELECT * FROM points WHERE map_id = ? ORDER BY updated_at DESC", (map_id,)
                ).fetchall()
            else:
                rows = self._conn.execute("SELECT * FROM points ORDER BY updated_at DESC").fetchall()
        return [self._point_from_row(row) for row in rows]

    def get_point(self, point_id: str) -> Point:
        with self._lock:
            row = self._conn.execute("SELECT * FROM points WHERE point_id = ?", (point_id,)).fetchone()
        if not row:
            raise PatrolError("INVALID_DATA", f"点位不存在：{point_id}")
        return self._point_from_row(row)

    def upsert_point(self, point_id: str, map_id: str, name: str, x: float, y: float,
                     yaw: float, source: str, sampled_at: int,
                     expected_revision: int = 0, inspection_enabled: bool = False,
                     ptz_preset: int = 0, recognition_prompt: str = "") -> Point:
        import math
        if not all(math.isfinite(float(v)) for v in (x, y, yaw)):
            raise PatrolError("INVALID_DATA", "点位坐标必须是有限数值")
        name = validate_name(name, "点位名称")
        inspection_enabled, ptz_preset, recognition_prompt = \
            validate_inspection_settings(inspection_enabled, ptz_preset,
                                         recognition_prompt)
        self.get_map(map_id)
        now = now_ns()
        with self._transaction() as conn:
            row = None
            if point_id:
                row = conn.execute("SELECT * FROM points WHERE point_id = ?", (point_id,)).fetchone()
            if row:
                if int(expected_revision) != int(row["revision"]):
                    raise PatrolError("STALE_DATA", "点位已被其他会话修改，请刷新后重试")
                revision = int(row["revision"]) + 1
                conn.execute(
                    """UPDATE points SET map_id=?,name=?,x=?,y=?,yaw=?,source=?,sampled_at=?,
                       revision=?,updated_at=?,inspection_enabled=?,ptz_preset=?,
                       recognition_prompt=? WHERE point_id=?""",
                    (map_id, name, float(x), float(y), float(yaw), source, int(sampled_at),
                     revision, now, 1 if inspection_enabled else 0, int(ptz_preset),
                     recognition_prompt, point_id),
                )
                # A route's stored point_id is not enough to detect a changed
                # point value. Bump every referencing route so an old preview
                # cannot be executed against newer point coordinates.
                conn.execute(
                    """UPDATE routes SET revision=revision+1, updated_at=?
                       WHERE route_id IN (SELECT route_id FROM route_steps WHERE point_id=?)""",
                    (now, point_id),
                )
                chosen_id = point_id
            else:
                chosen_id = point_id or new_id("point")
                revision = 1
                conn.execute(
                    """INSERT INTO points(point_id,map_id,name,x,y,yaw,source,sampled_at,
                       revision,created_at,updated_at,inspection_enabled,ptz_preset,
                       recognition_prompt) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?)""",
                    (chosen_id, map_id, name, float(x), float(y), float(yaw), source,
                     int(sampled_at), revision, now, now,
                     1 if inspection_enabled else 0, int(ptz_preset),
                     recognition_prompt),
                )
            row = conn.execute("SELECT * FROM points WHERE point_id = ?", (chosen_id,)).fetchone()
            return self._point_from_row(row)

    def delete_point(self, point_id: str, expected_revision: int = 0) -> None:
        with self._transaction() as conn:
            row = conn.execute("SELECT * FROM points WHERE point_id = ?", (point_id,)).fetchone()
            if not row:
                raise PatrolError("INVALID_DATA", f"点位不存在：{point_id}")
            if expected_revision and int(expected_revision) != int(row["revision"]):
                raise PatrolError("STALE_DATA", "点位已被其他会话修改，请刷新后重试")
            references = conn.execute(
                """SELECT r.route_id, r.name FROM route_steps s JOIN routes r ON r.route_id=s.route_id
                   WHERE s.point_id = ?""", (point_id,)
            ).fetchall()
            if references:
                names = "、".join(f"{item['name']}({item['route_id']})" for item in references)
                raise PatrolError("INVALID_DATA", f"点位仍被路线引用：{names}")
            conn.execute("DELETE FROM points WHERE point_id = ?", (point_id,))

    # ---------------------------------------------------------------- routes
    def _steps_for_route(self, conn: sqlite3.Connection, route_id: str) -> list[RouteStep]:
        rows = conn.execute(
            """SELECT s.step_id, s.step_order, s.point_id, s.dwell_seconds,
                      p.name AS point_name, p.x, p.y, p.yaw,
                      p.inspection_enabled, p.ptz_preset, p.recognition_prompt
               FROM route_steps s JOIN points p ON p.point_id=s.point_id
               WHERE s.route_id=? ORDER BY s.step_order""", (route_id,)
        ).fetchall()
        return [
            RouteStep(
                point_id=row["point_id"], dwell_seconds=row["dwell_seconds"],
                order=row["step_order"], step_id=row["step_id"],
                point_name=row["point_name"], x=row["x"], y=row["y"], yaw=row["yaw"],
                inspection_enabled=bool(row["inspection_enabled"]),
                ptz_preset=int(row["ptz_preset"]),
                recognition_prompt=str(row["recognition_prompt"] or ""),
            ) for row in rows
        ]

    def _route_from_row(self, conn: sqlite3.Connection, row: sqlite3.Row) -> Route:
        return Route(
            route_id=row["route_id"], map_id=row["map_id"], name=row["name"],
            steps=self._steps_for_route(conn, row["route_id"]), revision=row["revision"],
            created_at_ns=row["created_at"], updated_at_ns=row["updated_at"],
            navigation_mode=row['navigation_mode'], network_id=row['network_id'],
            network_revision=row['network_revision'],
        )

    def list_routes(self, map_id: str) -> list[Route]:
        if map_id:
            self.get_map(map_id)
        with self._lock:
            if map_id:
                rows = self._conn.execute(
                    "SELECT * FROM routes WHERE map_id=? ORDER BY updated_at DESC", (map_id,)
                ).fetchall()
            else:
                rows = self._conn.execute("SELECT * FROM routes ORDER BY updated_at DESC").fetchall()
            return [self._route_from_row(self._conn, row) for row in rows]

    def get_route(self, route_id: str) -> Route:
        with self._lock:
            row = self._conn.execute("SELECT * FROM routes WHERE route_id=?", (route_id,)).fetchone()
            if not row:
                raise PatrolError("INVALID_DATA", f"路线不存在：{route_id}")
            return self._route_from_row(self._conn, row)

    def upsert_route(self, route_id: str, map_id: str, name: str,
                     steps: Iterable[Any], expected_revision: int = 0,
                     navigation_mode: str = '', network_id: str = '', network_revision: int = 0) -> Route:
        self.get_map(map_id)
        name = validate_name(name, "路线名称")
        normalized = validate_route_steps(steps)
        point_ids = {point_id for point_id, _ in normalized}
        with self._transaction() as conn:
            previous = conn.execute('SELECT * FROM routes WHERE route_id=?', (route_id,)).fetchone() if route_id else None
            if previous and previous['navigation_mode'] == 'RECORDED_NETWORK' and not navigation_mode:
                raise PatrolError('UNSUPPORTED_MODE', '固定路线需要支持此模式的客户端，禁止降级成自由导航')
            mode = navigation_mode or 'FREE_NAV'
            if mode not in ('FREE_NAV', 'RECORDED_NETWORK'):
                raise PatrolError('INVALID_DATA', '不支持的路线导航模式')
            if mode == 'RECORDED_NETWORK':
                net = self.get_network(network_id)
                mapping = self.get_map(map_id)
                if not net['published'] or net['map_id'] != map_id or net['map_revision'] != mapping.revision or net['revision'] != int(network_revision):
                    raise PatrolError('STALE_DATA', '路网未发布、地图不符或版本已变化')
                for point_id in point_ids:
                    if point_id not in net['document'].get('bindings', {}):
                        raise PatrolError('POINT_NOT_BOUND', f'点位未绑定路网：{point_id}')
            else:
                network_id, network_revision = '', 0
            for point_id in point_ids:
                row = conn.execute("SELECT map_id FROM points WHERE point_id=?", (point_id,)).fetchone()
                if not row or row["map_id"] != map_id:
                    raise PatrolError("MAP_MISMATCH", f"路线引用了不属于当前地图的点位：{point_id}")
            row = conn.execute("SELECT * FROM routes WHERE route_id=?", (route_id,)).fetchone() if route_id else None
            now = now_ns()
            if row:
                if int(expected_revision) != int(row["revision"]):
                    raise PatrolError("STALE_DATA", "路线已被其他会话修改，请刷新后重试")
                chosen_id = route_id
                revision = int(row["revision"]) + 1
                if row['map_id'] != map_id:
                    raise PatrolError('MAP_MISMATCH', '不能将已有路线移动到其他地图')
                conn.execute("UPDATE routes SET name=?,revision=?,updated_at=?,navigation_mode=?,network_id=?,network_revision=? WHERE route_id=?",
                             (name, revision, now, mode, network_id, network_revision, chosen_id))
                conn.execute("DELETE FROM route_steps WHERE route_id=?", (chosen_id,))
            else:
                chosen_id = route_id or new_id("route")
                revision = 1
                conn.execute(
                    "INSERT INTO routes(route_id,map_id,name,revision,created_at,updated_at,navigation_mode,network_id,network_revision)"
                    " VALUES(?,?,?,?,?,?,?,?,?)", (chosen_id, map_id, name, revision, now, now, mode, network_id, network_revision)
                )
            for order, (point_id, dwell) in enumerate(normalized):
                conn.execute(
                    "INSERT INTO route_steps(step_id,route_id,step_order,point_id,dwell_seconds)"
                    " VALUES(?,?,?,?,?)", (new_id("step"), chosen_id, order, point_id, dwell)
                )
            route_row = conn.execute("SELECT * FROM routes WHERE route_id=?", (chosen_id,)).fetchone()
            return self._route_from_row(conn, route_row)

    def delete_route(self, route_id: str, expected_revision: int = 0) -> None:
        with self._transaction() as conn:
            row = conn.execute("SELECT * FROM routes WHERE route_id=?", (route_id,)).fetchone()
            if not row:
                raise PatrolError("INVALID_DATA", f"路线不存在：{route_id}")
            if expected_revision and int(expected_revision) != int(row["revision"]):
                raise PatrolError("STALE_DATA", "路线已被其他会话修改，请刷新后重试")
            conn.execute("DELETE FROM routes WHERE route_id=?", (route_id,))

    # ------------------------------------------------------------------ runs
    def create_run(self, run_id: str, route_id: str, map_id: str, route_revision: int,
                   map_revision: int, route_snapshot: dict[str, Any], loop_count: int,
                   state: int, started_at: int) -> None:
        with self._transaction() as conn:
            conn.execute(
                """INSERT INTO runs(run_id,route_id,map_id,route_revision,map_revision,
                   route_snapshot_json,loop_count,state,started_at) VALUES(?,?,?,?,?,?,?,?,?)""",
                (run_id, route_id, map_id, int(route_revision), int(map_revision),
                 json.dumps(route_snapshot, ensure_ascii=False), int(loop_count), int(state),
                 int(started_at)),
            )

    def update_run_state(self, run_id: str, state: int) -> None:
        with self._transaction() as conn:
            conn.execute("UPDATE runs SET state=? WHERE run_id=?", (int(state), run_id))

    def finish_run(self, run_id: str, state: int, success: bool, completed_loops: int,
                   completed_steps: int, error_code: str, message: str,
                   finished_at: int) -> None:
        with self._transaction() as conn:
            conn.execute(
                """UPDATE runs SET state=?,success=?,completed_loops=?,completed_steps=?,
                   error_code=?,message=?,finished_at=? WHERE run_id=?""",
                (int(state), 1 if success else 0, int(completed_loops), int(completed_steps),
                 error_code, message, int(finished_at), run_id),
            )
            self._write_report(conn, run_id)

    @staticmethod
    def _write_report(conn, run_id):
        from .reporting import build_report
        report = build_report(conn, run_id)
        if report is not None:
            conn.execute(
                'INSERT INTO inspection_reports(run_id,generated_at,payload_json) VALUES(?,?,?) '
                'ON CONFLICT(run_id) DO UPDATE SET generated_at=excluded.generated_at,payload_json=excluded.payload_json',
                (run_id, report['generated_at_ns'], json.dumps(report, ensure_ascii=False, allow_nan=False)))

    def add_attempt(self, attempt_id: str, run_id: str, loop_index: int, step_index: int,
                    attempt_index: int, goal_uuid: str, target: tuple[float, float, float],
                    actual: tuple[float, float, float] | None, sampled_at: int,
                    result: str, error_code: str = "", started_at: int = 0,
                    finished_at: int = 0) -> None:
        actual_valid = actual is not None
        ax, ay, ayaw = actual if actual else (0.0, 0.0, 0.0)
        import math
        pos_error = math.hypot(target[0] - ax, target[1] - ay) if actual else 0.0
        yaw_error = math.atan2(math.sin(target[2] - ayaw), math.cos(target[2] - ayaw)) if actual else 0.0
        with self._transaction() as conn:
            conn.execute(
                """INSERT INTO stop_attempts(attempt_id,run_id,loop_index,step_index,
                   attempt_index,goal_uuid,target_x,target_y,target_yaw,actual_x,actual_y,
                   actual_yaw,actual_valid,sampled_at,position_error,yaw_error,started_at,
                   finished_at,result,error_code) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)""",
                (attempt_id, run_id, loop_index, step_index, attempt_index, goal_uuid,
                 float(target[0]), float(target[1]), float(target[2]), float(ax), float(ay),
                 float(ayaw), 1 if actual_valid else 0, int(sampled_at), pos_error, yaw_error,
                 int(started_at), int(finished_at), result, error_code),
            )

    def add_inspection_result(self, result: InspectionResult) -> None:
        with self._transaction() as conn:
            conn.execute(
                """INSERT OR REPLACE INTO inspection_results(
                   result_id,run_id,map_id,point_id,point_name,loop_index,step_index,
                   started_at,finished_at,ptz_preset,prompt,image_path,
                   text,model,success,error_code,message)
                   VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)""",
                (result.result_id, result.run_id, result.map_id, result.point_id,
                 result.point_name, int(result.loop_index), int(result.step_index),
                 int(result.started_at_ns), int(result.finished_at_ns),
                 int(result.ptz_preset),
                 result.prompt, result.image_path, result.text, result.model,
                 1 if result.success else 0, result.error_code, result.message),
            )

    def list_inspection_results(self, map_id: str = "", run_id: str = "",
                                limit: int = 100) -> list[InspectionResult]:
        limit = max(1, min(int(limit or 100), 1000))
        clauses = []
        values: list[Any] = []
        if map_id:
            clauses.append("map_id=?")
            values.append(map_id)
        if run_id:
            clauses.append("run_id=?")
            values.append(run_id)
        where = (" WHERE " + " AND ".join(clauses)) if clauses else ""
        with self._lock:
            rows = self._conn.execute(
                "SELECT * FROM inspection_results" + where +
                " ORDER BY started_at DESC LIMIT ?", (*values, limit)
            ).fetchall()
        return [self._inspection_from_row(row) for row in rows]

    def list_runs(self, map_id: str = "", route_id: str = "", limit: int = 50) -> list[RunRecord]:
        limit = max(1, min(int(limit or 50), 500))
        clauses = []
        values: list[Any] = []
        if map_id:
            clauses.append("map_id=?")
            values.append(map_id)
        if route_id:
            clauses.append("route_id=?")
            values.append(route_id)
        where = (" WHERE " + " AND ".join(clauses)) if clauses else ""
        with self._lock:
            rows = self._conn.execute(
                "SELECT * FROM runs" + where + " ORDER BY started_at DESC LIMIT ?",
                (*values, limit),
            ).fetchall()
        return [self._run_from_row(row) for row in rows]

    # ------------------------------------------------------------ home points
    def get_home_point(self, map_id: str) -> HomePoint | None:
        """Return the home point for *map_id* only when its revision matches."""
        map_entry = self.get_map(map_id)
        with self._lock:
            row = self._conn.execute(
                "SELECT * FROM home_points WHERE map_id = ?", (map_id,)).fetchone()
        if row is None:
            return None
        if int(row["map_revision"]) != int(map_entry.revision):
            return None
        return self._home_from_row(row)

    def set_home_point(self, map_id: str, expected_map_revision: int,
                       x: float, y: float, yaw: float) -> HomePoint:
        import math
        map_entry = self.get_map(map_id)
        if expected_map_revision and int(expected_map_revision) != int(map_entry.revision):
            raise PatrolError("STALE_DATA", "地图版本已变化，请刷新后重新保存返航点")
        if not all(math.isfinite(float(value)) for value in (x, y, yaw)):
            raise PatrolError("INVALID_DATA", "返航点坐标必须是有限数值")
        now = now_ns()
        with self._transaction() as conn:
            conn.execute(
                """INSERT INTO home_points(map_id,map_revision,x,y,yaw,updated_at)
                   VALUES(?,?,?,?,?,?)
                   ON CONFLICT(map_id) DO UPDATE SET
                     map_revision=excluded.map_revision, x=excluded.x, y=excluded.y,
                     yaw=excluded.yaw, updated_at=excluded.updated_at""",
                (map_id, int(map_entry.revision), float(x), float(y), float(yaw), now),
            )
        return HomePoint(map_id=map_id, map_revision=int(map_entry.revision),
                         x=float(x), y=float(y), yaw=float(yaw), updated_at_ns=now)

    def clear_home_point(self, map_id: str) -> None:
        self.get_map(map_id)
        with self._transaction() as conn:
            conn.execute("DELETE FROM home_points WHERE map_id = ?", (map_id,))

    # ---------------------------------------------------------- active map
    def set_active_map(self, map_id: str) -> None:
        """Persist the locally verified active map selection.

        Runtime /map verification remains the responsibility of patrol_node;
        this row only lets a restarted local process and read-only web backend
        agree on which map the operator selected.
        """
        self.get_map(map_id)
        now = now_ns()
        with self._transaction() as conn:
            conn.execute(
                """INSERT INTO active_map_state(singleton,map_id,updated_at)
                   VALUES(1,?,?)
                   ON CONFLICT(singleton) DO UPDATE SET
                     map_id=excluded.map_id, updated_at=excluded.updated_at""",
                (map_id, now),
            )

    def get_active_map_id(self) -> str:
        with self._lock:
            row = self._conn.execute(
                "SELECT map_id FROM active_map_state WHERE singleton = 1").fetchone()
        if row is None:
            return ""
        try:
            self.get_map(row["map_id"])
        except PatrolError:
            self.clear_active_map()
            return ""
        return str(row["map_id"])

    def clear_active_map(self) -> None:
        with self._transaction() as conn:
            conn.execute("DELETE FROM active_map_state WHERE singleton = 1")

    # ------------------------------------------------------- inspection summary
    def upsert_inspection_summary(
        self, run_id: str, inspection_date: str, map_id: str, route_id: str,
        face_capture_count: int, person_count: int,
        plates: Iterable[Any], updated_at: int | None = None,
    ) -> InspectionSummary:
        if not run_id or not inspection_date or not map_id or not route_id:
            raise PatrolError("INVALID_DATA", "巡检摘要缺少运行、日期、地图或路线标识")
        seen: dict[str, int] = {}
        for item in plates:
            if isinstance(item, str):
                plate, count = item.strip(), 1
            elif isinstance(item, dict):
                plate = str(item.get("plate", "")).strip()
                try:
                    count = max(1, int(item.get("count", 1)))
                except (TypeError, ValueError):
                    count = 1
            else:
                continue
            if not plate:
                continue
            seen[plate] = seen.get(plate, 0) + count
        normalized = [{"plate": plate, "count": count}
                      for plate, count in seen.items()]
        stamp = now_ns() if updated_at is None else int(updated_at)
        with self._transaction() as conn:
            conn.execute(
                """INSERT INTO inspection_summary(
                     run_id,inspection_date,map_id,route_id,face_capture_count,
                     person_count,plates_json,updated_at)
                   VALUES(?,?,?,?,?,?,?,?)
                   ON CONFLICT(run_id) DO UPDATE SET
                     inspection_date=excluded.inspection_date, map_id=excluded.map_id,
                     route_id=excluded.route_id,
                     face_capture_count=excluded.face_capture_count,
                     person_count=excluded.person_count,
                     plates_json=excluded.plates_json, updated_at=excluded.updated_at""",
                (run_id, inspection_date, map_id, route_id,
                 max(0, int(face_capture_count)), max(0, int(person_count)),
                 json.dumps(normalized, ensure_ascii=False, separators=(",", ":")), stamp),
            )
            row = conn.execute(
                "SELECT * FROM inspection_summary WHERE run_id = ?", (run_id,)).fetchone()
            # The recorder can deliver the last counts after the task's terminal
            # callback. Keep the persisted report in sync with those counts.
            self._write_report(conn, run_id)
            return self._summary_from_row(row)

    def list_inspection_summaries(self, limit: int = 100) -> list[InspectionSummary]:
        limit = max(1, min(int(limit or 100), 500))
        with self._lock:
            rows = self._conn.execute(
                "SELECT * FROM inspection_summary "
                "ORDER BY inspection_date DESC, updated_at DESC LIMIT ?", (limit,)
            ).fetchall()
        return [self._summary_from_row(row) for row in rows]
