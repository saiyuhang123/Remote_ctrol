"""Read-only access to the authoritative robot_patrol SQLite database.

The web backend never creates tables and never writes business data. The
patrol node and inspection_recorder are the only business-table writers.
"""
from __future__ import annotations

import os
from pathlib import Path
import sqlite3
import threading
from typing import Any, Iterable

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DB = (Path(os.environ.get("XDG_DATA_HOME", Path.home() / ".local/share"))
              / "robot_patrol" / "patrol.sqlite3")
DB_PATH = Path(os.environ.get("ROBOT_PATROL_DB", str(DEFAULT_DB))).expanduser()
WEB_MAP_DIR = Path(os.environ.get("ROBOT_WEB_MAP_DIR",
                                  str(PROJECT_ROOT / "web" / "maps"))).expanduser()
SUPPORTED_SCHEMA_VERSION = 7
SUPPORTED_SCHEMA_VERSIONS = frozenset(range(2, SUPPORTED_SCHEMA_VERSION + 1))

_local = threading.local()


class DatabaseUnavailable(RuntimeError):
    pass


def _connect() -> sqlite3.Connection:
    if not DB_PATH.is_file():
        raise DatabaseUnavailable(
            f"本地巡检数据库不存在：{DB_PATH}；请先启动 robot_patrol")
    uri = DB_PATH.resolve().as_uri() + "?mode=ro"
    try:
        connection = sqlite3.connect(uri, uri=True, timeout=5.0,
                                     check_same_thread=False)
        connection.row_factory = sqlite3.Row
        connection.execute("PRAGMA query_only = ON")
        connection.execute("PRAGMA busy_timeout = 5000")
        return connection
    except sqlite3.Error as error:
        raise DatabaseUnavailable(f"无法只读打开巡检数据库：{error}") from error


def conn() -> sqlite3.Connection:
    connection = getattr(_local, "connection", None)
    if connection is None:
        connection = _connect()
        _local.connection = connection
    return connection


def schema_version() -> int:
    try:
        row = conn().execute(
            "SELECT version FROM schema_version ORDER BY rowid DESC LIMIT 1").fetchone()
    except sqlite3.Error as error:
        raise DatabaseUnavailable(f"数据库缺少 schema_version：{error}") from error
    if row is None:
        raise DatabaseUnavailable("数据库缺少 schema_version 记录")
    version = int(row["version"])
    if version not in SUPPORTED_SCHEMA_VERSIONS:
        raise DatabaseUnavailable(
            f"数据库 schema={version}，网页支持 2～{SUPPORTED_SCHEMA_VERSION}；请升级匹配的软件，不要删除业务数据库")
    return version


def query(sql: str, args: Iterable[Any] = ()) -> list[dict[str, Any]]:
    try:
        return [dict(row) for row in conn().execute(sql, tuple(args)).fetchall()]
    except sqlite3.Error as error:
        raise DatabaseUnavailable(f"数据库查询失败：{error}") from error


def query_one(sql: str, args: Iterable[Any] = ()) -> dict[str, Any] | None:
    try:
        row = conn().execute(sql, tuple(args)).fetchone()
        return dict(row) if row is not None else None
    except sqlite3.Error as error:
        raise DatabaseUnavailable(f"数据库查询失败：{error}") from error


def table_columns(table: str) -> set[str]:
    try:
        return {str(row["name"]) for row in conn().execute(
            f"PRAGMA table_info({table})").fetchall()}
    except sqlite3.Error as error:
        raise DatabaseUnavailable(f"无法读取数据表结构：{error}") from error
