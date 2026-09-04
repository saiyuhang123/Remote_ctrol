"""SQLite 数据层：建表 + 轻量查询助手（无 ORM，保持简单）"""
import sqlite3
import threading
from pathlib import Path

DB_PATH = Path.home() / 'ros2Project' / 'Remote_ctrol' / 'data' / 'robot.db'

_local = threading.local()

SCHEMA = """
CREATE TABLE IF NOT EXISTS detection_record (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    ts REAL NOT NULL,               -- 事件时间戳
    event TEXT NOT NULL,            -- person / person_face / vehicle_plate
    label TEXT NOT NULL,            -- 姓名 / 未登记人员 / 车牌号
    score REAL DEFAULT 0,
    image TEXT DEFAULT '',          -- 抓拍图相对路径 web/ 下
    task_id INTEGER DEFAULT NULL,   -- 关联巡逻任务（可空）
    created_at REAL DEFAULT (strftime('%s','now'))
);
CREATE TABLE IF NOT EXISTS capture_record (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    ts REAL NOT NULL,
    label TEXT DEFAULT 'manual',    -- manual / detection / patrol
    image TEXT NOT NULL,
    task_id INTEGER DEFAULT NULL,
    created_at REAL DEFAULT (strftime('%s','now'))
);
CREATE TABLE IF NOT EXISTS patrol_point (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT NOT NULL,
    x REAL NOT NULL,
    y REAL NOT NULL,
    yaw REAL DEFAULT 0,
    actions TEXT DEFAULT '[]',      -- JSON: [{"type":"dwell","sec":10},{"type":"capture"},{"type":"tts","text":"..."}]
    created_at REAL DEFAULT (strftime('%s','now'))
);
CREATE TABLE IF NOT EXISTS route (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT NOT NULL,
    point_ids TEXT DEFAULT '[]',    -- JSON 数组，有序
    created_at REAL DEFAULT (strftime('%s','now'))
);
CREATE TABLE IF NOT EXISTS plan (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT NOT NULL,
    route_id INTEGER NOT NULL,
    schedule_type TEXT NOT NULL,    -- once / daily / interval
    schedule_value TEXT NOT NULL,   -- once: "2026-09-05 09:00"; daily: "09:00"; interval: 秒数
    enabled INTEGER DEFAULT 1,
    last_run REAL DEFAULT NULL,
    next_run REAL DEFAULT NULL,
    created_at REAL DEFAULT (strftime('%s','now'))
);
CREATE TABLE IF NOT EXISTS task_record (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    plan_id INTEGER DEFAULT NULL,
    route_id INTEGER DEFAULT NULL,
    start_time REAL,
    end_time REAL,
    status TEXT DEFAULT 'running',  -- running / success / failed / canceled
    detail TEXT DEFAULT '{}',       -- JSON：各点位执行结果
    report_path TEXT DEFAULT '',
    created_at REAL DEFAULT (strftime('%s','now'))
);
"""


def conn():
    if not hasattr(_local, 'c') or _local.c is None:
        DB_PATH.parent.mkdir(parents=True, exist_ok=True)
        _local.c = sqlite3.connect(DB_PATH)
        _local.c.row_factory = sqlite3.Row
    return _local.c


def init_db():
    c = conn()
    c.executescript(SCHEMA)
    c.commit()


def query(sql, args=()):
    return [dict(r) for r in conn().execute(sql, args).fetchall()]


def execute(sql, args=()):
    c = conn()
    cur = c.execute(sql, args)
    c.commit()
    return cur.lastrowid
