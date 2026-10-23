"""Idempotent HTTP event inbox; it never writes the robot's patrol database."""
from contextlib import closing
import json
import math
import os
from pathlib import Path
import re
import sqlite3
import time

ROOT = Path(__file__).resolve().parents[1]
DIRECTORY = Path(os.environ.get('ROBOT_WEB_RECORDS_DIR', ROOT / 'data/web_records')).expanduser()
KINDS = ('face_capture', 'person', 'vehicle_plate')
ID_PATTERN = re.compile(r'^[A-Za-z0-9_-]{1,128}$')


def normalize(payload):
    value = payload.get('metadata', payload)
    if not isinstance(value, dict):
        raise ValueError('识别记录必须是对象')
    event_id = str(value.get('event_id') or payload.get('event_id') or '')
    if not ID_PATTERN.fullmatch(event_id):
        raise ValueError('识别记录编号无效')
    kind = value.get('event_type') or {'person_face': 'face_capture', 'vehicle_plate': 'vehicle_plate', 'person': 'person'}.get(payload.get('event'))
    if kind not in KINDS:
        raise ValueError('识别类型无效')
    stamp = value.get('event_time_ns')
    if stamp is None:
        stamp = int(float(payload.get('ts', 0))*1e9)
    if type(stamp) is not int or not 0 < stamp < 2**63:
        raise ValueError('识别时间无效')
    event = dict(event_id=event_id, event_type=kind, event_time_ns=stamp)
    for key, limit in (('label', 2000), ('source', 200), ('map_id', 512), ('map_revision', 512), ('pose_status', 200), ('time_basis', 200)):
        event[key] = str(value.get(key) or '')[:limit]
    confidence = value.get('confidence', payload.get('score'))
    if confidence is not None and (type(confidence) not in (int, float) or not math.isfinite(confidence) or not 0 <= confidence <= 1):
        raise ValueError('识别置信度无效')
    event['confidence'] = confidence
    valid = value.get('pose_valid') is True
    for key in ('robot_x', 'robot_y', 'robot_yaw'):
        item = value.get(key)
        if valid and (type(item) not in (int, float) or not math.isfinite(item)):
            raise ValueError('识别坐标无效')
        event[key] = float(item) if valid else None
    event['pose_valid'] = valid
    return event


def connect():
    DIRECTORY.mkdir(parents=True, exist_ok=True)
    connection = sqlite3.connect(DIRECTORY / 'events.sqlite3', timeout=5)
    connection.row_factory = sqlite3.Row
    connection.execute('PRAGMA journal_mode=WAL')
    connection.executescript('''CREATE TABLE IF NOT EXISTS events(
        event_id TEXT PRIMARY KEY, stamp INTEGER NOT NULL, kind TEXT NOT NULL,
        map_id TEXT NOT NULL, payload TEXT NOT NULL, image TEXT NOT NULL DEFAULT '', received_at INTEGER NOT NULL);
        CREATE INDEX IF NOT EXISTS event_time ON events(stamp DESC,event_id DESC);
        CREATE INDEX IF NOT EXISTS event_map_time ON events(map_id,stamp DESC);''')
    return connection


def receive(event, image_name='', image_temp=None):
    with closing(connect()) as connection, connection:
        existing = connection.execute('SELECT * FROM events WHERE event_id=?', (event['event_id'],)).fetchone()
        if existing:
            previous = json.loads(existing['payload'])
            if (previous['event_time_ns'], previous['event_type']) != (event['event_time_ns'], event['event_type']):
                raise ValueError('识别记录编号已被其他事件使用')
            if image_name and not existing['image']:
                if image_temp is not None:
                    os.replace(image_temp, DIRECTORY/'media'/image_name)
                connection.execute('UPDATE events SET image=? WHERE event_id=?', (image_name, event['event_id']))
            return False
        if image_temp is not None:
            os.replace(image_temp, DIRECTORY/'media'/image_name)
        connection.execute('INSERT INTO events VALUES(?,?,?,?,?,?,?)',
                           (event['event_id'], event['event_time_ns'], event['event_type'], event['map_id'],
                            json.dumps(event, ensure_ascii=False, allow_nan=False), image_name, time.time_ns()))
        return True


def public(row):
    value = json.loads(row['payload'])
    value['image_url'] = '/api/vision/media/'+row['event_id'] if row['image'] else ''
    value['media_status'] = 'ready' if row['image'] else 'missing'
    return value


def query(kind='', limit=50, cursor='', map_id='', start_ns=None, end_ns=None, event_id='', include_unmapped=False):
    if kind and kind not in KINDS:
        raise ValueError('识别类型无效')
    clauses, arguments = [], []
    for column, item in (('kind', kind), ('event_id', event_id)):
        if item:
            clauses.append(column+'=?'); arguments.append(item)
    if map_id:
        clauses.append('(map_id=? OR map_id=\'\')' if include_unmapped else 'map_id=?')
        arguments.append(map_id)
    for operator, item in (('>=', start_ns), ('<=', end_ns)):
        if item is not None:
            clauses.append('stamp'+operator+'?'); arguments.append(item)
    base = ' WHERE '+' AND '.join(clauses) if clauses else ''
    with closing(connect()) as connection:
        total = connection.execute('SELECT COUNT(*) FROM events'+base, arguments).fetchone()[0]
        if cursor:
            at, identifier = json.loads(cursor)
            if type(at) is not int or not isinstance(identifier, str) or not ID_PATTERN.fullmatch(identifier):
                raise ValueError('分页游标无效')
            clauses.append('(stamp<? OR (stamp=? AND event_id<?))')
            arguments.extend((at, at, identifier))
        where = ' WHERE '+' AND '.join(clauses) if clauses else ''
        rows = connection.execute('SELECT * FROM events'+where+' ORDER BY stamp DESC,event_id DESC LIMIT ?',
                                  (*arguments, limit+1)).fetchall()
    selected = rows[:limit]
    following = json.dumps([selected[-1]['stamp'], selected[-1]['event_id']]) if len(rows)>limit else ''
    return dict(items=[public(row) for row in selected], total=total, next_cursor=following)


def image_path(event_id):
    with closing(connect()) as connection:
        row = connection.execute('SELECT image FROM events WHERE event_id=?', (event_id,)).fetchone()
    if row is None or not row['image']:
        return None
    root = (DIRECTORY / 'media').resolve()
    path = (root / row['image']).resolve()
    return path if path.parent == root and path.is_file() else None
