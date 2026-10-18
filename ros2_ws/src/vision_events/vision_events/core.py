"""Pure persistence, bounded pose history and media handling (no ROS/model imports)."""
import hashlib
import json
import math
import os
from pathlib import Path
import sqlite3
import threading
import time
import uuid
from collections import deque

TYPES = ('face_capture', 'person', 'vehicle_plate')


def canonical_event(raw):
    event = dict(raw)
    kind = event.get('event_type')
    if kind not in TYPES:
        raise ValueError('unknown event type')
    key = str(event.get('source_event_key', ''))
    if not key or len(key) > 4096:
        raise ValueError('missing/oversized source event key')
    stamp = int(event['event_time_ns'])
    if stamp <= 0:
        raise ValueError('invalid event time')
    event['schema_version'] = 1
    event['event_id'] = hashlib.sha256((str(event.get('source', '')) + '\0' + key).encode()).hexdigest()
    event['event_time_ns'] = stamp
    event.setdefault('received_time_ns', time.time_ns())
    event['stored_time_ns'] = time.time_ns()
    event.setdefault('label', {'face_capture': '人脸抓拍', 'person': '行人',
                               'vehicle_plate': '车牌'}[kind])
    # Reject non-finite payloads rather than emitting non-standard JSON.
    json.dumps(event, allow_nan=False)
    return event


class PoseHistory:
    def __init__(self, max_age=.3, retention=120.):
        self.max_age_ns = int(max_age * 1e9)
        self.retention_ns = int(retention * 1e9)
        self.samples = deque(maxlen=5000)
        self.contexts = []

    def context(self, map_id, revision='', at_ns=None):
        stamp = time.time_ns() if at_ns is None else at_ns
        self.contexts.append((stamp, str(uuid.uuid4()), map_id, revision))
        self.samples.clear()

    def add(self, stamp, x, y, yaw):
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            return
        if self.samples and stamp < self.samples[-1][0]:
            self.samples.clear()  # clock/TF reset; never mix epochs
        if not self.samples or stamp != self.samples[-1][0]:
            self.samples.append((stamp, x, y, yaw))
        while self.samples and stamp - self.samples[0][0] > self.retention_ns:
            self.samples.popleft()

    def associate(self, stamp):
        result = dict(pose_valid=False, pose_status='map_unassociated',
                      robot_x=None, robot_y=None, robot_yaw=None, frame_id='map',
                      map_id='', map_revision='', context_id='', pose_time_ns=None, pose_age_ms=None)
        contexts = [c for c in self.contexts if c[0] <= stamp]
        if not contexts or not contexts[-1][2]:
            return result
        begin, cid, map_id, revision = contexts[-1]
        result.update(map_id=map_id, map_revision=revision, context_id=cid, pose_status='tf_unavailable')
        # Context changes clear samples. Old events must not use the new map's TF.
        if cid != self.contexts[-1][1]:
            result['pose_status'] = 'context_changed'
            return result
        eligible = [p for p in self.samples if p[0] >= begin]
        if not eligible:
            return result
        sample = min(eligible, key=lambda p: abs(p[0] - stamp))
        age = abs(sample[0] - stamp)
        result.update(pose_time_ns=sample[0], pose_age_ms=age / 1e6)
        if age > self.max_age_ns:
            result['pose_status'] = 'tf_stale'
            return result
        result.update(pose_valid=True, pose_status='valid', robot_x=sample[1],
                      robot_y=sample[2], robot_yaw=sample[3])
        return result


class Store:
    def __init__(self, directory, upload=False):
        self.directory = Path(directory).expanduser().resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.upload = upload
        self.lock = threading.RLock()
        self.db = sqlite3.connect(self.directory / 'events.sqlite3', check_same_thread=False)
        if self.db.execute('PRAGMA user_version').fetchone()[0] > 1:
            self.db.close()
            raise ValueError('vision database version is newer than this application')
        self.db.execute('PRAGMA journal_mode=WAL')
        self.db.executescript('''
          CREATE TABLE IF NOT EXISTS events (
            event_id TEXT PRIMARY KEY, stamp INTEGER NOT NULL, kind TEXT NOT NULL, payload TEXT NOT NULL);
          CREATE INDEX IF NOT EXISTS events_time ON events(stamp DESC,event_id DESC);
          CREATE INDEX IF NOT EXISTS events_kind_time ON events(kind,stamp DESC);
          CREATE TABLE IF NOT EXISTS outbox (event_id TEXT PRIMARY KEY REFERENCES events(event_id), attempts INTEGER DEFAULT 0);
          CREATE TABLE IF NOT EXISTS delivered (event_id TEXT PRIMARY KEY);
          PRAGMA user_version=1;
        ''')
        self.db.commit()
        if self.upload:
            with self.db:
                self.db.execute('INSERT OR IGNORE INTO outbox(event_id) SELECT event_id FROM events WHERE event_id NOT IN (SELECT event_id FROM delivered)')

    def close(self):
        with self.lock:
            self.db.close()

    def exists(self, event_id):
        with self.lock:
            return self.db.execute('SELECT 1 FROM events WHERE event_id=?', (event_id,)).fetchone() is not None

    def insert(self, event):
        text = json.dumps(event, ensure_ascii=False, allow_nan=False)
        with self.lock, self.db:
            inserted = self.db.execute('INSERT OR IGNORE INTO events VALUES(?,?,?,?)',
                                      (event['event_id'], event['event_time_ns'], event['event_type'], text)).rowcount
            if inserted and self.upload:
                self.db.execute('INSERT INTO outbox(event_id) VALUES(?)', (event['event_id'],))
        return bool(inserted)

    def query(self, kind='', limit=50, cursor='', event_id=''):
        if kind and kind not in TYPES:
            raise ValueError('unknown event type')
        limit = min(max(int(limit or 50), 1), 200)
        where, args = [], []
        if kind:
            where.append('kind=?'); args.append(kind)
        if event_id:
            where.append('event_id=?'); args.append(event_id)
        base = (' WHERE ' + ' AND '.join(where)) if where else ''
        with self.lock:
            total = self.db.execute('SELECT COUNT(*) FROM events' + base, args).fetchone()[0]
            if cursor:
                at, eid = json.loads(cursor)
                where.append('(stamp < ? OR (stamp = ? AND event_id < ?))')
                args.extend((int(at), int(at), str(eid)))
            clause = (' WHERE ' + ' AND '.join(where)) if where else ''
            rows = self.db.execute('SELECT stamp,event_id,payload FROM events' + clause +
                                   ' ORDER BY stamp DESC,event_id DESC LIMIT ?', (*args, limit + 1)).fetchall()
        more = len(rows) > limit
        rows = rows[:limit]
        next_cursor = json.dumps([rows[-1][0], rows[-1][1]]) if more and rows else ''
        return [json.loads(r[2]) for r in rows], next_cursor, total

    def pending(self):
        with self.lock:
            row = self.db.execute('SELECT e.payload FROM outbox o JOIN events e USING(event_id) ORDER BY e.stamp LIMIT 1').fetchone()
        return json.loads(row[0]) if row else None

    def delivered(self, event_id):
        with self.lock, self.db:
            self.db.execute('INSERT OR IGNORE INTO delivered VALUES(?)', (event_id,))
            self.db.execute('DELETE FROM outbox WHERE event_id=?', (event_id,))


def approved_file(value, roots):
    path = Path(value).expanduser().resolve()
    if not any(path.is_relative_to(Path(root).resolve()) for root in roots):
        raise ValueError('media path outside configured roots')
    if not path.is_file():
        raise ValueError('media file missing')
    return path


def archive(event, directory, roots):
    import cv2
    src = event.get('image_path', '')
    event['media_status'] = 'missing'
    if not src:
        return event
    path = approved_file(src, roots)
    image = cv2.imread(str(path))
    if image is None or image.size == 0:
        raise ValueError('unreadable event image')
    date = time.strftime('%Y-%m-%d', time.localtime(event['event_time_ns'] / 1e9))
    destination = Path(directory) / 'media' / date / event['event_id']
    destination.mkdir(parents=True, exist_ok=True)
    target = destination / 'image.jpg'
    temporary = destination / 'image.tmp'
    # Keep a small durable copy; original scene images remain driver-owned.
    encoded = path.read_bytes()
    if not encoded.startswith(b'\xff\xd8'):
        ok, converted = cv2.imencode('.jpg', image)
        if not ok:
            raise ValueError('cannot encode event image')
        encoded = converted.tobytes()
    with temporary.open('wb') as stream:
        stream.write(encoded); stream.flush(); os.fsync(stream.fileno())
    os.replace(temporary, target)
    event['native_image_path'] = str(path)
    event['image_path'] = str(target)
    event['media_status'] = 'ready'
    return event
