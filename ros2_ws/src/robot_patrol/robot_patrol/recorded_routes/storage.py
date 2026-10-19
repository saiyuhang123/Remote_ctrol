"""Road-library mixin; uses PatrolStore's connection and transaction lock."""
from __future__ import annotations

import json
from ..models import PatrolError, new_id, now_ns, validate_name
from .graph import validate_network, digest


class RoadStorage:
    def initialize_roads(self):
        self._conn.executescript('''
          CREATE TABLE IF NOT EXISTS road_networks(
            network_id TEXT PRIMARY KEY, map_id TEXT NOT NULL REFERENCES maps(map_id),
            map_revision INTEGER NOT NULL, name TEXT NOT NULL, revision INTEGER NOT NULL,
            published INTEGER NOT NULL DEFAULT 0, document_json TEXT NOT NULL,
            updated_at INTEGER NOT NULL);
          CREATE TABLE IF NOT EXISTS road_drafts(
            draft_id TEXT PRIMARY KEY, map_id TEXT NOT NULL REFERENCES maps(map_id),
            payload_json TEXT NOT NULL, updated_at INTEGER NOT NULL);
          CREATE TABLE IF NOT EXISTS road_events(
            event_id INTEGER PRIMARY KEY AUTOINCREMENT, run_id TEXT NOT NULL,
            event_json TEXT NOT NULL, created_at INTEGER NOT NULL);
          CREATE TABLE IF NOT EXISTS road_navigation_context(
            map_id TEXT PRIMARY KEY, mode TEXT NOT NULL, network_id TEXT NOT NULL,
            network_revision INTEGER NOT NULL);
        ''')
        columns = {r['name'] for r in self._conn.execute('PRAGMA table_info(routes)')}
        for key, spec in [('navigation_mode', "TEXT NOT NULL DEFAULT 'FREE_NAV'"),
                          ('network_id', "TEXT NOT NULL DEFAULT ''"),
                          ('network_revision', 'INTEGER NOT NULL DEFAULT 0')]:
            if key not in columns:
                self._conn.execute(f'ALTER TABLE routes ADD COLUMN {key} {spec}')

    def list_networks(self, map_id):
        with self._lock:
            rows = self._conn.execute('SELECT * FROM road_networks WHERE map_id=? ORDER BY updated_at DESC', (map_id,)).fetchall()
        return [self._road_row(r) for r in rows]

    def set_navigation_context(self, map_id, mode, network_id='', network_revision=0):
        with self._transaction() as conn:
            conn.execute('INSERT OR REPLACE INTO road_navigation_context VALUES(?,?,?,?)',
                         (map_id, mode, network_id, network_revision))

    def navigation_context(self, map_id):
        with self._lock:
            row = self._conn.execute('SELECT * FROM road_navigation_context WHERE map_id=?',(map_id,)).fetchone()
        return dict(row) if row else dict(mode='FREE_NAV',network_id='',network_revision=0)

    @staticmethod
    def _road_row(row):
        return dict(network_id=row['network_id'], map_id=row['map_id'],
                    map_revision=row['map_revision'], name=row['name'], revision=row['revision'],
                    published=bool(row['published']), document=json.loads(row['document_json']))

    def get_network(self, network_id):
        with self._lock:
            row = self._conn.execute('SELECT * FROM road_networks WHERE network_id=?', (network_id,)).fetchone()
        if not row:
            raise PatrolError('INVALID_DATA', '道路路网不存在')
        return self._road_row(row)

    def save_network(self, map_id, name, document, network_id='', expected_revision=0, published=False):
        mapping = self.get_map(map_id)
        name = validate_name(name, '路网名称')
        validate_network(document,float(document.get('vehicle',{}).get('minimum_turning_radius',0.69)))
        chosen = network_id or new_id('network')
        with self._transaction() as conn:
            row = conn.execute('SELECT * FROM road_networks WHERE network_id=?', (chosen,)).fetchone()
            if row and (row['revision'] != expected_revision or row['map_id'] != map_id):
                raise PatrolError('STALE_DATA', '路网版本已变化或地图不匹配，请刷新')
            revision = row['revision']+1 if row else 1
            conn.execute('''INSERT INTO road_networks VALUES(?,?,?,?,?,?,?,?)
                ON CONFLICT(network_id) DO UPDATE SET name=excluded.name,
                map_revision=excluded.map_revision,revision=excluded.revision,
                published=excluded.published,document_json=excluded.document_json,updated_at=excluded.updated_at''',
                         (chosen, map_id, mapping.revision, name, revision, int(published),
                          json.dumps(document, ensure_ascii=False, allow_nan=False), now_ns()))
        return self.get_network(chosen)

    def delete_network(self, network_id, expected_revision):
        with self._transaction() as conn:
            row = conn.execute('SELECT * FROM road_networks WHERE network_id=?', (network_id,)).fetchone()
            if not row or row['revision'] != expected_revision:
                raise PatrolError('STALE_DATA', '路网不存在或版本变化')
            if conn.execute('SELECT 1 FROM routes WHERE network_id=?', (network_id,)).fetchone():
                raise PatrolError('IN_USE', '仍有巡检路线引用该路网，不能删除')
            conn.execute('DELETE FROM road_networks WHERE network_id=?', (network_id,))

    def save_road_draft(self, draft_id, map_id, payload):
        with self._transaction() as conn:
            conn.execute('INSERT OR REPLACE INTO road_drafts VALUES(?,?,?,?)',
                         (draft_id, map_id, json.dumps(payload, ensure_ascii=False, allow_nan=False), now_ns()))

    def list_road_drafts(self, map_id):
        with self._lock:
            rows = self._conn.execute('SELECT * FROM road_drafts WHERE map_id=?', (map_id,)).fetchall()
        result = [dict(draft_id=r['draft_id'], payload=json.loads(r['payload_json'])) for r in rows]
        return [item for item in result if item['payload'].get('state') not in ('SAVED','DISCARDED')]

    def road_event(self, run_id, value):
        with self._transaction() as conn:
            conn.execute('INSERT INTO road_events(run_id,event_json,created_at) VALUES(?,?,?)',
                         (run_id, json.dumps(value, ensure_ascii=False, allow_nan=False), now_ns()))

    def road_events(self, run_id, limit=200):
        with self._lock:
            rows = self._conn.execute('SELECT event_json,created_at FROM road_events WHERE run_id=? ORDER BY event_id DESC LIMIT ?',
                                      (run_id, min(1000, max(1, int(limit))))).fetchall()
        return [dict(json.loads(row['event_json']), timestamp_ns=row['created_at']) for row in rows]
