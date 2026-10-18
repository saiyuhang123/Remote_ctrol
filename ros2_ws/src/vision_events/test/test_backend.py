import json
import importlib
from pathlib import Path
import sqlite3
import pytest


def test_old_database_migration_and_retry_idempotence(tmp_path):
    from backend import db
    from backend.api_records import DetectionIn, add_detection
    previous = db.DB_PATH
    old_connection = getattr(db._local, 'c', None)
    db._local.c = None
    try:
        db.DB_PATH = tmp_path/'old.db'
        with sqlite3.connect(db.DB_PATH) as connection:
            connection.executescript(db.SCHEMA)
            connection.execute("INSERT INTO detection_record(ts,event,label) VALUES(1,'person','old')")
        db.init_db(); db.init_db()
        payload = DetectionIn(event='person_face', label='人脸抓拍', event_id='a'*64, metadata={'pose_valid': False})
        first = add_detection(payload)
        assert add_detection(payload)['id'] == first['id']
        assert len(db.query('SELECT * FROM detection_record')) == 2
        assert db.query('SELECT label FROM detection_record WHERE id=1')[0]['label'] == 'old'
        add_detection(DetectionIn(event='person', label='legacy'))
        assert len(db.query('SELECT * FROM detection_record')) == 3
    finally:
        if getattr(db._local, 'c', None):
            db._local.c.close()
        db._local.c = old_connection; db.DB_PATH = previous


def test_backend_media_rejects_unknown_and_outside_paths(tmp_path, monkeypatch):
    from backend.api_vision import media
    from fastapi import HTTPException
    from vision_events.core import Store, canonical_event
    monkeypatch.setenv('VISION_DATA_DIR', str(tmp_path))
    store = Store(tmp_path)
    event = canonical_event(dict(event_type='person', source='test', source_event_key='one', event_time_ns=1))
    event['image_path'] = '/etc/passwd'
    store.insert(event)
    with pytest.raises(HTTPException):
        media(event['event_id'])
    with pytest.raises(HTTPException):
        media('../etc/passwd')
    store.close()
