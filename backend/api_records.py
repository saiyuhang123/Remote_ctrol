"""识别/抓拍记录 API"""
import time
import json
from typing import Optional

from fastapi import APIRouter
from pydantic import BaseModel, Field

from . import db

router = APIRouter(prefix='/api/records', tags=['records'])


class DetectionIn(BaseModel):
    ts: Optional[float] = None
    event: str                      # person / person_face / vehicle_plate
    label: str
    score: float = 0.0
    image: str = ''
    task_id: Optional[int] = None
    event_id: Optional[str] = Field(default=None, min_length=1, max_length=128)
    metadata: dict = Field(default_factory=dict)


class CaptureIn(BaseModel):
    ts: Optional[float] = None
    label: str = 'manual'
    image: str
    task_id: Optional[int] = None


@router.post('/detections')
def add_detection(d: DetectionIn):
    db.execute(
        'INSERT OR IGNORE INTO detection_record(ts,event,label,score,image,task_id,event_id,metadata) VALUES(?,?,?,?,?,?,?,?)',
        (d.ts or time.time(), d.event, d.label, d.score, d.image, d.task_id,
         d.event_id, json.dumps(d.metadata, ensure_ascii=False, allow_nan=False)))
    rid = (db.query('SELECT id FROM detection_record WHERE event_id=?', (d.event_id,))[0]['id']
           if d.event_id else db.query('SELECT last_insert_rowid() AS id')[0]['id'])
    return {'ok': True, 'id': rid}


@router.post('/captures')
def add_capture(c: CaptureIn):
    rid = db.execute(
        'INSERT INTO capture_record(ts,label,image,task_id) VALUES(?,?,?,?)',
        (c.ts or time.time(), c.label, c.image, c.task_id))
    return {'ok': True, 'id': rid}


@router.get('/detections')
def list_detections(event: Optional[str] = None, limit: int = 100, offset: int = 0):
    sql = 'SELECT * FROM detection_record'
    args = []
    if event:
        sql += ' WHERE event=?'
        args.append(event)
    sql += ' ORDER BY id DESC LIMIT ? OFFSET ?'
    args += [limit, offset]
    return {'items': db.query(sql, args)}


@router.get('/captures')
def list_captures(limit: int = 100, offset: int = 0):
    return {'items': db.query(
        'SELECT * FROM capture_record ORDER BY id DESC LIMIT ? OFFSET ?',
        (limit, offset))}


@router.get('/summary')
def summary():
    return {
        'detections_total': db.query('SELECT COUNT(*) c FROM detection_record')[0]['c'],
        'captures_total': db.query('SELECT COUNT(*) c FROM capture_record')[0]['c'],
        'detections_today': db.query(
            "SELECT COUNT(*) c FROM detection_record WHERE ts > ?",
            (time.mktime(time.strptime(time.strftime('%Y-%m-%d'), '%Y-%m-%d')),))[0]['c'],
    }
