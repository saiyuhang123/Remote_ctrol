"""识别/抓拍记录 API"""
import time
from typing import Optional

from fastapi import APIRouter
from pydantic import BaseModel

from . import db

router = APIRouter(prefix='/api/records', tags=['records'])


class DetectionIn(BaseModel):
    ts: Optional[float] = None
    event: str                      # person / person_face / vehicle_plate
    label: str
    score: float = 0.0
    image: str = ''
    task_id: Optional[int] = None


class CaptureIn(BaseModel):
    ts: Optional[float] = None
    label: str = 'manual'
    image: str
    task_id: Optional[int] = None


@router.post('/detections')
def add_detection(d: DetectionIn):
    rid = db.execute(
        'INSERT INTO detection_record(ts,event,label,score,image,task_id) VALUES(?,?,?,?,?,?)',
        (d.ts or time.time(), d.event, d.label, d.score, d.image, d.task_id))
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
