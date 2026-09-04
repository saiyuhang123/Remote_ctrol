"""巡检点 CRUD API（合同：巡检点预置）。"""
import json
import time

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from . import db

router = APIRouter(prefix='/api/points', tags=['points'])


class PointIn(BaseModel):
    name: str
    x: float
    y: float
    yaw: float = 0.0
    actions: list = []           # [{"type":"dwell","sec":10},{"type":"capture"},{"type":"tts","text":"..."}]


def _row_out(r):
    r = dict(r)
    try:
        r['actions'] = json.loads(r.get('actions') or '[]')
    except Exception:
        r['actions'] = []
    return r


@router.post('')
def add_point(p: PointIn):
    rid = db.execute(
        'INSERT INTO patrol_point(name,x,y,yaw,actions,created_at) VALUES(?,?,?,?,?,?)',
        (p.name, p.x, p.y, p.yaw, json.dumps(p.actions, ensure_ascii=False), time.time()))
    return {'ok': True, 'id': rid}


@router.get('')
def list_points():
    rows = db.query('SELECT * FROM patrol_point ORDER BY id')
    return {'items': [_row_out(r) for r in rows]}


@router.put('/{pid}')
def update_point(pid: int, p: PointIn):
    n = db.execute(
        'UPDATE patrol_point SET name=?, x=?, y=?, yaw=?, actions=? WHERE id=?',
        (p.name, p.x, p.y, p.yaw, json.dumps(p.actions, ensure_ascii=False), pid))
    if not db.query('SELECT id FROM patrol_point WHERE id=?', (pid,)):
        raise HTTPException(404, '点位不存在')
    return {'ok': True}


@router.delete('/{pid}')
def delete_point(pid: int):
    if not db.query('SELECT id FROM patrol_point WHERE id=?', (pid,)):
        raise HTTPException(404, '点位不存在')
    db.execute('DELETE FROM patrol_point WHERE id=?', (pid,))
    return {'ok': True}
