"""巡检路线 CRUD（路线 = 有序巡检点集合）。"""
import json
import time

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from . import db

router = APIRouter(prefix='/api/routes', tags=['routes'])


class RouteIn(BaseModel):
    name: str
    point_ids: list[int] = []


def _row_out(r):
    r = dict(r)
    try:
        r['point_ids'] = json.loads(r.get('point_ids') or '[]')
    except Exception:
        r['point_ids'] = []
    return r


def _points_of(route):
    """按顺序展开路线的点位详情。"""
    pts = []
    for pid in route['point_ids']:
        rows = db.query('SELECT * FROM patrol_point WHERE id=?', (pid,))
        if rows:
            pts.append(_point_out(rows[0]))
    return pts


def _point_out(r):
    r = dict(r)
    try:
        r['actions'] = json.loads(r.get('actions') or '[]')
    except Exception:
        r['actions'] = []
    return r


@router.post('')
def add_route(r: RouteIn):
    rid = db.execute(
        'INSERT INTO route(name,point_ids,created_at) VALUES(?,?,?)',
        (r.name, json.dumps(r.point_ids), time.time()))
    return {'ok': True, 'id': rid}


@router.get('')
def list_routes():
    rows = db.query('SELECT * FROM route ORDER BY id')
    out = []
    for r in rows:
        r = _row_out(r)
        r['points'] = _points_of(r)
        out.append(r)
    return {'items': out}


@router.get('/{rid}')
def get_route(rid: int):
    rows = db.query('SELECT * FROM route WHERE id=?', (rid,))
    if not rows:
        raise HTTPException(404, '路线不存在')
    r = _row_out(rows[0])
    r['points'] = _points_of(r)
    return r


@router.put('/{rid}')
def update_route(rid: int, r: RouteIn):
    if not db.query('SELECT id FROM route WHERE id=?', (rid,)):
        raise HTTPException(404, '路线不存在')
    db.execute('UPDATE route SET name=?, point_ids=? WHERE id=?',
               (r.name, json.dumps(r.point_ids), rid))
    return {'ok': True}


@router.delete('/{rid}')
def delete_route(rid: int):
    if not db.query('SELECT id FROM route WHERE id=?', (rid,)):
        raise HTTPException(404, '路线不存在')
    db.execute('DELETE FROM route WHERE id=?', (rid,))
    return {'ok': True}
