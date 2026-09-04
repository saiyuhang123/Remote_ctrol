"""巡检计划 CRUD + 立即执行 + 任务记录查询。

schedule_type:
  once   — schedule_value="2026-09-05 09:00"
  daily  — schedule_value="09:00"
  interval_min — schedule_value="120"（分钟）
"""
import time
from datetime import datetime, timedelta

from fastapi import APIRouter, HTTPException
from fastapi.responses import RedirectResponse
from pydantic import BaseModel

from . import db
from . import scheduler

router = APIRouter(prefix='/api/plans', tags=['plans'])


class PlanIn(BaseModel):
    name: str
    route_id: int
    schedule_type: str            # once / daily / interval_min
    schedule_value: str
    enabled: bool = True


def _next_run(schedule_type, schedule_value, after=None):
    """计算下次执行时间（epoch 秒），无法计算返回 None。"""
    now = after or time.time()
    try:
        if schedule_type == 'once':
            return time.mktime(time.strptime(schedule_value, '%Y-%m-%d %H:%M'))
        if schedule_type == 'daily':
            hm = time.strptime(schedule_value, '%H:%M')
            today = datetime.now().replace(hour=hm.tm_hour, minute=hm.tm_min,
                                           second=0, microsecond=0)
            t = today.timestamp()
            return t if t > now else (today + timedelta(days=1)).timestamp()
        if schedule_type == 'interval_min':
            return now + float(schedule_value) * 60
    except Exception:
        return None
    return None


def _row_out(r):
    return dict(r)


@router.post('')
def add_plan(p: PlanIn):
    if not db.query('SELECT id FROM route WHERE id=?', (p.route_id,)):
        raise HTTPException(404, '路线不存在')
    nxt = _next_run(p.schedule_type, p.schedule_value)
    rid = db.execute(
        'INSERT INTO plan(name,route_id,schedule_type,schedule_value,enabled,next_run,created_at)'
        ' VALUES(?,?,?,?,?,?,?)',
        (p.name, p.route_id, p.schedule_type, p.schedule_value,
         1 if p.enabled else 0, nxt, time.time()))
    return {'ok': True, 'id': rid, 'next_run': nxt}


@router.get('')
def list_plans():
    return {'items': [_row_out(r) for r in db.query('SELECT * FROM plan ORDER BY id')]}


@router.put('/{pid}')
def update_plan(pid: int, p: PlanIn):
    if not db.query('SELECT id FROM plan WHERE id=?', (pid,)):
        raise HTTPException(404, '计划不存在')
    nxt = _next_run(p.schedule_type, p.schedule_value)
    db.execute(
        'UPDATE plan SET name=?, route_id=?, schedule_type=?, schedule_value=?, enabled=?, next_run=? WHERE id=?',
        (p.name, p.route_id, p.schedule_type, p.schedule_value,
         1 if p.enabled else 0, nxt, pid))
    return {'ok': True, 'next_run': nxt}


@router.delete('/{pid}')
def delete_plan(pid: int):
    if not db.query('SELECT id FROM plan WHERE id=?', (pid,)):
        raise HTTPException(404, '计划不存在')
    db.execute('DELETE FROM plan WHERE id=?', (pid,))
    return {'ok': True}


@router.post('/{pid}/run')
def run_now(pid: int):
    rows = db.query('SELECT * FROM plan WHERE id=?', (pid,))
    if not rows:
        raise HTTPException(404, '计划不存在')
    task_id = scheduler.start_task(rows[0])
    return {'ok': True, 'task_id': task_id}


# ---- 任务记录 ----
@router.get('/tasks')
def list_tasks(limit: int = 50):
    return {'items': [_row_out(r) for r in db.query(
        'SELECT * FROM task_record ORDER BY id DESC LIMIT ?', (limit,))]}


@router.get('/tasks/{tid}')
def get_task(tid: int):
    rows = db.query('SELECT * FROM task_record WHERE id=?', (tid,))
    if not rows:
        raise HTTPException(404, '任务不存在')
    return _row_out(rows[0])


@router.get('/report/{tid}')
def get_report(tid: int):
    """查看巡检报告；不存在则现场生成。"""
    rows = db.query('SELECT report_path FROM task_record WHERE id=?', (tid,))
    if not rows:
        raise HTTPException(404, '任务不存在')
    path = rows[0]['report_path']
    if not path:
        from . import report
        if not report.generate_report(tid):
            raise HTTPException(500, '报告生成失败')
        path = db.query('SELECT report_path FROM task_record WHERE id=?', (tid,))[0]['report_path']
    return RedirectResponse(url='/' + path)
