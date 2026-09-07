"""计划调度器 + 模拟执行器。

- asyncio 后台协程每 30s 扫一次 plan 表，到点（next_run <= now 且 enabled）即触发；
- 执行器当前为模拟执行（逐点位标记成功 + 动作记录），真车/patrol_manager 到位后
  仅需替换 _execute_point 为真实导航+动作调用（经网关 WebSocket 出口）；
- 断网可跑：全部本机执行，不依赖任何外部服务。
"""
import asyncio
import json
import time
from datetime import datetime, timedelta

from . import db

_running = {}          # plan_id -> asyncio.Task
_loop = None           # FastAPI 主事件循环（startup 时捕获；sync 端点里没有当前 loop）


def start_task(plan):
    """启动一次任务（模拟执行），返回 task_record id。"""
    routes = db.query('SELECT * FROM route WHERE id=?', (plan['route_id'],))
    route_name = routes[0]['name'] if routes else ''
    point_ids = json.loads(routes[0]['point_ids'] or '[]') if routes else []
    points = []
    for pid in point_ids:
        rows = db.query('SELECT * FROM patrol_point WHERE id=?', (pid,))
        if rows:
            r = dict(rows[0])
            r['actions'] = json.loads(r.get('actions') or '[]')
            points.append(r)

    task_id = db.execute(
        'INSERT INTO task_record(plan_id,route_id,start_time,status,detail)'
        ' VALUES(?,?,?,?,?)',
        (plan['id'], plan['route_id'], time.time(), 'running', '[]'))

    def _spawn():
        _running[plan['id']] = _loop.create_task(
            _run_task(task_id, plan, route_name, points))
    if _loop is not None:
        _loop.call_soon_threadsafe(_spawn)   # sync 端点线程里没有事件循环，投递回主 loop
    return task_id


async def _run_task(task_id, plan, route_name, points):
    detail = []
    try:
        for p in points:
            detail.append(await _execute_point(p))
        status = 'success'
    except asyncio.CancelledError:
        status = 'canceled'
        raise
    except Exception as e:
        status = 'failed'
        detail.append({'error': str(e)})
    finally:
        db.execute(
            'UPDATE task_record SET end_time=?, status=?, detail=? WHERE id=?',
            (time.time(), status, json.dumps(detail, ensure_ascii=False), task_id))
        _running.pop(plan['id'], None)
        # 任务结束自动生成巡检报告（合同：每完成单次巡逻生成巡检报告）
        try:
            from . import report
            report.generate_report(task_id)
        except Exception as e:
            print(f'[scheduler] 报告生成失败 task={task_id}: {e}')


async def _execute_point(p):
    """模拟执行单个点位：导航(1s) + 动作序列（停留按真实秒数，封顶 10s）。"""
    rec = {'point': p['name'], 'arrive_ts': time.time(), 'actions': [], 'result': 'ok'}
    await asyncio.sleep(1.0)                      # 模拟导航到位
    for a in p['actions']:
        if a.get('type') == 'dwell':
            sec = min(int(a.get('sec', 0)), 10)   # 模拟器停留封顶 10s
            await asyncio.sleep(sec)
            rec['actions'].append({'type': 'dwell', 'sec': sec, 'ok': True})
        elif a.get('type') == 'capture':
            rec['actions'].append({'type': 'capture', 'ok': True})
        elif a.get('type') == 'tts':
            rec['actions'].append({'type': 'tts', 'text': a.get('text', ''), 'ok': True})
        elif a.get('type') == 'audio':
            rec['actions'].append({'type': 'audio', 'file': a.get('file', ''), 'ok': True})
    return rec


async def _tick():
    while True:
        try:
            now = time.time()
            for plan in db.query(
                    'SELECT * FROM plan WHERE enabled=1 AND next_run IS NOT NULL'
                    ' AND next_run<=?', (now,)):
                if plan['id'] in _running:
                    continue
                start_task(plan)
                # 计算下一次执行时间
                if plan['schedule_type'] == 'once':
                    nxt = None
                elif plan['schedule_type'] == 'daily':
                    nxt = _next_daily(plan['schedule_value'], now)
                else:
                    nxt = now + float(plan['schedule_value']) * 60
                db.execute(
                    'UPDATE plan SET last_run=?, next_run=? WHERE id=?',
                    (now, nxt, plan['id']))
        except Exception as e:
            print(f'[scheduler] tick 异常: {e}')
        await asyncio.sleep(30)


def _next_daily(hhmm, after):
    hm = time.strptime(hhmm, '%H:%M')
    base = datetime.fromtimestamp(after).replace(
        hour=hm.tm_hour, minute=hm.tm_min, second=0, microsecond=0)
    t = base.timestamp()
    if t <= after:
        t = (base + timedelta(days=1)).timestamp()
    return t


def start_scheduler():
    global _loop
    _loop = asyncio.get_event_loop()
    _loop.create_task(_tick())
