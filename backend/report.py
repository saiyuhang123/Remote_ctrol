"""巡检报告生成（HTML）。

任务结束时由调度器自动调用；也可通过 GET /api/report/{task_id} 手动生成/查看。
报告文件落盘 web/reports/task_<id>.html（静态可访问，便于后台浏览/下载）。
"""
import html as html_lib
import json
import time
from pathlib import Path

from . import db

REPORTS_DIR = Path.home() / 'ros2Project' / 'Remote_ctrol' / 'web' / 'reports'

_STATUS_CN = {'success': '成功', 'failed': '失败', 'canceled': '已取消', 'running': '进行中'}

_CSS = """
body{font-family:"Microsoft YaHei",sans-serif;background:#f5f6f8;color:#222;margin:0;padding:24px}
h1{font-size:20px} h2{font-size:15px;color:#444;margin:22px 0 8px}
.card{background:#fff;border:1px solid #e0e3e8;border-radius:8px;padding:16px;margin-bottom:14px}
table{width:100%;border-collapse:collapse;font-size:13px}
th,td{padding:7px 10px;border-bottom:1px solid #eee;text-align:left}
th{background:#f0f2f5;color:#555}
.badge{padding:2px 10px;border-radius:10px;color:#fff;font-size:12px}
.ok{background:#43a047}.bad{background:#e53935}.mid{background:#fb8c00}
img.thumb{width:110px;border-radius:4px;cursor:pointer}
.meta{color:#666;font-size:13px;line-height:1.9}
"""


def _esc(s):
    return html_lib.escape(str(s))


def _fmt(ts):
    return time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(ts)) if ts else '-'


def generate_report(task_id):
    """生成巡检报告 HTML，返回文件路径。"""
    rows = db.query('SELECT * FROM task_record WHERE id=?', (task_id,))
    if not rows:
        return None
    task = rows[0]

    plan = db.query('SELECT * FROM plan WHERE id=?', (task['plan_id'],))
    route = db.query('SELECT * FROM route WHERE id=?', (task['route_id'],))
    plan_name = plan[0]['name'] if plan else f"#{task['plan_id']}"
    route_name = route[0]['name'] if route else ''

    try:
        detail = json.loads(task['detail'] or '[]')
    except Exception:
        detail = []

    t0, t1 = task['start_time'] or 0, task['end_time'] or time.time()
    detections = db.query(
        'SELECT * FROM detection_record WHERE ts>=? AND ts<=? ORDER BY ts', (t0, t1))
    captures = db.query(
        'SELECT * FROM capture_record WHERE ts>=? AND ts<=? ORDER BY ts', (t0, t1))

    dur = f"{int(t1 - t0)} 秒" if task['end_time'] else '-'
    status = task['status']
    badge = {'success': 'ok', 'failed': 'bad'}.get(status, 'mid')

    # 点位明细
    point_rows = ''
    for d in detail:
        acts = '、'.join(
            f"{ {'dwell':'停留'+str(a.get('sec',0))+'s','capture':'抓拍','tts':'播报'}.get(a.get('type'), a.get('type','')) }"
            for a in d.get('actions', []))
        point_rows += (f"<tr><td>{_esc(d.get('point',''))}</td><td>{_fmt(d.get('arrive_ts'))}</td>"
                       f"<td>{_esc(acts) or '-'}</td><td>{_esc(d.get('result',''))}</td></tr>")
    if not point_rows:
        point_rows = '<tr><td colspan="4">无</td></tr>'

    # 识别记录
    det_rows = ''
    for d in detections:
        img = f'<a href="/{_esc(d["image"])}" target="_blank"><img class="thumb" src="/{_esc(d["image"])}"></a>' if d['image'] else '-'
        det_rows += (f"<tr><td>{_fmt(d['ts'])}</td><td>{_esc(d['event'])}</td>"
                     f"<td>{_esc(d['label'])}</td><td>{d['score']:.2f}</td><td>{img}</td></tr>")
    if not det_rows:
        det_rows = '<tr><td colspan="5">期间无识别事件</td></tr>'

    # 抓拍
    cap_html = ''.join(
        f'<a href="/{_esc(c["image"])}" target="_blank">'
        f'<img class="thumb" src="/{_esc(c["image"])}" title="{_fmt(c["ts"])}"></a> '
        for c in captures) or '<span class="meta">无</span>'

    html = f"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><title>巡检报告 #{task_id}</title>
<style>{_CSS}</style></head><body>
<h1>巡检报告 <span class="badge {badge}">{_STATUS_CN.get(status, status)}</span></h1>
<div class="card"><div class="meta">
任务编号：#{task_id}　计划：{_esc(plan_name)}　路线：{_esc(route_name)}<br>
开始：{_fmt(t0)}　结束：{_fmt(task['end_time'])}　耗时：{dur}<br>
点位数：{len(detail)}　期间识别事件：{len(detections)}　期间抓拍：{len(captures)}
</div></div>

<h2>点位执行明细</h2>
<div class="card"><table>
<tr><th>点位</th><th>到达时间</th><th>执行动作</th><th>结果</th></tr>
{point_rows}
</table></div>

<h2>期间识别记录</h2>
<div class="card"><table>
<tr><th>时间</th><th>类型</th><th>内容</th><th>置信度</th><th>图片</th></tr>
{det_rows}
</table></div>

<h2>期间抓拍</h2>
<div class="card">{cap_html}</div>

<div class="meta" style="margin-top:16px">生成时间：{_fmt(time.time())} · 巡检机器人管理系统</div>
</body></html>"""

    REPORTS_DIR.mkdir(parents=True, exist_ok=True)
    path = REPORTS_DIR / f'task_{task_id}.html'
    path.write_text(html, encoding='utf-8')
    db.execute('UPDATE task_record SET report_path=? WHERE id=?',
               (f'reports/task_{task_id}.html', task_id))
    return str(path)
