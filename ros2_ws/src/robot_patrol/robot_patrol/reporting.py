"""Task reports shared by the patrol writer and the read-only web backend."""
from datetime import datetime
from html import escape
import json
import time


STATE_NAMES = {9: '已取消', 10: '已完成', 11: '失败', 12: '已中断'}


def build_report(connection, run_id):
    def exists(table):
        return connection.execute("SELECT 1 FROM sqlite_master WHERE type='table' AND name=?", (table,)).fetchone() is not None
    row = connection.execute('SELECT * FROM runs WHERE run_id=?', (run_id,)).fetchone()
    if row is None:
        return None
    run = dict(row)
    if not run['finished_at']:
        return None
    snapshot = json.loads(run.pop('route_snapshot_json'))
    mapping = connection.execute('SELECT name FROM maps WHERE map_id=?', (run['map_id'],)).fetchone()
    results = [dict(item) for item in connection.execute(
        'SELECT * FROM inspection_results WHERE run_id=? ORDER BY loop_index,step_index,started_at',
        (run_id,))] if exists('inspection_results') else []
    attempts = [dict(item) for item in connection.execute(
        'SELECT * FROM stop_attempts WHERE run_id=? ORDER BY loop_index,step_index,attempt_index',
        (run_id,))] if exists('stop_attempts') else []
    summary = connection.execute('SELECT * FROM inspection_summary WHERE run_id=?', (run_id,)).fetchone() if exists('inspection_summary') else None
    summary = dict(summary) if summary else {}
    plates = json.loads(summary.pop('plates_json', '[]'))
    return dict(
        schema_version=1, run_id=run_id, generated_at_ns=time.time_ns(),
        run=run, route=snapshot, map_name=mapping['name'] if mapping else run['map_id'],
        state_name=STATE_NAMES.get(run['state'], str(run['state'])),
        duration_seconds=max(0., (run['finished_at']-run['started_at'])/1e9),
        inspection_results=results, stop_attempts=attempts,
        summary=dict(face_capture_count=summary.get('face_capture_count', 0),
                     person_count=summary.get('person_count', 0), plates=plates),
        inspection_success_count=sum(bool(item['success']) for item in results),
        inspection_failure_count=sum(not item['success'] for item in results),
    )


def render_html(report):
    """Self-contained, escaped UTF-8 report suitable for printing to PDF."""
    def safe(value):
        return escape(str(value if value is not None else '—'), quote=True)

    def date(value):
        return datetime.fromtimestamp(value/1e9).astimezone().isoformat(sep=' ', timespec='seconds') if value else '—'

    def table(headers, rows):
        return '<table><thead><tr>' + ''.join('<th>'+safe(v)+'</th>' for v in headers) + \
            '</tr></thead><tbody>' + ''.join('<tr>'+''.join('<td>'+safe(v)+'</td>' for v in row)+'</tr>' for row in rows) + '</tbody></table>'

    run, route, summary = report['run'], report['route'], report['summary']
    body = '<h1>巡检报告</h1><p>' + safe(route.get('name', run['route_id'])) + '</p>'
    body += table(['项目', '结果'], [
        ('运行编号', run['run_id']), ('地图', report['map_name']),
        ('任务状态', report['state_name']), ('开始时间', date(run['started_at'])),
        ('结束时间', date(run['finished_at'])), ('耗时', f"{report['duration_seconds']:.1f} 秒"),
        ('完成圈数 / 趟数', run['completed_loops']), ('完成点数', run['completed_steps']),
        ('抓拍识别成功 / 失败', f"{report['inspection_success_count']} / {report['inspection_failure_count']}"),
        ('人脸 / 行人', f"{summary['face_capture_count']} / {summary['person_count']}"),
        ('车牌', '、'.join(str(v.get('plate', '')) for v in summary['plates']) or '无'),
        ('任务异常', run['error_code'] or '无'), ('任务说明', run['message'] or '—')])
    body += '<h2>巡检点结果</h2>' + table(['圈/趟', '点位', '结果', '识别文本', '异常'], [
        (v['loop_index'], v['point_name'] or v['point_id'], '成功' if v['success'] else '失败',
         v['text'], v['error_code'] or v['message']) for v in report['inspection_results']])
    for item in report['inspection_results']:
        if item.get('image_url'):
            body += '<figure><img loading="lazy" src="'+safe(item['image_url'])+'"><figcaption>'+safe(item['point_name'])+'</figcaption></figure>'
    body += '<h2>到点与停车记录</h2>' + table(['圈/趟', '点序', '尝试', '结果', '位置误差(m)', '异常'], [
        (v['loop_index'], v['step_index'], v['attempt_index'], v['result'],
         f"{v['position_error']:.3f}" if v['actual_valid'] else '无实测', v['error_code'])
        for v in report['stop_attempts']])
    events = report.get('vision_events', [])
    body += '<h2>视觉识别记录</h2>' + table(['时间', '类型', '识别内容', '地图位置'], [
        (date(v['event_time_ns']), v['event_type'], v.get('label', ''),
         f"{v['robot_x']:.2f}, {v['robot_y']:.2f}" if v.get('pose_valid') else '无定位') for v in events])
    for item in events:
        if item.get('image_url'):
            body += '<figure><img loading="lazy" src="'+safe(item['image_url'])+'"><figcaption>'+safe(item.get('label', ''))+'</figcaption></figure>'
    return '<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>巡检报告 '+safe(run['run_id'])+'</title><style>body{font:15px sans-serif;max-width:1100px;margin:32px auto;color:#172638}table{border-collapse:collapse;width:100%;margin:16px 0}th,td{border:1px solid #ccd5df;padding:9px;text-align:left;white-space:pre-wrap;overflow-wrap:anywhere}th{background:#eef3f8}figure{display:inline-block;max-width:46%;margin:10px}img{max-width:100%;max-height:360px}h2{margin-top:32px}@media print{figure, tr{break-inside:avoid}body{margin:0}}</style>'+body+'</html>'
