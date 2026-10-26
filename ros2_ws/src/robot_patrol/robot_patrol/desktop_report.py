"""Read-only task snapshots for the native desktop; no ROS or HTTP startup."""
from contextlib import closing
import argparse
import json
import os
from pathlib import Path
import sqlite3
import sys

from .reporting import build_report


def readonly(path):
    path = Path(path).expanduser().resolve()
    if not path.is_file():
        raise ValueError(f'记录数据库不存在：{path}')
    connection = sqlite3.connect(path.as_uri() + '?mode=ro', uri=True, timeout=5)
    connection.row_factory = sqlite3.Row
    connection.execute('PRAGMA query_only=ON')
    connection.execute('BEGIN')
    return connection


def approved_image(value, roots):
    if not value:
        return '', 'missing'
    path = Path(value).expanduser().resolve()
    if not any(path.is_relative_to(root) for root in roots):
        return '', 'outside_root'
    return (str(path), 'ready') if path.is_file() else ('', 'missing')


def load_report(run_id, project_root, patrol_db=None):
    project = Path(project_root).expanduser().resolve()
    data_home = Path(os.environ.get('XDG_DATA_HOME', Path.home() / '.local/share'))
    database = Path(patrol_db or os.environ.get('ROBOT_PATROL_DB',
                                              data_home / 'robot_patrol/patrol.sqlite3'))
    with closing(readonly(database)) as connection:
        row = connection.execute('SELECT finished_at FROM runs WHERE run_id=?', (run_id,)).fetchone()
        if row is None:
            raise ValueError('该任务不在本地巡航数据库中；请核对 ROBOT_PATROL_DB')
        if not row['finished_at']:
            raise ValueError('任务尚未结束，结束后可查看完整报告；当前到点结果可在执行记录中查看')
        report = build_report(connection, run_id)
        if report is None:
            raise ValueError('该任务尚未生成完整报告')
    inspection_root = Path(os.environ.get('ROBOT_INSPECTION_IMAGE_DIR',
                                           data_home / 'robot_patrol/inspections')).expanduser().resolve()
    local_root = Path(os.environ.get('ROBOT_VISION_DIR', project / 'data/vision')).expanduser().resolve()
    web_root = Path(os.environ.get('ROBOT_WEB_RECORDS_DIR', project / 'data/web_records')).expanduser().resolve()
    for result in report['inspection_results']:
        result['success'] = bool(result['success'])
        result['local_image_path'], result['media_status'] = approved_image(
            result.get('image_path', ''), [inspection_root])
    for attempt in report['stop_attempts']:
        attempt['actual_valid'] = bool(attempt['actual_valid'])

    run = report['run']
    events, warnings, readable = {}, [], 0
    # Read both the original local recorder and the durable HTTP inbox. The
    # former includes events that have not yet uploaded; event_id deduplicates.
    for directory, source in ((local_root, '本地视觉记录'), (web_root, '后台接收记录')):
        path = directory / 'events.sqlite3'
        if not path.is_file():
            if directory == local_root:
                warnings.append('本地视觉记录库不存在，未上报的本地视觉事件无法纳入；请核对 ROBOT_VISION_DIR。')
            continue
        try:
            with closing(readonly(path)) as connection:
                columns = {r['name'] for r in connection.execute('PRAGMA table_info(events)')}
                if not {'event_id', 'stamp', 'kind', 'payload'}.issubset(columns):
                    raise ValueError('视觉记录表格式不兼容')
                has_media = 'image' in columns
                rows = connection.execute(
                    "SELECT * FROM events WHERE stamp>=? AND stamp<=? "
                    "AND kind IN ('face_capture','person','vehicle_plate') ORDER BY stamp,event_id",
                    (run['started_at'], run['finished_at']))
                for row in rows:
                    event = json.loads(row['payload'])
                    if not isinstance(event, dict):
                        raise ValueError('视觉记录内容格式不兼容')
                    # Match the existing web report's task time/map scope.
                    # Unlocated events stay unlocated rather than receiving a
                    # fabricated pose from the task or the current vehicle.
                    if event.get('map_id', '') not in ('', run['map_id']):
                        continue
                    if event.get('run_id') and event['run_id'] != run_id:
                        continue
                    event.update(event_id=row['event_id'], event_time_ns=row['stamp'], event_type=row['kind'])
                    image = str(directory / 'media' / row['image']) if has_media and row['image'] else event.get('image_path', '')
                    event['local_image_path'], event['media_status'] = approved_image(image, [directory])
                    event.setdefault('image_path', image)
                    event['association'] = 'task_time_and_map'
                    previous = events.get(row['event_id'])
                    if previous is None:
                        event['record_sources'] = [source]
                        events[row['event_id']] = event
                    else:
                        previous['record_sources'].append(source)
                        if not previous.get('local_image_path') and event['local_image_path']:
                            previous['local_image_path'] = event['local_image_path']
                            previous['media_status'] = 'ready'
                readable += 1
        except (sqlite3.Error, ValueError, TypeError, KeyError, OSError) as error:
            warnings.append(f'{source}读取不完整：{error}')
    if not readable:
        warnings.append('未能读取独立视觉记录库；此报告保留到点结果和已有统计，不能据此认定没有人脸、行人或车牌事件。')
    report['vision_events'] = sorted(events.values(), key=lambda v: (v['event_time_ns'], v['event_id']))
    report['vision_event_counts'] = {kind: sum(v['event_type'] == kind for v in events.values())
                                     for kind in ('face_capture', 'person', 'vehicle_plate')}
    report['warnings'] = warnings
    report['vision_association'] = '按任务起止时间及地图匹配；无地图记录按时间归集，保留原定位有效性。'
    report['missing_image_count'] = sum(not item.get('local_image_path')
                                       for item in report['inspection_results'] + report['vision_events'])
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--project-root', required=True)
    parser.add_argument('--patrol-db')
    args = parser.parse_args(argv)
    try:
        report = load_report(args.run_id, args.project_root, args.patrol_db)
        sys.stdout.write(json.dumps(report, ensure_ascii=False, allow_nan=False))
    except (sqlite3.Error, OSError, ValueError, KeyError, TypeError) as error:
        sys.stderr.write(f'读取巡检报告失败：{error}\n')
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
