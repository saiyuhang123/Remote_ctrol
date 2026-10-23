"""Reports, inspection images and the durable vision-event HTTP inbox."""
from io import BytesIO
import base64
import json
import os
from pathlib import Path
import uuid

from fastapi import APIRouter, HTTPException, Query, Request
from fastapi.responses import FileResponse, Response
from PIL import Image

from . import db, vision_store
# Shared pure Python report code; a separate HTTP deployment needs no ROS runtime.
from ros2_ws.src.robot_patrol.robot_patrol.reporting import build_report, render_html

router = APIRouter(tags=['inspection'])
INSPECTION_IMAGE_DIR = Path(os.environ.get('ROBOT_INSPECTION_IMAGE_DIR',
    Path(os.environ.get('XDG_DATA_HOME', Path.home()/'.local/share'))/'robot_patrol/inspections')).expanduser().resolve()


def business():
    try:
        return db.schema_version()
    except db.DatabaseUnavailable as error:
        raise HTTPException(503, str(error))


def media_url(result):
    return '/api/inspection/results/'+result['result_id']+'/image' if result.get('image_path') else ''


@router.get('/api/local/maps/{map_id}/points')
def points(map_id: str):
    business()
    return {'items': db.query('SELECT * FROM points WHERE map_id=? ORDER BY name,point_id', (map_id,))}


@router.get('/api/local/maps/{map_id}/networks')
def networks(map_id: str):
    business()
    if not db.table_columns('road_networks'):
        return {'items': []}
    rows = db.query('SELECT network_id,name,map_revision,revision,published,document_json FROM road_networks WHERE map_id=? ORDER BY updated_at DESC', (map_id,))
    for row in rows:
        document = json.loads(row.pop('document_json'))
        row['bound_points'] = list(document.get('bindings', {}))
    return {'items': rows}


@router.get('/api/inspection/results')
def inspection_results(run_id: str = '', limit: int = Query(50, ge=1, le=200), offset: int = Query(0, ge=0)):
    business()
    if not db.table_columns('inspection_results'):
        return {'items': [], 'total': 0}
    where, args = (' WHERE run_id=?', (run_id,)) if run_id else ('', ())
    rows = db.query('SELECT * FROM inspection_results'+where+' ORDER BY started_at DESC,result_id DESC LIMIT ? OFFSET ?',
                    (*args, limit, offset))
    for item in rows:
        item['image_url'] = media_url(item)
    total = db.query_one('SELECT COUNT(*) AS n FROM inspection_results'+where, args)['n']
    return {'items': rows, 'total': total}


@router.get('/api/inspection/results/{result_id}/image')
def inspection_image(result_id: str):
    business()
    row = db.query_one('SELECT image_path FROM inspection_results WHERE result_id=?', (result_id,))
    if row is None or not row['image_path']:
        raise HTTPException(404, '巡检图片不存在')
    path = approved_inspection_path(row['image_path'])
    if path is None:
        raise HTTPException(404, '巡检图片不存在或不在配置的图片目录中')
    return FileResponse(path, media_type='image/jpeg')


def approved_inspection_path(value):
    path = Path(value).expanduser().resolve()
    return path if path.is_relative_to(INSPECTION_IMAGE_DIR) and path.is_file() else None


def run_filter(run_id):
    if not run_id:
        return {}
    business()
    run = db.query_one('SELECT map_id,started_at,finished_at FROM runs WHERE run_id=?', (run_id,))
    if run is None:
        raise HTTPException(404, '巡检任务不存在')
    return {'start_ns': run['started_at'], 'end_ns': run['finished_at'] or None,
            'map_id': run['map_id'], 'include_unmapped': True}


@router.get('/api/vision/events')
def vision_events(event_type: str = '', run_id: str = '', cursor: str = '',
                  limit: int = Query(50, ge=1, le=200)):
    try:
        return vision_store.query(kind=event_type, limit=limit, cursor=cursor, **run_filter(run_id))
    except (ValueError, TypeError, json.JSONDecodeError) as error:
        raise HTTPException(422, str(error))


@router.get('/api/vision/media/{event_id}')
def vision_image(event_id: str):
    path = vision_store.image_path(event_id)
    if path is None:
        raise HTTPException(404, '识别图片不存在')
    return FileResponse(path, media_type='image/jpeg')


@router.post('/api/records/detections')
async def receive_detection(request: Request):
    content_type = request.headers.get('content-type', '')
    image_bytes = b''
    temporary = None
    try:
        if content_type.startswith('multipart/form-data'):
            form = await request.form()
            payload = json.loads(form.get('event', '{}'))
            upload = form.get('image')
            if upload is not None:
                image_bytes = await upload.read(8*1024*1024+1)
                await upload.close()
                if len(image_bytes)>8*1024*1024:
                    raise HTTPException(413, '识别图片不能超过 8MB')
        else:
            raw = await request.body()
            if len(raw)>256*1024:
                raise HTTPException(413, '识别记录过大')
            payload = json.loads(raw)
        if not isinstance(payload, dict):
            raise ValueError('识别记录必须是对象')
        event = vision_store.normalize(payload)
        image_name = ''
        if image_bytes:
            with Image.open(BytesIO(image_bytes)) as original:
                if original.width*original.height > 20_000_000:
                    raise ValueError('图片尺寸过大')
                image = original.convert('RGB')
            directory = vision_store.DIRECTORY/'media'
            directory.mkdir(parents=True, exist_ok=True)
            image_name = event['event_id']+'.jpg'
            temporary = directory/(uuid.uuid4().hex+'.tmp')
            image.save(temporary, format='JPEG', quality=90)
        # Validate the event identity before promoting its image. Retries must
        # preserve the original picture and conflicting IDs cannot overwrite it.
        inserted = vision_store.receive(event, image_name, temporary)
        return {'ok': True, 'event_id': event['event_id'], 'inserted': inserted}
    except (ValueError, TypeError, KeyError, OSError, Image.DecompressionBombError) as error:
        raise HTTPException(422, str(error))
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


@router.get('/api/inspection/reports')
def reports(limit: int = Query(100, ge=1, le=500), offset: int = Query(0, ge=0)):
    business()
    rows = db.query('SELECT r.run_id,r.route_id,r.map_id,r.state,r.success,r.completed_steps,r.completed_loops,'
                    'r.started_at,r.finished_at,r.error_code,r.message,r.route_snapshot_json '
                    'FROM runs r WHERE r.finished_at>0 ORDER BY r.started_at DESC LIMIT ? OFFSET ?', (limit, offset))
    for row in rows:
        snapshot = json.loads(row.pop('route_snapshot_json'))
        row['route_name'] = snapshot.get('name', row['route_id'])
        row['report_url'] = '/api/inspection/reports/'+row['run_id']
    total = db.query_one('SELECT COUNT(*) AS n FROM runs WHERE finished_at>0')['n']
    return {'items': rows, 'total': total}


def load_report(run_id):
    business()
    report = None
    if db.table_columns('inspection_reports'):
        row = db.query_one('SELECT payload_json FROM inspection_reports WHERE run_id=?', (run_id,))
        if row:
            report = json.loads(row['payload_json'])
    if report is None:
        report = build_report(db.conn(), run_id)
    if report is None:
        raise HTTPException(404, '报告尚未生成，请等待任务结束')
    for item in report['inspection_results']:
        item['image_url'] = media_url(item)
    events, cursor = [], ''
    filters = run_filter(run_id)
    while True:
        page = vision_store.query(limit=200, cursor=cursor, **filters)
        events.extend(page['items'])
        cursor = page['next_cursor']
        if not cursor:
            break
    report['vision_events'] = events
    return report


@router.get('/api/inspection/reports/{run_id}')
def report_detail(run_id: str):
    return load_report(run_id)


@router.get('/api/inspection/reports/{run_id}/download')
def report_download(run_id: str, format: str = 'html'):
    if format not in ('html', 'json'):
        raise HTTPException(422, '仅支持 HTML 或 JSON 报告')
    report = load_report(run_id)
    if format == 'html':
        # Downloaded reports remain viewable away from the robot's web server.
        for item in report['inspection_results']+report['vision_events']:
            path = (approved_inspection_path(item.get('image_path', '')) if 'result_id' in item
                    else vision_store.image_path(item['event_id']))
            item['image_url'] = ''
            if path:
                try:
                    with Image.open(path) as original:
                        picture = original.convert('RGB')
                        picture.thumbnail((960, 960))
                        output = BytesIO()
                        picture.save(output, format='JPEG', quality=80)
                    item['image_url'] = 'data:image/jpeg;base64,'+base64.b64encode(output.getvalue()).decode('ascii')
                except (OSError, ValueError):
                    pass
    body = render_html(report) if format == 'html' else json.dumps(report, ensure_ascii=False, indent=2)
    # The filename is generated from a validated database identifier, never a path.
    filename = 'inspection-'+''.join(ch for ch in run_id if ch.isascii() and (ch.isalnum() or ch in '_-'))+'.'+format
    return Response(body, media_type='text/html' if format == 'html' else 'application/json',
                    headers={'Content-Disposition': 'attachment; filename="'+filename+'"'})
