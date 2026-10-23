from io import BytesIO
import json
import sqlite3
import wave

from PIL import Image
import pytest

from test_readonly_api import _build_fixture


def fixture(tmp_path, monkeypatch):
    monkeypatch.setenv('ROBOT_AUDIO_DIR', str(tmp_path/'audio'))
    monkeypatch.setenv('ROBOT_WEB_RECORDS_DIR', str(tmp_path/'inbox'))
    monkeypatch.setenv('ROBOT_INSPECTION_IMAGE_DIR', str(tmp_path/'inspections'))
    return _build_fixture(tmp_path, monkeypatch)


@pytest.mark.parametrize('version', [2,3,4,5,6])
def test_backend_accepts_supported_patrol_versions(tmp_path, monkeypatch, version):
    client, mapping, route = fixture(tmp_path, monkeypatch)
    with sqlite3.connect(tmp_path/'patrol.sqlite3') as connection:
        connection.execute('UPDATE schema_version SET version=?', (version,))
    response = client.get('/api/local/state')
    assert response.status_code==200, response.text
    assert response.json()['schema_version']==version
    assert response.json()['routes'][0]['route_id']==route.route_id
    assert client.get('/api/inspection/records').status_code==200


def test_unknown_database_version_is_rejected_without_modification(tmp_path, monkeypatch):
    client, _, _ = fixture(tmp_path, monkeypatch)
    with sqlite3.connect(tmp_path/'patrol.sqlite3') as connection:
        connection.execute('UPDATE schema_version SET version=99')
    assert client.get('/api/local/state').status_code==503
    with sqlite3.connect(tmp_path/'patrol.sqlite3') as connection:
        assert connection.execute('SELECT version FROM schema_version').fetchone()[0]==99


def jpeg(color='green'):
    stream=BytesIO()
    Image.new('RGB', (32,24), color).save(stream, format='JPEG')
    return stream.getvalue()


def event(identifier, stamp=12_000_000_000, map_id='', kind='face_capture', pose=False):
    return dict(event_id=identifier,event_time_ns=stamp,event_type=kind,label='<script>unsafe</script>',
                confidence=.91,map_id=map_id,pose_valid=pose,robot_x=1.,robot_y=2.,robot_yaw=.3,
                image_path='/private/device/image.jpg',native_image_path='/private/raw.jpg')


def test_vision_upload_is_idempotent_includes_picture_and_paginates(tmp_path, monkeypatch):
    client, mapping, _ = fixture(tmp_path, monkeypatch)
    value=event('one', map_id=mapping.map_id, pose=True)
    for inserted in (True, False):
        response=client.post('/api/records/detections',data={'event':json.dumps(value)},
                             files={'image':('image.jpg',jpeg(),'image/jpeg')})
        assert response.status_code==200, response.text
        assert response.json()['inserted'] is inserted
    assert client.post('/api/records/detections',json=event('two',kind='person')).status_code==200
    data=client.get('/api/vision/events?limit=1').json()
    assert data['total']==2 and data['next_cursor']
    following=client.get('/api/vision/events',params={'limit':1,'cursor':data['next_cursor']}).json()
    assert data['items'][0]['event_id']!=following['items'][0]['event_id']
    item=client.get('/api/vision/events?event_type=face_capture').json()['items'][0]
    assert item['pose_valid'] and item['robot_x']==1.
    assert 'image_path' not in item and 'native_image_path' not in item
    picture=client.get(item['image_url'])
    assert picture.status_code==200 and picture.content.startswith(b'\xff\xd8')
    for updated, status in ((value, 200), (value|{'event_time_ns':13_000_000_000}, 422)):
        retry=client.post('/api/records/detections',data={'event':json.dumps(updated)},
                          files={'image':('image.jpg',jpeg('red'),'image/jpeg')})
        assert retry.status_code==status
        assert client.get(item['image_url']).content==picture.content
    assert not list((tmp_path/'inbox/media').glob('*.tmp'))
    assert client.get('/api/vision/events?event_type=bad').status_code==422
    assert client.get('/api/vision/events?cursor=bad').status_code==422
    assert client.post('/api/records/detections',json=event('../escape')).status_code==422
    assert client.post('/api/records/detections',json=event('nan',pose=True)|{'robot_x':float('inf')}).status_code==422


def completed_task(tmp_path, mapping, route):
    from robot_patrol.store import PatrolStore
    from robot_patrol.models import InspectionResult
    store=PatrolStore(tmp_path/'patrol.sqlite3')
    steps=store.get_route(route.route_id).steps
    store.create_run('run1',route.route_id,mapping.map_id,1,1,
                     {'name':'仪表巡检<script>','steps':[{'point_id':steps[0].point_id}]},1,1,10_000_000_000)
    image_path=tmp_path/'inspections/run1/photo.jpg'
    image_path.parent.mkdir(parents=True)
    image_path.write_bytes(jpeg())
    store.add_inspection_result(InspectionResult(
        result_id='result1',run_id='run1',map_id=mapping.map_id,point_id=steps[0].point_id,
        point_name='仪表 A',loop_index=1,step_index=1,started_at_ns=11_000_000_000,
        finished_at_ns=13_000_000_000,ptz_preset=1,prompt='读取电压',image_path=str(image_path),
        text='电压 220 V <script>',model='fake',success=True,error_code='',message='OK'))
    store.add_attempt('attempt1','run1',1,1,1,'goal',(1.,2.,0.),(1.01,2.,0.),11_000_000_000,'SUCCEEDED')
    return store


def test_reports_generate_on_finish_and_download_with_offline_images(tmp_path, monkeypatch):
    client,mapping,route=fixture(tmp_path,monkeypatch)
    store=completed_task(tmp_path,mapping,route)
    assert client.get('/api/inspection/reports/run1').status_code==404
    store.finish_run('run1',10,True,1,1,'','巡检完成',20_000_000_000)
    for value in (event('face',map_id=mapping.map_id),event('no_pose',stamp=14_000_000_000),
                  event('other_map',stamp=15_000_000_000,map_id='another-map')):
        response=client.post('/api/records/detections',data={'event':json.dumps(value)},
                             files={'image':('face.jpg',jpeg(),'image/jpeg')})
        assert response.status_code==200, response.text
    report=client.get('/api/inspection/reports/run1').json()
    assert report['run']['success'] and report['inspection_success_count']==1
    assert len(report['vision_events'])==2
    assert len(report['stop_attempts'])==1
    assert client.get(report['inspection_results'][0]['image_url']).status_code==200
    html=client.get('/api/inspection/reports/run1/download?format=html')
    assert html.status_code==200 and 'attachment' in html.headers['content-disposition']
    assert 'data:image/jpeg;base64,' in html.text
    assert '<script>' not in html.text and '&lt;script&gt;' in html.text
    exported=client.get('/api/inspection/reports/run1/download?format=json').json()
    assert exported['run_id']=='run1' and exported['inspection_results'][0]['text']=='电压 220 V <script>'
    assert client.get('/api/inspection/reports').json()['items'][0]['run_id']=='run1'
    assert client.get('/api/inspection/results?run_id=run1').json()['total']==1
    store.close()


def test_v5_reports_are_read_without_backend_migrating_the_database(tmp_path, monkeypatch):
    client,mapping,route=fixture(tmp_path,monkeypatch)
    store=completed_task(tmp_path,mapping,route)
    store.finish_run('run1',9,False,0,0,'CANCELED','取消',20_000_000_000)
    store._conn.execute('DROP TABLE inspection_reports')
    store._conn.execute('UPDATE schema_version SET version=5')
    store.close()
    report=client.get('/api/inspection/reports/run1')
    assert report.status_code==200, report.text
    assert report.json()['run']['state']==9
    with sqlite3.connect(tmp_path/'patrol.sqlite3') as connection:
        assert not connection.execute("SELECT 1 FROM sqlite_master WHERE name='inspection_reports'").fetchone()
        assert connection.execute('SELECT version FROM schema_version').fetchone()[0]==5


def test_audio_upload_validates_content_and_lists_persistent_assets(tmp_path, monkeypatch):
    client,_,_=fixture(tmp_path,monkeypatch)
    stream=BytesIO()
    with wave.open(stream,'wb') as output:
        output.setnchannels(1); output.setsampwidth(2); output.setframerate(8000)
        output.writeframes(b'\0\0'*1600)
    audio=stream.getvalue()
    response=client.post('/api/audio/files',files={'file':('播报.wav',audio,'audio/wav')})
    assert response.status_code==200, response.text
    item=response.json()
    assert item['name']=='播报.wav' and .19<item['duration_seconds']<.21
    assert (tmp_path/'audio'/item['filename']).read_bytes()==audio
    assert client.get('/api/audio/files').json()['items'][0]['filename']==item['filename']
    assert client.post('/api/audio/files',files={'file':('fake.wav',b'not audio','audio/wav')}).status_code==422
    assert client.post('/api/audio/files',files={'file':('../bad.exe',audio,'audio/wav')}).status_code==422
    assert client.post('/api/audio/files',files={'file':('large.wav',audio,'audio/wav')},
                       headers={'content-length':str(22*1024*1024)}).status_code==413
    assert len(client.get('/api/audio/files').json()['items'])==1


def test_manual_captures_are_durable_paged_and_reject_path_escape(tmp_path, monkeypatch):
    client,_,_=fixture(tmp_path,monkeypatch)
    from backend import api_captures
    monkeypatch.setattr(api_captures,'WEB',tmp_path/'web')
    picture=api_captures.WEB/'captures/2026-10-22/one.jpg'
    picture.parent.mkdir(parents=True);picture.write_bytes(jpeg())
    value=dict(capture_id='capture1',label='<script>test</script>',ts=123.,image='captures/2026-10-22/one.jpg')
    assert client.post('/api/records/captures',json=value).status_code==200
    assert client.post('/api/records/captures',json=value).status_code==200
    assert client.post('/api/records/captures',json=value|{'image':'captures/2026-10-22/two.jpg'}).status_code==409
    assert client.post('/api/records/captures',json=value|{'image':'captures/../../private.jpg'}).status_code==422
    assert client.post('/api/records/captures',json=value|{'capture_id':'capture2','image':'captures/2026-10-22/two.jpg','ts':124.}).status_code==200
    page=client.get('/api/records/captures?limit=1').json()
    assert page['total']==2 and page['items'][0]['capture_id']=='capture2' and not page['items'][0]['available']
    following=client.get('/api/records/captures?limit=1&offset=1').json()
    assert following['items'][0]['available'] and following['items'][0]['image_url']=='/captures/2026-10-22/one.jpg'
