"""Real browser regressions, with local fake HTTP/WS endpoints only.

Run with WEB_BROWSER_TESTS=1. Requires playwright and its Chromium runtime.
No ROS, real gateway, camera, serial device or chassis is contacted.
"""
import asyncio
from io import BytesIO
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import threading
import time
from urllib.parse import urlsplit

import pytest
from PIL import Image
import websockets

pytestmark = pytest.mark.skipif(os.environ.get('WEB_BROWSER_TESTS') != '1',
                                reason='explicit browser test opt-in required')
WEB = Path(__file__).resolve().parents[2] / 'web'


def wait_for(predicate, timeout=4):
    until = time.monotonic() + timeout
    while time.monotonic() < until:
        if predicate():
            return
        time.sleep(.03)
    assert predicate()


class RobotFixture:
    def __init__(self):
        self.clients = set()
        self.owner = None
        self.token = None
        self.task = {'kind': '', 'running': False}
        self.mode = 'idle'
        self.heartbeat = 0
        self.accepted = self.closed = self.canceled = 0
        self.messages = []
        self.reject_route = False
        self.map_available = True
        self.revision = 1
        self.home_available = True
        self.home_revision = None
        self.gas_mode = 'fresh'
        self.battery_mode = 'fresh'
        self.ptz_held = False
        self.telemetry_paused = False
        self.video_available = False
        self.video_ready = False
        self.vision = {}
        self.audio_assets = []
        self.audio_uploads = []
        self.audio = dict(ready=True,available=True,age=.1,state='idle',message='播放器就绪')
        self.points=[dict(point_id='p1',revision=1,name='仪表点',x=-.4,y=-1.8,yaw=.3,inspection_enabled=True,ptz_preset=7,recognition_prompt='读取电压')]
        self.routes=[dict(route_id='r',revision=1,name='测试路线',valid=True,navigation_mode='FREE_NAV',requires_preview=False,steps=[dict(point_id='p1',point_name='P',dwell_seconds=0)])]
        self.captures=[]
        self.intercom_available=True;self.intercom_frames=[];self.intercom_active=False
        self.loop = asyncio.new_event_loop()
        self.stop = threading.Event()
        self.ready = threading.Event()
        self.thread = threading.Thread(target=self._serve, daemon=True)
        self.thread.start()
        assert self.ready.wait(5)
        robot = self
        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_):
                pass
            def do_GET(self):
                path = urlsplit(self.path).path
                content_type = 'application/json'
                code = 200
                if path == '/api/local/state':
                    payload = {'active_map': {'map_id': 'm', 'revision': robot.revision, 'name': '测试地图'},
                               'map_sync_ok': True, 'home': ({'map_id':'m','map_revision':robot.home_revision or robot.revision,'x':0,'y':0,'yaw':0} if robot.home_available else None),
                               'routes': robot.routes}
                    raw = json.dumps(payload).encode()
                elif path == '/api/video/status':
                    raw = json.dumps(dict(available=robot.video_available,ready=robot.video_ready,
                        message='视频服务未启动' if not robot.video_available else '正在等待相机画面')).encode()
                elif path == '/api/inspection/records':
                    raw = b'{"items": []}'
                elif path == '/api/map/meta':
                    if not robot.map_available:
                        code, raw = 404, json.dumps({'detail': '尚未激活地图'}).encode()
                    else:
                        raw = json.dumps(robot.meta()).encode()
                elif path == '/api/local/maps/m/points':
                    raw = json.dumps({'items':robot.points}).encode()
                elif path == '/api/local/maps/m/networks':
                    raw=json.dumps({'items':[dict(network_id='n1',revision=1,map_revision=robot.revision,published=True,name='录制道路',bound_points=['p1'])]}).encode()
                elif path == '/api/records/captures':
                    raw=json.dumps({'items':robot.captures,'total':len(robot.captures)}).encode()
                elif path.startswith('/captures/'):
                    content_type,raw='image/png',robot.png()
                elif path == '/api/audio/files':
                    raw = json.dumps({'items':robot.audio_assets}).encode()
                elif path == '/api/inspection/reports':
                    raw = json.dumps({'items':[dict(run_id='report1',route_name='仪表巡检',state=10,completed_steps=1,started_at=10_000_000_000)],'total':1}).encode()
                elif path == '/api/inspection/reports/report1':
                    raw = json.dumps(robot.report()).encode()
                elif path == '/api/inspection/reports/report1/download':
                    content_type,raw='text/html',b'<!doctype html><h1>Inspection report</h1>'
                    self.send_response(200)
                    self.send_header('Content-Disposition','attachment; filename="inspection-report1.html"')
                    self.send_header('Content-Type',content_type)
                    self.send_header('Content-Length',str(len(raw)))
                    self.end_headers();self.wfile.write(raw);return
                elif path == '/api/inspection/results':
                    raw = json.dumps({'items':robot.report()['inspection_results'],'total':1}).encode()
                elif path == '/api/vision/events':
                    raw = json.dumps({'items':[dict(event_id='v1',event_type='vehicle_plate',label='京A12345',event_time_ns=12_000_000_000,
                        pose_valid=True,robot_x=-.4,robot_y=-1.8,robot_yaw=.3,map_id='m',image_url='/api/vision/media/v1')],
                        'total':1,'next_cursor':''}).encode()
                elif path in ('/api/vision/media/v1','/api/inspection/results/result1/image'):
                    content_type,raw='image/png',robot.png()
                elif path == '/maps/active.png':
                    content_type, raw = 'image/png', robot.png()
                else:
                    target = WEB / path.lstrip('/')
                    if target.is_file():
                        content_type = {'.html': 'text/html', '.css': 'text/css', '.png': 'image/png'}.get(
                            target.suffix, 'application/javascript')
                        raw = target.read_bytes()
                    else:
                        code, raw = 404, b'not found'
                self.send_response(code)
                self.send_header('Content-Type', content_type + ('; charset=utf-8' if content_type != 'image/png' else ''))
                self.send_header('Content-Length', str(len(raw)))
                self.end_headers()
                self.wfile.write(raw)
            def do_POST(self):
                body=self.rfile.read(int(self.headers.get('Content-Length','0')))
                if urlsplit(self.path).path!='/api/audio/files':
                    self.send_error(404);return
                robot.audio_uploads.append(body)
                item=dict(filename='uploaded.wav',name='播报.wav',duration_seconds=.2,size=len(body))
                robot.audio_assets=[item]
                raw=json.dumps(item).encode()
                self.send_response(200);self.send_header('Content-Type','application/json')
                self.send_header('Content-Length',str(len(raw)));self.end_headers();self.wfile.write(raw)
        self.http = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        self.http_thread = threading.Thread(target=self.http.serve_forever, daemon=True)
        self.http_thread.start()
        self.url = f'http://127.0.0.1:{self.http.server_port}'

    def png(self):
        output = BytesIO()
        Image.new('L', (20, 10), 205 if self.revision == 1 else 240).save(output, format='PNG')
        return output.getvalue()

    def meta(self):
        return dict(map_id='m', revision=self.revision, image='active.png', width=20, height=10,
                    resolution=.05, origin=[-1, -2, 0] if self.revision == 1 else [2, 3, .5],
                    sha256=hashlib.sha256(self.png()).hexdigest(), sync_valid=True)

    def control(self):
        return dict(owner_present=self.owner is not None, mode=self.mode,
                    local_control=False, estop_known=True, estopped=False)

    def gas(self):
        return dict(status=dict(enabled=self.gas_mode != 'stopped', connected=self.gas_mode != 'error',
                                interval=2, last_error='串口断开' if self.gas_mode == 'error' else ''),
                    data=dict(raw_value=123, value_scaled=12.3, decimals=1, unit='ppm',
                              state='normal', decimal_scaling_verified=False),
                    status_age=.1, data_age=30 if self.gas_mode == 'stale' else .1)

    def telemetry(self):
        return dict(type='telemetry', control=self.control(), task=self.task, frame='map',
                    speed=0, turn=0, pose={'x': 0, 'y': 0, 'yaw': 0}, scan=[], paths={},
                    patrol=dict(active=self.task['running'], state=3, state_name='navigating',
                                route_id='r', run_id='run' if self.task['running'] else '',
                                loop_index=1, total_loops=1, step_index=1, step_count=1,
                                current_point_name='P', message='模拟巡航'),
                    nav={'active_goal_count': 1 if self.task['running'] else 0}, gas=self.gas(),
                    battery=dict(available=self.battery_mode != 'invalid',state='invalid' if self.battery_mode=='invalid' else 'ready',
                        percentage=0. if self.battery_mode=='empty' else 78.,voltage=25.4,current=-1.2,charging=False,
                        age=10. if self.battery_mode=='stale' else .1,timeout=5.),
                    vision=self.vision, audio=self.audio, capture=dict(available=True), ptz=dict(patrol={'active': self.ptz_held, 'enabled': False, 'inspection_held':self.ptz_held,
                        'state': 'inspection_paused' if self.ptz_held else 'stopped'},
                                        patrol_age=.1, driver_ready=True, service_ready=True, manual_locked=False))

    def report(self):
        return dict(run_id='report1',route={'name':'仪表巡检'},map_name='测试地图',state_name='已完成',duration_seconds=10.,
            run=dict(started_at=10_000_000_000,finished_at=20_000_000_000,completed_steps=1,error_code='',message='完成'),
            inspection_success_count=1,inspection_failure_count=0,summary=dict(face_capture_count=2,person_count=3,plates=[{'plate':'京A12345'}]),
            inspection_results=[dict(result_id='result1',point_name='仪表点',started_at=11_000_000_000,loop_index=1,
                text='仪表读数：220 V',message='成功',success=True,image_url='/api/inspection/results/result1/image')],
            stop_attempts=[dict(loop_index=1,step_index=1,result='SUCCEEDED',position_error=.03,actual_valid=True,error_code='')],vision_events=[])

    async def send(self, ws, data):
        await ws.send(json.dumps(data))

    def release(self, ws):
        if self.owner is not ws:
            return
        if self.task['running']:
            self.canceled += 1
        self.task = {'kind': '', 'running': False}
        self.owner = self.token = None
        self.mode = 'idle'

    async def handle(self, ws):
        if getattr(ws,'path',getattr(getattr(ws,'request',None),'path',''))=='/intercom':
            try:
                auth=json.loads(await ws.recv())
                if auth.get('session_id')!=self.token:return
                if not self.intercom_available:
                    await self.send(ws,dict(type='devices',devices=[]));return
                await self.send(ws,dict(type='devices',devices=[dict(id=0,name='USB microphone',input_channels=1,output_channels=0),dict(id=1,name='USB speaker',input_channels=0,output_channels=1)]))
                async for raw in ws:
                    if isinstance(raw,bytes):
                        self.intercom_frames.append(raw);await ws.send(b'\x01\x02'*960);continue
                    value=json.loads(raw)
                    if value.get('type')=='stop':break
                    if value.get('type')=='start':
                        self.intercom_active=True
                        await self.send(ws,dict(type='ready',sample_rate=48000,channels=1,format='s16le'))
            finally:self.intercom_active=False
            return
        self.clients.add(ws)
        self.accepted += 1
        try:
            async for raw in ws:
                data = json.loads(raw)
                self.messages.append(data)
                command = data['type']
                result = dict(type='command_result', command=command, request_id=data.get('request_id'), ok=True)
                if command == 'control_claim':
                    if self.owner is not None and self.owner is not ws:
                        await self.send(ws, dict(result, ok=False, code='CONTROL_BUSY'))
                        continue
                    self.owner, self.token, self.mode = ws, 'fixture_token', 'manual'
                    self.heartbeat = time.monotonic()
                    await self.send(ws, dict(result, session_id=self.token))
                    continue
                if command == 'estop':
                    self.release(self.owner)
                    continue
                if self.owner is not ws or data.get('session_id') != self.token:
                    await self.send(ws, dict(result, ok=False, code='NOT_OWNER'))
                    continue
                self.heartbeat = time.monotonic()
                if command in ('route_start','point_start'):
                    if self.reject_route:
                        self.reject_route = False
                        await self.send(ws, dict(result, ok=False, code='NAV_GOAL_ACTIVE', message='旧目标尚未结束'))
                        continue
                    kind='point' if command=='point_start' else 'route'
                    self.task = dict(kind=kind, running=True, request_id=data['request_id'])
                    self.mode = kind
                    await self.send(ws, dict(result, code='ACCEPTED'))
                    await self.send(ws, dict(type='task_accepted', kind=kind, request_id=data['request_id']))
                elif command == 'control_release':
                    self.release(ws)
                    await self.send(ws, result)
                elif command == 'return_home':
                    self.task = dict(kind='return', running=True, request_id=data['request_id'])
                    self.mode = 'return'
                    await self.send(ws, dict(result, code='ACCEPTED'))
                elif command in ('route_cancel','return_cancel'):
                    previous=self.task
                    self.task=dict(kind='',running=False);self.mode='manual';self.canceled+=1
                    await self.send(ws,result)
                    await self.send(ws,dict(type='task_finished',kind=previous.get('kind','point'),request_id=previous.get('request_id'),success=False,code='CANCELED',message='已取消'))
                elif command in ('audio_play','audio_stop'):
                    self.audio.update(state='playing' if command=='audio_play' else 'stopped',
                                      request_id=data['request_id'],message='机器人正在播放' if command=='audio_play' else '已停止')
                    await self.send(ws,result)
                elif command=='capture':
                    item=dict(capture_id=data['request_id'],stamp=time.time(),label='网页手动抓拍',image_url='/captures/2026-10-22/web.jpg',available=True)
                    self.captures.append(item)
                    await self.send(ws,dict(result,code='SAVED',message='抓拍已保存',data=dict(image='captures/2026-10-22/web.jpg')))
                elif command in ('point_save','route_save'):
                    point=command.startswith('point_');key='point_id' if point else 'route_id';items=self.points if point else self.routes
                    previous=next((item for item in items if item[key]==data.get(key)),None)
                    value=dict(data);value.pop('type');value[key]=data.get(key) or f'new_{len(items)}'
                    value['revision']=(previous['revision'] if previous else 0)+1
                    if not point:value.update(valid=True,requires_preview=data.get('navigation_mode')=='RECORDED_NETWORK')
                    if previous:items.remove(previous)
                    items.append(value);await self.send(ws,dict(result,code='SAVED',data={'point' if point else 'route':value}))
                elif command in ('point_delete','route_delete'):
                    key='point_id' if command.startswith('point_') else 'route_id';items=self.points if key=='point_id' else self.routes
                    items[:]=[item for item in items if item[key]!=data[key]];await self.send(ws,dict(result,code='SAVED'))
                elif command=='route_preview':
                    route=next(item for item in self.routes if item['route_id']==data['route_id'])
                    plan=dict(digest='a'*64,route_id=route['route_id'],route_revision=route['revision'],map_id='m',map_revision=self.revision,loop_count=data['loop_count'],blocks=[dict(direction=1,samples=[dict(x=-.8,y=-1.8,yaw=0),dict(x=-.4,y=-1.8,yaw=0)])])
                    await self.send(ws,dict(result,code='READY',data=plan))
                elif command in ('route_pause', 'route_resume', 'ptz_jog', 'ptz_stop', 'ptz_read'):
                    await self.send(ws, result)
                elif command in ('vision_face_capture', 'vision_person', 'vision_plate'):
                    feature = {'vision_face_capture': 'face', 'vision_person': 'person', 'vision_plate': 'plate'}[command]
                    self.vision['detector'][feature] = data['enabled']
                    self.vision['detector'][feature+'_requested'] = data['enabled']
                    await self.send(ws, result)
        finally:
            self.closed += 1
            self.release(ws)
            self.clients.discard(ws)

    def _serve(self):
        asyncio.set_event_loop(self.loop)
        async def run():
            async with websockets.serve(self.handle, '127.0.0.1', 0) as server:
                self.ws_port = server.sockets[0].getsockname()[1]
                self.ready.set()
                while not self.stop.is_set():
                    if self.owner is not None and time.monotonic() - self.heartbeat > 1:
                        self.release(self.owner)
                    for ws in list(self.clients):
                        try:
                            if not self.telemetry_paused:
                                await self.send(ws, self.telemetry())
                        except websockets.exceptions.ConnectionClosed:
                            pass
                    await asyncio.sleep(.05)
        self.loop.run_until_complete(run())
        self.loop.close()

    def close_owner(self):
        if self.owner:
            asyncio.run_coroutine_threadsafe(self.owner.close(), self.loop).result(4)

    def close(self):
        self.stop.set()
        self.thread.join(5)
        self.http.shutdown()
        self.http.server_close()
        self.http_thread.join(3)


@pytest.fixture
def web():
    from playwright.sync_api import sync_playwright
    robot = RobotFixture()
    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(headless=True,args=['--use-fake-device-for-media-stream','--use-fake-ui-for-media-stream'])
        context = browser.new_context(viewport={'width': 1366, 'height': 900})
        page = context.new_page()
        errors = []
        page.on('pageerror', lambda error: errors.append(str(error)))
        yield robot, page, errors
        context.close()
        browser.close()
    robot.close()
    assert not errors, errors


def open_view(robot, page, view='tasks'):
    page.goto(f'{robot.url}/{view}.html?ws_port={robot.ws_port}')
    page.wait_for_url(f'**/console.html?ws_port={robot.ws_port}#{view}')
    page.locator('#connection').get_by_text('已连接', exact=True).wait_for()
    return page.frame_locator('#view')


def start_route(robot, page):
    view = open_view(robot, page)
    view.locator('#btnClaim').click()
    view.locator('#routeBody input').check()
    view.locator('#btnStart').click()
    wait_for(lambda: robot.task['running'])
    return view


def test_view_navigation_keeps_one_socket_and_close_still_cancels(web):
    robot, page, _ = web
    start_route(robot, page)
    page.locator('header nav a[data-page=monitor]').click()
    monitor = page.frame_locator('#view')
    monitor.locator('#gasValue').get_by_text('12.3', exact=True).wait_for()
    assert monitor.locator('#controlTag').inner_text() == '本网页已取得'
    page.locator('header nav a[data-page=records]').click()
    page.frame_locator('#view').locator('#reportRows button').wait_for()
    page.wait_for_timeout(1400)  # longer than the gateway control lease
    assert robot.accepted == 1 and robot.closed == 0 and robot.canceled == 0
    assert robot.task['running']
    page.locator('header nav a[data-page=tasks]').click()
    page.frame_locator('#view').locator('#btnPause').click()
    wait_for(lambda: any(item['type'] == 'route_pause' for item in robot.messages))
    page.close()
    wait_for(lambda: robot.canceled == 1 and robot.owner is None)


def test_one_click_inspection_survives_view_change_and_can_cancel_from_tasks(web):
    robot,page,_=web
    monitor=open_view(robot,page,'monitor')
    monitor.locator('#btnClaim').click()
    monitor.locator('#targetPoint').select_option('p1')
    assert monitor.locator('#targetPreset').input_value()=='7'
    monitor.locator('#pointStart').click()
    wait_for(lambda:robot.task.get('kind')=='point')
    submitted=next(item for item in robot.messages if item['type']=='point_start')
    assert submitted['point_id']=='p1' and submitted['point_revision']==1
    page.locator('header nav a[data-page=records]').click()
    page.frame_locator('#view').locator('#reportRows button').wait_for()
    page.wait_for_timeout(1200)
    assert robot.task['running'] and robot.closed==0
    page.locator('header nav a[data-page=tasks]').click()
    page.frame_locator('#view').locator('#btnCancel').click()
    wait_for(lambda:not robot.task['running'])
    assert robot.canceled==1 and robot.accepted==1


def test_map_pick_uses_rotated_origin_for_one_click_target(web):
    robot,page,_=web
    robot.revision=2
    monitor=open_view(robot,page,'monitor')
    monitor.locator('#btnClaim').click()
    monitor.locator('#mapHint').get_by_text('revision 2',exact=False).wait_for()
    monitor.locator('#pickTarget').click()
    box=monitor.locator('#mapCanvas').bounding_box()
    cx,cy=box['x']+box['width']/2,box['y']+box['height']/2
    page.mouse.move(cx,cy);page.mouse.down();page.mouse.move(cx+50,cy);page.mouse.up()
    monitor.locator('#pointStart').click()
    wait_for(lambda:any(item['type']=='point_start' for item in robot.messages))
    target=next(item for item in robot.messages if item['type']=='point_start')
    assert abs(target['x']-(2+.5*__import__('math').cos(.5)-.25*__import__('math').sin(.5)))<.02
    assert abs(target['yaw']-.5)<.02
    assert target['inspection_enabled'] is True and target['ptz_preset']==1


@pytest.mark.parametrize('kind',['route','point'])
def test_global_return_preempts_task_from_records_and_can_be_canceled(web,kind):
    robot,page,_=web
    if kind=='route':
        start_route(robot,page)
    else:
        monitor=open_view(robot,page,'monitor')
        monitor.locator('#btnClaim').click()
        monitor.locator('#targetPoint').select_option('p1')
        monitor.locator('#pointStart').click()
        wait_for(lambda:robot.task['running'])
    page.locator('header nav a[data-page=records]').click()
    page.frame_locator('#view').locator('#reportRows button').wait_for()
    page.locator('#returnHome').click()
    wait_for(lambda:robot.task.get('kind')=='return')
    returns=[item for item in robot.messages if item['type']=='return_home']
    assert len(returns)==1
    assert returns[0]['map_id']=='m' and returns[0]['map_revision']==1
    assert returns[0]['session_id']==robot.token
    assert page.locator('#returnHome').is_disabled()
    page.locator('#cancelReturn').click()
    wait_for(lambda:not robot.task['running'])
    assert any(item['type']=='return_cancel' for item in robot.messages)
    assert robot.accepted==1 and robot.canceled==1


def test_global_return_requires_current_home_control_and_fresh_status(web):
    robot,page,_=web
    robot.home_available=False
    monitor=open_view(robot,page,'monitor')
    monitor.locator('#btnClaim').click()
    page.wait_for_timeout(500)
    assert page.locator('#returnHome').is_disabled()
    robot.home_available=True;robot.home_revision=99
    page.wait_for_timeout(5200)
    assert page.locator('#returnHome').is_disabled()
    robot.home_revision=1
    page.wait_for_function("!document.querySelector('#returnHome').disabled",timeout=7000)
    robot.telemetry_paused=True
    page.wait_for_function("document.querySelector('#returnHome').disabled",timeout=5000)
    assert not any(item['type']=='return_home' for item in robot.messages)


def test_audio_upload_and_owned_robot_play_stop(web):
    robot,page,_=web
    monitor=open_view(robot,page,'monitor')
    monitor.locator('#btnClaim').click()
    monitor.locator('#audioFile').set_input_files({'name':'播报.wav','mimeType':'audio/wav','buffer':b'RIFF-web-upload-fixture'})
    monitor.locator('#audioUpload').click()
    monitor.locator('#audioMessage').get_by_text('已上传',exact=False).wait_for()
    monitor.locator('#audioPlay').click()
    wait_for(lambda:any(item['type']=='audio_play' for item in robot.messages))
    command=next(item for item in robot.messages if item['type']=='audio_play')
    assert command['filename']=='uploaded.wav' and command['session_id']==robot.token
    assert len(robot.audio_uploads)==1 and b'RIFF-web-upload-fixture' in robot.audio_uploads[0]
    monitor.locator('#audioStop').click()
    wait_for(lambda:any(item['type']=='audio_stop' for item in robot.messages))


def test_fixed_route_preview_is_required_and_digest_reaches_start(web):
    robot,page,_=web
    robot.routes[0].update(navigation_mode='RECORDED_NETWORK',requires_preview=True,network_id='n1',network_revision=1)
    view=open_view(robot,page);view.locator('#btnClaim').click();view.locator('#routeBody input').check()
    assert view.locator('#btnStart').is_disabled()
    view.locator('#btnPreview').click()
    page.wait_for_function("document.getElementById('view').contentDocument.getElementById('btnStart').disabled===false")
    view.locator('#loopCount').fill('2')
    assert view.locator('#btnStart').is_disabled()
    view.locator('#btnPreview').click()
    view.locator('#btnStart').click();wait_for(lambda:robot.task['running'])
    command=next(item for item in robot.messages if item['type']=='route_start')
    assert command['plan_digest']=='a'*64 and command['loop_count']==2


def test_manual_capture_works_without_a_patrol_task_and_is_listed(web):
    robot,page,_=web;view=open_view(robot,page,'monitor');view.locator('#btnClaim').click()
    view.locator('#manualCapture').click();view.locator('#captureMessage').get_by_text('抓拍已保存',exact=True).wait_for()
    assert view.locator('#capturePreview img').get_attribute('src')=='/captures/2026-10-22/web.jpg'
    assert not robot.task['running']
    page.locator('nav a[data-page=records]').click();view=page.frame_locator('#view')
    view.locator('#recordsTabCaptures').click()
    view.locator('#captureGallery img').wait_for()


def test_intercom_uses_browser_audio_persists_across_views_and_releases(web):
    robot,page,_=web;view=open_view(robot,page,'monitor');view.locator('#btnClaim').click()
    page.context.grant_permissions(['microphone'],origin=robot.url)
    page.locator('#intercomOpen').click();page.locator('#intercomStart').click()
    page.locator('#intercomMessage').get_by_text('双向对讲已开启',exact=False).wait_for()
    wait_for(lambda:len(robot.intercom_frames)>=3)
    assert all(len(frame)==1920 for frame in robot.intercom_frames)
    before=len(robot.intercom_frames)
    page.locator('#intercomClose').click();page.locator('nav a[data-page=records]').click()
    wait_for(lambda:len(robot.intercom_frames)>before+3)
    assert robot.intercom_active and robot.accepted==1
    page.locator('#intercomOpen').click();page.locator('#intercomEnd').click()
    wait_for(lambda:not robot.intercom_active)
    assert page.locator('#intercomEnd').is_disabled()


def test_intercom_no_robot_devices_is_clear_and_does_not_start(web):
    robot,page,_=web;robot.intercom_available=False
    view=open_view(robot,page,'monitor');view.locator('#btnClaim').click();page.locator('#intercomOpen').click()
    page.locator('#intercomMessage').get_by_text('机器人尚未连接麦克风或音响',exact=False).wait_for()
    assert page.locator('#intercomStart').is_disabled() and not robot.intercom_frames


def test_report_view_download_and_recognition_gallery(web):
    robot,page,_=web
    view=open_view(robot,page,'records')
    view.locator('#reportRows button').click()
    view.locator('#reportDialog').wait_for(state='visible')
    assert '220 V' in view.locator('#reportBody').inner_text()
    with page.expect_download() as result:
        view.locator('#reportBody a[download]').click()
    assert result.value.suggested_filename=='inspection-report1.html'
    view.locator('#filterReportRun').click()
    assert view.locator('#recordRun').input_value()=='report1'
    view.locator('#visionGallery').get_by_text('车牌 · 京A12345',exact=True).wait_for()
    assert view.locator('#inspectionRows img').count()==1
    page.screenshot(path='/tmp/remote_web_records_preview.png',full_page=True)


def test_inspection_controls_mobile_layout(web):
    robot,page,_=web
    page.set_viewport_size({'width':390,'height':844})
    view=open_view(robot,page,'monitor')
    view.locator('#targetPoint').select_option('p1')
    view.locator('#audioUpload').scroll_into_view_if_needed()
    assert view.locator('#audioUpload').is_visible()
    frame=next(frame for frame in page.frames if '/monitor.html' in frame.url)
    assert frame.evaluate('document.documentElement.scrollWidth<=innerWidth+1')
    assert page.evaluate('document.documentElement.scrollWidth<=innerWidth+1')
    page.screenshot(path='/tmp/remote_web_monitor_mobile_preview.png',full_page=True)


def test_rejected_start_and_bad_loop_allow_retry(web):
    robot, page, _ = web
    view = open_view(robot, page)
    view.locator('#btnClaim').click()
    view.locator('#routeBody input').check()
    for invalid in ('-1', '1.5', '100001'):
        view.locator('#loopCount').fill(invalid)
        view.locator('#btnStart').click()
        assert not any(item['type'] == 'route_start' for item in robot.messages)
    view.locator('#loopCount').fill('1')
    robot.reject_route = True
    view.locator('#btnStart').click()
    view.locator('#taskMessage').get_by_text('旧目标尚未结束').wait_for()
    view.locator('#btnStart:enabled').wait_for()
    view.locator('#btnStart').click()
    wait_for(lambda: robot.task['running'])


def test_gas_stops_expires_and_recovers(web):
    robot, page, _ = web
    view = open_view(robot, page, 'monitor')
    view.locator('#gasValue').get_by_text('12.3', exact=True).wait_for()
    for mode in ('stopped', 'stale', 'error'):
        robot.gas_mode = mode
        view.locator('#gasValue').get_by_text('—', exact=True).wait_for()
        assert view.locator('#gasState').inner_text() == '未知'
        robot.gas_mode = 'fresh'
        view.locator('#gasValue').get_by_text('12.3', exact=True).wait_for()


def test_battery_invalid_expired_zero_and_silent_connection(web):
    robot, page, _ = web
    view = open_view(robot, page, 'monitor')
    view.locator('#batteryPercent').get_by_text('78', exact=True).wait_for()
    assert view.locator('#batteryVoltage').inner_text() == '25.4'
    assert view.locator('#batteryCurrent').inner_text() == '-1.2'
    for mode in ('invalid', 'stale'):
        robot.battery_mode = mode
        view.locator('#batteryPercent').get_by_text('—', exact=True).wait_for()
        assert view.locator('#batteryVoltage').inner_text() == '—'
        assert view.locator('#batteryState').inner_text() == '未知'
        robot.battery_mode = 'fresh'
        view.locator('#batteryPercent').get_by_text('78', exact=True).wait_for()
    robot.battery_mode = 'empty'
    view.locator('#batteryPercent').get_by_text('0', exact=True).wait_for()
    view.locator('#batteryFresh').get_by_text('电量偏低', exact=True).wait_for()
    robot.telemetry_paused = True
    view.locator('#batteryPercent').get_by_text('—', exact=True).wait_for(timeout=5000)
    robot.telemetry_paused = False
    view.locator('#batteryPercent').get_by_text('0', exact=True).wait_for()


def test_inspection_hold_is_visible_and_locks_manual_ptz(web):
    robot, page, _ = web
    robot.ptz_held = True
    view = open_view(robot, page, 'monitor')
    view.locator('#btnClaim').click()
    view.locator('#ptzAuto').get_by_text('巡检占用中', exact=True).wait_for()
    view.locator('#ptzPatrolState').get_by_text('巡检拍照中', exact=True).wait_for()
    assert view.locator('.ptz:enabled').count() == 0
    robot.ptz_held = False
    view.locator('#ptzAuto').get_by_text('手动可用', exact=True).wait_for()
    view.locator('.ptz:enabled').first.wait_for()


def test_video_missing_service_shows_actionable_message_without_error_iframe(web):
    robot,page,_=web
    view=open_view(robot,page,'monitor')
    view.locator('#btnVideo').click()
    view.locator('#videoMessage').get_by_text('视频服务未启动',exact=True).wait_for()
    assert view.locator('#btnVideo').inner_text()=='重试视频'
    assert view.locator('#video').get_attribute('src') in (None,'about:blank')


def test_video_wait_can_be_canceled_and_ready_video_can_be_reopened(web):
    robot,page,_=web
    page.route('**://127.0.0.1:8889/**',lambda route:route.fulfill(status=200,content_type='text/html; charset=utf-8',body='<p>模拟视频</p>'))
    robot.video_available=True
    view=open_view(robot,page,'monitor')
    view.locator('#btnVideo').click()
    view.locator('#videoMessage').get_by_text('正在等待相机画面',exact=True).wait_for()
    view.locator('#btnVideo').click()
    robot.video_ready=True
    page.wait_for_timeout(700)
    assert view.locator('#video').get_attribute('src')=='about:blank'
    assert view.locator('#btnVideo').inner_text()=='开启视频'
    view.locator('#btnVideo').click()
    view.frame_locator('#video').get_by_text('模拟视频',exact=True).wait_for()
    view.locator('#btnVideo').click()
    assert view.locator('#video').get_attribute('src')=='about:blank'


def test_host_vision_toggles_need_no_sdk_and_remain_usable_after_model_error(web):
    robot, page, _ = web
    robot.vision = dict(detector=dict(ready=True, face=False, person=False, plate=False),
                        detector_age=.1, detector_service_ready=True,
                        camera=dict(online=True), camera_age=.1)
    view = open_view(robot, page, 'monitor')
    view.locator('#btnClaim').click()
    view.locator('#toggleFace:enabled').wait_for()
    view.locator('#toggleFace').check()
    wait_for(lambda: any(item['type'] == 'vision_face_capture' and item['enabled']
                         for item in robot.messages))
    robot.vision['detector'].update(ready=False, face=False, face_requested=True, last_error='测试模型异常')
    view.locator('#visionError').get_by_text('测试模型异常', exact=True).wait_for()
    view.locator('#toggleFace').uncheck()
    wait_for(lambda: any(item['type'] == 'vision_face_capture' and not item['enabled']
                         for item in robot.messages))
    view.locator('#togglePerson').check()
    wait_for(lambda: robot.vision['detector']['person'])


def test_map_appears_late_and_revision_updates_without_reconnect(web):
    robot, page, _ = web
    robot.map_available = False
    view = open_view(robot, page, 'monitor')
    view.locator('#mapHint').get_by_text('尚未激活地图').wait_for()
    robot.map_available = True
    view.locator('#mapHint').get_by_text('revision 1', exact=False).wait_for(timeout=7000)
    robot.revision = 2
    view.locator('#mapHint').get_by_text('revision 2', exact=False).wait_for(timeout=7000)
    assert robot.accepted == 1


def test_ptz_hold_continues_past_five_seconds_and_stops_on_release(web):
    robot, page, _ = web
    view = open_view(robot, page, 'monitor')
    view.locator('#btnClaim').click()
    button = view.locator('button[data-action=left]')
    button.hover()
    page.mouse.down()
    page.wait_for_timeout(5600)
    jogs = [item for item in robot.messages if item['type'] == 'ptz_jog']
    assert len(jogs) >= 20 and robot.owner is not None
    page.mouse.up()
    wait_for(lambda: robot.messages[-1]['type'] == 'ptz_stop' or any(
        item['type'] == 'ptz_stop' for item in robot.messages[-5:]))
    count = len([item for item in robot.messages if item['type'] == 'ptz_jog'])
    page.wait_for_timeout(900)
    assert count == len([item for item in robot.messages if item['type'] == 'ptz_jog'])


def test_real_socket_loss_cancels_and_reconnect_does_not_resume(web):
    robot, page, _ = web
    start_route(robot, page)
    robot.close_owner()
    wait_for(lambda: robot.canceled == 1)
    page.locator('#connection').get_by_text('已连接', exact=True).wait_for(timeout=7000)
    page.wait_for_timeout(300)
    assert robot.owner is None and not robot.task['running']
    assert len([item for item in robot.messages if item['type'] == 'route_start']) == 1
