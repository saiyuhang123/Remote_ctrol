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
        self.gas_mode = 'fresh'
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
                               'map_sync_ok': True, 'home': {'x': 0, 'y': 0, 'yaw': 0},
                               'routes': [{'route_id': 'r', 'revision': 1, 'name': '测试路线', 'valid': True,
                                           'steps': [{'point_name': 'P'}]}]}
                    raw = json.dumps(payload).encode()
                elif path == '/api/inspection/records':
                    raw = b'{"items": []}'
                elif path == '/api/map/meta':
                    if not robot.map_available:
                        code, raw = 404, json.dumps({'detail': '尚未激活地图'}).encode()
                    else:
                        raw = json.dumps(robot.meta()).encode()
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
                    vision={}, ptz=dict(patrol={'active': False, 'enabled': False, 'state': 'stopped'},
                                        patrol_age=.1, service_ready=True, manual_locked=False))

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
                if command == 'route_start':
                    if self.reject_route:
                        self.reject_route = False
                        await self.send(ws, dict(result, ok=False, code='NAV_GOAL_ACTIVE', message='旧目标尚未结束'))
                        continue
                    self.task = dict(kind='route', running=True, request_id=data['request_id'])
                    self.mode = 'route'
                    await self.send(ws, dict(result, code='ACCEPTED'))
                    await self.send(ws, dict(type='task_accepted', kind='route', request_id=data['request_id']))
                elif command == 'control_release':
                    self.release(ws)
                    await self.send(ws, result)
                elif command == 'return_home':
                    self.task = dict(kind='return', running=True, request_id=data['request_id'])
                    self.mode = 'return'
                    await self.send(ws, dict(result, code='ACCEPTED'))
                elif command in ('route_pause', 'route_resume', 'ptz_jog', 'ptz_stop', 'ptz_read'):
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
        browser = playwright.chromium.launch(headless=True)
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
    page.frame_locator('#view').locator('#sessions').get_by_text('0', exact=True).wait_for()
    page.wait_for_timeout(1400)  # longer than the gateway control lease
    assert robot.accepted == 1 and robot.closed == 0 and robot.canceled == 0
    assert robot.task['running']
    page.locator('header nav a[data-page=tasks]').click()
    page.frame_locator('#view').locator('#btnPause').click()
    wait_for(lambda: any(item['type'] == 'route_pause' for item in robot.messages))
    page.close()
    wait_for(lambda: robot.canceled == 1 and robot.owner is None)


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
