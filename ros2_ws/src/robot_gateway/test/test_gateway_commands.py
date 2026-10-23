"""Explicit isolated command regressions; no real Nav2 or camera endpoints."""
import asyncio
from concurrent.futures import Future
import os
import time

import pytest
import rclpy
from std_msgs.msg import Bool
from yunle_msgs.msg import Battery
from patrol_interfaces.msg import OperatorHeartbeat
from robot_gateway.gateway_node import GatewayNode
from types import SimpleNamespace

pytestmark = pytest.mark.skipif(os.environ.get('ROBOT_GATEWAY_INTEGRATION') != '1',
                                reason='explicit isolated ROS test required')
if os.environ.get('ROBOT_GATEWAY_INTEGRATION') == '1':
    assert os.environ.get('ROS_DOMAIN_ID') == '226'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'


class Action:
    def __init__(self, raises=False):
        self.raises = raises
        self.goals = []
        self.future = Future()
    def wait_for_server(self, **kwargs):
        return True
    def send_goal_async(self, goal, **kwargs):
        if self.raises:
            raise RuntimeError('isolated dispatch failure')
        self.goals.append(goal)
        return self.future


@pytest.fixture
def gateway():
    rclpy.init()
    node = GatewayNode({'control': {'max_linear': .3, 'max_angular': .5, 'cmd_timeout': 1.0}})
    results = []
    node._command_result = lambda ws, command, ok, code='', message='', request_id='', **kw: results.append(
        dict(command=command, ok=ok, code=code, message=message, request_id=request_id, **kw))
    ws = object()
    node._on_estop(Bool(data=False))
    token, _, _ = node._claim_control(ws)
    payload = dict(type='route_start', session_id=token, request_id='start',
                   map_id='map', route_id='route', map_revision=1, route_revision=1, loop_count=1)
    yield node, ws, payload, results
    node._perform_release('test cleanup')
    node.destroy_node()
    rclpy.shutdown()


def test_battery_waiting_zero_soc_expiry_and_recovery(gateway):
    node, *_ = gateway
    assert node._battery_snapshot()['state'] == 'waiting'
    message = Battery(capacity=0., voltage=24.5, ampere=-1.2, charge_status=0)
    node._on_battery(message)
    value = node._battery_snapshot()
    assert value['available'] and value['percentage'] == 0.
    assert value['current'] == pytest.approx(-1.2) and value['charging'] is False
    node._battery_time -= 6.
    value = node._battery_snapshot()
    assert value['state'] == 'stale' and not value['available']
    assert 'percentage' not in value and 'voltage' not in value
    node._on_battery(Battery(capacity=85., voltage=25., ampere=2., charge_status=1))
    assert node._battery_snapshot()['charging'] is True
    assert node._battery_snapshot()['percentage'] == 85.


@pytest.mark.parametrize('capacity,voltage', [(101.,24.),(-1.,24.),(float('nan'),24.),(50.,0.),(50.,float('inf'))])
def test_invalid_battery_clears_previous_reading(gateway, capacity, voltage):
    node, *_ = gateway
    node._on_battery(Battery(capacity=50., voltage=24.))
    node._on_battery(Battery(capacity=capacity, voltage=voltage))
    value = node._battery_snapshot()
    assert value['state'] == 'invalid' and not value['available']
    assert 'percentage' not in value and 'voltage' not in value


@pytest.mark.parametrize('bad', [-1, 100001, 1.5, '2', True, None])
def test_invalid_loops_do_not_reserve_task_and_next_start_succeeds(gateway, bad):
    node, ws, data, results = gateway
    action = Action(); node._route_client = action
    asyncio.run(node.handle_command(ws, dict(data, loop_count=bad)))
    assert results[-1]['code'] == 'INVALID_DATA'
    assert not node._task_snapshot()['running'] and not action.goals
    asyncio.run(node.handle_command(ws, dict(data, request_id='valid', loop_count=0)))
    assert action.goals[0].loop_count == 0
    assert node._task_snapshot()['request_id'] == 'valid'


def test_send_exception_restores_slot_for_route_and_return(gateway):
    node, ws, data, results = gateway
    node._route_client = Action(raises=True)
    asyncio.run(node.handle_command(ws, data))
    assert not node._task_snapshot()['running']
    assert node._control_mode == 'manual' and node._cmd_pub is not None
    assert results[-1]['code'] == 'ACTION_FAILED'
    node._return_client = Action(raises=True)
    asyncio.run(node.handle_command(ws, dict(data, type='return_home', request_id='return')))
    assert not node._task_snapshot()['running']
    assert results[-1]['code'] == 'ACTION_FAILED'
    node._route_client = Action()
    asyncio.run(node.handle_command(ws, dict(data, request_id='retry')))
    assert node._task_snapshot()['request_id'] == 'retry'


def test_invalid_return_does_not_cancel_running_route(gateway):
    node, ws, data, results = gateway
    action = Action(); node._route_client = action
    asyncio.run(node.handle_command(ws, data))
    asyncio.run(node.handle_command(ws, dict(data, type='return_home', map_revision=-1)))
    assert results[-1]['code'] == 'INVALID_DATA'
    assert not node._task_cancel_pending and node._pending_return is None
    assert node._task_kind == 'route'


def test_point_validation_submission_and_return_preemption(gateway):
    node,ws,data,results=gateway
    node._point_client=Action()
    payload=dict(data,type='point_start',x=.2,y=.3,yaw=.1,inspection_enabled=True,ptz_preset=1)
    asyncio.run(node.handle_command(ws,dict(payload,x=float('inf'))))
    assert results[-1]['code']=='INVALID_DATA' and not node._task_kind
    asyncio.run(node.handle_command(ws,payload))
    assert node._task_kind=='point' and node._control_mode=='point'
    assert node._point_client.goals[0].x==.2
    asyncio.run(node.handle_command(ws,dict(data,type='return_home',request_id='home')))
    assert node._task_cancel_pending and node._pending_return is not None
    assert results[-1]['command']=='return_home' and results[-1]['ok']


def test_audio_requires_owner_fresh_player_and_uploaded_file(gateway,tmp_path):
    node,ws,data,results=gateway
    node._audio_dir=tmp_path
    (tmp_path/'voice.wav').write_bytes(b'fake audio fixture')
    published=[]
    node._audio_pub=SimpleNamespace(publish=lambda message:published.append(message.data))
    payload=dict(data,type='audio_play',filename='voice.wav')
    asyncio.run(node.handle_command(ws,payload))
    assert results[-1]['code']=='AUDIO_UNAVAILABLE' and not published
    node._audio_status={'ready':True,'state':'idle'};node._audio_time=time.monotonic()
    asyncio.run(node.handle_command(ws,dict(payload,session_id='wrong')))
    assert results[-1]['code']=='NOT_OWNER' and not published
    asyncio.run(node.handle_command(ws,dict(payload,filename='../voice.wav')))
    assert not results[-1]['ok'] and not published
    asyncio.run(node.handle_command(ws,payload))
    assert results[-1]['ok'] and len(published)==1


def test_cancel_point_uses_action_when_control_service_is_unavailable(gateway):
    node,ws,data,results=gateway
    assert node._begin_task('point','point1')[0]
    node._patrol_status={'run_id':'run1'}
    node._control_client=SimpleNamespace(service_is_ready=lambda:False)
    canceled=[]
    node._task_goal_handle=SimpleNamespace(cancel_goal_async=lambda:canceled.append(True))
    asyncio.run(node.handle_command(ws,dict(data,type='route_cancel')))
    assert results[-1]['ok'] and canceled and node._task_cancel_pending


def test_manual_capture_requires_a_fresh_video_and_correlates_completion(gateway):
    from std_msgs.msg import String
    import json
    node,ws,data,results=gateway
    published=[];node._capture_pub=SimpleNamespace(publish=lambda message:published.append(json.loads(message.data)))
    payload=dict(data,type='capture',request_id='capture1')
    asyncio.run(node.handle_command(ws,payload))
    assert results[-1]['code']=='CAMERA_UNAVAILABLE' and not published
    node._on_capture_status(String(data='{"ready":true}'))
    asyncio.run(node.handle_command(ws,payload))
    assert results[-1]['code']=='PENDING' and published[0]['request_id']=='capture1'
    node._on_capture_done(String(data='{"request_id":"unrelated","ok":true}'))
    assert results[-1]['code']=='PENDING'
    node._on_capture_done(String(data='{"request_id":"capture1","ok":true,"image":"captures/2026-10-22/image.jpg"}'))
    assert results[-1]['code']=='SAVED' and results[-1]['data']['ok']


def test_slow_web_preview_keeps_reading_control_heartbeats(gateway):
    import json
    from robot_gateway.gateway_node import _handle_client
    node,_,data,_=gateway
    observed=[]
    async def slow_preview(websocket,payload):
        await asyncio.sleep(1.05)
        observed.append(node._lease_valid())
    node._business.handle=slow_preview
    class Socket:
        remote_address='isolated-browser'
        path='/'
        count=0
        async def send(self,value):pass
        def __aiter__(self):return self
        async def __anext__(self):
            self.count+=1
            if self.count==1:return json.dumps(dict(data,type='route_preview'))
            await asyncio.sleep(.25)
            if self.count>6:raise StopAsyncIteration
            return json.dumps(dict(data,type='control_heartbeat'))
    websocket=Socket();node._owner_ws=websocket
    asyncio.run(_handle_client(node,websocket))
    assert observed==[True], 'preview blocked reading the owner heartbeat'


@pytest.mark.parametrize('takeover', [False, True])
def test_disconnect_or_local_takeover_still_cancels_owned_task(gateway, takeover):
    node, ws, data, _ = gateway
    class Handle:
        calls = 0
        def cancel_goal_async(self): self.calls += 1
    handle = Handle()
    node._route_client = Action()
    asyncio.run(node.handle_command(ws, data))
    node._task_goal_handle = handle
    if takeover:
        heartbeat = OperatorHeartbeat(client_id='desktop_ui', session_id='desktop', enabled=True)
        node._on_operator_heartbeat(heartbeat)
    else:
        node._remove_client(ws)
    assert handle.calls >= 1 and not node._owner_session
    assert node._cmd_pub is None


def test_ptz_requires_hold_renewal_and_pressed_estop_is_known(gateway, monkeypatch):
    node, ws, _, _ = gateway
    now = [time.monotonic()]
    monkeypatch.setattr('robot_gateway.gateway_node.time.monotonic', lambda: now[0])
    calls = []
    node._call_ptz = lambda command=0, stop=False, speed=3: (calls.append(stop) or True, '')
    class OfflinePose:
        def service_is_ready(self): return False
    node._ptz_pose_client = OfflinePose()
    for _ in range(35):
        node._ptz_jog(ws, {'action': 'left'})
        now[0] += .2
        node._poll_ptz_pose()
        assert node._ptz_jog_active
    now[0] += 1.0
    node._poll_ptz_pose()
    assert not node._ptz_jog_active and calls[-1] is True
    node._on_estop(Bool(data=True))
    snapshot = node._control_snapshot()
    assert snapshot['estop_known'] and snapshot['estopped']


@pytest.mark.parametrize('enabled', [True, False])
def test_face_command_changes_host_parameter_without_sdk_service(gateway, enabled):
    from rcl_interfaces.srv import SetParameters
    from rcl_interfaces.msg import SetParametersResult, ParameterType
    node, ws, data, results = gateway
    class Parameters:
        requests = []
        def service_is_ready(self): return True
        def call_async(self, request):
            self.requests.append(request)
            future = Future()
            future.set_result(SetParameters.Response(results=[SetParametersResult(successful=True)]))
            return future
    parameters = Parameters(); node._params_client = parameters
    asyncio.run(node.handle_command(ws, dict(data, type='vision_face_capture', enabled=enabled)))
    parameter = parameters.requests[0].parameters[0]
    assert parameter.name == 'enable_face'
    assert parameter.value.type == ParameterType.PARAMETER_BOOL and parameter.value.bool_value is enabled
    assert results[-1]['ok'] and results[-1]['command'] == 'vision_face_capture'
