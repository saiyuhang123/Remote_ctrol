"""Explicit isolated command regressions; no real Nav2 or camera endpoints."""
import asyncio
from concurrent.futures import Future
import os
import time

import pytest
import rclpy
from std_msgs.msg import Bool
from patrol_interfaces.msg import OperatorHeartbeat
from robot_gateway.gateway_node import GatewayNode

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
