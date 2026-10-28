"""Public ROS services + SDK replies in an isolated domain, with fake motors."""
import math
import os
import threading
import time
from types import SimpleNamespace
from unittest.mock import patch

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from hk_camera.srv import ControlPTZ, ControlPTZPatrol, GetPTZPose, SetPTZPose

from ptz_pelco.node import CMD_PAN, CMD_TILT, CMD_ZOOM, PelcoSerial, PtzPelcoNode


pytestmark = pytest.mark.skipif(
    os.environ.get('ROS_DOMAIN_ID') != '226' or os.environ.get('ROS_LOCALHOST_ONLY') != '1',
    reason='Run with ROS_DOMAIN_ID=226 ROS_LOCALHOST_ONLY=1; fake hardware only')


class FakeHead:
    def __init__(self):
        self.lock = threading.RLock()
        self.ser = object()
        self.port = 'fake'
        self.stats = {}
        self.angles = (125.67, -33.59)
        self.axes = []
        self.moves = []
        self.move_speeds = []
        self.move_axes = []
        self.fail_query = None
        self.stops = 0
        self.fail_axis = None
        self.stop_ok = True

    def open(self):
        return True, 'fake'

    def read_angles(self):
        return self.angles

    def read_xyz(self):
        # Deliberately different from the camera's real optical ratio.
        return (*self.angles, 23.)

    def set_axis(self, cmd, value, repeat=2):
        self.axes.append((cmd, value))
        return cmd != self.fail_axis

    def move(self, cmd, pan_speed, tilt_speed):
        self.moves.append(cmd)
        self.move_speeds.append(pan_speed)
        self.move_axes.append((cmd, pan_speed, tilt_speed))
        return True

    _dec = staticmethod(PelcoSerial._dec)

    def query(self, cmd, **kwargs):
        if cmd == self.fail_query or self.angles is None:
            return None
        value = self.angles[0] if cmd == 0x51 else self.angles[1]
        return PelcoSerial._enc(value * 100)

    def stop_move(self):
        self.stops += 1
        return self.stop_ok


def wait_for(predicate, timeout=3.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.01)
    assert predicate(), 'isolated PTZ condition timed out'


@pytest.fixture
def rig():
    rclpy.init()
    h = SimpleNamespace(head=FakeHead(), zoom=1.4, controls=[], zoom_sets=[],
                        sdk_ok=True, hang_get=False, extra_nodes=[])
    with patch('ptz_pelco.node.PelcoSerial', return_value=h.head):
        h.ptz = PtzPelcoNode()
    h.ptz.last_sync = time.monotonic()
    h.server, h.client = Node('fake_camera_lens'), Node('ptz_client_fixture')

    async def get_pose(request, response):
        if h.hang_get:
            h.pending_get = Future()
            await h.pending_get
        response.success = h.sdk_ok
        response.zoom = float(h.zoom)
        response.limits_valid = True
        response.zoom_min, response.zoom_max = 1., 23.
        return response

    def set_pose(request, response):
        h.zoom_sets.append((request.pan, request.tilt, request.zoom))
        response.success = h.sdk_ok
        if response.success:
            h.zoom = request.zoom
        return response

    def control(request, response):
        h.controls.append((request.command, request.stop))
        response.success = h.sdk_ok
        return response

    h.server.create_service(GetPTZPose, '/hk_camera/sdk/get_ptz_pose', get_pose)
    h.server.create_service(SetPTZPose, '/hk_camera/sdk/set_ptz_pose', set_pose)
    h.server.create_service(ControlPTZ, '/hk_camera/sdk/control_ptz', control)
    h.clients = {key: h.client.create_client(kind, '/hk_camera/' + name)
                 for key, kind, name in [('get', GetPTZPose, 'get_ptz_pose'),
                                         ('set', SetPTZPose, 'set_ptz_pose'),
                                         ('control', ControlPTZ, 'control_ptz')]}
    h.clients['patrol'] = h.client.create_client(ControlPTZPatrol, '/ptz/control_patrol')
    h.clients['head'] = h.client.create_client(ControlPTZ, '/ptz/control')
    h.clients['angles'] = h.client.create_client(GetPTZPose, '/ptz/get_angles')
    h.executor = MultiThreadedExecutor(num_threads=3)
    for node in (h.ptz, h.server, h.client):
        h.executor.add_node(node)
    worker = threading.Thread(target=h.executor.spin, daemon=True)
    worker.start()
    for client in [*h.clients.values(), h.ptz.sdk_get, h.ptz.sdk_set, h.ptz.sdk_control]:
        assert client.wait_for_service(timeout_sec=3.)

    def call(kind, request):
        future = h.clients[kind].call_async(request)
        wait_for(future.done)
        return future.result()

    h.call = call
    yield h
    if hasattr(h, 'pending_get'):
        h.pending_get.cancel()
    h.executor.shutdown(timeout_sec=1.)
    worker.join(3.)
    assert not worker.is_alive()
    for node in (*h.extra_nodes, h.ptz, h.server, h.client):
        node.destroy_node()
    rclpy.shutdown()


def test_pose_combines_signed_head_angles_and_actual_lens_zoom(rig):
    r = rig.call('get', GetPTZPose.Request())
    assert r.success
    assert r.tilt == pytest.approx(-33.59)
    assert r.zoom == pytest.approx(1.4)
    assert (r.tilt_min, r.tilt_max, r.zoom_max) == (-90., 25., 23.)


def test_absolute_pose_preserves_negative_tilt_and_uses_camera_lens(rig):
    rig.ptz.pan_sign = rig.ptz.tilt_sign = -1.
    r = rig.call('set', SetPTZPose.Request(pan=126., tilt=-34., zoom=23.))
    assert r.success
    assert rig.head.axes == [(CMD_PAN, 126.), (CMD_TILT, -34.)]
    assert rig.zoom_sets == [(0., 0., 23.)]
    assert not rig.ptz.pos_valid
    # The target is not substituted for actual head feedback.
    r = rig.call('get', GetPTZPose.Request())
    assert r.success and r.pan == pytest.approx(125.67)
    assert r.zoom == 23.


@pytest.mark.parametrize('pan,tilt,zoom', [(math.nan, 0., 2.), (0., -91., 2.),
                                         (0., 0., 24.), (math.inf, 0., 2.)])
def test_invalid_targets_are_rejected_without_actuating(rig, pan, tilt, zoom):
    r = rig.call('set', SetPTZPose.Request(pan=pan, tilt=tilt, zoom=zoom))
    assert not r.success
    assert not rig.head.axes and not rig.zoom_sets


def test_failed_axis_write_is_not_reported_as_success(rig):
    rig.head.fail_axis = CMD_TILT
    r = rig.call('set', SetPTZPose.Request(pan=126., tilt=-34., zoom=2.))
    assert not r.success
    assert rig.zoom_sets == [(0., 0., 2.)] and not rig.ptz.pos_valid


def test_unchanged_zoom_is_not_sent_during_absolute_head_move(rig):
    assert rig.call('get', GetPTZPose.Request()).success
    r = rig.call('set', SetPTZPose.Request(pan=132., tilt=-33.59, zoom=1.4))
    assert r.success
    assert rig.head.axes == [(CMD_PAN, 132.)]
    assert not rig.zoom_sets
    assert not rig.ptz.pos_valid


def test_sdk_zoom_stopping_head_does_not_interrupt_subsequent_positioning(rig):
    assert rig.call('get', GetPTZPose.Request()).success
    moving = []
    set_axis = rig.head.set_axis
    def start_head(cmd, value, repeat=2):
        moving.append(cmd)
        return set_axis(cmd, value, repeat)
    rig.head.set_axis = start_head
    # This camera's SDK zoom command stops any currently moving head axis.
    def zoom_stops_head(request, response):
        moving.clear()
        response.success = True
        return response
    rig.server.create_service(SetPTZPose, '/test/zoom_stops_head', zoom_stops_head)
    rig.ptz.sdk_set = rig.ptz.create_client(SetPTZPose, '/test/zoom_stops_head',
                                           callback_group=rig.ptz.sdk_group)
    assert rig.ptz.sdk_set.wait_for_service(timeout_sec=3.)
    assert rig.call('set', SetPTZPose.Request(pan=132., tilt=-33.59, zoom=2.)).success
    assert moving == [CMD_PAN]


def test_failed_sdk_zoom_does_not_start_head_positioning(rig):
    rig.sdk_ok = False
    r = rig.call('set', SetPTZPose.Request(pan=132., tilt=-33.59, zoom=2.))
    assert not r.success
    assert not rig.head.axes and not rig.ptz.pos_valid


def test_zoom_is_continuous_and_operator_lease_stops_it(rig):
    assert rig.call('control', ControlPTZ.Request(command=11, speed=40)).success
    time.sleep(.3)
    assert rig.call('control', ControlPTZ.Request(command=11, speed=40)).success
    assert not rig.head.axes  # No staircase of absolute zoom targets.
    assert rig.ptz.zoom == 1.  # No invented feedback while the lens moves.
    wait_for(lambda: (0, True) in rig.controls)
    assert rig.ptz.zoom_jog == 0
    assert sum(command == 11 and not stop for command, stop in rig.controls) >= 2


def test_switching_zoom_to_pan_stops_lens_before_moving_head(rig):
    assert rig.call('control', ControlPTZ.Request(command=11, speed=40)).success
    assert rig.call('control', ControlPTZ.Request(command=23, speed=40)).success
    assert rig.ptz.zoom_jog == 0
    assert (0, True) in rig.controls and rig.head.moves == [0x04]
    assert rig.call('control', ControlPTZ.Request(stop=True)).success


def test_sdk_speed_scale_maps_to_usable_pelco_speed(rig):
    assert rig.call('control', ControlPTZ.Request(command=23, speed=3)).success
    assert rig.head.move_speeds == [27]
    assert rig.call('control', ControlPTZ.Request(stop=True)).success


def test_failed_stop_is_retried_by_watchdog(rig):
    assert rig.call('control', ControlPTZ.Request(command=23, speed=40)).success
    rig.head.stop_ok = False
    assert not rig.call('control', ControlPTZ.Request(stop=True)).success
    assert rig.ptz.stop_pending
    failed_stops = rig.head.stops
    rig.head.stop_ok = True
    wait_for(lambda: not rig.ptz.stop_pending)
    assert rig.head.stops > failed_stops and rig.ptz.jog is None


def test_invalid_measurement_does_not_reuse_stale_success(rig):
    assert rig.call('get', GetPTZPose.Request()).success
    rig.zoom = 600.
    assert not rig.call('get', GetPTZPose.Request()).success
    assert not rig.ptz.pos_valid


def test_sdk_timeout_returns_failure_without_deadlocking_public_services(rig):
    rig.hang_get = True
    begin = time.monotonic()
    r = rig.call('get', GetPTZPose.Request())
    assert not r.success and '超时' in r.message
    assert time.monotonic() - begin < 1.5
    assert rig.call('control', ControlPTZ.Request(command=23, speed=40)).success


def test_successful_read_clears_a_previous_sdk_timeout(rig):
    rig.hang_get = True
    assert not rig.call('get', GetPTZPose.Request()).success
    assert '超时' in rig.ptz.last_error
    rig.hang_get = False
    rig.pending_get.set_result(None)
    assert rig.call('get', GetPTZPose.Request()).success
    assert rig.ptz.pos_valid and rig.ptz.last_error == ''


def test_direct_pelco_lens_remains_opt_in(rig):
    rig.ptz.zoom_backend = 'pelco'
    r = rig.call('get', GetPTZPose.Request())
    assert r.success and r.zoom == 23.
    assert rig.call('set', SetPTZPose.Request(pan=126., tilt=-34., zoom=20.)).success
    assert (CMD_ZOOM, 20.) in rig.head.axes
    assert not rig.zoom_sets


def test_zero_pelco_zoom_is_invalid_instead_of_inventing_one_x(rig):
    rig.ptz.zoom_backend = 'pelco'
    rig.head.read_xyz = lambda: (*rig.head.angles, 0.)
    r = rig.call('get', GetPTZPose.Request())
    assert not r.success and 'zoom=0.0' in r.message
    assert not rig.ptz.pos_valid


def patrol_request(**kwargs):
    values = dict(command=23, pan_speed=2, tilt_enabled=True,
                  tilt_min=-9., tilt_max=15., tilt_speed=1)
    values.update(kwargs)
    return ControlPTZPatrol.Request(**values)


def test_patrol_positions_outside_tilt_before_rotating_without_sdk(rig):
    r = rig.call('patrol', patrol_request())
    assert r.success and r.positioning
    assert rig.head.axes == [(CMD_TILT, -9.)]
    assert not rig.head.moves and not rig.controls and not rig.zoom_sets
    rig.head.angles = (125.67, -9.)
    assert rig.call('patrol', patrol_request()).success
    wait_for(lambda: 0x14 in rig.head.moves, timeout=.4)
    assert rig.head.move_axes[-1] == (0x14, 18, 9)
    assert rig.call('patrol', patrol_request(stop=True)).success
    assert not rig.controls and not rig.zoom_sets


def test_patrol_reverses_pitch_at_feedback_edges_while_pan_keeps_rotating(rig):
    rig.head.angles = (125., 0.)
    assert rig.call('patrol', patrol_request()).success
    wait_for(lambda: 0x14 in rig.head.moves, timeout=.4)
    rig.head.angles = (124., 14.9)
    assert rig.call('patrol', patrol_request()).success
    wait_for(lambda: 0x0C in rig.head.moves, timeout=.4)
    rig.head.angles = (123., -8.9)
    assert rig.call('patrol', patrol_request()).success
    wait_for(lambda: rig.head.moves[-1] == 0x14, timeout=.4)
    assert all(cmd & 0x04 for cmd in rig.head.moves)
    assert not rig.controls and not rig.zoom_sets


def test_patrol_feedback_loss_stops_and_rejects_renew_until_stop(rig):
    rig.head.angles = (125., 0.)
    assert rig.call('patrol', patrol_request()).success
    wait_for(lambda: rig.head.moves)
    rig.head.fail_query = 0x53
    wait_for(lambda: rig.ptz.patrol is None, timeout=.4)
    assert rig.head.stops > 0
    r = rig.call('patrol', patrol_request())
    assert not r.success and '反馈' in r.message
    assert rig.call('patrol', patrol_request(stop=True)).success
    assert not rig.controls


def test_patrol_lease_expiry_stops_both_head_axes_without_sdk(rig):
    rig.head.angles = (125., 0.)
    assert rig.call('patrol', patrol_request()).success
    wait_for(lambda: rig.head.moves)
    wait_for(lambda: rig.ptz.patrol is None, timeout=1.)
    assert rig.head.stops > 0 and not rig.controls


@pytest.mark.parametrize('lo,hi', [(-91., 15.), (15., -9.), (-9., 26.), (math.nan, 15.), (0., 1.)])
def test_patrol_rejects_invalid_pitch_limits_before_moving(rig, lo, hi):
    r = rig.call('patrol', patrol_request(tilt_min=lo, tilt_max=hi))
    assert not r.success and not rig.head.moves and not rig.head.axes


def test_head_stop_and_angle_feedback_do_not_require_camera_sdk(rig):
    rig.sdk_ok = False
    r = rig.call('angles', GetPTZPose.Request())
    assert r.success and r.tilt == pytest.approx(-33.59)
    assert rig.call('head', ControlPTZ.Request(stop=True)).success
    assert not rig.controls and not rig.ptz.stop_pending


def test_patrol_blocks_manual_motion_and_stops_if_pitch_is_out_of_bounds(rig):
    rig.head.angles = (125., 0.)
    assert rig.call('patrol', patrol_request()).success
    r = rig.call('control', ControlPTZ.Request(command=21, speed=3))
    assert not r.success and '巡航' in r.message
    rig.head.angles = (125., 20.)
    wait_for(lambda: rig.ptz.patrol is None, timeout=.4)
    assert '超出' in rig.ptz.patrol_fault


def test_pitch_slows_near_endpoint_while_horizontal_speed_stays_constant(rig):
    rig.head.angles = (125., 13.)
    assert rig.call('patrol', patrol_request()).success
    wait_for(lambda: bool(rig.head.move_axes), timeout=.4)
    assert rig.head.move_axes[-1] == (0x14, 18, 1)


def test_stop_during_blocked_feedback_never_restarts_old_movement(rig):
    rig.head.angles = (125., 0.)
    assert rig.call('patrol', patrol_request()).success
    wait_for(lambda: bool(rig.head.moves))
    entered, release = threading.Event(), threading.Event()
    original_query = rig.head.query

    def blocking_query(*args, **kwargs):
        with rig.head.lock:
            entered.set()
            assert release.wait(2.)
            return original_query(*args, **kwargs)

    rig.head.query = blocking_query
    try:
        assert entered.wait(1.)
        future = rig.clients['patrol'].call_async(patrol_request(stop=True))
        wait_for(lambda: rig.ptz.patrol is None)
        moves = len(rig.head.moves)
        release.set()
        wait_for(future.done)
        assert future.result().success
        time.sleep(.25)
        assert len(rig.head.moves) == moves
        assert not rig.ptz.stop_pending
    finally:
        release.set()


def test_pitch_resume_keeps_previous_direction_until_endpoint(rig):
    rig.head.angles = (125., 14.9)
    assert rig.call('patrol', patrol_request()).success
    wait_for(lambda: bool(rig.head.moves))
    assert rig.head.moves[-1] == 0x0C
    assert rig.call('patrol', patrol_request(stop=True)).success
    rig.head.angles = (123., 5.)
    count = len(rig.head.moves)
    assert rig.call('patrol', patrol_request()).success
    wait_for(lambda: len(rig.head.moves) > count)
    assert rig.head.moves[-1] == 0x0C
