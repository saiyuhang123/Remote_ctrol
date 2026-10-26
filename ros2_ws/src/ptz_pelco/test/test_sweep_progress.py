"""Measured sweep regressions; isolated ROS domain, no motor service calls."""
from concurrent.futures import Future
import importlib.util
from pathlib import Path
import time
from types import SimpleNamespace as NS

import pytest
import rclpy
from rclpy.parameter import Parameter

from test_node import pytestmark


spec = importlib.util.spec_from_file_location(
    'measured_sweep_under_test',
    Path(__file__).resolve().parents[2] / 'chassis_nav2/scripts/ptz_patrol.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


@pytest.fixture
def patrol():
    rclpy.init()
    node = module.PtzPatrol()
    node.commands = []
    node._command = lambda stop: node.commands.append(stop)
    node._ctrl = NS(service_is_ready=lambda: True)
    node._patrol_ctrl = NS(service_is_ready=lambda: True)
    yield node
    node.destroy_node()
    rclpy.shutdown()


def start(node, pan=137.25):
    assert node.set_parameters_atomically([Parameter('enabled', value=True)]).successful
    assert node._accept_pan(pan)
    node._moving = True


@pytest.mark.parametrize('direction,sign', [(module.PAN_LEFT, -1), (module.PAN_RIGHT, 1)])
def test_each_direction_completes_a_real_revolution_across_zero(patrol, direction, sign):
    start(patrol)
    patrol._direction = direction
    pan = 137.25
    for leg in range(2):
        expected = patrol._direction
        for _ in range(23):
            pan = (pan + sign * 15.) % 360.
            assert patrol._accept_pan(pan)
            patrol._tick()
            assert patrol._direction == expected
        assert abs(patrol._pan_travel) == pytest.approx(345.)
        pan = (pan + sign * 15.) % 360.
        patrol._accept_pan(pan)
        patrol._tick()
        assert patrol._direction != expected
        assert patrol._completed_sweeps == leg + 1
        assert patrol._pan_travel == 0.
        # Brief motion in the old direction must cancel against the return,
        # rather than being counted as additional return travel.
        patrol._accept_pan((pan + sign * .5) % 360.)
        patrol._accept_pan(pan)
        assert patrol._pan_travel == pytest.approx(0.)
        sign = -sign


def test_elapsed_time_does_not_reverse_a_partial_revolution(patrol):
    start(patrol)
    patrol._last_flip = time.monotonic() - 180.
    patrol._accept_pan(92.25)
    patrol._tick()
    assert patrol._direction == module.PAN_LEFT
    assert patrol._pan_travel == pytest.approx(-45.)
    assert patrol.get_parameter('enabled').value


def test_repeated_heading_and_jitter_do_not_complete_a_revolution(patrol):
    start(patrol, 359.75)
    for _ in range(1000):
        patrol._accept_pan(.15)
        patrol._accept_pan(359.75)
    patrol._tick()
    assert patrol._completed_sweeps == 0
    assert patrol._pan_travel == pytest.approx(0.)
    assert patrol._direction == module.PAN_LEFT


def test_start_waits_for_actual_angle_before_moving(patrol):
    future = Future()
    patrol._pose = NS(service_is_ready=lambda: True, call_async=lambda request: future)
    assert patrol.set_parameters_atomically([Parameter('enabled', value=True)]).successful
    patrol._tick()
    assert not patrol.commands
    future.set_result(NS(success=True, pan=213.5))
    patrol._tick()
    assert patrol.commands == [False]
    assert patrol._pan_previous == 213.5
    assert patrol._pan_travel == 0.


def test_horizontal_only_mode_reads_feedback_without_bounds(patrol):
    start(patrol)
    future = Future()
    patrol._pose = NS(service_is_ready=lambda: True, call_async=lambda request: future)
    patrol._poll_pose()
    future.set_result(NS(success=True, pan=136.25))
    assert patrol._pan == 136.25
    assert patrol._pan_travel == pytest.approx(-1.)


def test_stale_feedback_stops_instead_of_reversing(patrol):
    start(patrol)
    patrol._pan_stamp -= 3.
    patrol._tick()
    assert patrol.commands == [True]
    assert not patrol.get_parameter('enabled').value
    assert '反馈超时' in patrol._error
    assert patrol._completed_sweeps == 0


def test_unchanging_feedback_stops_instead_of_rotating_forever(patrol):
    start(patrol)
    patrol._pan_motion_stamp -= 4.
    patrol._tick()
    assert patrol.commands == [True]
    assert not patrol.get_parameter('enabled').value
    assert '无变化' in patrol._error


@pytest.mark.parametrize('pan', [float('nan'), float('inf'), -1., 361.])
def test_invalid_pose_feedback_disables_rotation(patrol, pan):
    start(patrol)
    future = Future()
    patrol._pose_pending = future
    future.set_result(NS(success=True, pan=pan))
    patrol._on_pose_done(future)
    assert not patrol.get_parameter('enabled').value
    assert patrol._stop_due
    assert '反馈无效' in patrol._error


def test_invalid_patrol_feedback_disables_both_axes(patrol):
    start(patrol)
    future = Future()
    patrol._ctrl_pending = future
    future.set_result(NS(success=True, positioning=False, pose_valid=False, pan=137., tilt=0.))
    patrol._on_ctrl_done(future, stop=False, patrol_mode=True)
    assert not patrol.get_parameter('enabled').value
    assert patrol._stop_due


def test_late_control_ack_preserves_feedback_fault_and_stop(patrol):
    start(patrol)
    patrol._fail_feedback('isolated feedback fault')
    future = Future()
    patrol._ctrl_pending = future
    future.set_result(NS(success=True))
    patrol._on_ctrl_done(future, stop=False)
    patrol._tick()
    assert patrol._error == 'isolated feedback fault'
    assert not patrol.get_parameter('enabled').value
    assert patrol.commands == [True]


def test_inspection_repositioning_is_not_counted_as_sweep_travel(patrol):
    start(patrol)
    patrol._accept_pan(100.)
    patrol._reset_pan_progress()
    patrol._inspection_owner = 'isolated_capture'
    patrol._accept_pan(280.)
    patrol._accept_pan(20.)
    assert patrol._pan_travel == 0.
    patrol._inspection_owner = ''
    patrol._reset_pan_progress(invalidate=True)
    patrol._accept_pan(240.)
    patrol._accept_pan(230.)
    assert patrol._pan_travel == pytest.approx(-10.)


def test_full_circle_bounds_also_require_a_full_revolution(patrol):
    assert patrol.set_parameters_atomically([
        Parameter('pan_min', value=0.), Parameter('pan_max', value=360.),
        Parameter('pan_sweep_degrees', value=30.)]).successful
    start(patrol)
    patrol._accept_pan(107.25)
    patrol._tick()
    assert patrol._direction == module.PAN_LEFT
    assert patrol._sweep_target() == 360.


@pytest.mark.parametrize('angle', [0., -1., 361., float('nan'), float('inf')])
def test_invalid_sweep_angles_are_rejected(patrol, angle):
    result = patrol.set_parameters_atomically([Parameter('pan_sweep_degrees', value=angle)])
    assert not result.successful
    assert patrol.get_parameter('pan_sweep_degrees').value == 360.
