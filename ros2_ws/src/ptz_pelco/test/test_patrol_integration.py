"""Actual sweep + RS485 driver services, with fake motors in domain 226."""
import importlib.util
from pathlib import Path
import time

import pytest
from rclpy.parameter import Parameter
from patrol_interfaces.srv import PtzInspectionControl

from test_node import rig, wait_for, pytestmark

spec = importlib.util.spec_from_file_location('rs485_sweep_under_test',
    Path(__file__).resolve().parents[2] / 'chassis_nav2/scripts/ptz_patrol.py')
sweep_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sweep_module)


@pytest.fixture
def sweep(rig):
    rig.head.angles = (125., 0.)
    node = sweep_module.PtzPatrol()
    rig.extra_nodes.append(node)
    rig.executor.add_node(node)
    assert node._patrol_ctrl.wait_for_service(timeout_sec=3.)
    assert node.set_parameters_atomically([
        Parameter('tilt_enabled', value=True), Parameter('tilt_min', value=-9.),
        Parameter('tilt_max', value=15.), Parameter('tilt_speed', value=1)]).successful
    rig.clients['inspection'] = rig.client.create_client(PtzInspectionControl, '/ptz_patrol/inspection_control')
    assert rig.clients['inspection'].wait_for_service(timeout_sec=3.)
    yield node
    node.set_parameters_atomically([Parameter('enabled', value=False)])
    wait_for(lambda: not node._moving and rig.ptz.patrol is None)


def start(node, rig):
    assert node.set_parameters_atomically([Parameter('enabled', value=True)]).successful
    wait_for(lambda: node._moving and bool(rig.head.moves))


def inspection(rig, command, resume=False, lease=2.):
    return rig.call('inspection', PtzInspectionControl.Request(
        command=command, owner_id='isolated_rs485_inspection', resume=resume, lease_seconds=lease))


def test_two_axis_patrol_pauses_both_axes_for_inspection_and_resumes(sweep, rig):
    start(sweep, rig)
    assert rig.head.move_axes[-1] == (0x14, 18, 9)
    assert inspection(rig, 1).success
    wait_for(sweep._inspection_stopped)
    assert rig.ptz.patrol is None
    count = len(rig.head.moves)
    time.sleep(.3)
    assert len(rig.head.moves) == count
    assert inspection(rig, 3, resume=True).success
    wait_for(lambda: sweep._moving and len(rig.head.moves) > count)
    assert not rig.controls and not rig.zoom_sets


def test_pitch_feedback_fault_stops_and_disables_sweep(sweep, rig):
    start(sweep, rig)
    rig.head.fail_query = 0x53
    wait_for(lambda: not sweep.get_parameter('enabled').value and not sweep._moving)
    assert rig.ptz.patrol is None and '反馈' in sweep._error
    assert not rig.controls


def test_operator_stop_during_inspection_does_not_restart_pitch(sweep, rig):
    start(sweep, rig)
    assert inspection(rig, 1).success
    wait_for(sweep._inspection_stopped)
    assert sweep.set_parameters_atomically([Parameter('enabled', value=False)]).successful
    count = len(rig.head.moves)
    assert inspection(rig, 3, resume=True).success
    time.sleep(.35)
    assert not sweep._moving and len(rig.head.moves) == count


def test_lost_inspection_lease_keeps_both_axes_stopped(sweep, rig):
    start(sweep, rig)
    assert inspection(rig, 1, lease=1.).success
    wait_for(sweep._inspection_stopped)
    count = len(rig.head.moves)
    wait_for(lambda: bool(sweep._inspection_fault))
    assert not sweep.get_parameter('enabled').value
    assert inspection(rig, 3, resume=True).success
    time.sleep(.3)
    assert not sweep._moving and len(rig.head.moves) == count


def test_pitch_settings_cannot_change_during_rotation(sweep, rig):
    start(sweep, rig)
    result = sweep.set_parameters_atomically([Parameter('tilt_min', value=-5.)])
    assert not result.successful and sweep.get_parameter('tilt_min').value == -9.
    result = sweep.set_parameters_atomically([Parameter('tilt_max', value=-10.)])
    assert not result.successful


def test_slow_serial_feedback_does_not_starve_lease_renewals(sweep, rig):
    original_query = rig.head.query
    original_move = rig.head.move

    def slow_query(*args, **kwargs):
        time.sleep(.24)
        return original_query(*args, **kwargs)

    def slow_move(*args, **kwargs):
        time.sleep(.18)
        return original_move(*args, **kwargs)

    rig.head.query = slow_query
    rig.head.move = slow_move
    start(sweep, rig)
    time.sleep(1.7)
    assert sweep.get_parameter('enabled').value and sweep._moving
    assert rig.ptz.patrol is not None and not rig.ptz.patrol_fault


def test_feedback_blocked_longer_than_lease_still_accepts_renewals(sweep, rig):
    import threading
    entered, release = threading.Event(), threading.Event()
    original_query = rig.head.query

    def blocked_query(*args, **kwargs):
        with rig.head.lock:
            entered.set()
            assert release.wait(3.)
            return original_query(*args, **kwargs)

    rig.head.query = blocked_query
    try:
        assert sweep.set_parameters_atomically([Parameter('enabled', value=True)]).successful
        assert entered.wait(1.)
        deadline = time.monotonic() + .9
        while time.monotonic() < deadline:
            assert rig.ptz.lease_deadline > time.monotonic()
            assert sweep.get_parameter('enabled').value and not sweep._error
            time.sleep(.05)
        release.set()
        wait_for(lambda: bool(rig.head.moves))
        assert not rig.ptz.patrol_fault
    finally:
        release.set()


def test_acknowledged_commands_with_frozen_angles_stop_patrol(sweep, rig):
    start(sweep, rig)
    wait_for(lambda: not sweep.get_parameter('enabled').value and not sweep._moving, timeout=4.5)
    assert '无变化' in sweep._error
    assert rig.ptz.patrol is None


def test_measured_full_revolutions_reverse_both_ways_with_pitch_sweep(sweep, rig):
    original_move = rig.head.move
    reversals = []
    previous_bit, travel = None, 0.

    def moving_head(cmd, pan_speed, tilt_speed):
        nonlocal previous_bit, travel
        bit = cmd & 0x06
        if previous_bit is not None and bit != previous_bit:
            reversals.append(travel)
            travel = 0.
        previous_bit = bit
        pan, tilt = rig.head.angles
        pan = (pan + (-12. if bit == 0x04 else 12.)) % 360.
        tilt = max(-9., min(15., tilt + (1. if cmd & 0x10 else -1.)))
        rig.head.angles = (pan, tilt)
        travel += 12.
        return original_move(cmd, pan_speed, tilt_speed)

    rig.head.move = moving_head
    start(sweep, rig)
    wait_for(lambda: len(reversals) >= 2, timeout=20.)
    assert all(angle >= 360. for angle in reversals[:2])
    assert sweep._completed_sweeps >= 2
    assert sweep.get_parameter('enabled').value and not sweep._error
    assert any(cmd & 0x10 for cmd in rig.head.moves)
    assert any(cmd & 0x08 for cmd in rig.head.moves)
    assert not rig.controls and not rig.zoom_sets
