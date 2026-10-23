"""Gear and curvature regressions; no ROS graph or hardware needed."""
import math

from chassis_nav2.drive_control import GearInterlock
from chassis_nav2.geometry import ackermann_command, ackermann_command_4ws


def test_speed_caps_preserve_forward_and_reverse_curvature():
    for direction in (1, -1):
        for convert in (ackermann_command, ackermann_command_4ws):
            fast = convert(direction*.6, .3, .5, 24., .6)
            limited = convert(direction*.6, .3, .5, 24., .3)
            assert math.isclose(limited[0], fast[0]/2)
            assert math.isclose(limited[1], fast[1])
            assert limited[-2] == (1 if direction>0 else 3)


def test_reverse_requires_continuous_standstill_and_a_settling_pause():
    state = GearInterlock()
    assert state.update(1, .2, 0.) == (1, True)
    for now in (.1, .3, .6):
        assert state.update(3, .1, now) == (1, False)
    assert state.update(3, 0., .7) == (1, False)
    assert state.update(3, .04, .9) == (1, False)  # movement resets dwell
    assert state.update(3, 0., 1.) == (1, False)
    assert state.update(3, 0., 1.26) == (3, False)
    assert state.update(3, 0., 1.4) == (3, False)
    assert state.update(3, 0., 1.47) == (3, True)
    assert state.update(None, 0., 1.5) == (3, False)  # a zero must keep R
    assert state.update(3, -.1, 1.6) == (3, True)
    assert state.update(1, -.1, 1.7) == (3, False)


def test_jitter_stale_feedback_and_estop_cannot_finish_a_gear_change():
    state = GearInterlock()
    for now in (0., .2, .4, .6):
        assert state.update(3, 0., now) == (1, False)
        assert state.update(1, 0., now+.1) == (1, True)
    state.update(3, 0., 1.)
    assert state.update(3, 0., 2., active=False) == (1, False)
    assert state.update(3, 0., 2.1) == (1, False)
    assert state.update(3, float('nan'), 3.) == (1, False)
    assert state.update(3, 0., 3.1) == (1, False)
    assert state.update(None, 0., 4.) == (1, False)
