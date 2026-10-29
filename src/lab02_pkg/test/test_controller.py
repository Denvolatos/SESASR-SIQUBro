"""Bump&Go state machine, tested without ROS."""

import math

import pytest

from lab02_pkg.controller import BumpAndGoController, ControllerConfig, wrap_angle


def make(**overrides):
    cfg = ControllerConfig(stop_at_goal=False, **overrides)
    return BumpAndGoController(cfg)


def test_go_when_front_is_free():
    c = make()
    assert c.step(0, 0, 0, front=2.0, left=1, right=1) == (0.22, 0.0)
    assert c.state == 'GO'


def test_bump_turns_towards_the_more_open_side():
    c = make()
    assert c.step(0, 0, 0, front=0.4, left=2.0, right=0.5) == (0.0, 0.0)
    assert c.state == 'ROTATE'
    assert c.target_yaw == pytest.approx(math.pi / 2)       # left
    c = make()
    c.step(0, 0, 0, front=0.4, left=0.5, right=2.0)
    assert c.target_yaw == pytest.approx(-math.pi / 2)      # right


def test_no_side_data_stops_without_turning():
    c = make()
    assert c.step(0, 0, 0, front=0.4, left=None, right=1.0) == (0.0, 0.0)
    assert c.state == 'GO'


def test_no_front_data_stops():
    assert make().step(0, 0, 0, front=None, left=1, right=1) == (0.0, 0.0)


def test_rotation_uses_p_control_and_finishes():
    c = make()
    c.step(0, 0, 0, front=0.4, left=2.0, right=0.5)
    v, w = c.step(0, 0, 0, front=0.4, left=2.0, right=0.5)
    assert v == 0.0 and w == pytest.approx(1.5)              # 2 * 90 deg, clipped
    c.step(0, 0, math.radians(88), front=0.4, left=2.0, right=0.5)
    assert c.state == 'GO'                                   # within 5 deg


def test_small_command_is_raised_to_the_dead_zone():
    c = make()
    c.step(0, 0, 0, front=0.4, left=2.0, right=0.5)
    _, w = c.step(0, 0, math.radians(84), front=0.4, left=2.0, right=0.5)
    assert w == pytest.approx(0.3)                           # 2 * 6 deg = 0.21 -> 0.3


def test_snap_to_axes():
    c = make()
    # placed 50 deg off the axes: 50 + 90 = 140 deg is snapped to 180 deg, a 130 deg turn
    c.step(0, 0, math.radians(50), front=0.4, left=2.0, right=0.5)
    assert c.target_yaw == pytest.approx(math.pi)
    c = make(snap_to_axes=False)
    c.step(0, 0, math.radians(50), front=0.4, left=2.0, right=0.5)
    assert c.target_yaw == pytest.approx(math.radians(140))


def test_turn_until_clear_stops_early():
    c = make(turn_until_clear=True)
    c.step(0, 0, 0, front=0.4, left=2.0, right=0.5)
    c.step(0, 0, math.radians(10), front=1.0, left=2.0, right=0.5)
    assert c.state == 'ROTATE'                               # less than 20 deg turned
    c.step(0, 0, math.radians(30), front=1.0, left=2.0, right=0.5)
    assert c.state == 'GO'                                   # front free (> 0.8 m)


def test_goal_stops_the_robot():
    c = BumpAndGoController(ControllerConfig())
    assert c.step(6.5, 3.0, 0, front=2.0, left=1, right=1) == (0.0, 0.0)
    assert c.state == 'DONE'


def test_wrap_angle():
    assert wrap_angle(3 * math.pi / 2) == pytest.approx(-math.pi / 2)
