"""Check that missing or invalid sensor data cannot command motion."""

import math
import time
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from lab02_pkg.bump_and_go import BumpAndGo


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_sector_ignores_invalid_ranges():
    scan = LaserScan()
    scan.angle_min = -0.1
    scan.angle_increment = 0.1
    scan.range_min = 0.12
    scan.range_max = 3.5
    scan.ranges = [math.nan, -math.inf, 0.0]
    node = SimpleNamespace(scan=scan)

    assert BumpAndGo.sector_distance(node, 0.0, 15.0) is None

    scan.ranges = [math.inf, 0.4, math.nan]
    assert BumpAndGo.sector_distance(node, 0.0, 15.0) == pytest.approx(0.4)


def test_stale_sensor_is_not_ready():
    now = time.monotonic()
    node = SimpleNamespace(
        scan=object(), got_odom=True, sensor_timeout=1.0,
        last_scan_time=now - 2.0, last_odom_time=now,
    )

    assert not BumpAndGo.sensors_ready(node)
    node.last_scan_time = now
    assert BumpAndGo.sensors_ready(node)


def test_missing_sensor_commands_stop():
    publisher = Publisher()
    node = SimpleNamespace(
        sensors_ready=lambda: False, cmd_pub=publisher, last_cmd=Twist(),
    )

    BumpAndGo.control_loop(node)

    assert len(publisher.messages) == 1
    assert publisher.messages[0].linear.x == 0.0
    assert publisher.messages[0].angular.z == 0.0


def test_missing_side_ranges_prevent_turn():
    publisher = Publisher()
    node = SimpleNamespace(
        sensors_ready=lambda: True,
        sector_distance=lambda center, width, **kw: 0.4 if center == 0.0 else None,
        cmd_pub=publisher, last_cmd=Twist(), stop_at_goal=False,
        state='GO', threshold=0.5,
    )

    BumpAndGo.control_loop(node)

    assert len(publisher.messages) == 1
    assert publisher.messages[0].linear.x == 0.0
    assert publisher.messages[0].angular.z == 0.0
