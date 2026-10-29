"""ROS node: missing or stale sensor data must command zero velocity."""

import time
from types import SimpleNamespace

from geometry_msgs.msg import Twist

from lab02_pkg.bump_and_go_node import BumpAndGoNode


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_stale_sensor_is_not_ready():
    now = time.monotonic()
    node = SimpleNamespace(
        scan=object(), got_odom=True, sensor_timeout=1.0,
        last_scan_time=now - 2.0, last_odom_time=now,
    )
    assert not BumpAndGoNode.sensors_ready(node)
    node.last_scan_time = now
    assert BumpAndGoNode.sensors_ready(node)


def test_missing_sensor_commands_stop():
    publisher = Publisher()
    node = SimpleNamespace(sensors_ready=lambda: False, cmd_pub=publisher, last_cmd=Twist())
    BumpAndGoNode.control_loop(node)
    assert len(publisher.messages) == 1
    assert publisher.messages[0].linear.x == 0.0
    assert publisher.messages[0].angular.z == 0.0
