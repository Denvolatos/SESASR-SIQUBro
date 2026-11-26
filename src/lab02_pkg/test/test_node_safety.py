# Copyright 2026 SESASR Group 203 (SIQUBro)
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


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
