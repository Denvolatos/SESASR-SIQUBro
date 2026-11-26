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


"""
Single-trial Gazebo test driver for lab07_pkg (ROS 2 Humble).

Subscribes to /nav_result, /ground_truth, and /scan BEFORE publishing
/goal_pose, waits until dwa_node is connected (get_subscription_count >= 1),
and reports simulation-time duration, path length, and minimum scan distance.
"""

import argparse
import math
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class SingleTrialRunner(Node):
    """ROS 2 node that sends one goal safely and records simulation-time metrics."""

    def __init__(self) -> None:
        """Create subscriptions before publisher so no result message is missed."""
        super().__init__('single_trial_runner')
        self.result = None
        self.path = []
        self.min_scan = float('inf')
        self.got_gt = False
        self.got_scan = False
        self.recording = False

        self.create_subscription(
            Odometry, '/ground_truth', self._gt_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data
        )
        self.create_subscription(String, '/nav_result', self._result_cb, 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

    def _gt_cb(self, msg: Odometry) -> None:
        self.got_gt = True
        if not self.recording:
            return
        t_sim = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        pos = msg.pose.pose.position
        self.path.append((t_sim, float(pos.x), float(pos.y)))

    def _scan_cb(self, msg: LaserScan) -> None:
        self.got_scan = True
        if not self.recording:
            return
        valid = [r for r in msg.ranges if math.isfinite(r) and r > 0.05]
        if valid:
            d_min = min(valid)
            if d_min < self.min_scan:
                self.min_scan = d_min

    def _result_cb(self, msg: String) -> None:
        if self.recording and self.result is None:
            self.result = str(msg.data)

    def wait_ready(self, timeout_sec: float) -> bool:
        """Wait until /goal_pose has a subscriber and sensor topics are live."""
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.goal_pub.get_subscription_count() >= 1 and self.got_gt and self.got_scan:
                return True
        return False

    def run_goal(self, gx: float, gy: float, frame_id: str, timeout_sec: float) -> dict:
        """Publish goal pose and spin until /nav_result or timeout."""
        self.path.clear()
        self.min_scan = float('inf')
        self.result = None
        self.recording = True

        msg = PoseStamped()
        msg.header.frame_id = frame_id
        msg.pose.position.x = float(gx)
        msg.pose.position.y = float(gy)
        self.goal_pub.publish(msg)

        deadline = time.monotonic() + timeout_sec
        while self.result is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        sim_sec = 0.0
        path_m = 0.0
        if len(self.path) >= 2:
            sim_sec = max(0.0, self.path[-1][0] - self.path[0][0])
            for p0, p1 in zip(self.path, self.path[1:]):
                path_m += math.hypot(p1[1] - p0[1], p1[2] - p0[2])

        return {
            'result': self.result if self.result is not None else 'ScriptTimeout',
            'sim_sec': sim_sec,
            'path_m': path_m,
            'min_scan_m': self.min_scan if math.isfinite(self.min_scan) else -1.0,
        }


def main() -> None:
    """Parse CLI arguments, wait for dwa_node, run one goal trial, and print metrics."""
    parser = argparse.ArgumentParser(description='Run one Gazebo Task 1 trial safely.')
    parser.add_argument('x', type=float, help='Goal x coordinate [m]')
    parser.add_argument('y', type=float, help='Goal y coordinate [m]')
    parser.add_argument('--frame', default='world', help='Goal frame_id')
    parser.add_argument('--ready-timeout', type=float, default=30.0, help='Wait timeout [s]')
    parser.add_argument('--trial-timeout', type=float, default=140.0, help='Wall timeout [s]')
    args = parser.parse_args()

    rclpy.init()
    node = SingleTrialRunner()
    try:
        if not node.wait_ready(args.ready_timeout):
            print('RESULT=NotReady SIM_TIME=0.00 PATH=0.00 MIN_SCAN=-1.00', flush=True)
            return
        res = node.run_goal(args.x, args.y, args.frame, args.trial_timeout)
        print(
            f"RESULT={res['result']} SIM_TIME={res['sim_sec']:.2f} "
            f"PATH={res['path_m']:.2f} MIN_SCAN={res['min_scan_m']:.3f}",
            flush=True,
        )
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
