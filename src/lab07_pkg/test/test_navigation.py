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


"""Unit tests for lab07_pkg.navigation (unittest)."""

import math
import unittest

import numpy as np

from lab07_pkg.dwa import DWAConfig
from lab07_pkg.navigation import (
    NavConfig,
    NavigationManager,
    RESULT_COLLISION,
    RESULT_GOAL,
    RESULT_TIMEOUT,
    ScanData,
    world_to_odom,
    yaw_from_quaternion,
)
from lab07_pkg.scan_utils import ScanConfig


def make_open_scan(n_rays: int = 36, value: float = 3.5) -> ScanData:
    """Verify make open scan."""
    return ScanData(
        ranges=[value] * n_rays,
        angle_min=-math.pi,
        angle_increment=2.0 * math.pi / n_rays,
        range_min=0.12,
        range_max=3.5,
    )


class TestNavigationManager(unittest.TestCase):
    """Verify TestNavigationManager."""

    def test_yaw_from_quaternion_and_world_to_odom(self):
        """Verify test yaw from quaternion and world to odom."""
        yaw = math.radians(90.0)
        q_yaw = yaw_from_quaternion(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
        self.assertAlmostEqual(q_yaw, yaw, places=9)

        # odom origin at (1, 2) rotated +90 deg in world; world point (1, 5) -> odom (3, 0)
        xo, yo = world_to_odom(1.0, 5.0, (1.0, 2.0, math.pi / 2.0))
        self.assertAlmostEqual(xo, 3.0, places=9)
        self.assertAlmostEqual(yo, 0.0, places=9)

    def test_frame_validation_and_midway_goal_replacement(self):
        """Verify test frame validation and midway goal replacement."""
        nav_cfg = NavConfig(odom_origin_in_world=(1.0, 0.0, 0.0), max_num_steps=100)
        mgr = NavigationManager(nav_config=nav_cfg)

        # Unknown frame rejected
        ok, msg = mgr.set_goal(2.0, 1.0, 'base_link')
        self.assertFalse(ok)
        self.assertIn('unsupported', msg.lower())
        self.assertFalse(mgr.active)

        # Empty frame rejected
        ok_empty, _ = mgr.set_goal(2.0, 1.0, '')
        self.assertFalse(ok_empty)

        # Valid world goal transformed to odom
        ok_w, _ = mgr.set_goal(3.0, 1.0, 'world')
        self.assertTrue(ok_w)
        np.testing.assert_allclose(mgr.goal_odom, [2.0, 1.0])

        # Advance 5 steps, then replace goal mid-navigation -> step_count resets to 0
        mgr.update_odom((0.0, 0.0, 0.0), (0.0, 0.0), stamp_sec=1.0)
        mgr.update_scan(make_open_scan(), stamp_sec=1.0)
        for i in range(5):
            mgr.step(now_sec=1.0 + 0.05 * i)
        self.assertEqual(mgr.step_count, 5)

        ok_new, _ = mgr.set_goal(0.0, 4.0, 'odom')
        self.assertTrue(ok_new)
        self.assertEqual(mgr.step_count, 0)
        np.testing.assert_allclose(mgr.goal_odom, [0.0, 4.0])

    def test_returns_idle_without_goal_or_sensors(self):
        """Verify test returns idle without goal or sensors."""
        mgr = NavigationManager()
        # No goal
        out = mgr.step(1.0)
        self.assertFalse(out.publish_cmd)

        # Goal set, no sensors yet
        mgr.set_goal(2.0, 0.0, 'odom')
        out = mgr.step(1.0)
        self.assertFalse(out.publish_cmd)

    def test_sensor_timeout_stops_robot(self):
        """Verify test sensor timeout stops robot."""
        mgr = NavigationManager(nav_config=NavConfig(sensor_timeout_sec=0.5))
        mgr.set_goal(3.0, 0.0, 'odom')
        mgr.update_odom((0.0, 0.0, 0.0), (0.1, 0.0), stamp_sec=1.0)
        mgr.update_scan(make_open_scan(), stamp_sec=1.0)

        # Fresh sensors -> normal command
        out_ok = mgr.step(now_sec=1.2)
        self.assertTrue(out_ok.publish_cmd)
        self.assertIsNone(out_ok.warning)

        # Stale sensors -> zero velocity + warning
        out_stale = mgr.step(now_sec=2.0)
        self.assertTrue(out_stale.publish_cmd)
        self.assertEqual(out_stale.cmd_vel, (0.0, 0.0))
        self.assertIsNotNone(out_stale.warning)

    def test_three_terminal_results_goal_collision_timeout(self):
        # 1. Goal reached
        """Verify test three terminal results goal collision timeout."""
        mgr = NavigationManager(nav_config=NavConfig(goal_dist_tol=0.25))
        mgr.set_goal(0.1, 0.0, 'odom')
        mgr.update_odom((0.0, 0.0, 0.0), (0.0, 0.0), stamp_sec=1.0)
        mgr.update_scan(make_open_scan(), stamp_sec=1.0)
        out_goal = mgr.step(1.0)
        self.assertEqual(out_goal.event, RESULT_GOAL)
        self.assertEqual(out_goal.cmd_vel, (0.0, 0.0))
        self.assertFalse(mgr.active)

        # 2. Collision triggered by independent safety stop
        mgr2 = NavigationManager(
            dwa_config=DWAConfig(robot_radius=0.105, collision_tol=0.18),
            scan_config=ScanConfig(laser_offset_x=0.0),
        )
        mgr2.set_goal(3.0, 0.0, 'odom')
        mgr2.update_odom((0.0, 0.0, 0.0), (0.1, 0.0), stamp_sec=1.0)
        close_scan = make_open_scan(value=0.20)  # < 0.285 m
        mgr2.update_scan(close_scan, stamp_sec=1.0)
        out_col = mgr2.step(1.0)
        self.assertEqual(out_col.event, RESULT_COLLISION)
        self.assertEqual(out_col.cmd_vel, (0.0, 0.0))
        self.assertFalse(mgr2.active)

        # 3. Timeout after max_num_steps
        mgr3 = NavigationManager(nav_config=NavConfig(max_num_steps=3))
        mgr3.set_goal(5.0, 0.0, 'odom')
        mgr3.update_odom((0.0, 0.0, 0.0), (0.0, 0.0), stamp_sec=1.0)
        mgr3.update_scan(make_open_scan(), stamp_sec=1.0)
        for _ in range(3):
            out = mgr3.step(1.0)
            self.assertIsNone(out.event)
        out_to = mgr3.step(1.0)
        self.assertEqual(out_to.event, RESULT_TIMEOUT)
        self.assertEqual(out_to.cmd_vel, (0.0, 0.0))
        self.assertFalse(mgr3.active)

    def test_feedback_every_50_steps(self):
        """Verify test feedback every 50 steps."""
        mgr = NavigationManager(nav_config=NavConfig(max_num_steps=200, feedback_rate=50))
        mgr.set_goal(4.0, 0.0, 'odom')
        mgr.update_odom((0.0, 0.0, 0.0), (0.0, 0.0), stamp_sec=1.0)
        mgr.update_scan(make_open_scan(), stamp_sec=1.0)

        feedback_steps = []
        for _ in range(105):
            out = mgr.step(1.0)
            if out.feedback_distance is not None:
                feedback_steps.append((out.step_count, out.feedback_distance))

        self.assertEqual([s for s, _ in feedback_steps], [50, 100])
        for _, d in feedback_steps:
            self.assertAlmostEqual(d, 4.0)


if __name__ == '__main__':
    unittest.main()
