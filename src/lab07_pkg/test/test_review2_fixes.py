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


"""Regression tests for second review items A-D (TDD: fail first)."""

import math
import os
import sys
import unittest

import numpy as np

SCRIPTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'scripts'))
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

from evaluate_nav import RunData, compute_metrics, summarize_batch  # noqa: E402
from lab07_pkg.dwa import DWAConfig, DWAController  # noqa: E402
from lab07_pkg.navigation import (  # noqa: E402
    NavigationManager,
    RESULT_COLLISION,
    ScanData,
)
from lab07_pkg.scan_utils import ScanConfig  # noqa: E402


def make_directional_scan(n_rays, front_range, other_range=math.inf, range_min=0.12):
    """Create a 360 deg scan with an obstacle ray straight ahead (angle 0)."""
    angle_min = -math.pi
    angle_inc = 2.0 * math.pi / n_rays
    ranges = [other_range] * n_rays
    front_idx = int(round((0.0 - angle_min) / angle_inc)) % n_rays
    ranges[front_idx] = front_range
    return ScanData(
        ranges=ranges,
        angle_min=angle_min,
        angle_increment=angle_inc,
        range_min=range_min,
        range_max=3.5,
    )


class TestItemAScanTimestampPose(unittest.TestCase):
    """Verify scan_to_obstacles uses the odom pose at scan timestamp."""

    def test_scan_uses_odom_pose_at_scan_timestamp_not_latest(self):
        """Obstacle projected from a 0.2 s old scan must match the older odom yaw."""
        scan_cfg = ScanConfig(num_ranges=24, laser_offset_x=0.0)
        mgr = NavigationManager(scan_config=scan_cfg)

        # At t=1.0 robot faces +x (yaw=0); at t=1.2 robot rotated to +y (yaw=pi/2)
        mgr.update_odom((0.0, 0.0, 0.0), (0.0, 1.5), stamp_sec=1.0)
        mgr.update_odom((0.0, 0.0, math.pi / 2.0), (0.0, 1.5), stamp_sec=1.2)

        # Scan was captured at t=1.0 with an obstacle 1.0 m directly ahead -> (1.0, 0.0)
        scan = make_directional_scan(36, front_range=1.0)
        mgr.update_scan(scan, stamp_sec=1.0, now_sec=1.2)

        self.assertIsNotNone(mgr.cached_obstacles_odom)
        self.assertEqual(mgr.cached_obstacles_odom.shape, (1, 2))
        np.testing.assert_allclose(mgr.cached_obstacles_odom[0], [1.0, 0.0], atol=0.1)


class TestItemBFollowSuccessCriterion(unittest.TestCase):
    """Verify follow mode never reports stationary or lost runs as success."""

    def test_stationary_robot_is_not_follow_success(self):
        """A robot that never moves must have 0% follow success rate."""
        t = np.linspace(0.0, 30.0, 31)
        poses = np.zeros((31, 3), dtype=np.float64)
        target = np.column_stack((np.linspace(1.0, 10.0, 31), np.zeros(31)))
        run = RunData(
            robot_times=t,
            robot_poses=poses,
            cmd_times=t,
            cmd_vels=np.zeros((31, 2)),
            scan_min_dists=np.ones(31),
            result='Unknown',
            target_times=t,
            target_positions=target,
        )
        summary = summarize_batch([run], mode='follow')
        self.assertEqual(summary['success_rate'], 0.0)
        self.assertIn('Lost', summary['failures'])

    def test_well_tracked_follow_run_succeeds(self):
        """A robot tracking within distance threshold to the end succeeds."""
        t = np.linspace(0.0, 20.0, 41)
        x_r = np.linspace(0.0, 3.0, 41)
        poses = np.column_stack((x_r, np.zeros(41), np.zeros(41)))
        target = np.column_stack((x_r + 0.85, np.zeros(41)))
        run = RunData(
            robot_times=t,
            robot_poses=poses,
            cmd_times=t,
            cmd_vels=np.column_stack((np.full(41, 0.15), np.zeros(41))),
            scan_min_dists=np.ones(41),
            result='Unknown',
            target_times=t,
            target_positions=target,
        )
        m = compute_metrics(run, desired_target_dist=0.8, follow_dist_threshold=2.0)
        self.assertAlmostEqual(m['in_range_ratio'], 1.0)
        self.assertAlmostEqual(m['final_target_dist_m'], 0.85, places=6)
        summary = summarize_batch([run], mode='follow')
        self.assertEqual(summary['success_rate'], 1.0)


class TestItemCMarginBandDeadlock(unittest.TestCase):
    """Verify robot escapes when inside [stop_radius, planning_radius) band."""

    def test_margin_band_allows_non_decreasing_clearance_and_turning(self):
        """In margin band (0.30 m), DWA must not deadlock with all_colliding."""
        cfg = DWAConfig(
            robot_radius=0.105,
            collision_tol=0.18,
            planning_margin=0.04,
            dt=1.0 / 15.0,
            max_ang_acc=3.2,
        )
        dwa = DWAController(cfg)
        # Obstacle directly ahead at 0.32 m (between 0.315 band floor and 0.325 planning);
        # goal is behind robot at (-2.0, 0.0). Robot should turn in place, not freeze!
        decision = dwa.compute_cmd_detailed(
            goal_pose=[-2.0, 0.0],
            robot_pose=[0.0, 0.0, 0.0],
            current_vel=[0.0, 0.0],
            obstacles=[[0.32, 0.0]],
        )
        self.assertFalse(decision.all_colliding)
        self.assertGreater(decision.num_safe, 0)
        self.assertGreater(abs(float(decision.cmd_vel[1])), 0.0)


class TestItemDResetNavRecovery(unittest.TestCase):
    """Verify /reset_nav enables safe escape from stop zone without disabling hard stop."""

    def test_reset_nav_allows_escape_away_from_obstacle_while_keeping_hard_stop(self):
        """After /reset_nav at 0.27 m, robot can turn/escape without instant re-Collision."""
        dwa_cfg = DWAConfig(robot_radius=0.105, collision_tol=0.18, planning_margin=0.04)
        scan_cfg = ScanConfig(laser_offset_x=0.0)
        mgr = NavigationManager(dwa_config=dwa_cfg, scan_config=scan_cfg)

        # Trigger Collision at 0.27 m (< 0.285 m)
        mgr.set_goal(2.0, 0.0, 'odom')
        mgr.update_odom((0.0, 0.0, 0.0), (0.0, 0.0), stamp_sec=1.0)
        mgr.update_scan(make_directional_scan(36, front_range=0.27), stamp_sec=1.0)
        out_col = mgr.step(now_sec=1.0)
        self.assertEqual(out_col.event, RESULT_COLLISION)

        # Reset and set a new goal behind the robot; must NOT immediately re-Collision
        mgr.reset_navigation()
        mgr.set_goal(-2.0, 0.0, 'odom')
        out_rec = mgr.step(now_sec=1.05)
        self.assertIsNone(out_rec.event)
        self.assertTrue(out_rec.publish_cmd)
        self.assertGreater(abs(out_rec.cmd_vel[1]), 0.0)

        # Hard stop MUST still fire if obstacle gets dangerously closer (0.20 m)
        mgr.update_scan(make_directional_scan(36, front_range=0.20), stamp_sec=1.1)
        out_hard = mgr.step(now_sec=1.1)
        self.assertEqual(out_hard.event, RESULT_COLLISION)


if __name__ == '__main__':
    unittest.main()
