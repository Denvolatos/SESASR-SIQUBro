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


"""Regression tests for Must-Fix 1-5 from review feedback (TDD: fail first)."""

import math
import os
import sys
import unittest

import numpy as np

SCRIPTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'scripts'))
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

from evaluate_nav import (  # noqa: E402
    RunData,
    align_point_to_origin,
    compute_metrics,
    nearest_indices,
    reconstruct_landmark_trajectory,
    scan_min_center_distance,
    summarize_batch,
)
from lab07_pkg.dwa import DWAConfig, DWAController  # noqa: E402
from lab07_pkg.navigation import (  # noqa: E402
    NavConfig,
    NavigationManager,
    RESULT_COLLISION,
    ScanData,
)
from lab07_pkg.scan_utils import ScanConfig, check_scan_health  # noqa: E402


def make_scan(ranges, range_min=0.0, range_max=3.5):
    """Verify make scan."""
    n = len(ranges)
    return ScanData(
        ranges=ranges,
        angle_min=-math.pi,
        angle_increment=2.0 * math.pi / max(n, 1),
        range_min=range_min,
        range_max=range_max,
    )


class TestMustFix1ScanFailure(unittest.TestCase):
    """Verify TestMustFix1ScanFailure."""

    def test_all_zero_scan_stops_robot_and_warns(self):
        """Verify test all zero scan stops robot and warns."""
        cfg = ScanConfig(min_valid_ratio=0.2, nan_policy='max')
        healthy, reason = check_scan_health([0.0] * 36, -math.pi, 0.1, 0.0, 3.5, cfg)
        self.assertFalse(healthy)
        self.assertTrue(len(reason) > 0)

        mgr = NavigationManager(scan_config=cfg)
        mgr.set_goal(3.0, 0.0, 'odom')
        mgr.update_odom((0.0, 0.0, 0.0), (0.1, 0.0), stamp_sec=1.0)
        mgr.update_scan(make_scan([0.0] * 36), stamp_sec=1.0)
        out = mgr.step(now_sec=1.0)
        self.assertTrue(out.publish_cmd)
        self.assertEqual(out.cmd_vel, (0.0, 0.0))
        self.assertIsNotNone(out.warning)

    def test_all_nan_under_max_policy_exceeds_max_nan_ratio(self):
        """Verify test all nan under max policy exceeds max nan ratio."""
        cfg = ScanConfig(nan_policy='max', max_nan_ratio=0.95)
        healthy, reason = check_scan_health([math.nan] * 36, -math.pi, 0.1, 0.0, 3.5, cfg)
        self.assertFalse(healthy)
        self.assertIn('nan', reason.lower())


class TestMustFix2CollisionLock(unittest.TestCase):
    """Verify TestMustFix2CollisionLock."""

    def test_follow_mode_collision_locks_until_explicit_reset(self):
        """Verify test follow mode collision locks until explicit reset."""
        mgr = NavigationManager(
            dwa_config=DWAConfig(robot_radius=0.105, collision_tol=0.18),
            scan_config=ScanConfig(laser_offset_x=0.0),
            nav_config=NavConfig(mode='follow'),
        )
        mgr.update_odom((0.0, 0.0, 0.0), (0.1, 0.0), stamp_sec=1.0)
        mgr.update_scan(make_scan([0.20] * 36, range_min=0.12), stamp_sec=1.0)
        mgr.update_dynamic_goal(2.0, 0.0, 'world', stamp_sec=1.0)

        out1 = mgr.step(now_sec=1.0)
        self.assertEqual(out1.event, RESULT_COLLISION)
        self.assertTrue(mgr.collision_locked)

        # Next target message must NOT reactivate or re-emit Collision
        ok, _ = mgr.update_dynamic_goal(2.1, 0.0, 'world', stamp_sec=1.1)
        self.assertFalse(ok)
        # Even if obstacle clears, robot must stay locked and silent
        mgr.update_scan(make_scan([3.0] * 36, range_min=0.12), stamp_sec=1.2)
        out2 = mgr.step(now_sec=1.2)
        self.assertIsNone(out2.event)
        self.assertFalse(out2.publish_cmd)

        # Explicit reset unlocks navigation
        reset_ok, _ = mgr.reset_navigation()
        self.assertTrue(reset_ok)
        self.assertFalse(mgr.collision_locked)
        ok_after, _ = mgr.update_dynamic_goal(2.1, 0.0, 'world', stamp_sec=1.3)
        self.assertTrue(ok_after)


class TestMustFix3PlanningMargin(unittest.TestCase):
    """Verify TestMustFix3PlanningMargin."""

    def test_planning_radius_exceeds_stop_radius_by_planning_margin(self):
        """Verify test planning radius exceeds stop radius by planning margin."""
        cfg = DWAConfig(robot_radius=0.105, collision_tol=0.18, planning_margin=0.04)
        dwa = DWAController(cfg)
        self.assertAlmostEqual(dwa.collision_radius, 0.285, places=6)
        self.assertAlmostEqual(dwa.planning_collision_radius, 0.325, places=6)

        # Obstacle at 0.30 m is outside hard stop (0.285) but inside planning radius (0.325)
        paths = np.array([[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]], dtype=np.float64)
        _, colliding = dwa.score_obstacles(paths, np.array([[0.30, 0.0]]))
        self.assertTrue(bool(colliding[0]))


class TestMustFix4TargetIdRequired(unittest.TestCase):
    """Verify TestMustFix4TargetIdRequired."""

    def test_require_target_id_rejects_unset_id(self):
        """Verify test require target id rejects unset id."""
        with self.assertRaises(ValueError):
            NavigationManager(
                nav_config=NavConfig(mode='follow', require_target_id=True, target_id=-1)
            )
        # Setting an explicit target_id >= 0 succeeds
        mgr = NavigationManager(
            nav_config=NavConfig(mode='follow', require_target_id=True, target_id=5)
        )
        self.assertEqual(mgr.nav_cfg.target_id, 5)


class TestMustFix5EvaluateNav(unittest.TestCase):
    """Verify TestMustFix5EvaluateNav."""

    def test_follow_mode_success_without_goal_event(self):
        """Verify test follow mode success without goal event."""
        t = np.linspace(0.0, 10.0, 21)
        x_r = np.linspace(0.0, 2.0, 21)
        run = RunData(
            robot_times=t,
            robot_poses=np.column_stack((x_r, np.zeros(21), np.zeros(21))),
            cmd_times=t,
            cmd_vels=np.column_stack((np.full(21, 0.2), np.zeros(21))),
            scan_min_dists=np.full(21, 0.8),
            result='Unknown',
            target_times=t,
            target_positions=np.column_stack((x_r + 0.8, np.zeros(21))),
        )
        summary = summarize_batch([run], mode='follow')
        self.assertAlmostEqual(summary['success_rate'], 1.0)

    def test_duration_uses_nav_start_and_end_window(self):
        """Verify test duration uses nav start and end window."""
        run = RunData(
            robot_times=np.array([0.0, 5.0, 10.0, 20.0]),
            robot_poses=np.array(
                [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 0.0, 0.0]]
            ),
            cmd_times=np.array([5.0, 10.0]),
            cmd_vels=np.array([[0.2, 0.0], [0.0, 0.0]]),
            scan_min_dists=np.array([1.0]),
            result='Goal',
            start_time=5.0,
            end_time=10.0,
        )
        m = compute_metrics(run)
        self.assertAlmostEqual(m['duration_s'], 5.0)

    def test_nearest_indices_picks_closest_not_next(self):
        """Verify test nearest indices picks closest not next."""
        src = np.array([0.0, 1.0, 2.0])
        q = np.array([0.1, 0.9, 1.2])
        np.testing.assert_array_equal(nearest_indices(src, q), [0, 1, 1])

    def test_scan_min_center_distance_ignores_nan_and_applies_offset(self):
        """Verify test scan min center distance ignores nan and applies offset."""
        cfg = ScanConfig(nan_policy='min', laser_offset_x=-0.032, laser_offset_y=0.0)
        # Ray 0 at -0.1 rad is NaN; ray 1 at 0.0 rad is 0.50 m -> center dist = 0.468 m
        d = scan_min_center_distance([math.nan, 0.50], -0.1, 0.1, 0.12, 3.5, cfg)
        self.assertAlmostEqual(d, 0.468, places=6)

    def test_reconstruct_landmark_trajectory_and_align_static_goal(self):
        """Verify test reconstruct landmark trajectory and align static goal."""
        robot_t = np.array([0.0, 1.0])
        robot_p = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, math.pi / 2.0]])
        pts = reconstruct_landmark_trajectory(
            robot_t, robot_p, np.array([0.1, 0.95]), np.array([1.0, 1.0]), np.array([0.0, 0.0])
        )
        np.testing.assert_allclose(pts[0], [1.0, 0.0], atol=1e-9)
        np.testing.assert_allclose(pts[1], [1.0, 1.0], atol=1e-9)

        # Static goal (2, 4) when first odom pose was (2, 3, pi/2) -> aligned (1, 0)
        gx, gy = align_point_to_origin((2.0, 4.0), (2.0, 3.0, math.pi / 2.0))
        self.assertAlmostEqual(gx, 1.0, places=9)
        self.assertAlmostEqual(gy, 0.0, places=9)


if __name__ == '__main__':
    unittest.main()
