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


"""Unit tests for Stage B (Task 2 dynamic goal following)."""

import math
import unittest

import numpy as np

from lab07_pkg.dwa import DWAConfig, DWAController
from lab07_pkg.navigation import (
    LandmarkObservation,
    NavConfig,
    NavigationManager,
    ScanData,
    landmark_to_odom,
)


def make_open_scan(n_rays: int = 36, value: float = 3.5) -> ScanData:
    """Verify make open scan."""
    return ScanData(
        ranges=[value] * n_rays,
        angle_min=-math.pi,
        angle_increment=2.0 * math.pi / n_rays,
        range_min=0.12,
        range_max=3.5,
    )


class TestFollowMode(unittest.TestCase):
    """Verify TestFollowMode."""

    def test_vel_prime_slows_down_below_threshold_and_is_continuous(self):
        """Verify test vel prime slows down below threshold and is continuous."""
        cfg = DWAConfig(
            follow_mode=True,
            max_lin_vel=0.22,
            slowdown_dist_threshold=1.0,
        )
        dwa = DWAController(cfg)
        vels = np.array([[0.0, 0.0], [0.11, 0.0], [0.22, 0.0]], dtype=np.float64)

        # Far from goal (d = 2.0 >= 1.0): vel' equals raw v
        far_paths = np.zeros((3, 2, 3), dtype=np.float64)
        far_scores = dwa.score_vel_prime(vels, far_paths, np.array([2.0, 0.0]))
        np.testing.assert_allclose(far_scores, [0.0, 0.11, 0.22])

        # Halfway inside threshold (d = 0.5 -> v_ref = 0.11): middle velocity peaks
        mid_scores = dwa.score_vel_prime(vels, far_paths, np.array([0.5, 0.0]))
        self.assertGreater(mid_scores[1], mid_scores[0])
        self.assertGreater(mid_scores[1], mid_scores[2])

    def test_dist_target_prefers_desired_distance_and_zero_bearing(self):
        """Verify test dist target prefers desired distance and zero bearing."""
        cfg = DWAConfig(
            follow_mode=True,
            desired_target_dist=0.8,
            target_dist_sigma=0.5,
            camera_fov_half_rad=0.576,
        )
        dwa = DWAController(cfg)
        goal = np.array([1.0, 0.0])
        # Path 0: at (0.2, 0.0), yaw=0 -> dist=0.8, bearing=0 (optimal)
        # Path 1: at (0.2, 0.0), yaw=1.0 -> dist=0.8, bearing=-1.0 (outside FoV)
        # Path 2: at (-1.0, 0.0), yaw=0 -> dist=2.0, bearing=0 (too far)
        paths = np.array([
            [[0.0, 0.0, 0.0], [0.2, 0.0, 0.0]],
            [[0.0, 0.0, 0.0], [0.2, 0.0, 1.0]],
            [[0.0, 0.0, 0.0], [-1.0, 0.0, 0.0]],
        ], dtype=np.float64)
        scores = dwa.score_target_following(paths, goal)
        self.assertAlmostEqual(scores[0], 1.0, places=9)
        self.assertGreater(scores[0], scores[1])
        self.assertGreater(scores[0], scores[2])

    def test_follow_mode_does_not_stop_when_close_to_target(self):
        """Verify test follow mode does not stop when close to target."""
        nav_cfg = NavConfig(mode='follow', goal_dist_tol=0.25, max_num_steps=0)
        mgr = NavigationManager(nav_config=nav_cfg)
        mgr.update_odom((0.0, 0.0, 0.0), (0.0, 0.0), stamp_sec=1.0)
        mgr.update_scan(make_open_scan(), stamp_sec=1.0)
        mgr.update_dynamic_goal(0.1, 0.0, 'world', stamp_sec=1.0)

        out = mgr.step(now_sec=1.0)
        self.assertIsNone(out.event)
        self.assertTrue(mgr.active)
        self.assertTrue(out.publish_cmd)

    def test_dynamic_goal_jump_detection(self):
        """Verify test dynamic goal jump detection."""
        nav_cfg = NavConfig(mode='follow', target_jump_threshold=2.5)
        mgr = NavigationManager(nav_config=nav_cfg)
        ok1, _ = mgr.update_dynamic_goal(-7.0, 7.5, 'world', stamp_sec=1.0)
        self.assertTrue(ok1)
        ok2, info2 = mgr.update_dynamic_goal(1.5, -0.5, 'world', stamp_sec=1.1)
        self.assertTrue(ok2)
        self.assertIn('jump', info2.lower())
        self.assertGreater(mgr.last_target_jump_m, 10.0)
        np.testing.assert_allclose(mgr.goal_odom, [1.5, -0.5])

    def test_landmark_conversion_uses_closest_odom_stamp_and_target_id(self):
        # Verify pure coordinate conversion first
        """Verify test landmark conversion uses closest odom stamp and target id."""
        gx, gy = landmark_to_odom((1.0, 2.0, math.pi / 2.0), range_m=1.0, bearing_rad=0.0)
        self.assertAlmostEqual(gx, 1.0, places=9)
        self.assertAlmostEqual(gy, 3.0, places=9)

        # Two odom stamps: at t=1.0 robot is at (0,0,0), at t=2.0 robot moved to (5,0,0)
        nav_cfg = NavConfig(mode='follow', target_id=7)
        mgr = NavigationManager(nav_config=nav_cfg)
        mgr.update_odom((0.0, 0.0, 0.0), (0.0, 0.0), stamp_sec=1.0)
        mgr.update_odom((5.0, 0.0, 0.0), (0.0, 0.0), stamp_sec=2.0)

        # Landmark observed at t=1.05 should use pose near t=1.0 (0,0,0) and pick id=7
        lms = [
            LandmarkObservation(landmark_id=3, range_m=0.5, bearing_rad=0.0),
            LandmarkObservation(landmark_id=7, range_m=1.2, bearing_rad=0.0),
        ]
        ok, _ = mgr.update_landmarks(lms, stamp_sec=1.05)
        self.assertTrue(ok)
        np.testing.assert_allclose(mgr.goal_odom, [1.2, 0.0], atol=1e-9)

        # Default target_id=-1 picks nearest landmark (id=3 at 0.5 m) using latest pose at t=2.0
        mgr_nearest = NavigationManager(nav_config=NavConfig(mode='follow', target_id=-1))
        mgr_nearest.update_odom((5.0, 0.0, 0.0), (0.0, 0.0), stamp_sec=2.0)
        ok_n, _ = mgr_nearest.update_landmarks(lms, stamp_sec=2.0)
        self.assertTrue(ok_n)
        np.testing.assert_allclose(mgr_nearest.goal_odom, [5.5, 0.0], atol=1e-9)

    def test_target_loss_holds_briefly_then_stops_and_resumes(self):
        """Verify test target loss holds briefly then stops and resumes."""
        nav_cfg = NavConfig(mode='follow', target_loss_timeout_sec=1.5, sensor_timeout_sec=5.0)
        mgr = NavigationManager(nav_config=nav_cfg)
        mgr.update_odom((0.0, 0.0, 0.0), (0.1, 0.0), stamp_sec=1.0)
        mgr.update_scan(make_open_scan(), stamp_sec=1.0)
        mgr.update_landmarks([LandmarkObservation(1, 1.5, 0.0)], stamp_sec=1.0)

        # Empty array at t=1.5 should NOT refresh last_target_time
        ok_empty, _ = mgr.update_landmarks([], stamp_sec=1.5)
        self.assertFalse(ok_empty)

        # Within timeout (age = 1.0 s <= 1.5 s): keep pursuing last target
        out_hold = mgr.step(now_sec=2.0)
        self.assertFalse(out_hold.target_lost)
        self.assertGreater(out_hold.cmd_vel[0], 0.0)

        # Beyond timeout (age = 2.0 s > 1.5 s): stop safely, remain active
        out_lost = mgr.step(now_sec=3.0)
        self.assertTrue(out_lost.target_lost)
        self.assertEqual(out_lost.cmd_vel, (0.0, 0.0))
        self.assertTrue(mgr.active)

        # Reacquire tag at t=3.2: resumes immediately
        mgr.update_landmarks([LandmarkObservation(1, 1.5, 0.0)], stamp_sec=3.2)
        out_resume = mgr.step(now_sec=3.3)
        self.assertFalse(out_resume.target_lost)
        self.assertGreater(out_resume.cmd_vel[0], 0.0)


if __name__ == '__main__':
    unittest.main()
