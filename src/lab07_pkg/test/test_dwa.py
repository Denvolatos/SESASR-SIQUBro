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


"""Unit tests for lab07_pkg.dwa (unittest)."""

import math
import unittest

import numpy as np

from lab07_pkg.dwa import DWAConfig, DWAController, normalize_angle, normalize_scores


class TestDWAController(unittest.TestCase):
    """Verify TestDWAController."""

    def test_dynamic_window_uses_odom_vel_and_control_dt(self):
        """Verify test dynamic window uses odom vel and control dt."""
        cfg = DWAConfig(
            dt=0.1,
            time_granularity=0.2,  # intentionally distinct from dt
            max_lin_vel=0.22,
            min_lin_vel=0.0,
            max_ang_vel=2.0,
            min_ang_vel=-2.0,
            max_lin_acc=0.5,
            max_ang_acc=3.0,
        )
        dwa = DWAController(cfg)
        v_min, v_max, w_min, w_max = dwa.compute_dynamic_window((0.10, 0.50))
        self.assertAlmostEqual(v_min, 0.05)
        self.assertAlmostEqual(v_max, 0.15)
        self.assertAlmostEqual(w_min, 0.20)
        self.assertAlmostEqual(w_max, 0.80)

        # Clamping at kinematic limits
        v_min2, v_max2, w_min2, w_max2 = dwa.compute_dynamic_window((0.0, 1.9))
        self.assertAlmostEqual(v_min2, 0.0)
        self.assertAlmostEqual(v_max2, 0.05)
        self.assertAlmostEqual(w_max2, 2.0)

    def test_simulate_paths_uses_time_granularity_not_dt(self):
        """Verify test simulate paths uses time granularity not dt."""
        cfg = DWAConfig(dt=1.0 / 15.0, sim_time=1.0, time_granularity=0.25)
        dwa = DWAController(cfg)
        vels = np.array([[0.2, 0.0]], dtype=np.float64)
        paths = dwa.simulate_paths([0.0, 0.0, 0.0], vels)
        # sim_steps = round(1.0 / 0.25) = 4 -> 5 states (t=0..1.0)
        self.assertEqual(paths.shape, (1, 5, 3))
        self.assertAlmostEqual(paths[0, -1, 0], 0.2, places=9)

    def test_task1_score_vel_is_raw_linear_velocity(self):
        """Verify test task1 score vel is raw linear velocity."""
        dwa = DWAController()
        vels = np.array([[0.0, 1.0], [0.11, -0.5], [0.22, 0.0]], dtype=np.float64)
        np.testing.assert_allclose(dwa.score_vel(vels), [0.0, 0.11, 0.22])

    def test_colliding_trajectories_filtered_before_normalization(self):
        # Obstacle directly ahead at (0.35, 0.0); robot at origin facing +x.
        # Straight trajectories will collide (< 0.105 + 0.18 = 0.285 m),
        # while turning trajectories remain safe and must be selected.
        """Verify test colliding trajectories filtered before normalization."""
        cfg = DWAConfig(
            dt=0.2,
            sim_time=2.0,
            time_granularity=0.1,
            max_lin_vel=0.22,
            max_lin_acc=1.0,
            max_ang_acc=5.0,
            robot_radius=0.105,
            collision_tol=0.18,
            weight_heading=0.3,
            weight_vel=0.2,
            weight_obs=0.5,
        )
        dwa = DWAController(cfg)
        obstacles = np.array([[0.40, 0.0]], dtype=np.float64)
        decision = dwa.compute_cmd_detailed(
            goal_pose=[2.0, 0.0],
            robot_pose=[0.0, 0.0, 0.0],
            current_vel=[0.1, 0.0],
            obstacles=obstacles,
        )
        self.assertFalse(decision.all_colliding)
        self.assertLess(decision.num_safe, decision.num_candidates)
        self.assertGreater(decision.num_safe, 0)
        # Selected path must maintain clearance >= collision_radius
        dists = np.hypot(
            decision.best_path[:, 0] - obstacles[0, 0],
            decision.best_path[:, 1] - obstacles[0, 1],
        )
        self.assertGreaterEqual(float(np.min(dists)), dwa.collision_radius - 1e-9)

    def test_all_colliding_commands_zero_velocity(self):
        # Obstacle already inside collision_radius around robot -> all trajectories collide
        """Verify test all colliding commands zero velocity."""
        cfg = DWAConfig(robot_radius=0.105, collision_tol=0.18)
        dwa = DWAController(cfg)
        obstacles = np.array([[0.20, 0.0]], dtype=np.float64)
        decision = dwa.compute_cmd_detailed(
            goal_pose=[2.0, 0.0],
            robot_pose=[0.0, 0.0, 0.0],
            current_vel=[0.15, 0.2],
            obstacles=obstacles,
        )
        self.assertTrue(decision.all_colliding)
        self.assertEqual(decision.num_safe, 0)
        np.testing.assert_allclose(decision.cmd_vel, [0.0, 0.0])

    def test_list_goal_pose_accepted(self):
        # Regression test for course bug `if goal_pose is list`
        """Verify test list goal pose accepted."""
        dwa = DWAController()
        cmd = dwa.compute_cmd([2.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0], [])
        self.assertEqual(cmd.shape, (2,))
        self.assertGreater(cmd[0], 0.0)

    def test_normalize_helpers(self):
        """Verify test normalize helpers."""
        self.assertAlmostEqual(normalize_angle(3.0 * math.pi), -math.pi)
        np.testing.assert_allclose(normalize_scores(np.array([5.0, 5.0])), [0.0, 0.0])
        np.testing.assert_allclose(normalize_scores(np.array([2.0, 4.0, 6.0])), [0.0, 0.5, 1.0])


if __name__ == '__main__':
    unittest.main()
