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


"""Unit tests for scripts/evaluate_nav.py (pure-Python metrics, alignment, plotting)."""

import math
import os
import sys
import tempfile
import unittest

import numpy as np

SCRIPTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'scripts'))
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

from evaluate_nav import (  # noqa: E402
    RunData,
    align_odom_to_origin,
    compute_metrics,
    plot_run,
    summarize_batch,
)


class TestEvaluateNav(unittest.TestCase):
    """Verify TestEvaluateNav."""

    def test_align_odom_to_origin(self):
        # Starts at (2, 3, pi/2), moves 1 m forward along +y -> aligned (1, 0, 0)
        """Verify test align odom to origin."""
        poses = np.array([
            [2.0, 3.0, math.pi / 2.0],
            [2.0, 4.0, math.pi / 2.0],
        ], dtype=np.float64)
        aligned = align_odom_to_origin(poses)
        np.testing.assert_allclose(aligned[0], [0.0, 0.0, 0.0], atol=1e-9)
        np.testing.assert_allclose(aligned[1], [1.0, 0.0, 0.0], atol=1e-9)

    def test_compute_metrics_task1_and_task2_rmse(self):
        """Verify test compute metrics task1 and task2 rmse."""
        times = np.array([0.0, 1.0, 2.0], dtype=np.float64)
        poses = np.array([
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
        ], dtype=np.float64)
        # Target stays exactly 1.0 m ahead (optimal=0.8 -> dist err = 0.2 m, bearing err = 0)
        target_pos = np.array([[1.0, 0.0], [2.0, 0.0], [3.0, 0.0]], dtype=np.float64)
        run = RunData(
            robot_times=times,
            robot_poses=poses,
            cmd_times=times,
            cmd_vels=np.array([[0.2, 0.0], [0.2, 0.1], [0.0, 0.0]]),
            scan_min_dists=np.array([0.5, 1.0, 1.5]),
            result='Goal',
            target_times=times,
            target_positions=target_pos,
        )
        m = compute_metrics(run, desired_target_dist=0.8)
        self.assertAlmostEqual(m['duration_s'], 2.0)
        self.assertAlmostEqual(m['path_length_m'], 2.0)
        self.assertAlmostEqual(m['min_obstacle_dist_m'], 0.5)
        self.assertAlmostEqual(m['avg_obstacle_dist_m'], 1.0)
        self.assertAlmostEqual(m['target_dist_rmse_m'], 0.2, places=9)
        self.assertAlmostEqual(m['target_bearing_rmse_rad'], 0.0, places=9)

    def test_summarize_batch_and_plot_generation(self):
        """Verify test summarize batch and plot generation."""
        base = RunData(
            robot_times=np.array([0.0, 1.0]),
            robot_poses=np.array([[0.0, 0.0, 0.0], [0.5, 0.0, 0.0]]),
            cmd_times=np.array([0.0, 1.0]),
            cmd_vels=np.array([[0.15, 0.0], [0.0, 0.0]]),
            scan_min_dists=np.array([0.8]),
            result='Goal',
            static_goal=(0.5, 0.0),
        )
        fail = RunData(
            robot_times=np.array([0.0, 1.0]),
            robot_poses=np.array([[0.0, 0.0, 0.0], [0.2, 0.0, 0.0]]),
            cmd_times=np.array([0.0, 1.0]),
            cmd_vels=np.array([[0.1, 0.0], [0.0, 0.0]]),
            scan_min_dists=np.array([0.2]),
            result='Collision',
        )
        summary = summarize_batch([base, fail])
        self.assertEqual(summary['num_runs'], 2)
        self.assertAlmostEqual(summary['success_rate'], 0.5)
        self.assertEqual(summary['failures'], {'Collision': 1})

        with tempfile.TemporaryDirectory() as tmpdir:
            p1, p2 = plot_run(base, os.path.join(tmpdir, 'test'))
            self.assertTrue(os.path.exists(p1))
            self.assertTrue(os.path.exists(p2))


if __name__ == '__main__':
    unittest.main()
