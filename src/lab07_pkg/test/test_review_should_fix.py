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


"""Regression tests for Should-Fix 6-9 from review feedback (TDD: fail first)."""

import math
import unittest

import numpy as np

from lab07_pkg.dwa import DWAConfig, DWAController
from lab07_pkg.navigation import (
    LandmarkObservation,
    NavConfig,
    NavigationManager,
    ScanData,
)


def make_open_scan(n_rays=36, value=3.5):
    """Verify make open scan."""
    return ScanData(
        ranges=[value] * n_rays,
        angle_min=-math.pi,
        angle_increment=2.0 * math.pi / n_rays,
        range_min=0.12,
        range_max=3.5,
    )


class TestShouldFix6ZeroAngularVelocity(unittest.TestCase):
    """Verify TestShouldFix6ZeroAngularVelocity."""

    def test_even_w_samples_still_includes_exact_zero_and_drives_straight(self):
        """Verify test even w samples still includes exact zero and drives straight."""
        cfg = DWAConfig(w_samples=20)
        dwa = DWAController(cfg)
        _, vels = dwa.get_trajectories([0.0, 0.0, 0.0], [0.0, 0.0])
        self.assertTrue(np.any(np.isclose(vels[:, 1], 0.0)))

        # Goal straight ahead with no obstacles -> exact w == 0.0
        cmd = dwa.compute_cmd([3.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0], [])
        self.assertAlmostEqual(float(cmd[1]), 0.0, places=9)


class TestShouldFix7WindowSource(unittest.TestCase):
    """Verify TestShouldFix7WindowSource."""

    def test_window_source_last_cmd_ramps_velocity_even_if_odom_stalls(self):
        """Verify test window source last cmd ramps velocity even if odom stalls."""
        nav_cfg = NavConfig(window_source='last_cmd')
        dwa_cfg = DWAConfig(dt=1.0 / 15.0, max_lin_acc=0.5, max_lin_vel=0.22)
        mgr = NavigationManager(dwa_config=dwa_cfg, nav_config=nav_cfg)
        mgr.set_goal(3.0, 0.0, 'odom')
        mgr.update_scan(make_open_scan(), stamp_sec=1.0)
        # Odom velocity stays 0.0 (e.g. static friction on real robot)
        mgr.update_odom((0.0, 0.0, 0.0), (0.0, 0.0), stamp_sec=1.0)

        out1 = mgr.step(now_sec=1.0)
        out2 = mgr.step(now_sec=1.05)
        self.assertGreater(out2.cmd_vel[0], out1.cmd_vel[0])


class TestShouldFix8RecoverySpin(unittest.TestCase):
    """Verify TestShouldFix8RecoverySpin."""

    def test_optional_recovery_spin_when_all_trajectories_collide(self):
        """Verify test optional recovery spin when all trajectories collide."""
        cfg = DWAConfig(
            robot_radius=0.105,
            collision_tol=0.18,
            planning_margin=0.04,
            enable_recovery_spin=True,
            recovery_ang_vel=0.4,
        )
        dwa = DWAController(cfg)
        # Obstacle at 0.20 m: inside hard-stop collision_radius (0.285) -> all collide
        decision = dwa.compute_cmd_detailed(
            [2.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0], [[0.20, 0.0]]
        )
        self.assertTrue(decision.all_colliding)
        self.assertAlmostEqual(float(decision.cmd_vel[0]), 0.0, places=9)
        self.assertAlmostEqual(float(decision.cmd_vel[1]), 0.4, places=9)


class TestShouldFix9ClockSkewCheck(unittest.TestCase):
    """Verify TestShouldFix9ClockSkewCheck."""

    def test_landmark_clock_skew_warns_and_falls_back_safely(self):
        """Verify test landmark clock skew warns and falls back safely."""
        nav_cfg = NavConfig(mode='follow', max_stamp_skew_sec=0.5)
        mgr = NavigationManager(nav_config=nav_cfg)
        mgr.update_odom((1.0, 0.0, 0.0), (0.0, 0.0), stamp_sec=100.0)
        # Observation timestamp from mismatched clock (skew = 90 s > 0.5 s)
        ok, info = mgr.update_landmarks([LandmarkObservation(1, 1.0, 0.0)], stamp_sec=10.0)
        self.assertTrue(ok)
        self.assertIn('skew', info.lower())
        np.testing.assert_allclose(mgr.goal_odom, [2.0, 0.0], atol=1e-9)


if __name__ == '__main__':
    unittest.main()
