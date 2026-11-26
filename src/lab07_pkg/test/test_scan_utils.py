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


"""Unit tests for lab07_pkg.scan_utils (unittest)."""

import math
import unittest

import numpy as np

from lab07_pkg.scan_utils import (
    ScanConfig,
    check_safety_stop,
    clean_scan_ranges,
    downsample_scan,
    scan_to_obstacles,
)


class TestScanUtils(unittest.TestCase):
    """Verify TestScanUtils."""

    def test_clean_scan_handles_nan_inf_zero_and_over_max(self):
        """Verify test clean scan handles nan inf zero and over max."""
        ranges = [math.nan, math.inf, -math.inf, 0.0, 0.02, 0.5, 4.8]
        angle_min = -0.3
        angle_inc = 0.1

        # Policy 'min' (sim / PDF): NaN -> eff_min (0.12), +inf -> 3.5, 4.8 -> 3.5;
        # -inf, 0.0, 0.02 are irregular and dropped.
        cfg_min = ScanConfig(max_scan_range=3.5, min_valid_range=0.05, nan_policy='min')
        r_min, a_min, eff_max = clean_scan_ranges(
            ranges, angle_min, angle_inc, range_min=0.12, range_max=10.0, config=cfg_min
        )
        self.assertAlmostEqual(eff_max, 3.5)
        np.testing.assert_allclose(r_min, [0.12, 3.5, 0.5, 3.5])
        np.testing.assert_allclose(a_min, [-0.3, -0.2, 0.2, 0.3])

        # Policy 'max' (real LDS-02 with range_min=0.0): NaN -> 3.5, 0.0 & 0.02 dropped.
        cfg_max = ScanConfig(max_scan_range=3.5, min_valid_range=0.05, nan_policy='max')
        r_max, a_max, _ = clean_scan_ranges(
            ranges, angle_min, angle_inc, range_min=0.0, range_max=3.5, config=cfg_max
        )
        np.testing.assert_allclose(r_max, [3.5, 3.5, 0.5, 3.5])
        np.testing.assert_allclose(a_max, [-0.3, -0.2, 0.2, 0.3])

    def test_nan_policy_prevents_false_collision_on_real_lds02(self):
        # Real LDS-02: range_min=0.0, open space returns NaN, plus a 0.0 glitch
        """Verify test nan policy prevents false collision on real lds02."""
        ranges = [math.nan] * 10 + [0.0, 1.2] + [math.nan] * 10
        cfg_real = ScanConfig(nan_policy='max', min_valid_range=0.05, laser_offset_x=0.0)
        is_col, min_dist = check_safety_stop(
            ranges, 0.0, 0.1, 0.0, 3.5, robot_radius=0.105, collision_tol=0.18, config=cfg_real
        )
        self.assertFalse(is_col)
        self.assertAlmostEqual(min_dist, 1.2)

        # Under 'min' policy, NaN becomes 0.05 m -> triggers safety stop
        cfg_sim = ScanConfig(nan_policy='min', min_valid_range=0.05, laser_offset_x=0.0)
        is_col_sim, min_dist_sim = check_safety_stop(
            ranges, 0.0, 0.1, 0.0, 3.5, robot_radius=0.105, collision_tol=0.18, config=cfg_sim
        )
        self.assertTrue(is_col_sim)
        self.assertAlmostEqual(min_dist_sim, 0.05)

    def test_downsample_preserves_sector_min_and_true_angle(self):
        # 213 rays (real LDS-02 count, not hardcoded 270 or 360) compressed to 12 sectors
        """Verify test downsample preserves sector min and true angle."""
        n_rays = 213
        angle_min = -math.pi
        angle_inc = 2.0 * math.pi / n_rays
        ranges = [3.0] * n_rays
        # Put a real multi-ray close obstacle inside sector 0 (minimum at ray index 7)
        target_idx = 7
        ranges[target_idx] = 0.65
        ranges[target_idx + 1] = 0.65
        expected_angle = angle_min + target_idx * angle_inc

        cfg = ScanConfig(num_ranges=12, max_scan_range=3.5)
        sec_r, sec_a, _ = downsample_scan(ranges, angle_min, angle_inc, 0.12, 3.5, cfg)
        self.assertEqual(len(sec_r), 12)
        self.assertAlmostEqual(sec_r[0], 0.65)
        self.assertAlmostEqual(sec_a[0], expected_angle)

    def test_scan_to_obstacles_coordinate_transform_with_offset(self):
        # Robot at (1.0, 2.0) facing +90 deg (pi/2).
        # Laser offset (-0.1, 0.0, 0.0) in base frame.
        # Single obstacle ray directly ahead of laser (theta = 0) at r = 1.1 m:
        # In base frame: bx = -0.1 + 1.1 = 1.0, by = 0.0.
        # Rotated by +90 deg: ox = 1.0 + 0 = 1.0, oy = 2.0 + 1.0 = 3.0.
        """Verify test scan to obstacles coordinate transform with offset."""
        n_rays = 12
        ranges = [math.inf] * n_rays
        ranges[0] = 1.1
        cfg = ScanConfig(
            num_ranges=12,
            max_scan_range=3.5,
            laser_offset_x=-0.1,
            laser_offset_y=0.0,
            laser_offset_yaw=0.0,
        )
        obs = scan_to_obstacles(
            robot_pose=(1.0, 2.0, math.pi / 2.0),
            ranges=ranges,
            angle_min=0.0,
            angle_increment=2.0 * math.pi / n_rays,
            range_min=0.12,
            range_max=3.5,
            config=cfg,
        )
        self.assertEqual(obs.shape, (1, 2))
        np.testing.assert_allclose(obs[0], [1.0, 3.0], atol=1e-9)

    def test_safety_stop_accounts_for_laser_offset(self):
        # Laser is at x = -0.032 m in base frame.
        # Ray straight ahead (theta=0) at r = 0.30 m -> center distance = 0.268 m.
        # Threshold = 0.105 + 0.18 = 0.285 m -> collision!
        """Verify test safety stop accounts for laser offset."""
        cfg = ScanConfig(laser_offset_x=-0.032, laser_offset_y=0.0)
        is_col, dist = check_safety_stop(
            [0.30], 0.0, 0.1, 0.05, 3.5, robot_radius=0.105, collision_tol=0.18, config=cfg
        )
        self.assertTrue(is_col)
        self.assertAlmostEqual(dist, 0.268, places=6)


if __name__ == '__main__':
    unittest.main()
