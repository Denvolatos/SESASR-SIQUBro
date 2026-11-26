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


"""Regression tests for Task 10 3-point LiDAR median filter (TDD: fail first)."""

import math
import os
import unittest

import numpy as np
import yaml

from lab07_pkg.scan_utils import (
    ScanConfig,
    check_safety_stop,
    clean_scan_ranges,
    median3_filter,
    scan_to_obstacles,
)


PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))


class TestTask10Median3Filter(unittest.TestCase):
    """Verify circular 3-point median filter suppresses single-ray spikes safely."""

    def test_isolated_single_ray_spike_is_suppressed_and_does_not_stop(self):
        """Verify [0.41, 0.286, 0.39] filters to >= 0.39 and avoids false safety stop."""
        ranges = [1.5] * 36
        ranges[17] = 0.41
        ranges[18] = 0.286
        ranges[19] = 0.39

        filtered = median3_filter(ranges)
        self.assertGreaterEqual(float(filtered[18]), 0.39)

        cfg = ScanConfig(laser_offset_x=-0.032, laser_offset_y=0.0)
        is_col, min_dist = check_safety_stop(
            ranges,
            angle_min=-math.pi,
            angle_increment=2.0 * math.pi / 36,
            range_min=0.12,
            range_max=3.5,
            robot_radius=0.105,
            collision_tol=0.18,
            config=cfg,
        )
        self.assertFalse(is_col)
        self.assertGreaterEqual(min_dist, 0.35)

        # Also verify scan_to_obstacles sees filtered distances (not the 0.286 m spike)
        obs = scan_to_obstacles(
            robot_pose=(0.0, 0.0, 0.0),
            ranges=ranges,
            angle_min=-math.pi,
            angle_increment=2.0 * math.pi / 36,
            range_min=0.12,
            range_max=3.5,
            config=cfg,
        )
        self.assertGreater(obs.shape[0], 0)
        self.assertGreaterEqual(float(np.min(np.hypot(obs[:, 0], obs[:, 1]))), 0.35)

    def test_wall_edge_next_to_inf_excludes_inf_and_returns_0435(self):
        """Verify [0.435, 0.293, inf] produces 0.435 (inf excluded) and avoids false stop."""
        ranges = [math.inf] * 36
        ranges[17] = 0.435
        ranges[18] = 0.293
        ranges[19] = math.inf

        filtered = median3_filter(ranges)
        self.assertAlmostEqual(float(filtered[18]), 0.435, places=6)
        self.assertTrue(math.isinf(float(filtered[19])))

        cfg = ScanConfig(laser_offset_x=-0.032, laser_offset_y=0.0)
        is_col, min_dist = check_safety_stop(
            ranges,
            angle_min=-math.pi,
            angle_increment=2.0 * math.pi / 36,
            range_min=0.12,
            range_max=3.5,
            robot_radius=0.105,
            collision_tol=0.18,
            config=cfg,
        )
        self.assertFalse(is_col)
        self.assertGreaterEqual(min_dist, 0.40)

    def test_real_wall_three_consecutive_rays_still_triggers_safety_stop(self):
        """Verify 3+ consecutive rays at 0.27 m still trigger hard safety stop."""
        ranges = [1.5] * 36
        ranges[17] = 0.27
        ranges[18] = 0.27
        ranges[19] = 0.27

        cfg = ScanConfig(laser_offset_x=0.0, laser_offset_y=0.0)
        is_col, min_dist = check_safety_stop(
            ranges,
            angle_min=-math.pi,
            angle_increment=2.0 * math.pi / 36,
            range_min=0.12,
            range_max=3.5,
            robot_radius=0.105,
            collision_tol=0.18,
            config=cfg,
        )
        self.assertTrue(is_col)
        self.assertAlmostEqual(min_dist, 0.27, places=6)

    def test_nan_and_inf_policies_unchanged_and_isolated_valid_kept(self):
        """Verify NaN/inf handling for both nan_policy modes and isolated finite ray."""
        ranges = [math.nan, math.nan, 0.8, math.nan, math.inf, 1.2, math.inf]
        filtered = median3_filter(ranges)
        self.assertTrue(math.isnan(float(filtered[0])))
        self.assertAlmostEqual(float(filtered[2]), 0.8, places=6)
        self.assertTrue(math.isinf(float(filtered[4])))
        self.assertAlmostEqual(float(filtered[5]), 1.2, places=6)

        cfg_min = ScanConfig(max_scan_range=3.5, min_valid_range=0.05, nan_policy='min')
        r_min, _, _ = clean_scan_ranges(ranges, 0.0, 0.1, 0.12, 3.5, cfg_min)
        np.testing.assert_allclose(r_min, [0.12, 0.12, 0.8, 0.12, 3.5, 1.2, 3.5])

        cfg_max = ScanConfig(max_scan_range=3.5, min_valid_range=0.05, nan_policy='max')
        r_max, _, _ = clean_scan_ranges(ranges, 0.0, 0.1, 0.0, 3.5, cfg_max)
        np.testing.assert_allclose(r_max, [3.5, 3.5, 0.8, 3.5, 3.5, 1.2, 3.5])

    def test_circular_wraparound_first_and_last_rays_are_neighbors(self):
        """Verify ray 0 and ray -1 wrap around 360 deg for median filtering."""
        # Spike at index 0 surrounded by ray -1 (0.41) and ray 1 (0.39)
        r_first = [1.5] * 36
        r_first[-1] = 0.41
        r_first[0] = 0.286
        r_first[1] = 0.39
        f_first = median3_filter(r_first)
        self.assertAlmostEqual(float(f_first[0]), 0.39, places=6)

        # Spike at index -1 surrounded by ray -2 (0.39) and ray 0 (0.41)
        r_last = [1.5] * 36
        r_last[-2] = 0.39
        r_last[-1] = 0.286
        r_last[0] = 0.41
        f_last = median3_filter(r_last)
        self.assertAlmostEqual(float(f_last[-1]), 0.39, places=6)

        cfg = ScanConfig(laser_offset_x=-0.032, laser_offset_y=0.0)
        is_col, _ = check_safety_stop(
            r_first, -math.pi, 2.0 * math.pi / 36, 0.12, 3.5, 0.105, 0.18, cfg
        )
        self.assertFalse(is_col)

    def test_scan_median_filter_default_and_yaml_configs(self):
        """Verify scan_median_filter defaults to True in ScanConfig and both YAMLs."""
        self.assertTrue(ScanConfig().scan_median_filter)
        for fname in ('sim.yaml', 'real_robot.yaml'):
            with open(os.path.join(PKG_ROOT, 'config', fname), 'r', encoding='utf-8') as fh:
                params = yaml.safe_load(fh)['dwa_node']['ros__parameters']
            self.assertTrue(bool(params['scan_median_filter']))


if __name__ == '__main__':
    unittest.main()
