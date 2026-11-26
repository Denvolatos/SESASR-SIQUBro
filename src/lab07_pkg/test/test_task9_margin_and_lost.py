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


"""Regression tests for Task 9 margin-band creep and non-stopping Lost (TDD)."""

import math
import os
import unittest

import numpy as np
import yaml

from lab07_pkg.dwa import DWAConfig, DWAController
from lab07_pkg.navigation import (
    NavConfig,
    NavigationManager,
    RESULT_LOST,
    ScanData,
)


PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))


def make_open_scan(n_rays=36, value=3.5):
    """Create a 360 deg open-space LiDAR scan."""
    return ScanData(
        ranges=[value] * n_rays,
        angle_min=-math.pi,
        angle_increment=2.0 * math.pi / n_rays,
        range_min=0.12,
        range_max=3.5,
    )


class TestTask9MarginBandAndLost(unittest.TestCase):
    """Verify margin-band anchor, band_floor_buffer, and non-stopping Lost."""

    def test_no_creep_toward_emergency_stop_in_margin_band(self):
        """Verify 50 integrated steps in margin band never drop below stop + buffer."""
        cfg = DWAConfig(
            follow_mode=True,
            dt=1.0 / 15.0,
            robot_radius=0.105,
            collision_tol=0.18,
            planning_margin=0.10,
            band_floor_buffer=0.03,
            w_samples=21,
        )
        dwa = DWAController(cfg)

        # Wall along y = -3.0 (side-front); robot starts 0.36 m away inside margin band,
        # goal is behind the wall at (-1.5, -2.5) pulling robot toward the wall.
        wall = np.column_stack((np.arange(-2.0, 2.001, 0.05), np.full(81, -3.0)))
        pose = np.array([0.0, -3.36, math.radians(170.0)], dtype=np.float64)
        vel = np.array([0.15, 0.0], dtype=np.float64)
        goal = np.array([-1.5, -2.5], dtype=np.float64)

        min_seen = float('inf')
        for _ in range(50):
            dec = dwa.compute_cmd_detailed(goal, pose, vel, wall)
            vel = dec.cmd_vel
            pose = np.array([
                pose[0] + vel[0] * math.cos(pose[2]) * cfg.dt,
                pose[1] + vel[0] * math.sin(pose[2]) * cfg.dt,
                pose[2] + vel[1] * cfg.dt,
            ], dtype=np.float64)
            d_now = float(np.min(np.hypot(pose[0] - wall[:, 0], pose[1] - wall[:, 1])))
            min_seen = min(min_seen, d_now)

        floor_expected = 0.285 + 0.03 - 1e-6
        self.assertGreaterEqual(min_seen, floor_expected)

    def test_band_entry_min_recorded_and_cleared_on_exit(self):
        """Verify band_entry_min locks on band entry, clears on exit, and re-records."""
        cfg = DWAConfig(
            follow_mode=True,
            robot_radius=0.105,
            collision_tol=0.18,
            planning_margin=0.10,
            band_floor_buffer=0.03,
        )
        dwa = DWAController(cfg)
        self.assertIsNone(dwa.band_entry_min)

        # Enter margin band at 0.36 m (< 0.385 m)
        dwa.compute_cmd_detailed([2.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0], [[0.0, 0.36]])
        self.assertAlmostEqual(dwa.band_entry_min, 0.36, places=6)

        # Even if queried at 0.34 m while still in band, entry anchor must NOT move down
        dwa.compute_cmd_detailed([2.0, 0.0], [0.0, 0.02, 0.0], [0.0, 0.0], [[0.0, 0.36]])
        self.assertAlmostEqual(dwa.band_entry_min, 0.36, places=6)

        # Exit margin band (>= 0.385 m) -> cleared
        dwa.compute_cmd_detailed([2.0, 0.0], [0.0, -0.20, 0.0], [0.0, 0.0], [[0.0, 0.36]])
        self.assertIsNone(dwa.band_entry_min)

        # Re-enter at 0.34 m -> records new entry distance 0.34 m
        dwa.compute_cmd_detailed([2.0, 0.0], [0.0, 0.02, 0.0], [0.0, 0.0], [[0.0, 0.36]])
        self.assertAlmostEqual(dwa.band_entry_min, 0.34, places=6)

    def test_lost_reports_once_keeps_chasing_and_rearms_after_return(self):
        """Verify Lost publishes once without stopping, stays active, and re-arms <= 3 m."""
        nav_cfg = NavConfig(
            mode='follow',
            max_num_steps=0,
            lost_dist=3.0,
            lost_dist_time_sec=10.0,
            sensor_timeout_sec=40.0,
            target_loss_timeout_sec=40.0,
        )
        mgr = NavigationManager(nav_config=nav_cfg)
        mgr.update_odom((0.0, 0.0, 0.0), (0.1, 0.0), stamp_sec=0.0)
        mgr.update_scan(make_open_scan(), stamp_sec=0.0)
        mgr.update_dynamic_goal(4.5, 0.0, 'world', stamp_sec=0.0)

        out0 = mgr.step(now_sec=0.0)
        self.assertIsNone(out0.event)

        # Exceeds 10 s beyond 3 m -> emits Lost ONCE, robot keeps moving (v > 0), active=True
        out10 = mgr.step(now_sec=10.1)
        self.assertEqual(out10.event, RESULT_LOST)
        self.assertGreater(out10.cmd_vel[0], 0.0)
        self.assertTrue(mgr.active)

        # Next tick still > 3 m -> does NOT emit duplicate Lost, keeps chasing
        out11 = mgr.step(now_sec=11.0)
        self.assertIsNone(out11.event)
        self.assertGreater(out11.cmd_vel[0], 0.0)

        # Target comes back within 3 m -> re-arms Lost detector
        mgr.update_dynamic_goal(2.0, 0.0, 'world', stamp_sec=12.0)
        out12 = mgr.step(now_sec=12.0)
        self.assertIsNone(out12.event)

        # Exceeds 3 m again for 10 s -> emits Lost a second time
        mgr.update_dynamic_goal(4.5, 0.0, 'world', stamp_sec=13.0)
        self.assertIsNone(mgr.step(now_sec=13.0).event)
        out23 = mgr.step(now_sec=23.1)
        self.assertEqual(out23.event, RESULT_LOST)
        self.assertGreater(out23.cmd_vel[0], 0.0)

    def test_band_floor_buffer_validation_and_yaml_configs(self):
        """Verify band_floor_buffer parameter bounds and YAML defaults."""
        with self.assertRaises(ValueError):
            DWAConfig(planning_margin=0.10, band_floor_buffer=-0.01)
        with self.assertRaises(ValueError):
            DWAConfig(planning_margin=0.10, band_floor_buffer=0.10)

        for fname in ('sim.yaml', 'real_robot.yaml'):
            with open(os.path.join(PKG_ROOT, 'config', fname), 'r', encoding='utf-8') as fh:
                params = yaml.safe_load(fh)['dwa_node']['ros__parameters']
            self.assertAlmostEqual(float(params['band_floor_buffer']), 0.03, places=6)


if __name__ == '__main__':
    unittest.main()
