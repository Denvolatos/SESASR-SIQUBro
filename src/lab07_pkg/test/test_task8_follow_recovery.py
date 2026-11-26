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


"""Regression tests for Task 8 follow corridor recovery (TDD: fail first)."""

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
from lab07_pkg.scan_utils import ScanConfig


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


def make_scan_from_points(robot_pose, obs_points, n_rays=230):
    """Build a synthetic 360 deg LaserScan from 2D world obstacle points."""
    rx, ry, ryaw = float(robot_pose[0]), float(robot_pose[1]), float(robot_pose[2])
    angle_min = -math.pi
    angle_inc = 2.0 * math.pi / n_rays
    ranges = [math.inf] * n_rays
    for ox, oy in obs_points:
        dx = float(ox) - rx
        dy = float(oy) - ry
        dist = math.hypot(dx, dy)
        if dist < 0.12 or dist > 3.5:
            continue
        rel = (math.atan2(dy, dx) - ryaw + math.pi) % (2.0 * math.pi) - math.pi
        idx = int(round((rel - angle_min) / angle_inc)) % n_rays
        if dist < ranges[idx]:
            ranges[idx] = dist
    return ScanData(
        ranges=ranges,
        angle_min=angle_min,
        angle_increment=angle_inc,
        range_min=0.12,
        range_max=3.5,
    )


class TestTask8CorridorRecovery(unittest.TestCase):
    """Verify corridor stuck recovery, trail aiming, obs norm, and Lost detection."""

    def test_corridor_stuck_recovers_and_turns_toward_180_deg(self):
        """Verify robot stuck in 1 m corridor enters recovery, moves (v>0), aligns ~180 deg."""
        w13 = np.column_stack((np.arange(-1.0, 1.001, 0.05), np.full(41, -3.0)))
        w15 = np.column_stack((np.arange(-1.0, 0.001, 0.05), np.full(21, -4.0)))
        obs = np.vstack((w13, w15))

        dwa_cfg = DWAConfig(
            dt=1.0 / 15.0,
            planning_margin=0.10,
            w_samples=21,
            obs_norm_min_span=0.05,
        )
        scan_cfg = ScanConfig(num_ranges=24, laser_offset_x=0.0)
        nav_cfg = NavConfig(
            mode='follow',
            max_num_steps=0,
            follow_use_trail=True,
            stuck_time_sec=2.0,
            recovery_exit_dist=0.3,
            recovery_max_sec=5.0,
            sensor_timeout_sec=5.0,
            target_loss_timeout_sec=10.0,
        )
        mgr = NavigationManager(dwa_cfg, scan_cfg, nav_cfg)

        pose = np.array([0.05, -3.42, math.radians(165.0)], dtype=np.float64)
        vel = np.array([0.10, 0.0], dtype=np.float64)
        target = (-1.56, -2.89)
        dt = dwa_cfg.dt

        entered_recovery = False
        max_v_in_recovery = 0.0
        max_yaw_deg = math.degrees(pose[2])
        for step_idx in range(150):
            t_now = step_idx * dt
            mgr.update_odom(pose, vel, stamp_sec=t_now)
            scan = make_scan_from_points(pose, obs)
            mgr.update_scan(scan, stamp_sec=t_now, now_sec=t_now)
            mgr.update_dynamic_goal(target[0], target[1], 'world', stamp_sec=t_now)
            out = mgr.step(now_sec=t_now)
            self.assertIsNone(out.event, f'Unexpected event {out.event} at step {step_idx}')
            if mgr.stuck_recovery_active:
                entered_recovery = True
                max_v_in_recovery = max(max_v_in_recovery, out.cmd_vel[0])
            vel = np.array(out.cmd_vel, dtype=np.float64)
            pose = np.array([
                pose[0] + vel[0] * math.cos(pose[2]) * dt,
                pose[1] + vel[0] * math.sin(pose[2]) * dt,
                pose[2] + vel[1] * dt,
            ], dtype=np.float64)
            max_yaw_deg = max(max_yaw_deg, math.degrees(pose[2]))

        self.assertTrue(entered_recovery, 'Robot should enter stuck recovery within 2 s')
        self.assertGreater(max_v_in_recovery, 0.05)
        self.assertGreater(max_yaw_deg, 175.0)
        self.assertLess(pose[0], -0.50)

    def test_trail_add_dedup_pop_and_clear_on_jump(self):
        """Verify target trail deduplication (>=0.10 m), popping (<=0.4 m), and jump clear."""
        nav_cfg = NavConfig(
            mode='follow',
            follow_use_trail=True,
            trail_spacing=0.10,
            trail_reach_dist=0.40,
            target_jump_threshold=2.5,
        )
        mgr = NavigationManager(nav_config=nav_cfg)
        mgr.update_odom((0.0, 0.0, 0.0), (0.0, 0.0), stamp_sec=0.0)
        mgr.update_scan(make_open_scan(), stamp_sec=0.0)

        mgr.update_dynamic_goal(1.0, 0.0, 'world', stamp_sec=0.1)
        mgr.update_dynamic_goal(1.04, 0.0, 'world', stamp_sec=0.2)  # < 0.10 m -> deduped
        mgr.update_dynamic_goal(1.50, 0.0, 'world', stamp_sec=0.3)  # >= 0.10 m -> added
        self.assertEqual(len(mgr.target_trail), 2)

        # Robot moves within 0.40 m of first trail point (1.0, 0.0) -> pops only first
        mgr.update_odom((0.8, 0.0, 0.0), (0.1, 0.0), stamp_sec=0.4)
        mgr.step(now_sec=0.4)
        self.assertEqual(len(mgr.target_trail), 1)
        np.testing.assert_allclose(mgr.target_trail[0], [1.50, 0.0])

        # Target jump > 2.5 m clears old trail and starts fresh
        mgr.update_dynamic_goal(6.0, 0.0, 'world', stamp_sec=0.5)
        self.assertEqual(len(mgr.target_trail), 1)
        np.testing.assert_allclose(mgr.target_trail[0], [6.0, 0.0])

    def test_recovery_never_relaxes_safety_and_exits_on_timeout(self):
        """Verify recovery only picks safe trajectories and exits after recovery_max_sec."""
        dwa_cfg = DWAConfig(
            follow_mode=True,
            robot_radius=0.105,
            collision_tol=0.18,
            planning_margin=0.10,
        )
        dwa = DWAController(dwa_cfg)
        # All trajectories collide (obstacle at 0.20 m inside hard stop) -> must stop
        dec = dwa.compute_cmd_detailed(
            [2.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0],
            [[0.20, 0.0]],
            use_progress_recovery=True,
        )
        self.assertTrue(dec.all_colliding)
        np.testing.assert_allclose(dec.cmd_vel, [0.0, 0.0])

        # Timeout exit after recovery_max_sec=1.0 s when robot stays stationary
        nav_cfg = NavConfig(
            mode='follow',
            max_num_steps=0,
            stuck_time_sec=0.5,
            recovery_max_sec=1.0,
            sensor_timeout_sec=10.0,
            target_loss_timeout_sec=10.0,
        )
        mgr = NavigationManager(dwa_cfg, ScanConfig(laser_offset_x=0.0), nav_cfg)
        mgr.stuck_recovery_active = True
        mgr.stuck_recovery_start_time = 1.0
        mgr.stuck_recovery_start_pose = np.array([0.0, 0.0], dtype=np.float64)
        mgr.update_odom((0.0, 0.0, 0.0), (0.0, 0.0), stamp_sec=2.1)
        mgr.update_scan(make_open_scan(), stamp_sec=2.1)
        mgr.update_dynamic_goal(2.0, 0.0, 'world', stamp_sec=2.1)
        out = mgr.step(now_sec=2.1)
        self.assertFalse(mgr.stuck_recovery_active)
        self.assertIsNotNone(out.info)

    def test_obs_norm_min_span_prevents_noise_and_preserves_goal_open_space(self):
        """Verify small clearance span is not stretched and goal mode open space is unchanged."""
        cfg = DWAConfig(
            follow_mode=False,
            obstacle_max_dist=1.0,
            obs_norm_min_span=0.05,
            weight_heading=0.4,
            weight_vel=0.3,
            weight_obs=0.3,
        )
        dwa = DWAController(cfg)
        # Small span (0.400 to 0.404 m, span=0.004 < 0.05): must NOT stretch to [0, 1]
        s_small = dwa.normalize_obstacle_scores(np.array([0.400, 0.402, 0.404]))
        np.testing.assert_allclose(s_small, [0.400, 0.402, 0.404], atol=1e-9)

        # Large span (0.40 to 0.80 m, span=0.40 >= 0.05): normal min-max to [0, 1]
        s_large = dwa.normalize_obstacle_scores(np.array([0.40, 0.60, 0.80]))
        np.testing.assert_allclose(s_large, [0.0, 0.5, 1.0], atol=1e-9)

        # Goal mode in open space: drives straight forward unchanged
        cmd = dwa.compute_cmd([3.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0], [])
        self.assertGreater(float(cmd[0]), 0.0)
        self.assertAlmostEqual(float(cmd[1]), 0.0, places=9)

    def test_lost_after_10s_beyond_3m_and_auto_resumes_within_3m(self):
        """Verify Lost event fires after 10 s > 3 m without locking, and resumes <= 3 m."""
        nav_cfg = NavConfig(
            mode='follow',
            max_num_steps=0,
            lost_dist=3.0,
            lost_dist_time_sec=10.0,
            sensor_timeout_sec=30.0,
            target_loss_timeout_sec=30.0,
        )
        mgr = NavigationManager(nav_config=nav_cfg)
        mgr.update_odom((0.0, 0.0, 0.0), (0.1, 0.0), stamp_sec=0.0)
        mgr.update_scan(make_open_scan(), stamp_sec=0.0)
        mgr.update_dynamic_goal(4.5, 0.0, 'world', stamp_sec=0.0)

        # At t=0.0 and t=9.5 s -> still pursuing (not Lost yet)
        out0 = mgr.step(now_sec=0.0)
        self.assertIsNone(out0.event)
        out9 = mgr.step(now_sec=9.5)
        self.assertIsNone(out9.event)

        # At t=10.1 s (> 10.0 s beyond 3.0 m) -> publishes Lost once, keeps chasing, NOT locked
        out10 = mgr.step(now_sec=10.1)
        self.assertEqual(out10.event, RESULT_LOST)
        self.assertGreater(out10.cmd_vel[0], 0.0)
        self.assertTrue(mgr.active)
        self.assertFalse(mgr.collision_locked)

        # Target returns within 3.0 m -> re-arms Lost and continues following
        mgr.update_dynamic_goal(2.0, 0.0, 'world', stamp_sec=11.0)
        out_resume = mgr.step(now_sec=11.0)
        self.assertIsNone(out_resume.event)
        self.assertGreater(out_resume.cmd_vel[0], 0.0)

    def test_yaml_configs_contain_all_task8_parameters(self):
        """Verify sim.yaml and real_robot.yaml define all new Task 8 parameters."""
        required = (
            'follow_use_trail',
            'trail_spacing',
            'trail_reach_dist',
            'stuck_time_sec',
            'recovery_exit_dist',
            'recovery_max_sec',
            'obs_norm_min_span',
            'lost_dist',
            'lost_dist_time_sec',
        )
        for fname in ('sim.yaml', 'real_robot.yaml'):
            with open(os.path.join(PKG_ROOT, 'config', fname), 'r', encoding='utf-8') as fh:
                params = yaml.safe_load(fh)['dwa_node']['ros__parameters']
            for key in required:
                self.assertIn(key, params, f'{key} missing in {fname}')


if __name__ == '__main__':
    unittest.main()
