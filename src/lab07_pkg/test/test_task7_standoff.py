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


"""Regression tests for Task 7 landmark_goal_standoff (TDD: fail first)."""

import math
import os
import unittest

import numpy as np
import yaml

from lab07_pkg.dwa import DWAConfig
from lab07_pkg.navigation import (
    LandmarkObservation,
    NavConfig,
    NavigationManager,
)


PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))


class TestTask7LandmarkStandoff(unittest.TestCase):
    """Verify landmark_goal_standoff backs off static tag goals without affecting follow."""

    def test_robot_at_origin_tag_ahead_2m_backs_off_to_1_5m(self):
        """Verify robot at (0, 0, 0) with tag 2 m ahead locks goal at (1.5, 0)."""
        nav_cfg = NavConfig(
            mode='goal',
            goal_source='landmark',
            target_id=5,
            landmark_goal_standoff=0.5,
        )
        mgr = NavigationManager(nav_config=nav_cfg)
        mgr.update_odom((0.0, 0.0, 0.0), (0.0, 0.0), stamp_sec=1.0)
        ok, info = mgr.update_landmarks([LandmarkObservation(5, 2.0, 0.0)], stamp_sec=1.0)
        self.assertTrue(ok)
        np.testing.assert_allclose(mgr.goal_odom, [1.5, 0.0], atol=1e-6)
        self.assertIn('2.00', info)
        self.assertIn('1.50', info)

    def test_robot_facing_90deg_tag_ahead_1m_backs_off_to_0_5m(self):
        """Verify robot facing +y with tag 1 m ahead locks goal at (0, 0.5)."""
        nav_cfg = NavConfig(
            mode='goal',
            goal_source='landmark',
            target_id=5,
            landmark_goal_standoff=0.5,
        )
        mgr = NavigationManager(nav_config=nav_cfg)
        mgr.update_odom((0.0, 0.0, math.pi / 2.0), (0.0, 0.0), stamp_sec=1.0)
        ok, _ = mgr.update_landmarks([LandmarkObservation(5, 1.0, 0.0)], stamp_sec=1.0)
        self.assertTrue(ok)
        np.testing.assert_allclose(mgr.goal_odom, [0.0, 0.5], atol=1e-6)

    def test_tag_closer_than_standoff_sets_goal_to_robot_position(self):
        """Verify tag at 0.3 m < 0.5 m standoff locks goal at robot position."""
        nav_cfg = NavConfig(
            mode='goal',
            goal_source='landmark',
            target_id=5,
            landmark_goal_standoff=0.5,
        )
        mgr = NavigationManager(nav_config=nav_cfg)
        mgr.update_odom((1.2, -0.4, 0.3), (0.0, 0.0), stamp_sec=1.0)
        ok, _ = mgr.update_landmarks([LandmarkObservation(5, 0.3, 0.0)], stamp_sec=1.0)
        self.assertTrue(ok)
        np.testing.assert_allclose(mgr.goal_odom, [1.2, -0.4], atol=1e-6)

    def test_follow_mode_unaffected_by_landmark_goal_standoff(self):
        """Verify follow mode keeps the raw tag position and uses desired_target_dist."""
        nav_cfg = NavConfig(
            mode='follow',
            target_id=5,
            landmark_goal_standoff=0.5,
        )
        mgr = NavigationManager(nav_config=nav_cfg)
        mgr.update_odom((0.0, 0.0, 0.0), (0.0, 0.0), stamp_sec=1.0)
        ok, _ = mgr.update_landmarks([LandmarkObservation(5, 2.0, 0.0)], stamp_sec=1.0)
        self.assertTrue(ok)
        np.testing.assert_allclose(mgr.goal_odom, [2.0, 0.0], atol=1e-6)

    def test_standoff_validation_warning_and_yaml_configs(self):
        """Verify negative standoff raises, small standoff warns, and YAMLs set 0.5 m."""
        with self.assertRaises(ValueError):
            NavConfig(landmark_goal_standoff=-0.1)

        # Small standoff (<= robot_radius + collision_tol) warns but does not raise
        dwa_cfg = DWAConfig(robot_radius=0.105, collision_tol=0.20)
        nav_cfg = NavConfig(
            mode='goal',
            goal_source='landmark',
            target_id=5,
            landmark_goal_standoff=0.20,
        )
        mgr = NavigationManager(dwa_config=dwa_cfg, nav_config=nav_cfg)
        self.assertIsNotNone(mgr.standoff_warning)

        for fname in ('sim.yaml', 'real_robot.yaml'):
            with open(os.path.join(PKG_ROOT, 'config', fname), 'r', encoding='utf-8') as fh:
                params = yaml.safe_load(fh)['dwa_node']['ros__parameters']
            self.assertAlmostEqual(float(params['landmark_goal_standoff']), 0.5, places=6)


if __name__ == '__main__':
    unittest.main()
