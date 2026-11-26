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


"""Regression tests for fourth review items 1-4 (TDD: fail first)."""

import ast
import glob
import math
import os
import subprocess
import unittest

import numpy as np
import yaml

from lab07_pkg.dwa import DWAConfig
from lab07_pkg.navigation import (
    LandmarkObservation,
    NavConfig,
    NavigationManager,
    RESULT_GOAL,
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


class TestReview4Item1LintRules(unittest.TestCase):
    """Verify F401 unused imports and D213/D406/D407/D413 docstring rules."""

    def test_no_unused_imports_or_d213_or_google_sections(self):
        """Verify all package Python files have zero F401, D213, and section errors."""
        files = sorted(glob.glob(os.path.join(PKG_ROOT, '**', '*.py'), recursive=True))
        problems = []
        for path in files:
            with open(path, 'r', encoding='utf-8') as fh:
                src = fh.read()
            lines = src.splitlines()
            tree = ast.parse(src, path)

            # Check F401
            imported = []
            used = set()
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    for alias in node.names:
                        imported.append((node.lineno, alias.asname or alias.name.split('.')[0]))
                elif isinstance(node, ast.ImportFrom):
                    for alias in node.names:
                        imported.append((node.lineno, alias.asname or alias.name))
                elif isinstance(node, ast.Name):
                    used.add(node.id)
            for ln, name in imported:
                if name not in used:
                    problems.append(f'{path}:{ln} F401 unused {name}')

            # Check D213 & Args:/Returns: headers
            for node in ast.walk(tree):
                if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef)):
                    if (
                        node.body
                        and isinstance(node.body[0], ast.Expr)
                        and isinstance(node.body[0].value, ast.Constant)
                        and isinstance(node.body[0].value.value, str)
                    ):
                        expr = node.body[0]
                        doc = expr.value.value
                        if '\n' in doc:
                            after = lines[expr.lineno - 1].split('"""', 1)[1].strip()
                            if after:
                                problems.append(f'{path}:{expr.lineno} D213')
                            for bad in ('Args:', 'Returns:', 'Rules:'):
                                if bad in doc:
                                    problems.append(f'{path}:{expr.lineno} {bad}')
        self.assertEqual(problems, [], '\n'.join(problems))


class TestReview4Item2TrialScript(unittest.TestCase):
    """Verify run_goal_trials.sh uses setsid, process-group kill, and Python runner."""

    def test_trial_script_uses_setsid_and_python_runner_with_sim_time(self):
        """Verify run_goal_trials.sh and run_single_trial.py satisfy VM requirements."""
        sh_path = os.path.join(PKG_ROOT, 'scripts', 'run_goal_trials.sh')
        py_path = os.path.join(PKG_ROOT, 'scripts', 'run_single_trial.py')
        self.assertTrue(os.path.isfile(py_path), 'run_single_trial.py is missing')

        proc = subprocess.run(['bash', '-n', sh_path], capture_output=True, text=True)
        self.assertEqual(proc.returncode, 0, proc.stderr)

        with open(sh_path, 'r', encoding='utf-8') as fh:
            sh_text = fh.read()
        self.assertIn('setsid', sh_text)
        self.assertIn('run_single_trial.py', sh_text)
        self.assertIn('sim_sec', sh_text)

        with open(py_path, 'r', encoding='utf-8') as fh:
            py_text = fh.read()
        ast.parse(py_text, py_path)
        self.assertIn('get_subscription_count', py_text)
        self.assertIn('/nav_result', py_text)


class TestReview4Item3RealWindowSource(unittest.TestCase):
    """Verify real_robot.yaml defaults to last_cmd and ramps to max_lin_vel."""

    def test_real_config_uses_last_cmd_and_sim_uses_odom(self):
        """Verify window_source is last_cmd in real_robot.yaml and odom in sim.yaml."""
        with open(os.path.join(PKG_ROOT, 'config', 'real_robot.yaml'), 'r', encoding='utf-8') as f:
            real_p = yaml.safe_load(f)['dwa_node']['ros__parameters']
        with open(os.path.join(PKG_ROOT, 'config', 'sim.yaml'), 'r', encoding='utf-8') as f:
            sim_p = yaml.safe_load(f)['dwa_node']['ros__parameters']
        self.assertEqual(real_p['window_source'], 'last_cmd')
        self.assertEqual(sim_p['window_source'], 'odom')

    def test_last_cmd_ramps_to_max_lin_vel_when_odom_stays_zero(self):
        """Verify commanded v climbs steadily to max_lin_vel when odom v stays 0."""
        dwa_cfg = DWAConfig(dt=1.0 / 15.0, max_lin_acc=0.4, max_lin_vel=0.15)
        nav_cfg = NavConfig(mode='goal', window_source='last_cmd')
        mgr = NavigationManager(dwa_config=dwa_cfg, nav_config=nav_cfg)
        mgr.set_goal(5.0, 0.0, 'odom')
        mgr.update_odom((0.0, 0.0, 0.0), (0.0, 0.0), stamp_sec=1.0)
        mgr.update_scan(make_open_scan(), stamp_sec=1.0)

        cmds = []
        for i in range(10):
            t = 1.0 + i * (1.0 / 15.0)
            mgr.update_odom((0.0, 0.0, 0.0), (0.0, 0.0), stamp_sec=t)
            out = mgr.step(now_sec=t)
            cmds.append(out.cmd_vel[0])

        self.assertGreater(cmds[1], cmds[0])
        self.assertAlmostEqual(cmds[-1], 0.15, places=6)


class TestReview4Item4LandmarkStaticGoal(unittest.TestCase):
    """Verify goal_source='landmark' locks one-shot static goal from AprilTag."""

    def test_landmark_goal_locks_once_and_ignores_subsequent_updates(self):
        """Verify first target_id detection locks static goal and ignores later moves."""
        nav_cfg = NavConfig(
            mode='goal',
            goal_source='landmark',
            target_id=7,
            require_target_id=True,
            goal_dist_tol=0.25,
        )
        mgr = NavigationManager(nav_config=nav_cfg)
        mgr.update_odom((0.0, 0.0, 0.0), (0.0, 0.0), stamp_sec=1.0)
        mgr.update_scan(make_open_scan(), stamp_sec=1.0)

        # Wrong tag ID ignored
        ok_wrong, _ = mgr.update_landmarks([LandmarkObservation(3, 1.5, 0.0)], stamp_sec=1.0)
        self.assertFalse(ok_wrong)
        self.assertFalse(mgr.active)

        # Target tag ID=7 at 2.0 m locks static goal at (1.5, 0.0) with default 0.5 m standoff
        ok_lock, _ = mgr.update_landmarks([LandmarkObservation(7, 2.0, 0.0)], stamp_sec=1.0)
        self.assertTrue(ok_lock)
        self.assertTrue(mgr.active)
        np.testing.assert_allclose(mgr.goal_odom, [1.5, 0.0], atol=1e-9)

        # Subsequent tag movement MUST NOT overwrite the locked static goal
        ok_again, _ = mgr.update_landmarks([LandmarkObservation(7, 4.0, 0.5)], stamp_sec=1.2)
        self.assertFalse(ok_again)
        np.testing.assert_allclose(mgr.goal_odom, [1.5, 0.0], atol=1e-9)

        # Reaching the locked position emits Goal
        mgr.update_odom((1.4, 0.0, 0.0), (0.1, 0.0), stamp_sec=2.0)
        mgr.update_scan(make_open_scan(), stamp_sec=2.0)
        out = mgr.step(now_sec=2.0)
        self.assertEqual(out.event, RESULT_GOAL)

    def test_goal_source_landmark_requires_target_id_when_enforced(self):
        """Verify goal mode with goal_source='landmark' rejects unset target_id."""
        with self.assertRaises(ValueError):
            NavigationManager(
                nav_config=NavConfig(
                    mode='goal',
                    goal_source='landmark',
                    require_target_id=True,
                    target_id=-1,
                )
            )


if __name__ == '__main__':
    unittest.main()
