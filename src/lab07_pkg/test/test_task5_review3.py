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


"""Regression tests for third review (Task 5 items 1-6, TDD: fail first)."""

import os
import subprocess
import unittest

import yaml


PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))


class TestTask5Review3(unittest.TestCase):
    """Verify Task 5 configuration, trial script, and planning_margin updates."""

    def test_sim_and_real_configs_use_planning_margin_0_10(self):
        """Verify both sim.yaml and real_robot.yaml set planning_margin to 0.10 m."""
        for fname in ('sim.yaml', 'real_robot.yaml'):
            path = os.path.join(PKG_ROOT, 'config', fname)
            with open(path, 'r', encoding='utf-8') as fh:
                params = yaml.safe_load(fh)['dwa_node']['ros__parameters']
            self.assertAlmostEqual(float(params['planning_margin']), 0.10, places=6)

    def test_run_goal_trials_script_exists_valid_and_has_8_goals(self):
        """Verify scripts/run_goal_trials.sh exists, passes bash -n, and lists 8 goals."""
        script = os.path.join(PKG_ROOT, 'scripts', 'run_goal_trials.sh')
        self.assertTrue(os.path.isfile(script), 'run_goal_trials.sh is missing')
        self.assertTrue(os.access(script, os.X_OK), 'run_goal_trials.sh must be executable')

        proc = subprocess.run(['bash', '-n', script], capture_output=True, text=True)
        self.assertEqual(proc.returncode, 0, proc.stderr)

        with open(script, 'r', encoding='utf-8') as fh:
            content = fh.read()
        self.assertIn('TRIALS=${TRIALS:-3}', content)
        for coord in (
            '1.5 -0.5',
            '2.0 1.0',
            '2.0 -1.0',
            '2.2 -3.4',
            '3.5 -3.0',
            '-3.45 0.8',
            '-4.0 0.0',
            '-3.5 -0.5',
        ):
            self.assertIn(coord, content)


if __name__ == '__main__':
    unittest.main()
