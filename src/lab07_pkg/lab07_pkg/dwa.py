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


"""
Dynamic Window Approach (DWA) local planner for Task 1.

Adapted from SESASR-Course/planning_control_methods (GPL-3.0) with bug fixes:
- Separates control period dt from rollout step time_granularity.
- Computes dynamic window from current /odom velocity.
- Uses raw linear velocity for Task 1 (J = alpha*heading + beta*vel + gamma*dist_obst).
- Filters colliding trajectories BEFORE score normalization; stops if all collide.
- Uses consistent robot-center-to-obstacle threshold (robot_radius + collision_tol).

Pure Python + NumPy (no rclpy dependency).
"""

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np


@dataclass
class DWAConfig:
    """Configuration parameters for the DWA controller."""

    dt: float = 1.0 / 15.0         # [s] control period (15 Hz)
    sim_time: float = 1.5          # [s] trajectory rollout horizon
    time_granularity: float = 0.1  # [s] rollout integration step
    v_samples: int = 10
    w_samples: int = 21            # odd count so w=0 is naturally included

    max_lin_vel: float = 0.22      # [m/s]
    min_lin_vel: float = 0.0       # [m/s]
    max_ang_vel: float = 1.5       # [rad/s]
    min_ang_vel: float = -1.5      # [rad/s]
    max_lin_acc: float = 0.5       # [m/s^2]
    max_ang_acc: float = 3.2       # [rad/s^2]

    robot_radius: float = 0.105    # [m] circular footprint radius
    collision_tol: float = 0.18    # [m] minimum surface clearance for hard stop
    planning_margin: float = 0.10  # [m] extra buffer for DWA planning over hard stop
    band_floor_buffer: float = 0.03  # [m] margin-band lower floor above hard stop
    enable_recovery_spin: bool = False  # optional in-place spin when all paths collide
    recovery_ang_vel: float = 0.4  # [rad/s] recovery spin rate when enabled

    weight_heading: float = 0.40   # alpha
    weight_vel: float = 0.30       # beta
    weight_obs: float = 0.30       # gamma
    weight_target: float = 0.35    # delta (Task 2 follow mode only)
    obstacle_max_dist: float = 1.0  # [m] saturation cap for obstacle score
    obs_norm_min_span: float = 0.05  # [m] skip min-max stretch when clearance span < this

    # Task 2 (dynamic goal following) parameters
    follow_mode: bool = False
    slowdown_dist_threshold: float = 1.2  # [m] start vel' slowdown below this distance
    desired_target_dist: float = 0.8      # [m] optimal following distance
    target_dist_sigma: float = 0.5        # [m] Gaussian width for distance keeping
    camera_fov_half_rad: float = 0.576    # [rad] ~33 deg half-FoV measured on real camera

    def __post_init__(self) -> None:
        if self.dt <= 0.0:
            raise ValueError(f'dt must be positive, got {self.dt}')
        if self.time_granularity <= 0.0 or self.sim_time <= 0.0:
            raise ValueError('sim_time and time_granularity must be positive')
        if self.v_samples < 1 or self.w_samples < 1:
            raise ValueError('v_samples and w_samples must be >= 1')
        if self.slowdown_dist_threshold <= 0.0:
            raise ValueError('slowdown_dist_threshold must be positive')
        if self.desired_target_dist <= 0.0:
            raise ValueError('desired_target_dist must be positive')
        if self.obs_norm_min_span < 0.0:
            raise ValueError(f'obs_norm_min_span must be >= 0, got {self.obs_norm_min_span}')
        if not (0.0 <= self.band_floor_buffer < self.planning_margin):
            raise ValueError(
                f'band_floor_buffer ({self.band_floor_buffer}) must satisfy '
                f'0 <= band_floor_buffer < planning_margin ({self.planning_margin})'
            )


@dataclass
class DWADecision:
    """Detailed output of one DWA planning step."""

    cmd_vel: np.ndarray            # shape (2,): [v, w]
    all_colliding: bool
    num_candidates: int
    num_safe: int
    best_score: float
    best_path: Optional[np.ndarray]


def normalize_angle(theta):
    """Wrap angle(s) into [-pi, pi)."""
    wrapped = (theta + np.pi) % (2.0 * np.pi) - np.pi
    return wrapped


def normalize_scores(arr: np.ndarray) -> np.ndarray:
    """
    Min-max normalize a 1D score array into [0, 1].

    Returns zeros when all entries are equal (within floating-point tolerance).
    """
    arr = np.asarray(arr, dtype=np.float64)
    if arr.size == 0:
        return arr
    min_val = float(np.min(arr))
    max_val = float(np.max(arr))
    if np.isclose(max_val - min_val, 0.0):
        return np.zeros_like(arr)
    return (arr - min_val) / (max_val - min_val)


class DWAController:
    """Task 1 DWA controller with original objective J = a*heading + b*vel + g*dist_obst."""

    def __init__(self, config: Optional[DWAConfig] = None) -> None:
        """Initialize the DWA controller with optional configuration."""
        self.cfg = config if config is not None else DWAConfig()
        self.sim_steps = max(1, int(round(self.cfg.sim_time / self.cfg.time_granularity)))
        self.band_entry_min: Optional[float] = None

    def reset_margin_band(self) -> None:
        """Clear the recorded margin-band entry distance."""
        self.band_entry_min = None

    @property
    def collision_radius(self) -> float:
        """Hard-stop center-to-obstacle threshold: robot_radius + collision_tol."""
        return self.cfg.robot_radius + self.cfg.collision_tol

    @property
    def band_floor_radius(self) -> float:
        """Margin-band lower floor: collision_radius + band_floor_buffer."""
        return self.collision_radius + self.cfg.band_floor_buffer

    @property
    def planning_collision_radius(self) -> float:
        """DWA planning threshold: robot_radius + collision_tol + planning_margin."""
        return self.cfg.robot_radius + self.cfg.collision_tol + self.cfg.planning_margin

    def compute_dynamic_window(
        self, current_vel: Sequence[float]
    ) -> Tuple[float, float, float, float]:
        """Compute reachable (v_min, v_max, w_min, w_max) over one control step dt."""
        v_cur = float(current_vel[0])
        w_cur = float(current_vel[1])
        dt = self.cfg.dt

        v_min = max(self.cfg.min_lin_vel, v_cur - self.cfg.max_lin_acc * dt)
        v_max = min(self.cfg.max_lin_vel, v_cur + self.cfg.max_lin_acc * dt)
        if v_min > v_max:
            clamped = float(np.clip(v_cur, self.cfg.min_lin_vel, self.cfg.max_lin_vel))
            v_min = v_max = clamped

        w_min = max(self.cfg.min_ang_vel, w_cur - self.cfg.max_ang_acc * dt)
        w_max = min(self.cfg.max_ang_vel, w_cur + self.cfg.max_ang_acc * dt)
        if w_min > w_max:
            clamped = float(np.clip(w_cur, self.cfg.min_ang_vel, self.cfg.max_ang_vel))
            w_min = w_max = clamped

        return v_min, v_max, w_min, w_max

    def simulate_paths(
        self, robot_pose: Sequence[float], velocities: np.ndarray
    ) -> np.ndarray:
        """
        Roll out differential-drive trajectories using time_granularity.

        Takes robot_pose [x, y, yaw] and velocities of shape (n_paths, 2), and
        returns simulated paths of shape (n_paths, sim_steps + 1, 3).
        """
        pose = np.asarray(robot_pose, dtype=np.float64)
        n_paths = velocities.shape[0]
        paths = np.empty((n_paths, self.sim_steps + 1, 3), dtype=np.float64)
        paths[:, 0, :] = pose

        step_dt = self.cfg.time_granularity
        v = velocities[:, 0]
        w = velocities[:, 1]

        for i in range(1, self.sim_steps + 1):
            prev_th = paths[:, i - 1, 2]
            paths[:, i, 0] = paths[:, i - 1, 0] + v * np.cos(prev_th) * step_dt
            paths[:, i, 1] = paths[:, i - 1, 1] + v * np.sin(prev_th) * step_dt
            paths[:, i, 2] = prev_th + w * step_dt

        return paths

    def get_trajectories(
        self, robot_pose: Sequence[float], current_vel: Sequence[float]
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Sample velocities inside the dynamic window and simulate their trajectories."""
        v_min, v_max, w_min, w_max = self.compute_dynamic_window(current_vel)
        v_values = np.linspace(v_min, v_max, self.cfg.v_samples)
        w_values = np.linspace(w_min, w_max, self.cfg.w_samples)
        # Ensure exact w=0.0 is sampled whenever 0 lies inside the angular window.
        if w_min <= 0.0 <= w_max and not np.any(np.isclose(w_values, 0.0)):
            nearest_zero = int(np.argmin(np.abs(w_values)))
            w_values[nearest_zero] = 0.0

        vv, ww = np.meshgrid(v_values, w_values, indexing='xy')
        velocities = np.column_stack((vv.ravel(), ww.ravel()))
        paths = self.simulate_paths(robot_pose, velocities)
        return paths, velocities

    def score_heading_angle(
        self, paths: np.ndarray, goal_pose: np.ndarray
    ) -> np.ndarray:
        """Heading score in [0, pi]: pi - |angle_to_goal - terminal_yaw|."""
        last_x = paths[:, -1, 0]
        last_y = paths[:, -1, 1]
        last_th = paths[:, -1, 2]

        angle_to_goal = np.arctan2(goal_pose[1] - last_y, goal_pose[0] - last_x)
        err = np.abs(normalize_angle(angle_to_goal - last_th))
        return np.pi - err

    def score_vel(self, velocities: np.ndarray) -> np.ndarray:
        """Task 1 velocity score: raw forward linear velocity."""
        return velocities[:, 0].copy()

    def score_vel_prime(
        self, velocities: np.ndarray, paths: np.ndarray, goal_pose: np.ndarray
    ) -> np.ndarray:
        """
        Compute Task 2 decreasing velocity score vel' near target.

        For terminal distance d >= slowdown_dist_threshold, v_ref = max_lin_vel
        so score equals raw linear velocity v (continuous transition).
        For d < slowdown_dist_threshold, v_ref scales linearly with d / threshold
        and scores peak at v == v_ref.
        """
        v = velocities[:, 0]
        dist_to_goal = np.hypot(paths[:, -1, 0] - goal_pose[0], paths[:, -1, 1] - goal_pose[1])
        ratio = np.clip(dist_to_goal / self.cfg.slowdown_dist_threshold, 0.0, 1.0)
        v_ref = self.cfg.max_lin_vel * ratio
        return self.cfg.max_lin_vel - np.abs(v - v_ref)

    def score_target_following(
        self,
        paths: np.ndarray,
        goal_pose: np.ndarray,
        distance_only: bool = False,
    ) -> np.ndarray:
        """Task 2 dist_target score: keep target at desired distance and centered in FoV."""
        last_x = paths[:, -1, 0]
        last_y = paths[:, -1, 1]
        last_th = paths[:, -1, 2]

        dist_to_goal = np.hypot(goal_pose[0] - last_x, goal_pose[1] - last_y)
        dist_err = dist_to_goal - self.cfg.desired_target_dist
        s_dist = np.exp(-0.5 * (dist_err / self.cfg.target_dist_sigma) ** 2)
        if distance_only:
            return s_dist

        angle_to_goal = np.arctan2(goal_pose[1] - last_y, goal_pose[0] - last_x)
        bearing_err = np.abs(normalize_angle(angle_to_goal - last_th))
        sigma_bearing = max(self.cfg.camera_fov_half_rad * 0.5, 1e-3)
        s_vis = np.exp(-0.5 * (bearing_err / sigma_bearing) ** 2)
        return 0.5 * s_dist + 0.5 * s_vis

    def score_progress(
        self, paths: np.ndarray, robot_pose: np.ndarray, aim_goal: np.ndarray
    ) -> np.ndarray:
        """Compute distance reduction from current pose to trajectory end toward aim_goal."""
        start_dist = float(np.hypot(robot_pose[0] - aim_goal[0], robot_pose[1] - aim_goal[1]))
        end_dist = np.hypot(paths[:, -1, 0] - aim_goal[0], paths[:, -1, 1] - aim_goal[1])
        return start_dist - end_dist

    def normalize_obstacle_scores(self, safe_obs: np.ndarray) -> np.ndarray:
        """
        Normalize obstacle clearances without amplifying sub-centimeter noise.

        When the clearance span across safe trajectories is smaller than
        obs_norm_min_span, scales linearly by obstacle_max_dist instead of
        stretching tiny differences to [0, 1].
        """
        safe_obs = np.asarray(safe_obs, dtype=np.float64)
        if safe_obs.size == 0:
            return safe_obs
        span = float(np.max(safe_obs) - np.min(safe_obs))
        if span < self.cfg.obs_norm_min_span:
            return safe_obs / self.cfg.obstacle_max_dist
        return normalize_scores(safe_obs)

    def score_obstacles(
        self, paths: np.ndarray, obstacles: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute minimum clearance to obstacles and collision mask per trajectory.

        Uses center-to-obstacle distance < (robot_radius + collision_tol +
        planning_margin) across all rollout states so trajectories moving into
        the safety margin are rejected.
        """
        n_paths = paths.shape[0]
        if obstacles.size == 0:
            return (
                np.full(n_paths, self.cfg.obstacle_max_dist, dtype=np.float64),
                np.zeros(n_paths, dtype=bool),
            )

        # Broadcast: paths (N, T, 1, 2) - obstacles (1, 1, M, 2)
        dx = paths[:, :, np.newaxis, 0] - obstacles[np.newaxis, np.newaxis, :, 0]
        dy = paths[:, :, np.newaxis, 1] - obstacles[np.newaxis, np.newaxis, :, 1]
        dists = np.hypot(dx, dy)
        min_dists = np.min(dists, axis=(1, 2))

        colliding = min_dists < self.planning_collision_radius
        clearance = np.minimum(min_dists, self.cfg.obstacle_max_dist)
        return clearance, colliding

    def compute_cmd_detailed(
        self,
        goal_pose: Sequence[float],
        robot_pose: Sequence[float],
        current_vel: Sequence[float],
        obstacles: Optional[Sequence[Sequence[float]]] = None,
        allow_stop_zone_escape: bool = False,
        heading_goal: Optional[Sequence[float]] = None,
        use_progress_recovery: bool = False,
    ) -> DWADecision:
        """
        Compute optimal (v, w) command and return diagnostic details.

        When the robot is already inside the margin band [collision_radius,
        planning_collision_radius), trajectories whose future clearance does not
        decrease (including in-place rotation) are allowed so the robot can
        exit the band instead of deadlocking. When allow_stop_zone_escape is
        True (post-/reset_nav recovery), the same non-decreasing-clearance rule
        applies while strictly above physical contact. In follow mode, an
        optional heading_goal lets heading/progress aim at a trail point while
        vel' and dist_target still reference goal_pose.
        """
        goal = np.asarray(goal_pose, dtype=np.float64).reshape(2)
        aim = (
            goal
            if heading_goal is None
            else np.asarray(heading_goal, dtype=np.float64).reshape(2)
        )
        pose = np.asarray(robot_pose, dtype=np.float64).reshape(3)
        vel = np.asarray(current_vel, dtype=np.float64).reshape(2)

        if obstacles is None or len(obstacles) == 0:
            obs = np.empty((0, 2), dtype=np.float64)
            self.band_entry_min = None
            cur_min = float('inf')
        else:
            obs = np.asarray(obstacles, dtype=np.float64).reshape(-1, 2)
            cur_min = float(np.min(np.hypot(pose[0] - obs[:, 0], pose[1] - obs[:, 1])))
            if cur_min >= self.planning_collision_radius:
                self.band_entry_min = None
            elif self.band_floor_radius <= cur_min < self.planning_collision_radius:
                if self.band_entry_min is None:
                    self.band_entry_min = cur_min

        paths, velocities = self.get_trajectories(pose, vel)
        n_candidates = paths.shape[0]

        clearance, colliding = self.score_obstacles(paths, obs)
        safe_mask = ~colliding

        # Margin-band or post-reset escape: anchor at band_entry_min (never creep lower).
        active_floor = self.planning_collision_radius
        if not np.any(safe_mask) and obs.size > 0:
            in_margin_band = self.band_floor_radius <= cur_min < self.planning_collision_radius
            in_escape_band = (
                allow_stop_zone_escape
                and (self.cfg.robot_radius + 0.02) <= cur_min < self.band_floor_radius
            )
            if in_margin_band or in_escape_band:
                dx = paths[:, 1:, np.newaxis, 0] - obs[np.newaxis, np.newaxis, :, 0]
                dy = paths[:, 1:, np.newaxis, 1] - obs[np.newaxis, np.newaxis, :, 1]
                future_min = np.min(np.hypot(dx, dy), axis=(1, 2))
                if in_margin_band:
                    anchor = self.band_entry_min if self.band_entry_min is not None else cur_min
                    active_floor = max(anchor, self.band_floor_radius)
                    safe_mask = future_min >= (active_floor - 1e-6)
                    # If inertia/scan ripple dipped cur_min slightly below anchor (while still
                    # >= band_floor_radius), allow only strictly non-worsening outward moves
                    # (end_min > cur_min) or in-place turns (v == 0) so the robot does not freeze.
                    if not np.any(safe_mask):
                        end_min = np.min(
                            np.hypot(
                                paths[:, -1, np.newaxis, 0] - obs[np.newaxis, :, 0],
                                paths[:, -1, np.newaxis, 1] - obs[np.newaxis, :, 1],
                            ),
                            axis=1,
                        )
                        non_worsening = (
                            (future_min >= cur_min - 1e-6)
                            & (future_min >= self.band_floor_radius - 1e-6)
                        )
                        outward_or_spin = (end_min > cur_min + 1e-4) | (velocities[:, 0] <= 1e-6)
                        safe_mask = non_worsening & outward_or_spin
                else:
                    active_floor = cur_min
                    safe_mask = future_min >= (cur_min - 1e-6)
                clearance = np.minimum(future_min, self.cfg.obstacle_max_dist)

        n_safe = int(np.sum(safe_mask))

        # If every candidate trajectory collides, stop (or spin in place if recovery enabled).
        if n_safe == 0:
            recovery_w = self.cfg.recovery_ang_vel if self.cfg.enable_recovery_spin else 0.0
            return DWADecision(
                cmd_vel=np.array([0.0, recovery_w], dtype=np.float64),
                all_colliding=True,
                num_candidates=n_candidates,
                num_safe=0,
                best_score=-np.inf,
                best_path=None,
            )

        safe_paths = paths[safe_mask]
        safe_vels = velocities[safe_mask]
        safe_obs = clearance[safe_mask]

        # Normalize each objective term over safe trajectories only.
        s_obs = self.normalize_obstacle_scores(safe_obs)

        if self.cfg.follow_mode:
            s_vel = normalize_scores(self.score_vel_prime(safe_vels, safe_paths, goal))
            if use_progress_recovery:
                raw_prog = self.score_progress(safe_paths, pose, aim)
                # When all safe candidates are in-place turns (progress == 0), break
                # ties by probing 0.25 m ahead along each candidate's terminal yaw so
                # the robot turns toward the safe open direction rather than w_min.
                if float(np.max(raw_prog)) <= 1e-6 and obs.size > 0:
                    th = safe_paths[:, -1, 2]
                    px0 = pose[0] + 0.003 * np.cos(th)
                    py0 = pose[1] + 0.003 * np.sin(th)
                    px1 = pose[0] + 0.08 * np.cos(th)
                    py1 = pose[1] + 0.08 * np.sin(th)
                    px2 = pose[0] + 0.25 * np.cos(th)
                    py2 = pose[1] + 0.25 * np.sin(th)
                    clr0 = np.min(
                        np.hypot(px0[:, np.newaxis] - obs[np.newaxis, :, 0],
                                 py0[:, np.newaxis] - obs[np.newaxis, :, 1]),
                        axis=1,
                    )
                    clr1 = np.min(
                        np.hypot(px1[:, np.newaxis] - obs[np.newaxis, :, 0],
                                 py1[:, np.newaxis] - obs[np.newaxis, :, 1]),
                        axis=1,
                    )
                    clr2 = np.min(
                        np.hypot(px2[:, np.newaxis] - obs[np.newaxis, :, 0],
                                 py2[:, np.newaxis] - obs[np.newaxis, :, 1]),
                        axis=1,
                    )
                    p_clr = np.minimum(np.minimum(clr0, clr1), clr2)
                    p_prog = (
                        float(np.hypot(pose[0] - aim[0], pose[1] - aim[1]))
                        - np.hypot(px2 - aim[0], py2 - aim[1])
                    )
                    probe_floor = min(active_floor, self.planning_collision_radius)
                    p_ok = (
                        (clr0 >= probe_floor + 1e-4)
                        & (clr1 >= probe_floor + 1e-4)
                        & (clr2 >= self.planning_collision_radius)
                    ).astype(np.float64)
                    if float(np.max(p_ok)) > 0.0:
                        total_scores = (
                            p_ok
                            + 0.5 * normalize_scores(p_clr)
                            + 0.5 * normalize_scores(p_prog)
                        )
                    else:
                        # No heading in window is open yet: turn away from blocking wall first.
                        total_scores = normalize_scores(p_clr)
                else:
                    s_primary = normalize_scores(raw_prog)
                    s_target = normalize_scores(
                        self.score_target_following(safe_paths, goal, distance_only=True)
                    )
                    total_scores = (
                        self.cfg.weight_heading * s_primary
                        + self.cfg.weight_vel * s_vel
                        + self.cfg.weight_obs * s_obs
                        + self.cfg.weight_target * s_target
                    )
            else:
                s_primary = normalize_scores(self.score_heading_angle(safe_paths, aim))
                s_target = normalize_scores(self.score_target_following(safe_paths, goal))
                total_scores = (
                    self.cfg.weight_heading * s_primary
                    + self.cfg.weight_vel * s_vel
                    + self.cfg.weight_obs * s_obs
                    + self.cfg.weight_target * s_target
                )
        else:
            s_heading = normalize_scores(self.score_heading_angle(safe_paths, goal))
            s_vel = normalize_scores(self.score_vel(safe_vels))
            total_scores = (
                self.cfg.weight_heading * s_heading
                + self.cfg.weight_vel * s_vel
                + self.cfg.weight_obs * s_obs
            )
        best_idx = int(np.argmax(total_scores))

        return DWADecision(
            cmd_vel=safe_vels[best_idx].copy(),
            all_colliding=False,
            num_candidates=n_candidates,
            num_safe=n_safe,
            best_score=float(total_scores[best_idx]),
            best_path=safe_paths[best_idx],
        )

    def compute_cmd(
        self,
        goal_pose: Sequence[float],
        robot_pose: Sequence[float],
        current_vel: Sequence[float],
        obstacles: Optional[Sequence[Sequence[float]]] = None,
    ) -> np.ndarray:
        """Return optimal velocity command [v, w] as a 1D float64 array."""
        return self.compute_cmd_detailed(goal_pose, robot_pose, current_vel, obstacles).cmd_vel
