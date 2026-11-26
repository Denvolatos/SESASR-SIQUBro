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
Goal manager, frame conversion, safety supervision, and Task 1 / Task 2 state machine.

Pure Python + NumPy (no rclpy dependency).
"""

from collections import deque
from dataclasses import dataclass
import math
from typing import Deque, Optional, Sequence, Tuple

import numpy as np

from lab07_pkg.dwa import DWAConfig, DWAController
from lab07_pkg.scan_utils import (
    ScanConfig,
    check_safety_stop,
    check_scan_health,
    scan_to_obstacles,
)

RESULT_GOAL = 'Goal'
RESULT_COLLISION = 'Collision'
RESULT_TIMEOUT = 'Timeout'
RESULT_LOST = 'Lost'


@dataclass
class ScanData:
    """Lightweight container mirroring sensor_msgs/LaserScan fields."""

    ranges: Sequence[float]
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float


@dataclass
class LandmarkObservation:
    """Lightweight container mirroring landmark_msgs/Landmark fields."""

    landmark_id: int
    range_m: float
    bearing_rad: float


@dataclass
class NavConfig:
    """Navigation task parameters for Task 1 (goal) and Task 2 (follow)."""

    mode: str = 'goal'                        # 'goal' (Task 1) or 'follow' (Task 2)
    goal_source: str = 'topic'                # 'topic' (/goal_pose) or 'landmark' (one-shot tag)
    landmark_goal_standoff: float = 0.5       # [m] backoff from tag in goal/landmark mode
    goal_dist_tol: float = 0.25               # [m] static goal success radius
    max_num_steps: int = 1500                 # control steps before Timeout
    feedback_rate: int = 50                   # publish distance every N steps
    sensor_timeout_sec: float = 1.0           # [s] stop if odom/scan is older than this
    target_loss_timeout_sec: float = 2.0      # [s] hold last target briefly, then stop
    target_jump_threshold: float = 2.5        # [m] detect simulation loop reset jumps
    follow_use_trail: bool = True             # aim heading at earliest unreached trail point
    trail_spacing: float = 0.10               # [m] min spacing between stored trail points
    trail_reach_dist: float = 0.40            # [m] pop trail point when robot is within this
    stuck_time_sec: float = 2.0               # [s] low-v duration before stuck recovery
    stuck_vel_thresh: float = 0.03            # [m/s] velocity below this counts as stuck
    stuck_dist_margin: float = 0.5            # [m] stuck only if dist > desired + margin
    recovery_exit_dist: float = 0.3           # [m] travel distance to exit stuck recovery
    recovery_exit_margin: float = 0.2         # [m] exit recovery if dist <= desired + margin
    recovery_max_sec: float = 5.0             # [s] max duration of one stuck recovery episode
    lost_dist: float = 3.0                    # [m] distance threshold for Lost detection
    lost_dist_time_sec: float = 10.0          # [s] continuous time > lost_dist to report Lost
    target_id: int = -1                       # landmark id to follow (-1 = nearest)
    require_target_id: bool = False           # true on real robot to avoid fixed arena tags
    camera_offset_x: float = 0.0              # [m] camera x offset in base frame
    camera_offset_y: float = 0.0              # [m] camera y offset in base frame
    camera_offset_yaw: float = 0.0            # [rad] camera yaw offset in base frame
    odom_buffer_size: int = 60                # ~3 s at 20 Hz for landmark timestamp matching
    max_stamp_skew_sec: float = 0.5           # [s] warn & fall back if landmark/odom clocks skew
    window_source: str = 'odom'               # 'odom' (default) or 'last_cmd'
    odom_frame: str = 'odom'
    world_frame: str = 'world'
    odom_origin_in_world: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        if self.mode not in ('goal', 'follow'):
            raise ValueError(f"mode must be 'goal' or 'follow', got '{self.mode}'")
        if self.goal_source not in ('topic', 'landmark'):
            raise ValueError(
                f"goal_source must be 'topic' or 'landmark', got '{self.goal_source}'"
            )
        if self.landmark_goal_standoff < 0.0:
            raise ValueError(
                f'landmark_goal_standoff must be >= 0, got {self.landmark_goal_standoff}'
            )
        for name in (
            'trail_spacing',
            'trail_reach_dist',
            'stuck_time_sec',
            'recovery_exit_dist',
            'recovery_max_sec',
            'lost_dist',
            'lost_dist_time_sec',
        ):
            if getattr(self, name) <= 0.0:
                raise ValueError(f'{name} must be > 0, got {getattr(self, name)}')
        for name in ('stuck_vel_thresh', 'stuck_dist_margin', 'recovery_exit_margin'):
            if getattr(self, name) < 0.0:
                raise ValueError(f'{name} must be >= 0, got {getattr(self, name)}')
        if self.window_source not in ('odom', 'last_cmd'):
            raise ValueError(
                f"window_source must be 'odom' or 'last_cmd', got '{self.window_source}'"
            )


@dataclass
class StepOutput:
    """Output of a single navigation control tick."""

    cmd_vel: Tuple[float, float] = (0.0, 0.0)
    publish_cmd: bool = False
    event: Optional[str] = None
    feedback_distance: Optional[float] = None
    step_count: int = 0
    warning: Optional[str] = None
    info: Optional[str] = None
    all_colliding: bool = False
    target_lost: bool = False


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Compute planar yaw [rad] from quaternion without tf_transformations."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def world_to_odom(
    x_world: float,
    y_world: float,
    odom_origin_in_world: Sequence[float],
) -> Tuple[float, float]:
    """
    Transform a 2D point from world frame into odom frame.

    odom_origin_in_world = (ox, oy, oyaw) is the pose of the odom origin
    expressed in the world frame.
    """
    ox, oy, oyaw = (
        float(odom_origin_in_world[0]),
        float(odom_origin_in_world[1]),
        float(odom_origin_in_world[2]),
    )
    dx = float(x_world) - ox
    dy = float(y_world) - oy
    c = math.cos(oyaw)
    s = math.sin(oyaw)
    x_odom = dx * c + dy * s
    y_odom = -dx * s + dy * c
    return x_odom, y_odom


def landmark_to_odom(
    robot_pose: Sequence[float],
    range_m: float,
    bearing_rad: float,
    camera_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> Tuple[float, float]:
    """Convert a (range, bearing) landmark measurement in base/camera frame to odom (x, y)."""
    rx, ry, ryaw = float(robot_pose[0]), float(robot_pose[1]), float(robot_pose[2])
    cx, cy, cyaw = camera_offset
    theta_base = float(bearing_rad) + cyaw
    bx = cx + float(range_m) * math.cos(theta_base)
    by = cy + float(range_m) * math.sin(theta_base)

    c = math.cos(ryaw)
    s = math.sin(ryaw)
    ox = rx + bx * c - by * s
    oy = ry + bx * s + by * c
    return ox, oy


class NavigationManager:
    """Task 1 (goal) and Task 2 (follow) state machine."""

    def __init__(
        self,
        dwa_config: Optional[DWAConfig] = None,
        scan_config: Optional[ScanConfig] = None,
        nav_config: Optional[NavConfig] = None,
    ) -> None:
        """Initialize NavigationManager with DWA, scan, and navigation configs."""
        self.nav_cfg = nav_config if nav_config is not None else NavConfig()
        needs_tag_id = (
            self.nav_cfg.mode == 'follow'
            or (self.nav_cfg.mode == 'goal' and self.nav_cfg.goal_source == 'landmark')
        )
        if needs_tag_id and self.nav_cfg.require_target_id and self.nav_cfg.target_id < 0:
            raise ValueError(
                'target_id must be explicitly set (>= 0) when require_target_id is True'
            )
        cfg = dwa_config if dwa_config is not None else DWAConfig()
        # Synchronize DWA follow_mode with NavConfig.mode
        cfg.follow_mode = self.nav_cfg.mode == 'follow'
        self.dwa = DWAController(cfg)
        self.scan_cfg = scan_config if scan_config is not None else ScanConfig()
        self.standoff_warning: Optional[str] = None
        if (
            self.nav_cfg.mode == 'goal'
            and self.nav_cfg.goal_source == 'landmark'
            and self.nav_cfg.landmark_goal_standoff <= self.dwa.collision_radius
        ):
            self.standoff_warning = (
                f'landmark_goal_standoff ({self.nav_cfg.landmark_goal_standoff:.3f} m) '
                f'<= stop radius ({self.dwa.collision_radius:.3f} m); '
                'robot may hit safety stop before reaching goal'
            )

        self.goal_odom: Optional[np.ndarray] = None
        self.active: bool = False
        self.collision_locked: bool = False
        self.static_landmark_locked: bool = False
        self.escape_recovery_active: bool = False
        self.escape_start_dist: Optional[float] = None
        self.step_count: int = 0
        self.last_target_time: Optional[float] = None
        self.last_target_jump_m: float = 0.0
        self.target_trail: Deque[np.ndarray] = deque()
        self.stuck_since: Optional[float] = None
        self.stuck_recovery_active: bool = False
        self.stuck_recovery_start_time: Optional[float] = None
        self.stuck_recovery_start_pose: Optional[np.ndarray] = None
        self.far_since: Optional[float] = None
        self.lost_reported: bool = False

        self.robot_pose: Optional[np.ndarray] = None
        self.robot_vel: Optional[np.ndarray] = None
        self.last_cmd_vel: np.ndarray = np.array([0.0, 0.0], dtype=np.float64)
        self.last_odom_time: Optional[float] = None
        self.odom_history: Deque[Tuple[float, np.ndarray]] = deque(
            maxlen=max(1, self.nav_cfg.odom_buffer_size)
        )

        self.last_scan: Optional[ScanData] = None
        self.last_scan_stamp: Optional[float] = None
        self.last_scan_time: Optional[float] = None
        self.cached_obstacles_odom: Optional[np.ndarray] = None

    def _transform_goal_frame(
        self, x: float, y: float, frame_id: str
    ) -> Tuple[bool, Optional[Tuple[float, float]], str]:
        frame = (frame_id or '').strip().lstrip('/')
        odom_frame = self.nav_cfg.odom_frame.strip().lstrip('/')
        world_frame = self.nav_cfg.world_frame.strip().lstrip('/')

        if not math.isfinite(x) or not math.isfinite(y):
            return False, None, f'Non-finite goal coordinates: ({x}, {y})'

        if frame == odom_frame:
            return True, (float(x), float(y)), odom_frame
        if frame == world_frame:
            return True, world_to_odom(x, y, self.nav_cfg.odom_origin_in_world), world_frame
        return (
            False,
            None,
            f"Rejected goal with unsupported frame_id '{frame_id}' "
            f"(expected '{odom_frame}' or '{world_frame}')",
        )

    def reset_navigation(self) -> Tuple[bool, str]:
        """Clear collision lock and enter guarded escape recovery if in stop zone."""
        self.collision_locked = False
        self.static_landmark_locked = False
        self.escape_recovery_active = True
        self.escape_start_dist = None
        self.active = False
        self.goal_odom = None
        self.step_count = 0
        self.last_target_time = None
        self.last_target_jump_m = 0.0
        self.target_trail.clear()
        self.stuck_since = None
        self.stuck_recovery_active = False
        self.stuck_recovery_start_time = None
        self.stuck_recovery_start_pose = None
        self.far_since = None
        self.lost_reported = False
        self.dwa.reset_margin_band()
        self.last_cmd_vel = np.array([0.0, 0.0], dtype=np.float64)
        return True, 'Navigation state reset and collision lock cleared'

    def _record_trail_point(self, point: np.ndarray, jumped: bool) -> None:
        if self.nav_cfg.mode != 'follow':
            return
        if jumped:
            self.target_trail.clear()
        if (
            not self.target_trail
            or float(np.linalg.norm(point - self.target_trail[-1])) >= self.nav_cfg.trail_spacing
        ):
            self.target_trail.append(point.copy())

    def set_goal(self, x: float, y: float, frame_id: str) -> Tuple[bool, str]:
        """Set or replace the static navigation goal (Task 1) at any instant."""
        if self.collision_locked:
            return False, 'Navigation locked after Collision; call /reset_nav to recover'
        ok, coords, info = self._transform_goal_frame(x, y, frame_id)
        if not ok or coords is None:
            return False, info

        self.goal_odom = np.array(coords, dtype=np.float64)
        self.active = True
        self.step_count = 0
        return True, (
            f'Accepted goal ({coords[0]:.3f}, {coords[1]:.3f}) '
            f"in '{self.nav_cfg.odom_frame}' (from '{info}')"
        )

    def update_dynamic_goal(
        self,
        x: float,
        y: float,
        frame_id: str,
        stamp_sec: float,
    ) -> Tuple[bool, str]:
        """Update moving target pose in simulation (/dynamic_goal_pose)."""
        if self.collision_locked:
            return False, 'Navigation locked after Collision; call /reset_nav to recover'
        ok, coords, info = self._transform_goal_frame(x, y, frame_id)
        if not ok or coords is None:
            return False, info

        new_goal = np.array(coords, dtype=np.float64)
        warning_note = ''
        jumped = False
        if self.goal_odom is not None:
            jump = float(np.linalg.norm(new_goal - self.goal_odom))
            self.last_target_jump_m = jump
            if jump > self.nav_cfg.target_jump_threshold:
                jumped = True
                warning_note = f' (target jump {jump:.2f} m detected; reacquiring)'
        else:
            self.last_target_jump_m = 0.0

        self.goal_odom = new_goal
        self._record_trail_point(new_goal, jumped)
        self.last_target_time = float(stamp_sec)
        self.active = True
        return True, f'Updated dynamic target ({coords[0]:.3f}, {coords[1]:.3f}){warning_note}'

    def _pose_near_stamp(self, stamp_sec: float) -> Tuple[Optional[np.ndarray], float]:
        """Return (pose, abs_dt_sec) for the odom entry closest to stamp_sec."""
        if not self.odom_history:
            return self.robot_pose, float('inf')
        best_pose = None
        best_dt = float('inf')
        for t, p in self.odom_history:
            dt = abs(t - stamp_sec)
            if dt < best_dt:
                best_dt = dt
                best_pose = p
        return best_pose, best_dt

    def update_landmarks(
        self,
        landmarks: Sequence[LandmarkObservation],
        stamp_sec: float,
        now_sec: Optional[float] = None,
    ) -> Tuple[bool, str]:
        """
        Update target from /camera/landmarks (follow mode or one-shot static goal).

        In follow mode, continuously updates the target and ignores empty arrays
        so target_loss_timeout_sec counts from the last valid detection. In goal
        mode with goal_source='landmark', locks the first valid detection of
        target_id as a one-shot static goal until /reset_nav.
        """
        if self.collision_locked:
            return False, 'Navigation locked after Collision; call /reset_nav to recover'
        if self.nav_cfg.mode == 'goal':
            if self.nav_cfg.goal_source != 'landmark':
                return False, 'Ignoring landmarks because goal_source is not landmark'
            if self.static_landmark_locked:
                return False, 'Static landmark goal already locked'

        valid = [
            lm for lm in landmarks
            if math.isfinite(lm.range_m) and lm.range_m > 0.0 and math.isfinite(lm.bearing_rad)
        ]
        if not valid:
            return False, 'No visible landmarks'

        if self.nav_cfg.target_id >= 0:
            matches = [lm for lm in valid if lm.landmark_id == self.nav_cfg.target_id]
            if not matches:
                return False, f'Target landmark id {self.nav_cfg.target_id} not visible'
            chosen = min(matches, key=lambda lm: lm.range_m)
        else:
            chosen = min(valid, key=lambda lm: lm.range_m)

        pose_at_obs, dt_skew = self._pose_near_stamp(float(stamp_sec))
        if pose_at_obs is None:
            return False, 'Cannot project landmark without odometry'

        skew_note = ''
        if dt_skew > self.nav_cfg.max_stamp_skew_sec and self.robot_pose is not None:
            pose_at_obs = self.robot_pose
            skew_note = (
                f' (clock skew {dt_skew:.2f}s > {self.nav_cfg.max_stamp_skew_sec:.2f}s; '
                'used latest odom)'
            )

        tx, ty = landmark_to_odom(
            pose_at_obs,
            chosen.range_m,
            chosen.bearing_rad,
            (
                self.nav_cfg.camera_offset_x,
                self.nav_cfg.camera_offset_y,
                self.nav_cfg.camera_offset_yaw,
            ),
        )
        if self.nav_cfg.mode == 'goal':
            rx, ry = float(pose_at_obs[0]), float(pose_at_obs[1])
            dx = tx - rx
            dy = ty - ry
            dist = math.hypot(dx, dy)
            standoff = self.nav_cfg.landmark_goal_standoff
            if dist <= standoff or dist < 1e-9:
                gx, gy = rx, ry
            else:
                scale = (dist - standoff) / dist
                gx = rx + dx * scale
                gy = ry + dy * scale
        else:
            gx, gy = tx, ty

        new_goal = np.array([gx, gy], dtype=np.float64)
        jumped = False
        if self.goal_odom is not None:
            jump = float(np.linalg.norm(new_goal - self.goal_odom))
            self.last_target_jump_m = jump
            jumped = jump > self.nav_cfg.target_jump_threshold
        else:
            self.last_target_jump_m = 0.0

        self.goal_odom = new_goal
        if self.nav_cfg.mode == 'follow':
            self._record_trail_point(new_goal, jumped)
        # Track target freshness using wall/node time if provided, else stamp_sec
        self.last_target_time = float(now_sec) if now_sec is not None else float(stamp_sec)
        self.active = True
        if self.nav_cfg.mode == 'goal':
            self.step_count = 0
            self.static_landmark_locked = True
            return True, (
                f'Locked one-shot static goal from landmark id={chosen.landmark_id}: '
                f'tag=({tx:.2f}, {ty:.2f}) -> goal=({gx:.2f}, {gy:.2f}) '
                f'(standoff={self.nav_cfg.landmark_goal_standoff:.2f} m){skew_note}'
            )
        return True, (
            f'Updated landmark target id={chosen.landmark_id} '
            f'(r={chosen.range_m:.2f} m, b={math.degrees(chosen.bearing_rad):.1f} deg) '
            f'-> ({gx:.2f}, {gy:.2f}){skew_note}'
        )

    def update_odom(
        self,
        pose: Sequence[float],
        vel: Sequence[float],
        stamp_sec: float,
    ) -> None:
        """Record latest robot pose [x, y, yaw] and velocity [v, w] in odom frame."""
        pose_arr = np.asarray(pose, dtype=np.float64).reshape(3)
        self.robot_pose = pose_arr
        self.robot_vel = np.asarray(vel, dtype=np.float64).reshape(2)
        self.last_odom_time = float(stamp_sec)
        self.odom_history.append((float(stamp_sec), pose_arr.copy()))

    def update_scan(
        self,
        scan: ScanData,
        stamp_sec: float,
        now_sec: Optional[float] = None,
    ) -> None:
        """Record latest LiDAR scan and project obstacles using scan-time odom pose."""
        self.last_scan = scan
        self.last_scan_stamp = float(stamp_sec)
        self.last_scan_time = float(now_sec) if now_sec is not None else float(stamp_sec)

        pose_at_scan, dt_skew = self._pose_near_stamp(self.last_scan_stamp)
        if pose_at_scan is None:
            self.cached_obstacles_odom = None
            return
        if dt_skew > self.nav_cfg.max_stamp_skew_sec and self.robot_pose is not None:
            pose_at_scan = self.robot_pose

        self.cached_obstacles_odom = scan_to_obstacles(
            pose_at_scan,
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
            self.scan_cfg,
        )

    def _finish(self, event: str, warning: Optional[str] = None) -> StepOutput:
        self.active = False
        self.last_cmd_vel = np.array([0.0, 0.0], dtype=np.float64)
        if event == RESULT_COLLISION:
            self.collision_locked = True
        return StepOutput(
            cmd_vel=(0.0, 0.0),
            publish_cmd=True,
            event=event,
            step_count=self.step_count,
            warning=warning,
        )

    def step(self, now_sec: float) -> StepOutput:
        """Execute one 15 Hz navigation control tick (non-blocking)."""
        if self.collision_locked or not self.active or self.goal_odom is None:
            return StepOutput(publish_cmd=False, step_count=self.step_count)

        if (
            self.robot_pose is None
            or self.robot_vel is None
            or self.last_odom_time is None
            or self.last_scan is None
            or self.last_scan_time is None
        ):
            return StepOutput(publish_cmd=False, step_count=self.step_count)

        # Sensor freshness check: stop robot if either odom or scan timed out.
        odom_age = float(now_sec) - self.last_odom_time
        scan_age = float(now_sec) - self.last_scan_time
        if (
            odom_age > self.nav_cfg.sensor_timeout_sec
            or scan_age > self.nav_cfg.sensor_timeout_sec
        ):
            return StepOutput(
                cmd_vel=(0.0, 0.0),
                publish_cmd=True,
                step_count=self.step_count,
                warning=(
                    f'Sensor timeout (odom_age={odom_age:.2f}s, '
                    f'scan_age={scan_age:.2f}s); stopping robot'
                ),
            )

        # LiDAR health check: never drive blind when scan is all-zero or NaN-saturated.
        healthy, health_reason = check_scan_health(
            self.last_scan.ranges,
            self.last_scan.angle_min,
            self.last_scan.angle_increment,
            self.last_scan.range_min,
            self.last_scan.range_max,
            self.scan_cfg,
        )
        if not healthy:
            return StepOutput(
                cmd_vel=(0.0, 0.0),
                publish_cmd=True,
                step_count=self.step_count,
                warning=health_reason,
            )

        # Follow mode: target loss timeout stops robot without terminating task
        if self.nav_cfg.mode == 'follow' and self.last_target_time is not None:
            target_age = float(now_sec) - self.last_target_time
            if target_age > self.nav_cfg.target_loss_timeout_sec:
                return StepOutput(
                    cmd_vel=(0.0, 0.0),
                    publish_cmd=True,
                    step_count=self.step_count,
                    warning=f'Target lost for {target_age:.2f} s; stopping until reacquired',
                    target_lost=True,
                )

        dist_to_goal = float(np.linalg.norm(self.robot_pose[:2] - self.goal_odom))

        # 1. Static goal reached check (ONLY in goal mode; follow mode never stops on arrival)
        if self.nav_cfg.mode == 'goal' and dist_to_goal < self.nav_cfg.goal_dist_tol:
            return self._finish(RESULT_GOAL)

        # 2. Independent laser-based safety stop check (guarded escape after /reset_nav)
        is_collision, min_obs_dist = check_safety_stop(
            self.last_scan.ranges,
            self.last_scan.angle_min,
            self.last_scan.angle_increment,
            self.last_scan.range_min,
            self.last_scan.range_max,
            self.dwa.cfg.robot_radius,
            self.dwa.cfg.collision_tol,
            self.scan_cfg,
        )
        allow_escape = False
        if not is_collision:
            # Exited stop zone cleanly: disable temporary escape recovery.
            self.escape_recovery_active = False
            self.escape_start_dist = None
        elif self.escape_recovery_active:
            physical_floor = self.dwa.cfg.robot_radius + 0.02
            if self.escape_start_dist is None:
                self.escape_start_dist = min_obs_dist
            # Keep hard stop active if physical contact is imminent or clearance worsens.
            if min_obs_dist < physical_floor or min_obs_dist < (self.escape_start_dist - 0.02):
                return self._finish(
                    RESULT_COLLISION,
                    warning=(
                        f'Safety stop during escape: min center distance {min_obs_dist:.3f} m'
                    ),
                )
            allow_escape = True
        else:
            return self._finish(
                RESULT_COLLISION,
                warning=(
                    f'Safety stop triggered: min center distance {min_obs_dist:.3f} m '
                    f'< {self.dwa.collision_radius:.3f} m'
                ),
            )

        # 3. Step counter and timeout check (in follow mode only if max_num_steps > 0)
        self.step_count += 1
        if self.nav_cfg.max_num_steps > 0 and self.step_count > self.nav_cfg.max_num_steps:
            return self._finish(RESULT_TIMEOUT)

        # 4. Follow-mode Lost check (> lost_dist for lost_dist_time_sec -> report once, keep going)
        info_notes = []
        lost_event = None
        lost_warning = None
        if self.nav_cfg.mode == 'follow':
            if dist_to_goal > self.nav_cfg.lost_dist:
                if self.far_since is None:
                    self.far_since = float(now_sec)
                if (
                    not self.lost_reported
                    and float(now_sec) - self.far_since >= self.nav_cfg.lost_dist_time_sec
                ):
                    self.lost_reported = True
                    lost_event = RESULT_LOST
                    lost_warning = (
                        f'Target remained > {self.nav_cfg.lost_dist:.2f} m for '
                        f'{self.nav_cfg.lost_dist_time_sec:.1f} s; reporting Lost and '
                        'continuing pursuit'
                    )
            else:
                self.far_since = None
                if self.lost_reported:
                    self.lost_reported = False
                    info_notes.append(f'Target back within {self.nav_cfg.lost_dist:.2f} m')

        # 5. Follow-mode trail pruning and stuck-recovery exit check
        heading_goal = None
        use_progress = False
        if self.nav_cfg.mode == 'follow':
            while (
                self.target_trail
                and float(np.linalg.norm(self.robot_pose[:2] - self.target_trail[0]))
                <= self.nav_cfg.trail_reach_dist
            ):
                self.target_trail.popleft()
            if self.nav_cfg.follow_use_trail and self.target_trail:
                heading_goal = self.target_trail[0]

            if self.stuck_recovery_active:
                moved = (
                    float(np.linalg.norm(self.robot_pose[:2] - self.stuck_recovery_start_pose))
                    if self.stuck_recovery_start_pose is not None
                    else 0.0
                )
                elapsed = (
                    float(now_sec) - self.stuck_recovery_start_time
                    if self.stuck_recovery_start_time is not None
                    else 0.0
                )
                close_enough = (
                    dist_to_goal
                    <= self.dwa.cfg.desired_target_dist + self.nav_cfg.recovery_exit_margin
                )
                exit_reason = None
                if moved >= self.nav_cfg.recovery_exit_dist:
                    exit_reason = f'moved {moved:.2f} m'
                elif close_enough:
                    exit_reason = f'within target margin ({dist_to_goal:.2f} m)'
                elif elapsed >= self.nav_cfg.recovery_max_sec:
                    exit_reason = f'timeout ({elapsed:.1f} s)'
                if exit_reason is not None:
                    self.stuck_recovery_active = False
                    self.stuck_since = None
                    info_notes.append(f'Exiting follow stuck recovery ({exit_reason})')
            use_progress = self.stuck_recovery_active

        # 6. Use odom-frame obstacles projected at scan timestamp
        if self.cached_obstacles_odom is not None:
            obstacles = self.cached_obstacles_odom
        else:
            obstacles = scan_to_obstacles(
                self.robot_pose,
                self.last_scan.ranges,
                self.last_scan.angle_min,
                self.last_scan.angle_increment,
                self.last_scan.range_min,
                self.last_scan.range_max,
                self.scan_cfg,
            )
        vel_center = (
            self.last_cmd_vel if self.nav_cfg.window_source == 'last_cmd' else self.robot_vel
        )
        decision = self.dwa.compute_cmd_detailed(
            self.goal_odom,
            self.robot_pose,
            vel_center,
            obstacles,
            allow_stop_zone_escape=allow_escape,
            heading_goal=heading_goal,
            use_progress_recovery=use_progress,
        )
        self.last_cmd_vel = decision.cmd_vel.copy()

        # 7. Follow-mode stuck-recovery entry check
        if self.nav_cfg.mode == 'follow' and not self.stuck_recovery_active:
            far_enough = (
                dist_to_goal
                > self.dwa.cfg.desired_target_dist + self.nav_cfg.stuck_dist_margin
            )
            if float(decision.cmd_vel[0]) < self.nav_cfg.stuck_vel_thresh and far_enough:
                if self.stuck_since is None:
                    self.stuck_since = float(now_sec)
                elif float(now_sec) - self.stuck_since >= self.nav_cfg.stuck_time_sec:
                    self.stuck_recovery_active = True
                    self.stuck_recovery_start_time = float(now_sec)
                    self.stuck_recovery_start_pose = self.robot_pose[:2].copy()
                    self.stuck_since = None
                    info_notes.append(
                        f'Entering follow stuck recovery (dist={dist_to_goal:.2f} m, '
                        f'v={decision.cmd_vel[0]:.3f} m/s)'
                    )
            else:
                self.stuck_since = None

        # 8. Periodic intermediate feedback every N = feedback_rate steps
        feedback_dist = None
        if self.nav_cfg.feedback_rate > 0 and self.step_count % self.nav_cfg.feedback_rate == 0:
            feedback_dist = dist_to_goal

        warning = lost_warning
        if decision.all_colliding:
            col_warn = 'All candidate DWA trajectories collide; commanding zero velocity'
            warning = f'{warning}; {col_warn}' if warning else col_warn

        return StepOutput(
            cmd_vel=(float(decision.cmd_vel[0]), float(decision.cmd_vel[1])),
            publish_cmd=True,
            event=lost_event,
            feedback_distance=feedback_dist,
            step_count=self.step_count,
            warning=warning,
            info='; '.join(info_notes) if info_notes else None,
            all_colliding=decision.all_colliding,
            target_lost=self.lost_reported,
        )
