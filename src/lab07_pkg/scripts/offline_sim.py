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
Pure-Python 2D offline simulator for Lab 07 Task 1 (NOT Gazebo).

Parses wall/obstacle boxes from hard_maze.world, simulates a 230-ray 360 deg
LiDAR and TurtleBot3 Burger kinematics at 15 Hz, runs NavigationManager on
verified goals inside [-6, 6] m, performs alpha/beta/gamma weight sensitivity
analysis, and benchmarks compute_cmd latency against the 66.7 ms budget.
"""

import argparse
from dataclasses import dataclass
import math
import os
import sys
import time
from typing import Dict, List, Sequence, Tuple
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.patches as patches  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402

# Allow running directly without colcon install
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, '..'))
if PKG_ROOT not in sys.path:
    sys.path.insert(0, PKG_ROOT)

from lab07_pkg.dwa import DWAConfig, DWAController, normalize_angle  # noqa: E402
from lab07_pkg.navigation import NavConfig, NavigationManager, ScanData  # noqa: E402
from lab07_pkg.scan_utils import ScanConfig  # noqa: E402


@dataclass
class Box2D:
    """2D oriented bounding box parsed from an SDF world."""

    name: str
    cx: float
    cy: float
    yaw: float
    sx: float
    sy: float


def _parse_pose(text: str) -> Tuple[float, float, float]:
    vals = [float(v) for v in text.strip().split()] if text else [0.0] * 6
    return vals[0], vals[1], vals[5]


def load_world_boxes(world_path: str) -> List[Box2D]:
    """Parse all static collision boxes (border + obstacles) from hard_maze.world."""
    tree = ET.parse(world_path)
    world = tree.getroot().find('world')
    boxes: List[Box2D] = []

    for model in world.findall('model'):
        mname = model.attrib.get('name', '')
        if mname == 'goal_box':
            continue
        mx, my, myaw = _parse_pose(model.findtext('pose'))
        mc, ms = math.cos(myaw), math.sin(myaw)

        for link in model.findall('link'):
            lx, ly, lyaw = _parse_pose(link.findtext('pose'))
            wx = mx + lx * mc - ly * ms
            wy = my + lx * ms + ly * mc
            wyaw = myaw + lyaw

            for col in link.findall('collision'):
                cx, cy, cyaw = _parse_pose(col.findtext('pose'))
                cc, cs = math.cos(wyaw), math.sin(wyaw)
                fx = wx + cx * cc - cy * cs
                fy = wy + cx * cs + cy * cc
                fyaw = wyaw + cyaw
                size_el = col.find('geometry/box/size')
                if size_el is not None and size_el.text:
                    sx, sy, _ = [float(v) for v in size_el.text.strip().split()]
                    name = f"{mname}/{link.attrib.get('name', '')}"
                    boxes.append(Box2D(name, fx, fy, fyaw, sx, sy))
    return boxes


def point_clearance(px: float, py: float, boxes: Sequence[Box2D]) -> float:
    """Minimum Euclidean distance from point (px, py) to any box surface (0 if inside)."""
    best = float('inf')
    for b in boxes:
        dx = px - b.cx
        dy = py - b.cy
        c = math.cos(-b.yaw)
        s = math.sin(-b.yaw)
        lx = dx * c - dy * s
        ly = dx * s + dy * c
        ex = max(abs(lx) - b.sx * 0.5, 0.0)
        ey = max(abs(ly) - b.sy * 0.5, 0.0)
        d = math.hypot(ex, ey)
        if d < best:
            best = d
    return best


def simulate_lidar(
    robot_pose: Sequence[float],
    boxes: Sequence[Box2D],
    scan_cfg: ScanConfig,
    n_rays: int = 230,
    range_min: float = 0.12,
    range_max: float = 3.5,
) -> ScanData:
    """Vectorized 2D ray-OBB LiDAR simulation matching Gazebo TurtleBot3 Burger (~230 rays)."""
    rx, ry, ryaw = float(robot_pose[0]), float(robot_pose[1]), float(robot_pose[2])
    c_r, s_r = math.cos(ryaw), math.sin(ryaw)

    # Sensor origin in world/odom frame
    ox = rx + scan_cfg.laser_offset_x * c_r - scan_cfg.laser_offset_y * s_r
    oy = ry + scan_cfg.laser_offset_x * s_r + scan_cfg.laser_offset_y * c_r

    angle_min = -math.pi
    angle_inc = 2.0 * math.pi / float(n_rays)
    local_angles = angle_min + np.arange(n_rays, dtype=np.float64) * angle_inc
    world_angles = ryaw + scan_cfg.laser_offset_yaw + local_angles
    dx_w = np.cos(world_angles)
    dy_w = np.sin(world_angles)

    best_t = np.full(n_rays, np.inf, dtype=np.float64)

    for b in boxes:
        c = math.cos(-b.yaw)
        s = math.sin(-b.yaw)
        # Transform ray origin and direction into box-local frame
        rel_x = ox - b.cx
        rel_y = oy - b.cy
        ro_x = rel_x * c - rel_y * s
        ro_y = rel_x * s + rel_y * c
        rd_x = dx_w * c - dy_w * s
        rd_y = dx_w * s + dy_w * c

        hx = b.sx * 0.5
        hy = b.sy * 0.5

        with np.errstate(divide='ignore', invalid='ignore'):
            tx1 = (-hx - ro_x) / rd_x
            tx2 = (hx - ro_x) / rd_x
            ty1 = (-hy - ro_y) / rd_y
            ty2 = (hy - ro_y) / rd_y

        in_x = np.abs(ro_x) <= hx
        in_y = np.abs(ro_y) <= hy
        has_x = np.abs(rd_x) > 1e-12
        has_y = np.abs(rd_y) > 1e-12
        tmin_x = np.where(has_x, np.minimum(tx1, tx2), np.where(in_x, -np.inf, np.inf))
        tmax_x = np.where(has_x, np.maximum(tx1, tx2), np.where(in_x, np.inf, -np.inf))
        tmin_y = np.where(has_y, np.minimum(ty1, ty2), np.where(in_y, -np.inf, np.inf))
        tmax_y = np.where(has_y, np.maximum(ty1, ty2), np.where(in_y, np.inf, -np.inf))

        t_enter = np.maximum(tmin_x, tmin_y)
        t_exit = np.minimum(tmax_x, tmax_y)

        hit = (t_exit >= np.maximum(t_enter, 0.0)) & (t_enter >= 0.0)
        t_hit = np.where(hit, t_enter, np.inf)
        best_t = np.minimum(best_t, t_hit)

    # Rays beyond range_max return +inf (matching Gazebo LiDAR)
    ranges = np.where((best_t >= range_min) & (best_t <= range_max), best_t, np.inf)
    return ScanData(
        ranges=ranges,
        angle_min=angle_min,
        angle_increment=angle_inc,
        range_min=range_min,
        range_max=range_max,
    )


def load_configs_from_yaml(yaml_path: str) -> Tuple[DWAConfig, ScanConfig, NavConfig]:
    """Load DWAConfig, ScanConfig, and NavConfig from config/sim.yaml."""
    with open(yaml_path, 'r', encoding='utf-8') as f:
        params = yaml.safe_load(f)['dwa_node']['ros__parameters']

    dt = 1.0 / float(params['control_rate'])
    dwa_cfg = DWAConfig(
        dt=dt,
        sim_time=float(params['sim_time']),
        time_granularity=float(params['time_granularity']),
        v_samples=int(params['v_samples']),
        w_samples=int(params['w_samples']),
        max_lin_vel=float(params['max_lin_vel']),
        min_lin_vel=float(params['min_lin_vel']),
        max_ang_vel=float(params['max_ang_vel']),
        min_ang_vel=float(params['min_ang_vel']),
        max_lin_acc=float(params['max_lin_acc']),
        max_ang_acc=float(params['max_ang_acc']),
        robot_radius=float(params['robot_radius']),
        collision_tol=float(params['collision_tol']),
        planning_margin=float(params.get('planning_margin', 0.10)),
        band_floor_buffer=float(params.get('band_floor_buffer', 0.03)),
        enable_recovery_spin=bool(params.get('enable_recovery_spin', False)),
        recovery_ang_vel=float(params.get('recovery_ang_vel', 0.4)),
        weight_heading=float(params['weight_heading']),
        weight_vel=float(params['weight_vel']),
        weight_obs=float(params['weight_obs']),
        weight_target=float(params.get('weight_target', 0.35)),
        obstacle_max_dist=float(params['obstacle_max_dist']),
        slowdown_dist_threshold=float(params.get('slowdown_dist_threshold', 1.2)),
        desired_target_dist=float(params.get('desired_target_dist', 0.8)),
        target_dist_sigma=float(params.get('target_dist_sigma', 0.5)),
        camera_fov_half_rad=float(params.get('camera_fov_half_rad', 0.576)),
    )
    scan_cfg = ScanConfig(
        max_scan_range=float(params['max_scan_range']),
        min_valid_range=float(params['min_valid_range']),
        min_valid_ratio=float(params.get('min_valid_ratio', 0.20)),
        max_nan_ratio=float(params.get('max_nan_ratio', 0.95)),
        scan_median_filter=bool(params.get('scan_median_filter', True)),
        num_ranges=int(params['num_ranges']),
        nan_policy=str(params['nan_policy']),
        laser_offset_x=float(params['laser_offset_x']),
        laser_offset_y=float(params['laser_offset_y']),
        laser_offset_yaw=float(params['laser_offset_yaw']),
    )
    origin = params['odom_origin_in_world']
    nav_cfg = NavConfig(
        goal_dist_tol=float(params['goal_dist_tol']),
        max_num_steps=int(params['max_num_steps']),
        feedback_rate=int(params['feedback_rate']),
        sensor_timeout_sec=float(params['sensor_timeout_sec']),
        odom_frame=str(params['odom_frame']),
        world_frame=str(params['world_frame']),
        odom_origin_in_world=(float(origin[0]), float(origin[1]), float(origin[2])),
    )
    return dwa_cfg, scan_cfg, nav_cfg


def run_episode(
    goal_xy: Tuple[float, float],
    boxes: Sequence[Box2D],
    dwa_cfg: DWAConfig,
    scan_cfg: ScanConfig,
    nav_cfg: NavConfig,
    init_pose: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    scan_rate_hz: float = 5.0,
    odom_rate_hz: float = 20.0,
) -> Dict:
    """Run one pure-Python simulation episode with async 5 Hz LiDAR and 20 Hz odom."""
    mgr = NavigationManager(dwa_cfg, scan_cfg, nav_cfg)
    ok, _ = mgr.set_goal(goal_xy[0], goal_xy[1], 'world')
    assert ok

    pose = np.array(init_pose, dtype=np.float64)
    vel = np.array([0.0, 0.0], dtype=np.float64)
    dt = dwa_cfg.dt
    sub_dt = 1.0 / max(odom_rate_hz, 1.0 / dt)
    scan_period = 1.0 / scan_rate_hz

    traj = [pose.copy()]
    cmds = []
    min_scan_dists = []
    event = 'Timeout'
    t_sim = 0.0
    next_scan_t = 0.0
    next_ctrl_t = 0.0
    cmd_v, cmd_w = 0.0, 0.0

    # Initial sensor readings at t = 0
    mgr.update_odom(pose, vel, stamp_sec=0.0)
    init_scan = simulate_lidar(pose, boxes, scan_cfg)
    mgr.update_scan(init_scan, stamp_sec=0.0, now_sec=0.0)
    next_scan_t = scan_period

    max_sim_time = (nav_cfg.max_num_steps + 2) * dt
    while t_sim <= max_sim_time:
        # 20 Hz odometry update
        mgr.update_odom(pose, vel, stamp_sec=t_sim)

        # 5 Hz LiDAR update (stamped at capture time)
        if t_sim + 1e-9 >= next_scan_t:
            scan = simulate_lidar(pose, boxes, scan_cfg)
            finite_r = scan.ranges[np.isfinite(scan.ranges)]
            if finite_r.size > 0:
                min_scan_dists.append(float(np.min(finite_r)))
            mgr.update_scan(scan, stamp_sec=t_sim, now_sec=t_sim)
            next_scan_t += scan_period

        # 15 Hz DWA control tick
        if t_sim + 1e-9 >= next_ctrl_t:
            out = mgr.step(now_sec=t_sim)
            if out.event is not None:
                event = out.event
                cmds.append((0.0, 0.0))
                break
            cmd_v, cmd_w = out.cmd_vel
            cmds.append((cmd_v, cmd_w))
            next_ctrl_t += dt

        # Kinematics step at odom sub_dt
        pose = np.array([
            pose[0] + cmd_v * math.cos(pose[2]) * sub_dt,
            pose[1] + cmd_v * math.sin(pose[2]) * sub_dt,
            pose[2] + cmd_w * sub_dt,
        ], dtype=np.float64)
        vel = np.array([cmd_v, cmd_w], dtype=np.float64)
        traj.append(pose.copy())
        t_sim += sub_dt

        if point_clearance(pose[0], pose[1], boxes) < dwa_cfg.robot_radius:
            event = 'Collision'
            break

    traj_arr = np.asarray(traj, dtype=np.float64)
    diffs = np.diff(traj_arr[:, :2], axis=0)
    path_len = float(np.sum(np.hypot(diffs[:, 0], diffs[:, 1]))) if len(diffs) > 0 else 0.0
    final_dist = float(np.hypot(traj_arr[-1, 0] - goal_xy[0], traj_arr[-1, 1] - goal_xy[1]))

    return {
        'goal': goal_xy,
        'result': event,
        'steps': mgr.step_count,
        'duration_s': mgr.step_count * dt,
        'path_len_m': path_len,
        'final_dist_m': final_dist,
        'min_obs_m': float(np.min(min_scan_dists)) if min_scan_dists else float('inf'),
        'avg_obs_m': float(np.mean(min_scan_dists)) if min_scan_dists else float('inf'),
        'traj': traj_arr,
        'cmds': np.asarray(cmds, dtype=np.float64),
    }


def draw_maze(ax, boxes: Sequence[Box2D], xlim=(-6.5, 6.5), ylim=(-6.5, 6.5)) -> None:
    """Draw 2D boxes from hard_maze.world onto a Matplotlib Axes."""
    for b in boxes:
        # Rectangle corner before rotation
        corners = np.array([
            [-b.sx / 2.0, -b.sy / 2.0],
            [b.sx / 2.0, -b.sy / 2.0],
            [b.sx / 2.0, b.sy / 2.0],
            [-b.sx / 2.0, b.sy / 2.0],
        ])
        c, s = math.cos(b.yaw), math.sin(b.yaw)
        rot = np.array([[c, -s], [s, c]])
        pts = corners @ rot.T + np.array([b.cx, b.cy])
        poly = patches.Polygon(
            pts, closed=True, facecolor='#3a3a3a', edgecolor='#1a1a1a', linewidth=0.5
        )
        ax.add_patch(poly)
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect('equal')
    ax.grid(True, linestyle=':', alpha=0.5)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')


def load_goal_box_waypoints(world_path: str) -> List[Tuple[float, float, float]]:
    """Parse (time, x, y) waypoints of the moving red cube (goal_box) from hard_maze.world."""
    tree = ET.parse(world_path)
    world = tree.getroot().find('world')
    waypoints: List[Tuple[float, float, float]] = []
    for model in world.findall('model'):
        if model.attrib.get('name') == 'goal_box':
            for wp in model.findall('.//waypoint'):
                t = float(wp.findtext('time', '0.0').strip())
                px, py, _ = _parse_pose(wp.findtext('pose'))
                waypoints.append((t, px, py))
    return waypoints


def sample_waypoint_trajectory(
    waypoints: Sequence[Tuple[float, float, float]], t: float
) -> Tuple[float, float]:
    """Linearly interpolate moving target position at time t [s]."""
    if t <= waypoints[0][0]:
        return waypoints[0][1], waypoints[0][2]
    for i in range(len(waypoints) - 1):
        t0, x0, y0 = waypoints[i]
        t1, x1, y1 = waypoints[i + 1]
        if t0 <= t <= t1:
            alpha = (t - t0) / max(t1 - t0, 1e-9)
            return x0 + alpha * (x1 - x0), y0 + alpha * (y1 - y0)
    return waypoints[-1][1], waypoints[-1][2]


def run_follow_episode(
    waypoints: Sequence[Tuple[float, float, float]],
    boxes: Sequence[Box2D],
    dwa_cfg: DWAConfig,
    scan_cfg: ScanConfig,
    nav_cfg: NavConfig,
    init_pose: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    scan_rate_hz: float = 5.0,
    odom_rate_hz: float = 20.0,
    follow_dist_threshold: float = 2.0,
    min_in_range_ratio: float = 0.70,
    max_final_dist_m: float = 2.0,
) -> Dict:
    """Run one pure-Python Task 2 episode with async 5 Hz LiDAR and honest Lost/Success."""
    follow_dwa = DWAConfig(**{**dwa_cfg.__dict__, 'follow_mode': True})
    follow_nav = NavConfig(**{**nav_cfg.__dict__, 'mode': 'follow', 'max_num_steps': 0})
    mgr = NavigationManager(follow_dwa, scan_cfg, follow_nav)

    pose = np.array(init_pose, dtype=np.float64)
    vel = np.array([0.0, 0.0], dtype=np.float64)
    dt = follow_dwa.dt
    sub_dt = 1.0 / max(odom_rate_hz, 1.0 / dt)
    scan_period = 1.0 / scan_rate_hz
    total_time = waypoints[-1][0]

    traj = [pose.copy()]
    target_traj = []
    min_scan_dists = []
    target_dists = []
    dist_errors = []
    bearing_errors = []
    event = None
    t_sim = 0.0
    next_scan_t = 0.0
    next_ctrl_t = 0.0
    cmd_v, cmd_w = 0.0, 0.0

    mgr.update_odom(pose, vel, stamp_sec=0.0)
    init_scan = simulate_lidar(pose, boxes, scan_cfg)
    mgr.update_scan(init_scan, stamp_sec=0.0, now_sec=0.0)
    next_scan_t = scan_period

    while t_sim <= total_time:
        tx, ty = sample_waypoint_trajectory(waypoints, t_sim)
        mgr.update_odom(pose, vel, stamp_sec=t_sim)
        mgr.update_dynamic_goal(tx, ty, 'world', stamp_sec=t_sim)

        if t_sim + 1e-9 >= next_scan_t:
            scan = simulate_lidar(pose, boxes, scan_cfg)
            finite_r = scan.ranges[np.isfinite(scan.ranges)]
            if finite_r.size > 0:
                min_scan_dists.append(float(np.min(finite_r)))
            mgr.update_scan(scan, stamp_sec=t_sim, now_sec=t_sim)
            next_scan_t += scan_period

        if t_sim + 1e-9 >= next_ctrl_t:
            target_traj.append((tx, ty))
            dx = tx - pose[0]
            dy = ty - pose[1]
            d_now = math.hypot(dx, dy)
            target_dists.append(d_now)
            dist_errors.append(d_now - follow_dwa.desired_target_dist)
            bearing_errors.append(float(normalize_angle(math.atan2(dy, dx) - pose[2])))

            out = mgr.step(now_sec=t_sim)
            if out.event is not None and out.event != 'Lost':
                event = out.event
                break
            cmd_v, cmd_w = out.cmd_vel
            next_ctrl_t += dt

        pose = np.array([
            pose[0] + cmd_v * math.cos(pose[2]) * sub_dt,
            pose[1] + cmd_v * math.sin(pose[2]) * sub_dt,
            pose[2] + cmd_w * sub_dt,
        ], dtype=np.float64)
        vel = np.array([cmd_v, cmd_w], dtype=np.float64)
        traj.append(pose.copy())
        t_sim += sub_dt

        if point_clearance(pose[0], pose[1], boxes) < follow_dwa.robot_radius:
            event = 'Collision'
            break

    traj_arr = np.asarray(traj, dtype=np.float64)
    target_arr = np.asarray(target_traj, dtype=np.float64)
    diffs = np.diff(traj_arr[:, :2], axis=0)
    path_len = float(np.sum(np.hypot(diffs[:, 0], diffs[:, 1]))) if len(diffs) > 0 else 0.0

    dist_rmse = float(np.sqrt(np.mean(np.square(dist_errors)))) if dist_errors else float('inf')
    bearing_rmse = (
        float(np.sqrt(np.mean(np.square(bearing_errors)))) if bearing_errors else float('inf')
    )
    in_range_ratio = (
        float(np.mean(np.asarray(target_dists) <= follow_dist_threshold)) if target_dists else 0.0
    )
    end_dist = float(
        math.hypot(traj_arr[-1, 0] - target_arr[-1, 0], traj_arr[-1, 1] - target_arr[-1, 1])
    )
    if event is None:
        if (
            in_range_ratio >= min_in_range_ratio
            and end_dist <= max_final_dist_m
            and path_len >= 1.0
        ):
            event = 'Success'
        else:
            event = 'Lost'
    return {
        'result': event,
        'duration_s': t_sim,
        'path_len_m': path_len,
        'min_obs_m': float(np.min(min_scan_dists)) if min_scan_dists else float('inf'),
        'avg_obs_m': float(np.mean(min_scan_dists)) if min_scan_dists else float('inf'),
        'dist_rmse_m': dist_rmse,
        'bearing_rmse_rad': bearing_rmse,
        'in_range_ratio': in_range_ratio,
        'final_dist_m': end_dist,
        'traj': traj_arr,
        'target_traj': target_arr,
    }


def benchmark_compute_cmd(dwa_cfg: DWAConfig, n_iters: int = 300) -> Tuple[float, float, float]:
    """Benchmark compute_cmd wall-clock latency [ms] with 24 obstacle points."""
    ctrl = DWAController(dwa_cfg)
    angles = np.linspace(-math.pi, math.pi, 24, endpoint=False)
    obstacles = np.column_stack((1.8 * np.cos(angles), 1.8 * np.sin(angles)))
    pose = np.array([0.0, 0.0, 0.2], dtype=np.float64)
    vel = np.array([0.12, 0.1], dtype=np.float64)
    goal = np.array([3.0, -1.5], dtype=np.float64)

    # Warm-up
    for _ in range(10):
        ctrl.compute_cmd(goal, pose, vel, obstacles)

    times_ms = []
    for _ in range(n_iters):
        t0 = time.perf_counter()
        ctrl.compute_cmd(goal, pose, vel, obstacles)
        times_ms.append((time.perf_counter() - t0) * 1000.0)

    arr = np.asarray(times_ms, dtype=np.float64)
    return float(np.median(arr)), float(np.percentile(arr, 95)), float(np.max(arr))


def main() -> None:
    """Run offline Task 1, weight sensitivity, Task 2 follow, and latency benchmark."""
    parser = argparse.ArgumentParser(
        description='Pure-Python offline DWA simulation on hard_maze.world'
    )
    parser.add_argument(
        '--world',
        default=os.path.abspath(os.path.join(PKG_ROOT, '..', '资料', 'hard_maze.world')),
        help='Path to hard_maze.world',
    )
    parser.add_argument(
        '--config',
        default=os.path.join(PKG_ROOT, 'config', 'sim.yaml'),
        help='Path to sim.yaml',
    )
    parser.add_argument(
        '--out-dir',
        default=os.path.abspath(os.path.join(PKG_ROOT, '..', 'results')),
        help='Directory to save plots',
    )
    args = parser.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    boxes = load_world_boxes(args.world)
    base_dwa, base_scan, base_nav = load_configs_from_yaml(args.config)

    # 6 verified goals inside [-6, 6] x [-6, 6] m
    goals = [
        ('G1', (1.5, -0.5)),
        ('G2', (2.2, -3.4)),
        ('G3', (-2.45, -2.0)),
        ('G4', (-3.45, 0.8)),
        ('G5', (2.0, 3.0)),
        ('G6', (-2.0, 2.0)),
    ]

    print('=== Pure Python Simulation (NOT Gazebo) ===')
    print(f'Loaded {len(boxes)} boxes from {args.world}')
    print('\n--- Goal Verification & Episode Results (Baseline Weights) ---')
    print(
        f'Weights: alpha={base_dwa.weight_heading}, beta={base_dwa.weight_vel}, '
        f'gamma={base_dwa.weight_obs} | v_max={base_dwa.max_lin_vel} m/s | '
        f'collision_tol={base_dwa.collision_tol} m'
    )
    header = (
        f"{'ID':<4} {'Goal (x,y)':<14} {'WallClr[m]':>10} {'Result':<10} "
        f"{'Time[s]':>8} {'Path[m]':>8} {'MinObs[m]':>10} {'AvgObs[m]':>10} {'EndDist[m]':>10}"
    )
    print(header)
    print('-' * len(header))

    goal_results = []
    for gid, gxy in goals:
        clr = point_clearance(gxy[0], gxy[1], boxes)
        res = run_episode(gxy, boxes, base_dwa, base_scan, base_nav)
        res['id'] = gid
        res['wall_clearance'] = clr
        goal_results.append(res)
        print(
            f"{gid:<4} ({gxy[0]:5.2f},{gxy[1]:5.2f}) {clr:10.3f} {res['result']:<10} "
            f"{res['duration_s']:8.2f} {res['path_len_m']:8.2f} "
            f"{res['min_obs_m']:10.3f} {res['avg_obs_m']:10.3f} {res['final_dist_m']:10.3f}"
        )

    # Plot 6-goal trajectories
    fig, ax = plt.subplots(figsize=(8, 8))
    draw_maze(ax, boxes)
    ax.plot(0.0, 0.0, 'ks', markersize=8, label='Start (0, 0)')
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']
    for res, col in zip(goal_results, colors):
        tr = res['traj']
        gx, gy = res['goal']
        lbl = f"{res['id']} ({res['result']})"
        ax.plot(tr[:, 0], tr[:, 1], color=col, linewidth=1.8, label=lbl)
        ax.scatter([gx], [gy], color=col, marker='*', s=130, zorder=5)
    ax.set_title('Task 1 Pure-Python Simulation (NOT Gazebo): 6 Goals in hard_maze.world')
    ax.legend(loc='upper right', fontsize=8)
    goals_plot = os.path.join(args.out_dir, 'offline_goals.png')
    fig.tight_layout()
    fig.savefig(goals_plot, dpi=150)
    plt.close(fig)

    # Weight sensitivity analysis on G2 (2.2, -3.4) — requires navigating around walls
    weight_goal = (2.2, -3.4)
    weight_trials = [
        ('Baseline', base_dwa.weight_heading, base_dwa.weight_vel, base_dwa.weight_obs),
        ('Low alpha (0.10)', 0.10, base_dwa.weight_vel, base_dwa.weight_obs),
        ('High alpha (0.85)', 0.85, base_dwa.weight_vel, base_dwa.weight_obs),
        ('Low beta (0.05)', base_dwa.weight_heading, 0.05, base_dwa.weight_obs),
        ('High beta (0.80)', base_dwa.weight_heading, 0.80, base_dwa.weight_obs),
        ('Low gamma (0.05)', base_dwa.weight_heading, base_dwa.weight_vel, 0.05),
        ('High gamma (0.85)', base_dwa.weight_heading, base_dwa.weight_vel, 0.85),
    ]

    print(f'\n--- Weight Sensitivity Analysis on G2 {weight_goal} (Pure Python Simulation) ---')
    w_header = (
        f"{'Trial':<19} {'(a, b, g)':<18} {'Result':<10} "
        f"{'Time[s]':>8} {'Path[m]':>8} {'MinObs[m]':>10} {'AvgObs[m]':>10}"
    )
    print(w_header)
    print('-' * len(w_header))

    weight_results = []
    for name, a, b, g in weight_trials:
        cfg = DWAConfig(
            dt=base_dwa.dt,
            sim_time=base_dwa.sim_time,
            time_granularity=base_dwa.time_granularity,
            v_samples=base_dwa.v_samples,
            w_samples=base_dwa.w_samples,
            max_lin_vel=base_dwa.max_lin_vel,
            min_lin_vel=base_dwa.min_lin_vel,
            max_ang_vel=base_dwa.max_ang_vel,
            min_ang_vel=base_dwa.min_ang_vel,
            max_lin_acc=base_dwa.max_lin_acc,
            max_ang_acc=base_dwa.max_ang_acc,
            robot_radius=base_dwa.robot_radius,
            collision_tol=base_dwa.collision_tol,
            weight_heading=a,
            weight_vel=b,
            weight_obs=g,
            obstacle_max_dist=base_dwa.obstacle_max_dist,
        )
        res = run_episode(weight_goal, boxes, cfg, base_scan, base_nav)
        res['name'] = name
        res['weights'] = (a, b, g)
        weight_results.append(res)
        print(
            f"{name:<19} ({a:.2f},{b:.2f},{g:.2f})   {res['result']:<10} "
            f"{res['duration_s']:8.2f} {res['path_len_m']:8.2f} "
            f"{res['min_obs_m']:10.3f} {res['avg_obs_m']:10.3f}"
        )

    # Plot weight sensitivity comparison
    fig, ax = plt.subplots(figsize=(7.5, 7.5))
    draw_maze(ax, boxes, xlim=(-2.0, 5.0), ylim=(-5.5, 2.0))
    ax.plot(0.0, 0.0, 'ks', markersize=8, label='Start (0, 0)')
    ax.scatter(
        [weight_goal[0]], [weight_goal[1]],
        color='red', marker='*', s=150, zorder=6, label='Goal G2',
    )
    w_colors = ['#000000', '#1f77b4', '#aec7e8', '#2ca02c', '#98df8a', '#d62728', '#ff9896']
    for res, col in zip(weight_results, w_colors):
        tr = res['traj']
        lw = 2.4 if res['name'] == 'Baseline' else 1.5
        lbl = f"{res['name']} [{res['result']}]"
        ax.plot(tr[:, 0], tr[:, 1], color=col, linewidth=lw, label=lbl)
    ax.set_title('Weight Sensitivity (Pure-Python Simulation, NOT Gazebo) on G2')
    ax.legend(loc='upper right', fontsize=8)
    weights_plot = os.path.join(args.out_dir, 'offline_weights.png')
    fig.tight_layout()
    fig.savefig(weights_plot, dpi=150)
    plt.close(fig)

    # Task 2 Dynamic Following on red-cube trajectory from hard_maze.world
    waypoints = load_goal_box_waypoints(args.world)
    follow_res = run_follow_episode(waypoints, boxes, base_dwa, base_scan, base_nav)
    print('\n--- Task 2 Dynamic Following on goal_box Trajectory (Pure Python Simulation) ---')
    print(
        f"Result: {follow_res['result']} | Duration: {follow_res['duration_s']:.1f} s | "
        f"Path: {follow_res['path_len_m']:.2f} m | MinObs: {follow_res['min_obs_m']:.3f} m | "
        f"AvgObs: {follow_res['avg_obs_m']:.3f} m"
    )
    print(
        f'Distance RMSE (opt={base_dwa.desired_target_dist:.2f} m): '
        f"{follow_res['dist_rmse_m']:.3f} m | "
        f"Bearing RMSE (opt=0): {follow_res['bearing_rmse_rad']:.3f} rad "
        f"({math.degrees(follow_res['bearing_rmse_rad']):.1f} deg) | "
        f"InRange(<=2m): {follow_res['in_range_ratio'] * 100:.1f}% | "
        f"Final Dist: {follow_res['final_dist_m']:.2f} m"
    )

    fig, ax = plt.subplots(figsize=(8, 8))
    draw_maze(ax, boxes, xlim=(-8.0, 5.0), ylim=(-5.5, 8.5))
    ax.plot(0.0, 0.0, 'ks', markersize=8, label='Robot Start (0, 0)')
    ax.plot(
        follow_res['target_traj'][:, 0], follow_res['target_traj'][:, 1],
        'r--', linewidth=1.8, label='Red Cube (goal_box)',
    )
    ax.plot(
        follow_res['traj'][:, 0], follow_res['traj'][:, 1],
        'b-', linewidth=1.8, label=f"Robot ({follow_res['result']})",
    )
    ax.set_title(
        'Task 2 Pure-Python Simulation (NOT Gazebo): Dynamic Following in hard_maze.world'
    )
    ax.legend(loc='upper right', fontsize=8)
    follow_plot = os.path.join(args.out_dir, 'offline_follow.png')
    fig.tight_layout()
    fig.savefig(follow_plot, dpi=150)
    plt.close(fig)

    # Benchmark compute_cmd
    med_ms, p95_ms, max_ms = benchmark_compute_cmd(base_dwa, n_iters=300)
    print('\n--- compute_cmd Latency Benchmark (300 iterations, 210 trajectories, 24 sectors) ---')
    print(
        f'Median: {med_ms:.3f} ms | P95: {p95_ms:.3f} ms | '
        f'Max: {max_ms:.3f} ms (15 Hz budget: 66.67 ms)'
    )
    print(f'\nSaved plots: {goals_plot}, {weights_plot}, {follow_plot}')


if __name__ == '__main__':
    main()
