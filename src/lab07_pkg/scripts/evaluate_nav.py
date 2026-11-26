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
Evaluation and plotting script for Lab 07-08-09 rosbag2 recordings (Stage C).

Supports both simulation bags (--robot sim) and real-robot bags (--robot real).
Core metric computation and plotting are pure Python + NumPy + Matplotlib so
they can be unit-tested offline on macOS without ROS 2. Rosbag reading uses
rosbag2_py when executed inside the ROS 2 Humble Ubuntu environment.
"""

import argparse
from dataclasses import dataclass
import math
import os
import sys
from typing import Dict, List, Optional, Sequence, Tuple

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, '..'))
if PKG_ROOT not in sys.path:
    sys.path.insert(0, PKG_ROOT)

from lab07_pkg.dwa import normalize_angle  # noqa: E402
from lab07_pkg.navigation import landmark_to_odom, yaw_from_quaternion  # noqa: E402
from lab07_pkg.scan_utils import ScanConfig, clean_scan_ranges  # noqa: E402


@dataclass
class RunData:
    """Normalized time-series extracted from one rosbag run."""

    robot_times: np.ndarray        # shape (N,) [s]
    robot_poses: np.ndarray        # shape (N, 3) [x, y, yaw]
    cmd_times: np.ndarray          # shape (K,) [s]
    cmd_vels: np.ndarray           # shape (K, 2) [v, w]
    scan_min_dists: np.ndarray     # shape (S,) per-scan min center-to-obstacle distance [m]
    result: str = 'Unknown'        # 'Goal', 'Collision', 'Timeout', or 'Unknown'
    static_goal: Optional[Tuple[float, float]] = None
    target_times: Optional[np.ndarray] = None      # shape (M,) [s]
    target_positions: Optional[np.ndarray] = None  # shape (M, 2) [x, y] in same frame
    landmark_ranges: Optional[np.ndarray] = None
    landmark_bearings: Optional[np.ndarray] = None
    start_time: Optional[float] = None             # nav start (goal/motion start) [s]
    end_time: Optional[float] = None               # nav end (/nav_result or end) [s]


def align_odom_to_origin(poses: np.ndarray) -> np.ndarray:
    """Align /odom trajectory to origin by subtracting the first pose (x0, y0, yaw0)."""
    poses = np.asarray(poses, dtype=np.float64)
    if poses.size == 0:
        return poses.copy()
    x0, y0, yaw0 = float(poses[0, 0]), float(poses[0, 1]), float(poses[0, 2])
    dx = poses[:, 0] - x0
    dy = poses[:, 1] - y0
    c = math.cos(-yaw0)
    s = math.sin(-yaw0)
    aligned = np.empty_like(poses)
    aligned[:, 0] = dx * c - dy * s
    aligned[:, 1] = dx * s + dy * c
    aligned[:, 2] = normalize_angle(poses[:, 2] - yaw0)
    return aligned


def align_point_to_origin(
    point_xy: Sequence[float],
    first_pose: Sequence[float],
) -> Tuple[float, float]:
    """Transform a 2D point from raw /odom frame into the first-pose-aligned frame."""
    x0, y0, yaw0 = float(first_pose[0]), float(first_pose[1]), float(first_pose[2])
    dx = float(point_xy[0]) - x0
    dy = float(point_xy[1]) - y0
    c = math.cos(-yaw0)
    s = math.sin(-yaw0)
    return dx * c - dy * s, dx * s + dy * c


def nearest_indices(source_times: np.ndarray, query_times: np.ndarray) -> np.ndarray:
    """Return index of closest timestamp in sorted source_times for each query_time."""
    src = np.asarray(source_times, dtype=np.float64)
    qry = np.asarray(query_times, dtype=np.float64)
    if src.size == 0:
        return np.empty(0, dtype=np.int64)
    right = np.searchsorted(src, qry, side='left')
    right = np.clip(right, 0, src.size - 1)
    left = np.clip(right - 1, 0, src.size - 1)
    use_left = np.abs(qry - src[left]) <= np.abs(src[right] - qry)
    return np.where(use_left, left, right)


def scan_min_center_distance(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    scan_cfg: ScanConfig,
) -> float:
    """
    Compute minimum obstacle distance from robot center, ignoring NaNs.

    Always ignores NaN values (even if runtime uses nan_policy='min') so NaNs
    never distort obstacle distance statistics, and accounts for LiDAR mounting
    offset to match the safety-stop measurement convention.
    """
    eval_cfg = ScanConfig(
        max_scan_range=scan_cfg.max_scan_range,
        min_valid_range=scan_cfg.min_valid_range,
        num_ranges=scan_cfg.num_ranges,
        nan_policy='max',
        laser_offset_x=scan_cfg.laser_offset_x,
        laser_offset_y=scan_cfg.laser_offset_y,
        laser_offset_yaw=scan_cfg.laser_offset_yaw,
    )
    vr, va, eff_max = clean_scan_ranges(
        ranges, angle_min, angle_increment, range_min, range_max, eval_cfg
    )
    obs_mask = vr < (eff_max - 1e-9)
    if not np.any(obs_mask):
        return float('inf')
    r = vr[obs_mask]
    theta = va[obs_mask] + eval_cfg.laser_offset_yaw
    bx = eval_cfg.laser_offset_x + r * np.cos(theta)
    by = eval_cfg.laser_offset_y + r * np.sin(theta)
    return float(np.min(np.hypot(bx, by)))


def reconstruct_landmark_trajectory(
    robot_times: np.ndarray,
    robot_poses: np.ndarray,
    landmark_times: np.ndarray,
    landmark_ranges: np.ndarray,
    landmark_bearings: np.ndarray,
    camera_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> np.ndarray:
    """Project (range, bearing) detections onto 2D target positions using nearest robot pose."""
    if landmark_times.size == 0 or robot_times.size == 0:
        return np.empty((0, 2), dtype=np.float64)
    idx = nearest_indices(robot_times, landmark_times)
    pts = [
        landmark_to_odom(robot_poses[i], r, b, camera_offset)
        for i, r, b in zip(idx, landmark_ranges, landmark_bearings)
    ]
    return np.asarray(pts, dtype=np.float64).reshape(-1, 2)


def compute_metrics(
    run: RunData,
    desired_target_dist: float = 0.8,
    follow_dist_threshold: float = 2.0,
) -> Dict[str, float]:
    """Compute all Lab 07-08-09 report metrics for a single run."""
    metrics: Dict[str, float] = {}

    if run.start_time is not None and run.end_time is not None:
        metrics['duration_s'] = max(0.0, float(run.end_time - run.start_time))
    elif run.robot_times.size >= 2:
        metrics['duration_s'] = float(run.robot_times[-1] - run.robot_times[0])
    else:
        metrics['duration_s'] = 0.0

    # Slice robot poses to [start_time, end_time] when navigation window is known
    poses = run.robot_poses
    times = run.robot_times
    if run.start_time is not None and run.end_time is not None and times.size > 0:
        mask = (times >= run.start_time) & (times <= run.end_time)
        if np.sum(mask) >= 2:
            poses = poses[mask]
            times = times[mask]

    if poses.shape[0] >= 2:
        diffs = np.diff(poses[:, :2], axis=0)
        metrics['path_length_m'] = float(np.sum(np.hypot(diffs[:, 0], diffs[:, 1])))
    else:
        metrics['path_length_m'] = 0.0

    valid_scans = run.scan_min_dists[np.isfinite(run.scan_min_dists)]
    if valid_scans.size > 0:
        metrics['min_obstacle_dist_m'] = float(np.min(valid_scans))
        metrics['avg_obstacle_dist_m'] = float(np.mean(valid_scans))
    else:
        metrics['min_obstacle_dist_m'] = float('inf')
        metrics['avg_obstacle_dist_m'] = float('inf')

    # Task 2 RMSE and tracking quality metrics
    dists = None
    if (
        run.landmark_ranges is not None
        and run.landmark_bearings is not None
        and run.landmark_ranges.size > 0
    ):
        dists = run.landmark_ranges
        dist_err = dists - float(desired_target_dist)
        bearing_err = normalize_angle(run.landmark_bearings)
        metrics['target_dist_rmse_m'] = float(np.sqrt(np.mean(dist_err ** 2)))
        metrics['target_bearing_rmse_rad'] = float(np.sqrt(np.mean(bearing_err ** 2)))
    elif (
        run.target_times is not None
        and run.target_positions is not None
        and run.target_times.size > 0
        and times.size > 0
    ):
        idx = nearest_indices(run.target_times, times)
        tx = run.target_positions[idx, 0]
        ty = run.target_positions[idx, 1]
        dx = tx - poses[:, 0]
        dy = ty - poses[:, 1]
        dists = np.hypot(dx, dy)
        bearings = normalize_angle(np.arctan2(dy, dx) - poses[:, 2])
        metrics['target_dist_rmse_m'] = float(np.sqrt(np.mean((dists - desired_target_dist) ** 2)))
        metrics['target_bearing_rmse_rad'] = float(np.sqrt(np.mean(bearings ** 2)))

    if dists is not None and dists.size > 0:
        metrics['in_range_ratio'] = float(np.mean(dists <= float(follow_dist_threshold)))
        tail_n = max(1, int(math.ceil(0.1 * dists.size)))
        metrics['final_target_dist_m'] = float(np.mean(dists[-tail_n:]))
    else:
        metrics['in_range_ratio'] = 0.0
        metrics['final_target_dist_m'] = float('inf')

    return metrics


def summarize_batch(
    runs: Sequence[RunData],
    desired_target_dist: float = 0.8,
    mode: str = 'goal',
    follow_dist_threshold: float = 2.0,
    min_in_range_ratio: float = 0.70,
    max_final_dist_m: float = 2.0,
    min_follow_path_m: float = 1.0,
    min_follow_duration_s: float = 5.0,
) -> Dict:
    """
    Aggregate success rate, failure types, and per-run metrics across multiple runs.

    In 'goal' mode, success requires result == 'Goal'. In 'follow' mode,
    success requires no Collision or Timeout, sufficient movement and duration,
    distance <= follow_dist_threshold for at least min_in_range_ratio of
    samples, and final distance <= max_final_dist_m; otherwise labeled 'Lost'.
    """
    per_run = [
        compute_metrics(r, desired_target_dist, follow_dist_threshold) for r in runs
    ]
    n = len(runs)
    n_success = 0
    failures: Dict[str, int] = {}

    for r, m in zip(runs, per_run):
        if mode == 'follow':
            if r.result in ('Collision', 'Timeout'):
                failures[r.result] = failures.get(r.result, 0) + 1
                continue
            tracked = (
                m['duration_s'] >= min_follow_duration_s
                and m['path_length_m'] >= min_follow_path_m
                and m['in_range_ratio'] >= min_in_range_ratio
                and m['final_target_dist_m'] <= max_final_dist_m
            )
            if tracked:
                n_success += 1
            else:
                failures['Lost'] = failures.get('Lost', 0) + 1
        else:
            if r.result == 'Goal':
                n_success += 1
            else:
                failures[r.result] = failures.get(r.result, 0) + 1

    return {
        'num_runs': n,
        'success_rate': (float(n_success) / n) if n > 0 else 0.0,
        'failures': failures,
        'per_run': per_run,
    }


def plot_run(
    run: RunData, out_prefix: str, title_prefix: str = 'Navigation Run'
) -> Tuple[str, str]:
    """Generate 2D trajectory plot and (v, w) velocity profile plot."""
    fig, ax = plt.subplots(figsize=(6.5, 6.5))
    if run.robot_poses.size > 0:
        ax.plot(
            run.robot_poses[:, 0], run.robot_poses[:, 1],
            'b-', linewidth=1.8, label='Robot path',
        )
        ax.plot(
            run.robot_poses[0, 0], run.robot_poses[0, 1], 'gs', markersize=7, label='Start'
        )
        ax.plot(
            run.robot_poses[-1, 0], run.robot_poses[-1, 1], 'bo', markersize=6, label='End'
        )
    if run.static_goal is not None:
        ax.scatter(
            [run.static_goal[0]], [run.static_goal[1]],
            c='r', marker='*', s=140, zorder=5, label='Goal',
        )
    if run.target_positions is not None and run.target_positions.size > 0:
        ax.plot(
            run.target_positions[:, 0], run.target_positions[:, 1],
            'r--', linewidth=1.5, label='Target trajectory',
        )
    ax.set_aspect('equal')
    ax.grid(True, linestyle=':', alpha=0.5)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title(f'{title_prefix}: 2D Trajectory ({run.result})')
    ax.legend(loc='best')
    traj_path = f'{out_prefix}_trajectory.png'
    fig.tight_layout()
    fig.savefig(traj_path, dpi=150)
    plt.close(fig)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7.5, 5.0), sharex=True)
    if run.cmd_times.size > 0:
        t0 = run.start_time if run.start_time is not None else run.cmd_times[0]
        t_rel = run.cmd_times - t0
        ax1.plot(t_rel, run.cmd_vels[:, 0], 'b-', linewidth=1.4)
        ax2.plot(t_rel, run.cmd_vels[:, 1], 'r-', linewidth=1.4)
    ax1.set_ylabel('v [m/s]')
    ax1.grid(True, linestyle=':', alpha=0.5)
    ax1.set_title(f'{title_prefix}: Velocity Commands (v, w)')
    ax2.set_ylabel('w [rad/s]')
    ax2.set_xlabel('Time [s]')
    ax2.grid(True, linestyle=':', alpha=0.5)
    vel_path = f'{out_prefix}_velocities.png'
    fig.tight_layout()
    fig.savefig(vel_path, dpi=150)
    plt.close(fig)

    return traj_path, vel_path


def read_rosbag2(
    bag_path: str,
    robot: str = 'sim',
    static_goal: Optional[Tuple[float, float]] = None,
    target_id: int = -1,
    laser_offset_x: float = -0.032,
) -> RunData:
    """Read a ROS 2 Humble rosbag2 directory using rosbag2_py + rclpy.serialization."""
    try:
        from rclpy.serialization import deserialize_message
        import rosbag2_py
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError(
            'rosbag2_py / rclpy not available in this Python environment. '
            'Run evaluate_nav.py inside the ROS 2 Humble Ubuntu VM.'
        ) from exc

    scan_cfg = ScanConfig(nan_policy='max', laser_offset_x=laser_offset_x)

    reader = rosbag2_py.SequentialReader()
    storage_opts = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_opts = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader.open(storage_opts, converter_opts)

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    pose_topic = '/ground_truth' if robot == 'sim' else '/odom'

    robot_times: List[float] = []
    robot_poses: List[Tuple[float, float, float]] = []
    cmd_times: List[float] = []
    cmd_vels: List[Tuple[float, float]] = []
    scan_mins: List[float] = []
    target_times: List[float] = []
    target_positions: List[Tuple[float, float]] = []
    lm_times: List[float] = []
    lm_ranges: List[float] = []
    lm_bearings: List[float] = []
    result = 'Unknown'
    goal_from_bag: Optional[Tuple[float, float]] = None
    start_time: Optional[float] = None
    end_time: Optional[float] = None

    msg_cls_cache = {}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        t_sec = t_ns * 1e-9
        if topic not in topic_types:
            continue
        type_str = topic_types[topic]
        if type_str not in msg_cls_cache:
            msg_cls_cache[type_str] = get_message(type_str)
        msg = deserialize_message(data, msg_cls_cache[type_str])

        if topic == pose_topic:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            robot_times.append(t_sec)
            robot_poses.append((float(p.x), float(p.y), yaw_from_quaternion(q.x, q.y, q.z, q.w)))
        elif topic == '/cmd_vel':
            v_lin, w_ang = float(msg.linear.x), float(msg.angular.z)
            cmd_times.append(t_sec)
            cmd_vels.append((v_lin, w_ang))
            if start_time is None and (abs(v_lin) > 1e-4 or abs(w_ang) > 1e-4):
                start_time = t_sec
        elif topic == '/goal_pose':
            p = msg.pose.position
            goal_from_bag = (float(p.x), float(p.y))
            if start_time is None:
                start_time = t_sec
        elif topic == '/scan':
            d_min = scan_min_center_distance(
                msg.ranges, msg.angle_min, msg.angle_increment,
                msg.range_min, msg.range_max, scan_cfg,
            )
            if math.isfinite(d_min):
                scan_mins.append(d_min)
        elif topic == '/dynamic_goal_pose':
            p = msg.pose.pose.position
            target_times.append(t_sec)
            target_positions.append((float(p.x), float(p.y)))
        elif topic == '/camera/landmarks':
            valid = [lm for lm in msg.landmarks if math.isfinite(lm.range) and lm.range > 0.0]
            if target_id >= 0:
                valid = [lm for lm in valid if int(lm.id) == target_id]
            if valid:
                nearest = min(valid, key=lambda lm: lm.range)
                lm_times.append(t_sec)
                lm_ranges.append(float(nearest.range))
                lm_bearings.append(float(nearest.bearing))
        elif topic == '/nav_result':
            result = str(msg.data)
            if end_time is None:
                end_time = t_sec

    poses_arr = np.asarray(robot_poses, dtype=np.float64).reshape(-1, 3)
    times_arr = np.asarray(robot_times, dtype=np.float64)
    chosen_goal = static_goal if static_goal is not None else goal_from_bag

    if robot == 'real' and poses_arr.size > 0:
        first_pose = poses_arr[0].copy()
        poses_arr = align_odom_to_origin(poses_arr)
        if chosen_goal is not None:
            chosen_goal = align_point_to_origin(chosen_goal, first_pose)

    if end_time is None and times_arr.size > 0:
        end_time = float(times_arr[-1])
    if start_time is None and times_arr.size > 0:
        start_time = float(times_arr[0])

    # Reconstruct target trajectory from /camera/landmarks for real-robot follow plots
    lm_t_arr = np.asarray(lm_times, dtype=np.float64)
    lm_r_arr = np.asarray(lm_ranges, dtype=np.float64)
    lm_b_arr = np.asarray(lm_bearings, dtype=np.float64)
    if target_positions:
        t_pos_arr = np.asarray(target_positions, dtype=np.float64).reshape(-1, 2)
        t_time_arr = np.asarray(target_times, dtype=np.float64)
    elif lm_t_arr.size > 0 and times_arr.size > 0:
        t_pos_arr = reconstruct_landmark_trajectory(
            times_arr, poses_arr, lm_t_arr, lm_r_arr, lm_b_arr
        )
        t_time_arr = lm_t_arr
    else:
        t_pos_arr = None
        t_time_arr = None

    return RunData(
        robot_times=times_arr,
        robot_poses=poses_arr,
        cmd_times=np.asarray(cmd_times, dtype=np.float64),
        cmd_vels=np.asarray(cmd_vels, dtype=np.float64).reshape(-1, 2),
        scan_min_dists=np.asarray(scan_mins, dtype=np.float64),
        result=result,
        static_goal=chosen_goal,
        target_times=t_time_arr,
        target_positions=t_pos_arr,
        landmark_ranges=lm_r_arr if lm_r_arr.size > 0 else None,
        landmark_bearings=lm_b_arr if lm_b_arr.size > 0 else None,
        start_time=start_time,
        end_time=end_time,
    )


def main() -> None:
    """Parse CLI arguments and evaluate one or more rosbag2 directories."""
    parser = argparse.ArgumentParser(description='Evaluate ROS 2 navigation bags for Lab 07-08-09')
    parser.add_argument('bags', nargs='+', help='Paths to rosbag2 directories')
    parser.add_argument('--robot', choices=('sim', 'real'), default='sim')
    parser.add_argument('--mode', choices=('goal', 'follow'), default='goal')
    parser.add_argument('--goal', type=float, nargs=2, default=None, help='Static goal x y [m]')
    parser.add_argument('--target-id', type=int, default=-1, help='Filter landmarks by tag ID')
    parser.add_argument('--desired-dist', type=float, default=0.8, help='Optimal distance [m]')
    parser.add_argument('--out-dir', default='eval_plots', help='Output directory for plots')
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    static_goal = (args.goal[0], args.goal[1]) if args.goal else None

    runs = []
    for i, bag in enumerate(args.bags):
        run = read_rosbag2(
            bag, robot=args.robot, static_goal=static_goal, target_id=args.target_id
        )
        runs.append(run)
        prefix = os.path.join(args.out_dir, f'run_{i + 1}')
        plot_run(run, prefix, title_prefix=f'{args.robot} {args.mode} run {i + 1}')

    summary = summarize_batch(runs, desired_target_dist=args.desired_dist, mode=args.mode)
    print(
        f"Runs: {summary['num_runs']} | Mode: {args.mode} | "
        f"Success Rate: {summary['success_rate'] * 100:.1f}% | Failures: {summary['failures']}"
    )
    for idx, m in enumerate(summary['per_run'], start=1):
        print(f'Run {idx}: {m}')


if __name__ == '__main__':
    main()
