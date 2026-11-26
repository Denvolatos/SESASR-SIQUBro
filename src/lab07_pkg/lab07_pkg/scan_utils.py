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
LiDAR scan filtering, sector downsampling, coordinate conversion, and safety stop.

Pure Python + NumPy (no rclpy dependency).
"""

from dataclasses import dataclass
import math
from typing import Sequence, Tuple

import numpy as np


@dataclass
class ScanConfig:
    """Configuration for LiDAR scan processing."""

    max_scan_range: float = 3.5       # [m] ignore obstacles beyond this distance
    min_valid_range: float = 0.05     # [m] ignore zero / near-zero irregular rays
    num_ranges: int = 24              # [12..30] angular sectors
    nan_policy: str = 'min'           # 'min' (sim/PDF) or 'max' (real LDS-02)
    laser_offset_x: float = -0.032    # [m] LiDAR x offset in base frame
    laser_offset_y: float = 0.0       # [m] LiDAR y offset in base frame
    laser_offset_yaw: float = 0.0     # [rad] LiDAR yaw offset in base frame
    min_valid_ratio: float = 0.20     # minimum fraction of valid rays to trust scan
    max_nan_ratio: float = 0.95       # maximum fraction of NaN rays before flagging failure
    scan_median_filter: bool = True   # 3-point circular median filter against single-ray spikes

    def __post_init__(self) -> None:
        if self.nan_policy not in ('min', 'max'):
            raise ValueError(f"nan_policy must be 'min' or 'max', got '{self.nan_policy}'")
        if not (12 <= self.num_ranges <= 30):
            raise ValueError(f'num_ranges must be in [12, 30], got {self.num_ranges}')
        if self.max_scan_range <= 0.0:
            raise ValueError(f'max_scan_range must be positive, got {self.max_scan_range}')
        if self.min_valid_range < 0.0:
            raise ValueError(f'min_valid_range must be non-negative, got {self.min_valid_range}')
        if not (0.0 <= self.min_valid_ratio <= 1.0):
            raise ValueError(f'min_valid_ratio must be in [0, 1], got {self.min_valid_ratio}')
        if not (0.0 <= self.max_nan_ratio <= 1.0):
            raise ValueError(f'max_nan_ratio must be in [0, 1], got {self.max_nan_ratio}')


def _effective_bounds(
    range_min: float,
    range_max: float,
    config: ScanConfig,
) -> Tuple[float, float]:
    """Compute effective [min, max] range bounds respecting sensor and config limits."""
    eff_min = float(config.min_valid_range)
    if math.isfinite(range_min) and range_min > eff_min:
        eff_min = float(range_min)

    eff_max = float(config.max_scan_range)
    if math.isfinite(range_max) and range_max > 0.0:
        eff_max = min(eff_max, float(range_max))

    return eff_min, eff_max


def median3_filter(ranges: Sequence[float]) -> np.ndarray:
    """
    Apply a 3-point circular median filter across a 360-degree LiDAR scan.

    Each finite ray takes the median of itself and its immediate left/right
    circular neighbors (first and last rays wrap around). Non-finite values
    (NaN, +inf, -inf) are excluded from the median and preserved unchanged; a
    finite ray whose two neighbors are both non-finite keeps its original
    value. When only one neighbor is finite (e.g. a wall edge next to +inf),
    the upper median (max of the two finite rays) rejects single-ray inward
    spikes such as [0.435, 0.293, inf] -> 0.435.

    Trade-off: any physical obstacle spanning only a single ray width is
    filtered out (~8 mm wide at 0.3 m range, ~2.7 cm at 1.0 m for a ~230-ray
    360-degree scan), so arena boxes, walls, and human legs are unaffected.
    """
    raw = np.asarray(ranges, dtype=np.float64)
    n = raw.shape[0]
    if n < 3:
        return raw.copy()

    left = np.roll(raw, 1)
    right = np.roll(raw, -1)
    c_fin = np.isfinite(raw)
    l_fin = np.isfinite(left)
    r_fin = np.isfinite(right)

    out = raw.copy()

    # All 3 rays finite -> exact 3-point median
    m3 = c_fin & l_fin & r_fin
    if np.any(m3):
        out[m3] = np.median(np.vstack((left[m3], raw[m3], right[m3])), axis=0)

    # Center + exactly one finite neighbor -> upper median (max of the 2 finite rays)
    m2_l = c_fin & l_fin & (~r_fin)
    if np.any(m2_l):
        out[m2_l] = np.maximum(left[m2_l], raw[m2_l])

    m2_r = c_fin & (~l_fin) & r_fin
    if np.any(m2_r):
        out[m2_r] = np.maximum(raw[m2_r], right[m2_r])

    return out


def clean_scan_ranges(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    config: ScanConfig,
) -> Tuple[np.ndarray, np.ndarray, float]:
    """
    Clean raw LiDAR ranges and compute exact ray angles.

    When config.scan_median_filter is True, applies median3_filter before
    cleaning so both check_safety_stop and scan_to_obstacles reject single-ray
    spikes. Ray angles are computed from angle_min + i * angle_increment.
    Positive inf is mapped to effective max range, NaN follows nan_policy
    ('min' or 'max'), irregular values below eff_min (such as 0.0) are
    dropped, and finite values above eff_max are clamped. Returns
    (valid_ranges, valid_angles, eff_max).
    """
    raw = np.asarray(ranges, dtype=np.float64)
    n = raw.shape[0]
    eff_min, eff_max = _effective_bounds(range_min, range_max, config)

    if n == 0 or eff_max <= eff_min:
        return np.empty(0, dtype=np.float64), np.empty(0, dtype=np.float64), eff_max

    if config.scan_median_filter:
        raw = median3_filter(raw)

    angles = float(angle_min) + np.arange(n, dtype=np.float64) * float(angle_increment)
    cleaned = raw.copy()

    # Handle NaN according to nan_policy
    nan_mask = np.isnan(cleaned)
    if np.any(nan_mask):
        cleaned[nan_mask] = eff_min if config.nan_policy == 'min' else eff_max

    # Handle +inf as free space at eff_max
    pos_inf_mask = np.isposinf(cleaned)
    if np.any(pos_inf_mask):
        cleaned[pos_inf_mask] = eff_max

    # Drop -inf and irregular values below eff_min (e.g. 0.0 on LDS-02)
    valid_mask = np.isfinite(cleaned) & (cleaned >= eff_min)
    valid_ranges = np.minimum(cleaned[valid_mask], eff_max)
    valid_angles = angles[valid_mask]

    return valid_ranges, valid_angles, eff_max


def check_scan_health(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    config: ScanConfig,
) -> Tuple[bool, str]:
    """
    Verify the LiDAR scan has enough valid beams and is not saturated by NaNs.

    Under nan_policy='max', an all-NaN scan is indistinguishable from
    completely open space unless bounded by max_nan_ratio.
    """
    raw = np.asarray(ranges, dtype=np.float64)
    n = raw.shape[0]
    if n == 0:
        return False, 'Empty LiDAR scan; stopping robot'

    nan_ratio = float(np.sum(np.isnan(raw))) / float(n)
    if nan_ratio > config.max_nan_ratio:
        return (
            False,
            f'LiDAR NaN ratio {nan_ratio:.2f} exceeds max_nan_ratio {config.max_nan_ratio:.2f}; '
            'stopping robot',
        )

    valid_ranges, _, _ = clean_scan_ranges(
        raw, angle_min, angle_increment, range_min, range_max, config
    )
    valid_ratio = float(valid_ranges.size) / float(n)
    if valid_ratio < config.min_valid_ratio:
        return (
            False,
            f'Too few valid LiDAR rays ({valid_ratio:.2f} < {config.min_valid_ratio:.2f}); '
            'stopping robot',
        )

    return True, ''


def downsample_scan(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    config: ScanConfig,
) -> Tuple[np.ndarray, np.ndarray, float]:
    """
    Compress LiDAR scan into config.num_ranges angular sectors.

    For each sector, selects the minimum valid range and preserves the true ray
    angle of that minimum measurement. Returns (sector_ranges, sector_angles,
    eff_max).
    """
    n_total = len(ranges)
    valid_ranges, valid_angles, eff_max = clean_scan_ranges(
        ranges, angle_min, angle_increment, range_min, range_max, config
    )
    if valid_ranges.size == 0 or n_total == 0 or angle_increment == 0.0:
        return np.empty(0, dtype=np.float64), np.empty(0, dtype=np.float64), eff_max

    total_span = abs(float(angle_increment)) * n_total
    sector_width = total_span / float(config.num_ranges)
    if sector_width <= 0.0:
        return np.empty(0, dtype=np.float64), np.empty(0, dtype=np.float64), eff_max

    # Map each valid ray angle into its sector index [0, num_ranges - 1]
    if angle_increment > 0.0:
        rel_angles = valid_angles - float(angle_min)
    else:
        rel_angles = float(angle_min) - valid_angles

    sector_ids = np.floor(rel_angles / sector_width).astype(np.int64)
    sector_ids = np.clip(sector_ids, 0, config.num_ranges - 1)

    sec_ranges = []
    sec_angles = []
    for sec in range(config.num_ranges):
        mask = sector_ids == sec
        if not np.any(mask):
            continue
        r_sec = valid_ranges[mask]
        a_sec = valid_angles[mask]
        min_idx = int(np.argmin(r_sec))
        sec_ranges.append(float(r_sec[min_idx]))
        sec_angles.append(float(a_sec[min_idx]))

    return (
        np.asarray(sec_ranges, dtype=np.float64),
        np.asarray(sec_angles, dtype=np.float64),
        eff_max,
    )


def scan_to_obstacles(
    robot_pose: Sequence[float],
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    config: ScanConfig,
) -> np.ndarray:
    """
    Convert raw LiDAR scan into downsampled [x, y] obstacle points in odom frame.

    Rays at eff_max (free space or beyond max_scan_range) are excluded so they
    do not create artificial obstacles at the range boundary. Returns a float64
    array of shape (M, 2) in the same frame as robot_pose.
    """
    sec_ranges, sec_angles, eff_max = downsample_scan(
        ranges, angle_min, angle_increment, range_min, range_max, config
    )
    if sec_ranges.size == 0:
        return np.empty((0, 2), dtype=np.float64)

    # Keep only actual obstacle returns strictly within effective max range
    obs_mask = sec_ranges < (eff_max - 1e-9)
    if not np.any(obs_mask):
        return np.empty((0, 2), dtype=np.float64)

    r = sec_ranges[obs_mask]
    theta = sec_angles[obs_mask] + config.laser_offset_yaw

    # Obstacle coordinates in robot base frame
    bx = config.laser_offset_x + r * np.cos(theta)
    by = config.laser_offset_y + r * np.sin(theta)

    # Transform to robot_pose frame (odom)
    rx, ry, ryaw = float(robot_pose[0]), float(robot_pose[1]), float(robot_pose[2])
    c = math.cos(ryaw)
    s = math.sin(ryaw)
    ox = rx + bx * c - by * s
    oy = ry + bx * s + by * c

    return np.column_stack((ox, oy))


def check_safety_stop(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    robot_radius: float,
    collision_tol: float,
    config: ScanConfig,
) -> Tuple[bool, float]:
    """
    Run independent laser-based safety stop check.

    Measures obstacle distance from the robot center (accounting for LiDAR
    offset) and compares against (robot_radius + collision_tol). Returns
    (is_collision, min_center_dist).
    """
    valid_ranges, valid_angles, eff_max = clean_scan_ranges(
        ranges, angle_min, angle_increment, range_min, range_max, config
    )
    if valid_ranges.size == 0:
        return False, float('inf')

    obs_mask = valid_ranges < (eff_max - 1e-9)
    if not np.any(obs_mask):
        return False, float('inf')

    r = valid_ranges[obs_mask]
    theta = valid_angles[obs_mask] + config.laser_offset_yaw
    bx = config.laser_offset_x + r * np.cos(theta)
    by = config.laser_offset_y + r * np.sin(theta)
    center_dists = np.hypot(bx, by)
    min_dist = float(np.min(center_dists))

    threshold = float(robot_radius) + float(collision_tol)
    return min_dist < threshold, min_dist
