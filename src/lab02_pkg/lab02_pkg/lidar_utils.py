"""LiDAR helpers. Pure Python: they only need an object shaped like sensor_msgs/LaserScan."""

import math


def sector_ranges(scan, center_deg, half_width_deg):
    """Usable ranges within +-half_width_deg of center_deg (0 = front, +90 = left).

    +inf means "no return within the sensor's range" and counts as range_max.
    NaN, -inf and values outside [range_min, range_max] are ignored
    (the real LDS-02 reports NaN where the beam hits nothing).
    """
    if not math.isfinite(scan.range_max) or scan.range_max <= scan.range_min:
        return []
    values = []
    for i, r in enumerate(scan.ranges):
        angle = math.degrees(scan.angle_min + i * scan.angle_increment)
        diff = (angle - center_deg + 180.0) % 360.0 - 180.0   # angle difference in [-180, 180)
        if abs(diff) <= half_width_deg:
            if math.isinf(r) and r > 0:
                values.append(scan.range_max)
            elif math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                values.append(r)
    return values


def sector_min(scan, center_deg, half_width_deg):
    """Closest obstacle in the sector, or None when the sector has no usable ray."""
    values = sector_ranges(scan, center_deg, half_width_deg)
    return min(values) if values else None


def sector_median(scan, center_deg, half_width_deg):
    """How open the sector is (median range), or None when the sector has no usable ray.

    Used to choose the turn side: one close corner does not hide a long free corridor.
    """
    values = sorted(sector_ranges(scan, center_deg, half_width_deg))
    return values[len(values) // 2] if values else None


def count_zero_rays(scan):
    return sum(1 for r in scan.ranges if r == 0.0)
