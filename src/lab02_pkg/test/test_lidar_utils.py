"""LiDAR helpers: invalid rays must never look like obstacles or free space."""

import math
from types import SimpleNamespace

import pytest

from lab02_pkg.lidar_utils import count_zero_rays, sector_median, sector_min


def make_scan(ranges, angle_min=-0.1, angle_increment=0.1):
    return SimpleNamespace(angle_min=angle_min, angle_increment=angle_increment,
                           range_min=0.12, range_max=3.5, ranges=ranges)


def test_invalid_rays_give_no_data():
    scan = make_scan([math.nan, -math.inf, 0.0])
    assert sector_min(scan, 0.0, 15.0) is None
    assert sector_median(scan, 0.0, 15.0) is None


def test_inf_counts_as_free_and_nan_is_ignored():
    scan = make_scan([math.inf, 0.4, math.nan])
    assert sector_min(scan, 0.0, 15.0) == pytest.approx(0.4)
    assert sector_median(scan, 0.0, 15.0) == pytest.approx(3.5)


def test_median_ignores_one_close_corner():
    # five rays around the left side: one corner at 0.3 m, the rest is a corridor
    scan = make_scan([0.3, 2.0, 2.1, 2.2, 2.3], angle_min=math.radians(80),
                     angle_increment=math.radians(5))
    assert sector_min(scan, 90.0, 30.0) == pytest.approx(0.3)
    assert sector_median(scan, 90.0, 30.0) == pytest.approx(2.1)


def test_sector_wraps_around_zero_degrees():
    # rays at 350 deg and 10 deg are both in the front sector
    scan = make_scan([1.0] * 36, angle_min=0.0, angle_increment=math.radians(10))
    scan.ranges[35] = 0.2      # 350 deg
    assert sector_min(scan, 0.0, 15.0) == pytest.approx(0.2)


def test_count_zero_rays():
    assert count_zero_rays(make_scan([0.0, 1.0, 0.0])) == 2
