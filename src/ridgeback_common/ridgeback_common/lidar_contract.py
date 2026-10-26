"""Shared Ridgeback 2D LiDAR and merged-scan geometry.

The raw values are the UST-10LX ROS contract emitted by the Isaac point-cloud
assembler and by Gazebo's patched Clearpath description.  The merged range is
measured from ``base_link``, so it includes the largest planar sensor offset
in addition to the sensor-frame range.
"""
from __future__ import annotations

import math


RAW_SCAN_BINS = 1081
RAW_ANGLE_MIN = -3.0 * math.pi / 4.0
RAW_ANGLE_INCREMENT = math.radians(0.25)
RAW_RANGE_MIN = 0.06
RAW_RANGE_MAX = 10.0
RAW_SCAN_PERIOD = 1.0 / 40.0

# Gazebo's constant Gaussian range stddev, in metres. Isaac's UST-10LX noise is
# Gaussian, unbiased and independent per bin, but its stddev grows with range
# (about 0.0126 * r**0.80: 13 mm at 1 m, 38 mm at 4 m, 67 mm at 8 m), and
# Gazebo's lidar noise can only be constant. This is Isaac's value at the
# benchmark's median target distance, 3.7 m. Measured 2026-10-25; see
# archive/engineering/2026-10-25-gazebo-ust10lx-lidar.md.
GAZEBO_RANGE_NOISE_STDDEV = 0.036

FRONT_LIDAR_XY_YAW = (0.3922, 0.0, 0.0)
REAR_LIDAR_XY_YAW = (-0.3922, 0.0, math.pi)

MERGED_SCAN_BINS = int(round(2.0 * math.pi / RAW_ANGLE_INCREMENT))
MERGED_ANGLE_MIN = -math.pi
MERGED_ANGLE_MAX = (
    MERGED_ANGLE_MIN + (MERGED_SCAN_BINS - 1) * RAW_ANGLE_INCREMENT
)
MERGED_RANGE_PAD = max(
    math.hypot(*FRONT_LIDAR_XY_YAW[:2]),
    math.hypot(*REAR_LIDAR_XY_YAW[:2]),
)
MERGED_RANGE_MAX = RAW_RANGE_MAX + MERGED_RANGE_PAD


def slam_max_laser_range(slam_source: str) -> float:
    """Return slam_toolbox's range threshold for the selected scan frame."""
    if slam_source == 'front_only':
        return RAW_RANGE_MAX
    if slam_source == 'merged':
        return MERGED_RANGE_MAX
    raise ValueError(f'unsupported slam_source: {slam_source!r}')
