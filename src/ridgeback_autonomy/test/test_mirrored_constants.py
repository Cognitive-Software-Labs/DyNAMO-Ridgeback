"""Cross-check the by-value 'mirrored constant' pairs (audit C10).

The mask stack (``perception/core``) deliberately never imports the legacy
estimator stack (``geometry.py``); shared tuning values are duplicated by value
instead, which keeps the two stacks decoupled. The cost is that editing one
side can silently diverge from the other. This test turns such a divergence
into a loud failure -- one cheap assertion per mirrored pair.
"""

from __future__ import annotations

from ridgeback_autonomy.perception.core import (
    depth_common,
    euclidean_reconstruction,
    geometry,
    isolation_2d,
    isolation_3d,
    polar_profiling,
    projective_ranging,
)


def test_lidar_clip_constants_mirror_geometry() -> None:
    assert polar_profiling.LIDAR_RANGE_MIN_M_DEFAULT == geometry.LIDAR_MIN_RANGE_METERS
    assert polar_profiling.LIDAR_RANGE_MAX_M_DEFAULT == geometry.LIDAR_MAX_METERS
    assert polar_profiling.MIN_VALID_RAYS_DEFAULT == geometry.LIDAR_MIN_VALID_RAYS


def test_depth_max_mirrors_pointcloud_max() -> None:
    assert depth_common.DEPTH_MAX_METERS_DEFAULT == geometry.POINTCLOUD_MAX_METERS


def test_range_band_mirrors_pointcloud_inlier_window() -> None:
    assert isolation_3d.RANGE_BAND_PERCENTILE_DEFAULT == geometry.POINTCLOUD_FRONT_PERCENTILE
    assert isolation_3d.RANGE_BAND_AHEAD_M_DEFAULT == geometry.POINTCLOUD_INLIER_AHEAD_MARGIN_M
    assert isolation_3d.RANGE_BAND_BEHIND_M_DEFAULT == geometry.POINTCLOUD_INLIER_BEHIND_MARGIN_M


def test_min_valid_counts_mirror_pointcloud_min_valid_points() -> None:
    assert projective_ranging.MIN_VALID_PIXELS_DEFAULT == geometry.POINTCLOUD_MIN_VALID_POINTS
    assert euclidean_reconstruction.MIN_VALID_POINTS_DEFAULT == geometry.POINTCLOUD_MIN_VALID_POINTS


def test_near_band_width_shared_between_isolation_2d_and_polar() -> None:
    assert isolation_2d.NEAREST_MODE_BAND_M_DEFAULT == polar_profiling.RANGE_BAND_M_DEFAULT
