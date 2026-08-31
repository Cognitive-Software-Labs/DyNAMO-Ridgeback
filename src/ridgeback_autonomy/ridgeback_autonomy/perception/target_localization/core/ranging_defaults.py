"""Shared ranging defaults, independent of any estimator implementation.

Only defaults intentionally kept equal across paths belong here. Algorithm-
specific settings (LiDAR validity clips, ROI fractions, camera geometry, etc.)
remain with their owners even when their numeric values happen to coincide.
Imports bind plain constants once; there is no per-frame configuration layer.
"""

MAX_RANGE_M = 10.0
MIN_VALID_SAMPLES = 10
FRONT_PERCENTILE = 25.0
INLIER_AHEAD_MARGIN_M = 0.10
INLIER_BEHIND_MARGIN_M = 0.35
NEAR_SURFACE_BAND_M = 0.35
