"""Shared ranging defaults, independent of any estimator implementation.

Only defaults intentionally kept equal across paths belong here. Algorithm-
specific settings (LiDAR validity clips, ROI fractions, camera geometry, etc.)
remain with their owners even when their numeric values happen to coincide.
Imports bind plain constants once; there is no per-frame configuration layer.
"""

# The two mask depth rows' shared sufficiency floor, and shared on purpose:
# their pre-isolation guards count the same prepared ``valid_masked`` array, so
# one floor is what keeps them from disagreeing about a selection they were
# handed jointly. The pointcloud estimator's identically valued floor is a
# different quantity and lives with that estimator.
MIN_VALID_SAMPLES = 10
FRONT_PERCENTILE = 25.0
INLIER_AHEAD_MARGIN_M = 0.10
INLIER_BEHIND_MARGIN_M = 0.35
NEAR_SURFACE_BAND_M = 0.35
