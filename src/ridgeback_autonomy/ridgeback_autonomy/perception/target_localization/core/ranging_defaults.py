"""Shared ranging defaults, independent of any estimator implementation.

**Entry rule:** a constant belongs here only if two or more estimators have to
move *together* when it moves. Equal values are not enough, and neither is
similar wording -- the test is whether changing one reader without the other
would be a bug. Algorithm-specific settings (LiDAR validity clips, ROI
fractions, camera geometry) stay with their owners even when their numbers
coincide. Imports bind plain constants once; there is no per-frame
configuration layer.

That rule is load-bearing because this file attracts the opposite. Three
tenants have been evicted for failing it -- ``MAX_RANGE_M`` (79ccb8f, a planar
report clamp that only the pointcloud row applied), the pointcloud half of
``MIN_VALID_SAMPLES`` (84ad5be, raw cloud points rather than mask pixels), and
``NEAR_SURFACE_BAND_M``, whose two readers turned out to be a camera depth
window and a lidar run-merge distance. Each read as a shared policy for as long
as it sat here. ``test_shared_defaults.py`` pins the tenancy so the next one has
to be argued rather than imported.

Current tenants and why each earns it:

- ``MIN_VALID_SAMPLES`` -- projective ranging and euclidean reconstruction.
  Their pre-isolation guards count the same prepared ``valid_masked`` array, so
  one floor is what stops the two rows disagreeing about a selection they were
  handed jointly.
- ``FRONT_PERCENTILE``, ``INLIER_AHEAD_MARGIN_M``, ``INLIER_BEHIND_MARGIN_M`` --
  the pointcloud estimator and ``isolation_3d.RangeBand``, which is a port of
  that estimator's isolation kept so the benchmark can measure the two against
  each other. Split them and the port stops being one.
"""

MIN_VALID_SAMPLES = 10
FRONT_PERCENTILE = 25.0
INLIER_AHEAD_MARGIN_M = 0.10
INLIER_BEHIND_MARGIN_M = 0.35
