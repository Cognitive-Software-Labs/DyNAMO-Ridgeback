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
# How far IN FRONT of the anchor a point may sit and still count as target.
# Nothing real is in front of the near surface, so this is not an extent: it
# covers the gap between the anchor *statistic* and the actual nearest target
# point, plus whatever depth noise puts a sample early. Both anchors sit behind
# the surface they name -- ``nearest_significant_mode`` returns a bin CENTRE,
# so half a bin (0.025 at the shipped 0.05 width) of its own anchor bin is
# nearer than it, and ``RangeBand``'s 25th percentile has a quarter of the
# whole set nearer by construction. Zero is therefore not "no slack", it is a
# truncation: it would discard the near half of the anchor bin, or a quarter of
# the points, every frame.
#
# 0.10 is sized for the percentile anchor, which has to reach back toward the
# minimum. The default recipe is mode-anchored and wants roughly the half-bin.
# At 0.05 bins, 0.10 reaches back TWO bins and so readmits points from bins
# that failed ``min_bin_fraction`` -- partially undoing the significance test
# that was the reason for moving off the percentile anchor. Left as shipped:
# that is a value judgement, and it is now sweepable as isolation_3d_ahead_m.
# Note sim cannot settle it -- the renderer has no noise model, so only the
# quantization half of the term is exercised there.
INLIER_AHEAD_MARGIN_M = 0.10

# How far behind the anchor a point may sit and still count as target. Its job
# is to span the target's own front-to-back extent, so geometry brackets it:
# the G1 measures 0.4457 m fore/aft square-on and 0.5749 m yawed, and the
# nearest thing that must stay out is the occluder at 0.62 m standoff. That
# puts the usable window at roughly 0.45-0.62 m and leaves 0.35 **below the
# entire bracket** -- it truncates the target's own far surface, which biases
# the centroid near on every trial rather than only on hard ones.
#
# Left at 0.35 regardless: the bracket is an argument from geometry, not a
# measurement, and this file's values move on measurements. It is now
# sweepable as isolation_3d_behind_m (67b648c) on the euclidean side, so the
# way to settle it is to measure it. Note the pointcloud row reads the same
# constant against forward-axis distance rather than euclidean range, so a
# change here moves both.
INLIER_BEHIND_MARGIN_M = 0.35
