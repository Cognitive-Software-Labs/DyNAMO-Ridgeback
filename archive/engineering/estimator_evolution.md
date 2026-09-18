# Estimator evolution

Recorded dates: unknown

Tested revisions: unknown

Provenance: partial

This is dated evidence. Missing revisions or preserved worktree inputs limit
reproducibility; no new measurements were made during archive curation.

These historical transitions distinguish deleted implementations and old
measurements. The timeline's exact dates and tested worktrees were not preserved;
the dated measurements below identify their own scope.

## Timeline

1. A camera node produced `rgb`, `sensor_depth`, `depth_anything`, and
   `pointcloud`; a separate LiDAR node produced `lidar`.
2. Mask representation, aligned-depth handling, and three mask-based estimators
   were added alongside that stack: `projective_ranging`,
   `euclidean_reconstruction`, and `polar_profiling`.
3. Benchmarking exposed that the older rows mixed source choice, selection, and
   ranging policy and could not share the mask contract cleanly.
4. Commit `8640635f95c4e55788195977908d6afb6ca4872f` removed `rgb`, `sensor_depth`, `depth_anything`, and `lidar`
   from the public registry, extracted the surviving pointcloud algorithm, and
   left four operational estimators: `pointcloud` plus the three mask rows.
5. Depth-Anything remained supported as a depth *source* for the depth-based
   estimators instead of being reported as a separate ranging algorithm.

The final pre-removal implementations can be inspected directly:

```bash
git show d96b2f0ed5a6ffe009457a74d017e7a7ae103b79:src/ridgeback_autonomy/ridgeback_autonomy/perception/core/geometry.py
git show d96b2f0ed5a6ffe009457a74d017e7a7ae103b79:src/ridgeback_autonomy/ridgeback_autonomy/perception/core/depth_anything.py
git show d96b2f0ed5a6ffe009457a74d017e7a7ae103b79:src/ridgeback_autonomy/ridgeback_autonomy/perception/g1_camera_measurement_node.py
git show d96b2f0ed5a6ffe009457a74d017e7a7ae103b79:src/ridgeback_autonomy/ridgeback_autonomy/perception/g1_lidar_measurement_node.py
```

These paths and `g1` names are historical. They are included so the algorithmic
steps remain recoverable without pretending the deleted modules are supported API.

## Removed estimator mechanisms

The original RGB path intersected an image ray with an assumed ground plane;
sensor-depth and monocular paths reduced box depth without the later mask
representation. The original organized-cloud row used a percentile surface
anchor, and the LiDAR box window admitted background through the target's leg
gap. These mechanisms motivated separating mask production from the estimators.
The immutable removal commit above preserves their complete implementations.

## Projective-ranging comparison

The deleted depth algorithm is the closest predecessor of projective ranging,
but they are not equivalent:

| Aspect | Deleted depth row | Projective ranging |
|---|---|---|
| Region | Fixed focus crop, then full-box fallback | Actual box or silhouette mask |
| Foreground recovery | Gaussian centre assumption | Selected 2D isolation strategy for rectangular gates |
| Reduction | Deproject every pixel and average planar distances | Aggregate depth, then deproject one representative pixel |
| Output boundary | Vehicle-frame scalar distance and optional planar fields | Camera-frame 3D coordinate before shared conversion |

## Polar-profiling comparison

| Aspect | Deleted `lidar` row | Polar profiling |
|---|---|---|
| Selection | Box-derived horizontal bearing window | Projected scan points indexed by the 2D mask |
| Intrinsics | Horizontal FoV-derived focal length | Colour `CameraInfo` intrinsics |
| Recovery | 30th-percentile anchor plus fixed range band | Bearing runs, near-band merge, then median |
| Output boundary | Vehicle-frame position with front offset | Camera-optical planar coordinate before shared conversion |

## Historical measurements

The first recorded polar run on 2026-07-13 covered 14 trials: box-gated polar
profiling reported `0.093 m` MAE versus `0.064 m` for the deleted `lidar` row.
Later standing runs were recorded around `0.086-0.094 m` for polar and
`0.064 m` for `lidar`. These aggregates alone do not prove whether the gap came
from tuning, target-surface convention, or the changed selection algorithm.

The old 3D-isolation catalogue also recorded `0.156 m` MAE for its percentile
range-band baseline in a 70-trial simulation run on 2026-07-13. The exact run
identifier was not preserved in that account. `height_crop_range_band` remained
the default until 2026-08-28, when `height_crop_nearest_mode_band` and the
disabled mask-depth cutoff became the defaults.

All these measurements predate later scenario, scoring, estimator-registry, and
camera-geometry transitions. They are design evidence, not verified present-day
scores. No controlled comparison on later geometry is established here.
