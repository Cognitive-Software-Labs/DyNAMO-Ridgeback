# Estimator evolution

Historical account of how the target-localization estimator registry changed.
It describes deleted implementations and old measurements; it is not a source
of current estimator names, parameters, or runnable baselines. Current contracts
are documented by [projective ranging](../target_localization/projective_ranging.md),
[euclidean reconstruction](../target_localization/euclidean_reconstruction.md), and
[polar profiling](../target_localization/polar_profiling.md).

## Timeline

1. A camera node produced `rgb`, `sensor_depth`, `depth_anything`, and
   `pointcloud`; a separate LiDAR node produced `lidar`.
2. Mask representation, aligned-depth handling, and three mask-based estimators
   were added alongside that stack: `projective_ranging`,
   `euclidean_reconstruction`, and `polar_profiling`.
3. Benchmarking exposed that the older rows mixed source choice, selection, and
   ranging policy and could not share the mask contract cleanly.
4. Commit `8640635` removed `rgb`, `sensor_depth`, `depth_anything`, and `lidar`
   from the public registry, extracted the surviving pointcloud algorithm, and
   left four operational estimators: `pointcloud` plus the three mask rows.
5. Depth-Anything remained supported as a depth *source* for the depth-based
   estimators instead of being reported as a separate ranging algorithm.

The final pre-removal implementations can be inspected directly:

```bash
git show 8640635^:src/ridgeback_autonomy/ridgeback_autonomy/perception/core/geometry.py
git show 8640635^:src/ridgeback_autonomy/ridgeback_autonomy/perception/core/depth_anything.py
git show 8640635^:src/ridgeback_autonomy/ridgeback_autonomy/perception/g1_camera_measurement_node.py
git show 8640635^:src/ridgeback_autonomy/ridgeback_autonomy/perception/g1_lidar_measurement_node.py
```

These paths and `g1` names are historical. They are included so the algorithmic
steps remain recoverable without pretending the deleted modules are current API.

## Original estimator stack

### RGB ground-plane projection

`add_rgb_measurements` used only a detection box and fixed camera geometry:

1. Take the horizontal centre and bottom edge of the box.
2. Convert that pixel to a ray using horizontal and vertical FoV constants.
3. Rotate the ray for the configured camera pitch.
4. Intersect it with the ground plane using configured camera height.
5. Convert camera-right to vehicle-left, apply the vehicle-front reference
   offset, and report planar position and distance.

This assumed that the bottom of the detected box represented a ground contact
point. It required no measured depth, but box truncation, pose, and imperfect
ground contact directly changed the result. It has no current replacement row.

### Sensor-depth and Depth-Anything rows

`sensor_depth` and `depth_anything` shared `add_depth_source_measurements` and
`compute_weighted_planar_distance`. Only the input depth map differed:

- `sensor_depth` decoded the subscribed depth image;
- `depth_anything` ran the configured metric Depth-Anything model on RGB and
  resized its prediction to the image grid.

For each detection the shared algorithm:

1. Constructed a fixed inner focus crop (`24-76%` horizontally and `18-62%`
   vertically), falling back to the full box when the crop had no usable depth.
2. Filtered non-finite, non-positive, and over-range pixels.
3. Applied a Gaussian weight centred on the crop.
4. Deprojected every valid pixel using FoV-derived intrinsics.
5. Rotated points into the vehicle frame, applied the front offset, and
   returned the weighted mean of their planar distances.

The design conflated two independent choices: where depth came from and how a
target was ranged. The current stack selects `stereoscopic` or `monocular` as a
depth source and then applies the same selected ranging estimator. This makes a
source comparison possible without silently changing the localization
algorithm.

### Organized pointcloud row

The original pointcloud implementation read the organized `PointCloud2`, used
the same inner focus crop, transformed valid points when possible, and then:

1. rejected points behind the vehicle or beyond the range limit;
2. anchored on the 25th percentile of forward position;
3. kept a fixed band from `0.10 m` ahead to `0.35 m` behind the anchor;
4. returned the nearest inlier after applying the vehicle-front offset.

Unlike the four removed rows, `pointcloud` stayed in the registry. Commit
`8640635` extracted its ROS-free implementation from the legacy `geometry.py`
into the dedicated pointcloud-ranging module and its own measurement node. It
remains intentionally separate from euclidean reconstruction: the former reads
a published organized cloud and uses its own reduction, while the latter
deprojects masked aligned-depth pixels and applies the selected 3D isolation
recipe.

### LiDAR box-window row

`compute_lidar_measurement`, fed by `g1_lidar_measurement_node`, was an early
camera-guided planar-scan estimator:

1. Transform valid scan points into the camera optical frame.
2. Convert the box into a horizontal bearing centre and angular half-window.
3. Keep scan points whose camera bearing falls inside that window.
4. Anchor on the 30th percentile of planar range.
5. Keep points no more than `0.20 m` behind that anchor.
6. Take the median camera-frame target point, transform it back to the vehicle
   frame, apply the front offset, and report planar position and distance.

The gate was only horizontal: it could not ask whether a beam actually crossed
the selected silhouette at the scan's projected image row. A near occluder in
the same box could therefore become the target.

## Transition to mask-based estimators

The mask paths separated responsibilities that the older rows had bundled:

| Earlier row | What was retained | What changed |
|---|---|---|
| `rgb` | Camera/vehicle frame conventions | Ground-plane box projection was removed rather than generalized |
| `sensor_depth` | Registered depth as an input source | Fixed focus crop and Gaussian range average became explicit mask selection plus projective or euclidean reduction |
| `depth_anything` | Monocular metric-depth support | Became a depth-source strategy instead of a separate estimator identity |
| `pointcloud` | Organized-cloud estimator and percentile-band lineage | Extracted behind a dedicated module; remains independent of the mask node |
| `lidar` | Camera-guided scan localization | Horizontal box gating became mask membership and polar run segmentation |

Projective ranging now aggregates selected depth and deprojects one
representative pixel. Euclidean reconstruction deprojects selected pixels and
isolates foreground in 3D. Polar profiling projects scan points onto the colour
grid, applies actual mask membership, and segments retained beams in polar
space. All three output camera-frame coordinates before the shared downstream
vehicle-frame conversion.

This was not merely a rename. It made representation, source acquisition,
foreground selection, estimator reduction, and frame conversion independently
testable and reusable.

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
camera-geometry transitions. They are design evidence, not verified current
scores. Controlled isolation comparison remains tracked in the maintained
backlog.
