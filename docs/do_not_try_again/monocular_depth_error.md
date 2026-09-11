# Monocular depth error: not a bug, deferred to Isaac Sim

Investigated 2026-09-04 against the 2026-09-03 sweep at
`artifacts/benchmarks/20260903_202157_model_concurrency`.

## Conclusion: this is NOT A BUG

The `monocular` row reporting 8.8-9.8x the mean absolute error of the
`stereoscopic` row is **not evidence of a defect**. Two separate things produce
that number, and neither is broken code:

1. **The baseline is ground truth, not a sensor.** The simulated "stereo" depth
   is an exact render with no noise model, so the comparison is monocular
   against ground truth. Its residual 0.06 m is a geometric definition offset,
   not sensing error.
2. **The residual is ordinary out-of-domain model error.** The default
   checkpoint is the smallest metric Depth-Anything variant, trained on real
   indoor photographs, run against synthetic Gazebo renders.

No code change is warranted on this evidence. Do not "fix" the monocular path
in the Gazebo stack.

## Do not re-derive these

Each was tested and settled on 2026-09-04.

- **The resize is not the problem.** Depth-Anything returns `predicted_depth`
  already at the colour grid -- shape `(480, 640)` for a 480x640 input, aspect
  identical -- so `_resize_depth`'s `cv2.resize` is a **no-op** and cannot
  displace the depth field.
- **It is not a units or scale-factor error.** `est/truth` spans 0.841-1.420
  and crosses 1.0 in both directions. A mm/m or focal-length mistake is a
  constant multiplier and cannot change sign.
- **It is not a relative-vs-metric checkpoint mix-up.** `_resolve_usable_max`
  in `core/depth_sources.py` raises on any non-`metric` checkpoint precisely so
  unitless inverse depth cannot reach the estimators.
- **It is not the usable-range ceiling.** That resolves to 0.9 x 20 m = 18 m;
  every scored instance is 1.75-5.12 m.
- **It is not the mask, isolation recipe, estimator or reduction.** Both cells
  run the identical gate, the identical `nearest_mode_histogram` /
  `height_crop_nearest_mode_band` recipes, and the identical estimator over the
  *same* prepared region. Only the depth array differs.
- **It is not run variance.** Three process replicates agree to three decimals.
- **The scene is not too dark or too flat for the model.** This was checked
  from the per-trial PNGs and the check was invalid: those are 640x240
  two-panel collages with HUD text burned in, so their pixel statistics measure
  the overlay, not the scene. The images cannot answer appearance questions
  either way -- do not try to use them for it.

## Why the stereo baseline is ground truth

`intel_realsense.urdf.xacro` states "stereo baseline, no noise model", clip
0.3-100 m. The depth image is an exact render, not a stereo match: no disparity
quantization, no matching failure, no texture dependence.

Its residual 0.06 m is fully explained by geometry. The G1 torso mesh spans
body-x `-0.0666` to `+0.0836` m, and every scene sets `robot_yaw_rad: pi`, so a
pi rotation about z maps body point `bx` to world `x = spawn - bx` and the
surface nearest the camera sits **0.0836 m in front of the base origin**.
Ground truth is measured to the origin; the reduction sees the surface:

```
truth        2.250   (spawn 2.500 - ROBOT_FRONT_OFFSET_M 0.25)   matches the CSV
near surface 2.166   (2.250 - 0.0836)
stereo est   2.191   -- between the two, as a mode reduction should be
```

That is why stereo's absolute error is flat across range (0.046-0.070 m from
1.75 m to 5.12 m) while `est/truth` climbs 0.965 -> 0.991. Every estimator
inherits the same ~0.06 m near bias, because all of them reduce over a visible
surface while truth is an origin.

**Consequence for how the number is quoted:** the 8.8-9.8x ratio is monocular
against ground truth plus a fixed offset on the reference side. It is an upper
bound on monocular's disadvantage and says nothing about real D455 stereo,
which has range-dependent noise and dropouts this benchmark cannot represent.

## The measurements, for whoever retests

Silhouette gate, projective ranging, `r1_*`, 18 scored instances per cell.

| scene | truth | stereo est | ratio | monocular est | ratio |
|---|---|---|---|---|---|
| inter_robot_occlusion_near_far | 1.750 | 1.689 | 0.965 | 1.760 | 1.005 |
| multi_visible_near_mid_far | 1.929 | 1.860 | 0.964 | 2.549 | 1.321 |
| single_forward_2p5 | 2.250 | 2.191 | 0.974 | 1.891 | 0.841 |
| bed_occluder_single | 3.250 | 3.190 | 0.982 | 4.615 | 1.420 |
| multi_visible_near_mid_far | 3.351 | 3.292 | 0.982 | 3.692 | 1.102 |
| multi_visible_near_mid_far | 5.116 | 5.070 | 0.991 | 5.888 | 1.151 |

Aggregate: stereo 0.059 m / monocular 0.578 m (projective); stereo 0.064 m /
monocular 0.563 m (euclidean). `mean_rel_error` 0.193 for monocular against a
published ~0.07 AbsRel for this checkpoint on NYU -- about 2.5x worse, the
right order for out-of-domain input rather than a defect.

## The one thing still unexplained

Monocular error is **scene-dependent, not distance-dependent**: 1.005 at
1.750 m but 1.321 at 1.929 m, two nearly identical ranges with a 32% spread,
and it under-estimates in one scene while over-estimating in others. Two
candidates remain, both deferred:

- **Checkpoint capacity.** `Depth-Anything-V2-Metric-Indoor-Small-hf` is the
  smallest metric variant.
- **The reduction settling on a farther mode** in a smooth depth field.
  Monocular depth bleeds across discontinuities in a way an exact render never
  does, so a mode/histogram reduction can land on background even with a
  correct mask. Note the related known result that the RangeBand percentile
  anchor is composition-dependent with a cliff at 25% object share.

## Reopen when: the Isaac Sim migration lands

**This is the trigger. Do not reopen on the Gazebo stack.**

The Gazebo benchmark structurally cannot answer the question, because all three
confounds are properties of that renderer. The Isaac Sim 6 port changes every
one of them at once, which is why the retest belongs there and only there:

- **A fair stereo baseline becomes possible.** Isaac can model real depth
  sensor noise, so `stereoscopic` can stop being ground truth. Until then any
  monocular-vs-stereo ratio is comparing against a perfect reference.
- **Photorealistic rendering directly addresses the out-of-domain concern.**
  Depth-Anything is trained on real photographs; Isaac's RTX output is far
  closer to that distribution than Gazebo's. If the error collapses there, the
  Gazebo figure was an artifact of synthetic appearance and nothing needed
  fixing.
- **Resolution changes 640x480 -> 1280x720**, altering the model's input scale
  and therefore its accuracy independently of everything else.

### What to run on Isaac, in order

1. Re-measure the **geometric offset first**. Confirm whether ground truth is
   still the target origin and re-derive the near-surface offset for whatever
   target model Isaac spawns. Every error figure is meaningless until that
   constant is known -- on Gazebo it was 0.0836 m and accounted for the entire
   stereo error.
2. Re-run the four-cell mask-gate / depth-source matrix on the `full` scenario
   (88 scenes), not `examples`. The five example scenes give 18 scored
   instances at three distances: enough to show the shape, **not** enough to
   fit a range model.
3. Only if a large gap survives a *noise-modelled* stereo baseline, resume the
   two deferred candidates above -- reduction behaviour first, since it is free
   once a frame corpus exists and decides whether comparing checkpoints is even
   measuring the right thing.

### What would make this a bug after all

Reopen as a defect only on one of these, not on a large ratio alone:

- monocular error stays scene-dependent with a *noise-modelled* stereo baseline
  and photorealistic input, at which point the residual is no longer
  attributable to domain gap;
- the masked monocular depth histogram contains the target's true depth while
  the reduction selects a farther mode, which would make it a recipe defect
  rather than a depth-source one;
- a larger checkpoint does not move the error at all, which would point at the
  input or the post-processing rather than capacity.

### Trap to avoid on any rerun

Benchmark runs on this host can silently degrade: the camera collapses from
~28 Hz to ~5 Hz under GUI contention on the XRDP session, changing coverage
without changing RTF. Check the camera rate before trusting a new run. See
[`docs/troubleshooting.md`](../troubleshooting.md#camera-rate-collapses-under-software-rendering).
