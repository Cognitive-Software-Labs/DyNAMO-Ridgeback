# ROI-native mask migration

2026-08-31. Behaviour-preserving migration of the target-localization mask
pipeline to one ROI-native representation. The contract itself is documented in
`mask_component.md` Section 7; this file records what was proven and what was
measured.

Scope note: this is a representation change. It does not close the exact-stamp
depth-availability gap tracked in `refactor_validation.md`, and nothing here
should be read as having done so.

## What changed

| Boundary | Before | After |
|---|---|---|
| Producers | `rasterize_detection` / `mask_from_array` → full-grid `Mask` | `region_from_bbox` / `region_from_blob` → `MaskRegion` |
| Per-detection depth prep | `mask.data & frame_valid`, full-grid, once per estimator | one shared `PreparedDepthRegion` per detection, ROI-sized |
| Depth estimators | one entry point, precision fork inline | `select_foreground_pixels` / `select_foreground_points` policy + shared reduction, behind two entry points |
| Polar membership | `mask.data[v_px, u_px]` | `MaskRegion.contains_pixels`, bounds-tested before indexing |
| Debug union | full-grid write per mask | `MaskRegion.blit_into` one output buffer |
| `deproject_masked` | `np.asarray(depth, float64)[rows, cols]` | gather, then widen |

Preserved deliberately: `MaskPrecision.RECT`/`TIGHT` and the different
foreground policies behind them; recipe catalogues, defaults, thresholds and
point ordering; every miss reason at the same logical stage; `None` vs empty;
oversized-box and segmentation rejection; exact-stamp RGB/depth matching;
`PreparedColorFrame` identity; zero-or-one scan projection per batch; all ROS
topic and message contracts. No launch parameter, runtime toggle, dependency or
strategy framework was added.

Compatibility is confined to boundaries: `Mask`, `mask_from_array`,
`rasterize_bbox` / `rasterize_detection` / `rasterize_batch`, `masked_rgb`, and
the full-grid `localize_projective_ranging` / `localize_euclidean_reconstruction`
keep their signatures and meaning, backed by the same geometry and reduction code
rather than a second copy. A caller's precomputed `valid_masked` is still passed
to a custom isolation callable by identity.

## Correctness evidence

Reference results were captured from the **pre-migration** implementation and
checked in as `test/mask_region_pre_migration_reference.json`; the parity tests
compare against that file, not against a second wrapper of today's code. Twenty
reference cases cover both precisions, both 2D recipes and all three 3D recipes,
finite and unlimited depth gates, float32 and float64 depth, a scene carrying
zero / NaN / inf / negative returns, both starvation reasons, and polar beam
selection under both mask types.

Both entry points reproduce every case exactly: foreground pixel/point counts,
ordered point sets, representative global UV, optical XYZ, and original beam
indices, at `abs=1e-9`.

Tests: **541 → 633** (`541` measured at `f16afd0` before this work started, `+9`
from the concurrent TF fix since committed as `e9ff5f3`, **`+83` added here**).
Nothing was removed or skipped.

| File | Covers |
|---|---|
| `test_mask_region.py` | region contract, ownership/read-only, canonical empty, clamped/inverted/fractional boxes, tight-extent cropping, offset/edge/one-pixel windows, holes and disconnected components, negative and out-of-window membership, crop↔materialize round trips, the `Mask` crossing |
| `test_mask_region_parity.py` | the frozen pre-migration numbers, shortfall reasons, empty-region behaviour, polar beam parity and overlap independence, `deproject_masked` gather order on float32/float64/strided/empty input |
| `test_mask_region_allocation.py` | structural: no full-grid materialization in the pipeline (`to_full_array`/`to_mask` monkeypatched to raise), region-sized prepared arrays, no full-frame cast per euclidean call, one debug output rather than one mask per detection |

Allocation assertions use `tracemalloc`'s peak counter, which covers numpy data
buffers including temporaries freed before the call returns. They are bounds
relative to one full-frame float64 array — the thing being ruled out — not
timing assertions.

## Measured storage and CPU

Same deterministic inputs before and after; "before" is the full-grid entry
points, which are behaviour-identical to the pre-migration production body.
Warm medians over 60–200 repeats, single process, host under concurrent load
from other sessions. The higher-resolution case is synthetic test data only; no
camera configuration was changed.

### Storage per detection

| Frame | Case | Full-grid mask | Region | Ratio |
|---|---|---:|---:|---:|
| 640×480 | box / tight, ROI ≈ 2% of frame | 307 200 B | 6 084 B | 50.5× |
| 640×480 | box / tight, ROI ≈ 50% of frame | 307 200 B | ≈153 000 B | 2.0× |
| 1280×720 | box / tight, ROI ≈ 2% of frame | 921 600 B | ≈18 300 B | 50.4× |
| 1280×720 | box / tight, ROI ≈ 50% of frame | 921 600 B | ≈460 000 B | 2.0× |

Every region reports `data.base is None`: no region retains the segmenter's
frame-sized output as backing storage.

### Both depth estimators, one shared preparation (640×480)

| Detections × ROI | Before med/p95 ms | After med/p95 ms | Before peak B | After peak B |
|---|---|---|---:|---:|
| 1 × small | 0.87 / 0.92 | 0.28 / 0.29 | 1 025 084 | 800 744 |
| 4 × small | 3.38 / 3.55 | 0.98 / 1.00 | 1 025 216 | 800 928 |
| 12 × small | 10.10 / 10.29 | 2.98 / 3.18 | 1 026 900 | 802 981 |
| 1 × large | 3.77 / 4.18 | 3.55 / 4.16 | 6 400 852 | 7 342 945 |
| 4 × large | 13.27 / 13.98 | 11.74 / 12.39 | 5 358 788 | 6 076 200 |
| 12 × large | 16.78 / 19.19 | 9.91 / 10.31 | 2 142 976 | 2 166 203 |

At 1280×720 the same shape holds: 12 × small goes 29.6 → 7.7 ms, 12 × large
53.5 → 29.4 ms, 1 × large 11.8 → 11.4 ms.

### Other stages (640×480)

| Stage | Before | After |
|---|---|---|
| bbox construction | 0.0073 ms | 0.0029 ms |
| tight construction | 0.0006 ms | 0.0161 ms |
| debug union, 12 detections | 0.29 ms | 0.04 ms |
| `deproject_masked` | 0.067 ms, 2 498 200 B peak | 0.024 ms, 229 920 B peak |
| full-image mask, standalone API | 1.73 / 1.87 ms | 1.77 / 1.85 ms |

### Cases that got slower, or did not improve

- **Tight-mask construction is ~27× slower** (0.0006 → 0.0161 ms at 640×480;
  0.0006 → 0.0327 ms at 1280×720). The old producer wrapped the model's array in
  O(1); the new one scans for the nonzero extent and copies. This is the price of
  not pinning a frame-sized model output per detection, and it sits next to a
  segmentation forward pass costing tens of milliseconds.
- **Large-ROI peak allocation is higher** (1 × large at 640×480: 6.40 → 7.34 MB).
  The extra is the two global index arrays `global_pixels` produces for a
  non-zero origin. It is proportional to the selection rather than to the frame,
  which is the intended trade; wall time for the same case still improved
  slightly. A window at the origin returns its inputs instead, which is what
  keeps the full-image standalone case at parity.
- **1 × large is essentially unchanged in time** (3.77 → 3.55 ms). A mask
  covering half the frame has little ROI to save.

Smaller mask storage is not evidence of lower end-to-end latency and does not
address dropped frames. No live simulator run was performed for this change (see
below), so nothing here is an end-to-end claim.

## Simulator smoke, both mask gates

`examples` scenario, 5 scenes / 8 instances, 1 repeat, 2 s settle, 10 s capture,
all four estimators, stereoscopic depth, default recipes, unlimited mask-depth
gate, headless. Artifacts (temporary, not checked in):
`/tmp/dynamo-roi-smoke/box` and `/tmp/dynamo-roi-smoke/silhouette`.

**Box gate reproduces the pre-migration benchmark exactly.** Baseline column is
the pre-refactor run recorded in `refactor_validation.md`:

| Estimator | MAE now (m) | Pre-migration MAE (m) | Δ | Scored |
|---|---:|---:|---:|---|
| Point cloud | 0.129655 | 0.129655 | 0.000000 | 6/8 |
| Projective ranging | 0.055221 | 0.055221 | 0.000000 | 6/8 |
| Euclidean reconstruction | 0.056944 | 0.056944 | 0.000000 | 6/8 |
| Polar profiling | 0.081697 | 0.081697 | 0.000000 | 5/8 |

Miss reasons are the expected ones: `NO_DEPTH_FRAME` ×13 of 246 on both depth
rows — the residual exact-stamp gap, untouched by this change — and
`TOO_FEW_RAYS_SELECTED` ×41 for polar. Four scene collages rendered.

**Silhouette gate exercises `region_from_blob` in production.** There is no
pre-migration silhouette baseline to diff against, so this is an integration
check rather than a bit-exactness one — but a mis-placed crop origin would move
the representative pixel by tens of pixels and wreck the lateral component, so
the distances are the evidence:

| Estimator | Box MAE (m) | Silhouette MAE (m) | Scored (both) |
|---|---:|---:|---|
| Point cloud | 0.129655 | 0.129655 | 6/8 |
| Projective ranging | 0.055221 | 0.052476 | 6/8 |
| Euclidean reconstruction | 0.056944 | 0.060593 | 6/8 |
| Polar profiling | 0.081697 | 0.072268 | 5/8 |

`pointcloud` consumes no mask, and it is identical to six decimals across both
runs — which is the control: the harness is run-to-run deterministic here, so
the mask-row differences are attributable to the gate rather than to noise. Two
of the three mask rows improve, as a tighter mask should; euclidean's small
regression is the `tight` MAD pass versus the `rect` 3D recipe, the documented
precision fork, not a geometry error. SlimSAM ran 11 segmentation batches and
the mask rows produced 230 / 230 / 203 OK observations. `NO_COLOR_FRAME` ×3
appears only under this gate, which is correct — it is the one gate that needs
the exact-stamp color frame. Both runs recorded one `UNSET` batch.

## Checks run

- Direct ROS-sourced pytest, full package suite: **633 passed**.
- `colcon build --symlink-install`, then the full suite again against the
  rebuilt install tree, then `colcon test` / `colcon test-result`.
- `ros2 launch ... ridgeback_exploration.launch.py --show-args` and
  `target_distance_benchmark.launch.py --show-args`.
- `git diff --check`.
- Graphify rebuilt with the repository helper.

## Not performed

- **No exploration run.** Exploration shares the same node factories and is
  covered by `test_launch_layout`, but nothing here drove it live.
- **No hardware validation**, and no sweep-scale benchmarking.
- The silhouette gate has no pre-migration numeric baseline, so its parity
  evidence is the unit and reference tests rather than the benchmark run.
