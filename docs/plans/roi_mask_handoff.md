# Handoff: ROI-native masks and shared bbox/silhouette processing

Status: implementation plan, not implemented. Prepared against checkout `f16afd0` on 2026-08-31. Recheck HEAD and concurrent changes before starting.

## Assignment

Implement a behavior-preserving migration of the target-localization mask pipeline to one ROI-native mask representation. Bounding boxes and silhouettes must share spatial processing; retain their deliberately different foreground-selection policies. This is the larger ROI-native design, not merely slicing depth while continuing to allocate full-frame masks per detection.

Read this plan, `AGENTS.md`, `AI_CONTEXT.md`, and the current Graphify report before editing. Inspect the worktree and preserve unrelated work. Follow the safe-refactor discipline: establish a baseline, move one boundary at a time, rerun the same proof, and stop when the stated scope is complete. Do not turn this into another package-wide structural refactor.

## 1. Decisions already made

- ROI means a rectangular **storage window containing a boolean mask**, not replacing a silhouette with a rectangle.
- Introduce a clearly named `MaskRegion`; do not silently change the meaning of existing `Mask.data[v, u]` from global to local indexing.
- Make ROI-native masks canonical throughout production per-detection measurement processing. Full-frame arrays remain appropriate for camera inputs, one shared depth-validity image, model output at its existing boundary, and subscribed image/debug output.
- Keep `MaskPrecision.RECT` and `MaskPrecision.TIGHT`. Never infer precision from whether a crop happens to be all `True`.
- Share geometry and preparation. Keep separate box/segmentation producers and estimator-specific foreground policies. Do not force identical filtering on both mask types.
- Use small pure functions and, where useful, a small prepared-depth value object. No new strategy framework, plugin registry, dependency, launch parameter, or runtime toggle.
- Apply this inside the shared target-localization implementation, so exploration and benchmarking both receive it without separate implementations or launch changes.
- Preserve existing public full-frame helper contracts through thin boundary adapters where needed. Production must not convert ROI masks back to full-frame masks between stages. Compatibility must not become a second copy of the algorithms.

## 2. Current behavior to preserve

| Stage | Bbox / `RECT` | Silhouette / `TIGHT` | Intended ownership |
|---|---|---|---|
| Producer | Rasterize accepted detection box; no segmentation/color requirement for masking | Segment accepted prompts using the exact color frame; reject failures without bbox fallback | Separate producers, common `MaskRegion` output |
| Depth preparation | Valid pixels within the rectangle | Valid pixels within the silhouette | One ROI preparation implementation |
| Projective foreground | Configured 2D isolation recipe | Use valid masked depths directly | Explicit projective selection policies |
| Projective reduction | Foreground median depth and foreground pixel centroid | Same reduction | One reduction/deprojection implementation |
| Euclidean foreground | Configured 3D isolation recipe | Existing MAD outlier removal | Explicit Euclidean filtering policies |
| Euclidean geometry/reduction | Deproject valid selected pixels; centroid of surviving points | Same geometry/reduction | One implementation |
| Polar profiling | Mask membership, range segmentation, near-band merge | Same algorithm; silhouette does not remove LiDAR parallax | One implementation, already shared |

Preserve numerical defaults, units, thresholds, recipe catalogues, point ordering, result fields, TF/front-offset handling, detection order, and estimator-enable behavior.

Failure behavior is part of the contract:

- `None` is a rejected/unavailable mask; an empty region is a valid selector with zero selected pixels. Do not conflate them or introduce a blanket empty-mask shortcut.
- Preserve oversized-box rejection and segmentation score/empty rejection before measurement.
- Projective tight shortfall remains `TOO_FEW_VALID_PIXELS`; rectangular isolation shortfall remains `ISOLATION_EMPTY`.
- Euclidean pre-filter shortfall remains `TOO_FEW_VALID_POINTS`; post-filter shortfall remains `ISOLATION_EMPTY`.
- Polar preserves `NO_BEAMS_IN_VIEW`, `TOO_FEW_RAYS_SELECTED`, and `TOO_FEW_RAYS_MERGED`, including selected original beam indices on failure.
- Preserve frame-level failure precedence, including missing depth, missing scan, grid mismatch, and already-stamped mask failures. Do not stamp disabled estimator fields.
- Preserve exact-stamp RGB/depth matching, early/late color-hint behavior, shared `PreparedColorFrame` identity, and no persistent RGB cache.

## 3. Region contract

Recommended fields, with explicit coordinate names:

```python
MaskRegion(
    data,          # bool[roi_height, roi_width], owned and read-only
    origin_u,      # column offset in the original color image
    origin_v,      # row offset in the original color image
    image_width,   # original full-grid width
    image_height,  # original full-grid height
    precision,     # existing MaskPrecision
)
```

- Outside the window, membership is implicitly false.
- Global column = `origin_u + local_column`; global row = `origin_v + local_row`.
- Bounds are half-open. Box construction must retain the existing integer conversion, clamping, inverted-box, and degenerate-box behavior.
- Expose unambiguous full-grid shape and ROI shape. Avoid ambiguous `width`/`height` properties that callers could mistake for full-image dimensions.
- Validate boolean/2D payloads, integer bounds, and containment within the full grid. Keep new validation separate from legacy-wrapper compatibility behavior.
- Define a canonical empty region with a `(0, 0)` boolean payload, zero origin, retained full-grid dimensions, and retained precision. It is not `None`.
- Each published region owns its compact storage. A tight crop must not retain the full SAM output as its NumPy base, nor remain writable through an external alias. Avoid unnecessary double copies of already-owned producer buffers.
- Grid/stamp association remains batch-owned; do not introduce independent timestamps, resampling, adjusted camera intrinsics, or persistent region caches.

Provide only the geometry helpers consumers need: full-frame slicing, local-to-global pixel coordinates, vectorized `contains_pixels(u_px, v_px)`, empty checks, blitting into a destination image, and explicit full-grid materialization for compatibility/output. Membership must reject out-of-ROI coordinates before local indexing; negative indices must never wrap around. Cropping/materialization/blitting must preserve every selected pixel, including disconnected components and holes.

## 4. Implementation sequence

### A. Establish baseline and reference behavior

1. Read the ownership map below and identify all direct `Mask.data`, `rasterize_*`, `mask_from_array`, and estimator callers, including tests and rendering.
2. Run the existing affected tests and full package suite before editing. Record exact HEAD, worktree state, commands, failures, and test counts. Do not reuse an older count as today's baseline.
3. Capture deterministic pre-migration reference results for offset boxes, tight masks, invalid depths, recipe variants, and beam selection. Add characterization cases before changing production code where coverage is missing.
4. Record a small repeatable CPU/allocation baseline before removing the old production path. Include mask construction, ROI preparation, and estimator work separately; segmentation inference is a separate cost.

### B. Add the region primitive without switching producers

1. Add the validated region type and geometry helpers in the mask layer; keep it ROS/OpenCV/model-free.
2. Add direct region construction from bbox and from a full boolean segmentation blob.
3. For segmentation, crop to the **actual nonzero extent**, not the detection box. The padded prompt can yield valid silhouette pixels outside that box. Copy the crop, preserve all components, and let the full output expire after conversion. Do not change SAM prompting, postprocessing resolution, thresholds, or model behavior.
4. Add explicit legacy-mask adapters, with round-trip pixel-equivalence tests. Keep existing full-grid `Mask` semantics unchanged.

### C. Isolate the current selection policies

1. Make the projective and Euclidean precision-dependent selection stages explicit small functions, preferably within their owning estimator modules. Geometry helpers must not import estimator policy code.
2. Dispatch using the mask's actual precision at a clear boundary. Do not hard-wire policy from a node-wide gate in standalone APIs that can receive different precision tags.
3. Reuse the existing isolation catalogues and MAD implementation unchanged. Keep minimum-count checks and their failure reasons at the same logical stages.
4. Keep the common reduction/geometry code single-sourced. A conditional confined to policy selection is acceptable; removing every `if` is not the objective.

### D. Make depth consumers ROI-native

1. Introduce a common per-detection preparation helper, optionally returning `PreparedDepthRegion`, containing the region, an ROI depth view, and one ROI-local `valid_masked` array. It may retain a reference to the original depth frame for indexed deprojection; retaining a reference does not copy the frame.
2. Keep the existing lazy, at-most-once-per-batch `valid_depth(full_depth, depth_max)` calculation. Slice that validity image to each region and combine it with local mask data once. Pass the **same prepared selection** to both enabled depth estimators. Do not add depth work in polar-only runs or cache between batches.
3. Run built-in 2D isolation recipes on ROI-sized depth/mask/validity arrays. They must produce the same selected global pixel set as before, with the same order and histogram inputs. Do not alter recipe mathematics.
4. Projective: obtain foreground depths from the local depth view; restore full-image pixel coordinates before computing the representative pixel/deprojection. Keep original intrinsics and the current centroid calculation semantics, not the bbox center or a nearest-mask-pixel substitute.
5. Euclidean: obtain the selected local rows/columns, restore global rows/columns, and deproject only those pixels against the original color intrinsics. Never index cropped depth with global indices, and never compensate for an offset twice.
6. Fix the in-scope allocation in `deproject_masked`: it currently performs `np.asarray(depth_m, dtype=np.float64)[rows, cols]`, potentially converting the full frame for every detection. Gather selected depth values **before** converting them to float64. Preserve its signature, precision, ordering, and formulas; test float32 and float64 inputs, empty selections, and strided arrays. This permits reuse of the existing helper with the full depth reference and global indices without a full-frame cast.
7. Keep legacy standalone API adapters explicit. Existing callers supplying a full-frame `Mask`, optional full-frame `valid_masked`, or a custom isolation callable retain their documented array shapes and callback behavior, including the existing precomputed-array identity test. Do not silently pass an arbitrary legacy callable cropped arrays: it may depend on absolute coordinates. Adapt its returned selection into the common reduction path. ROI-native entry points must document local-array policy inputs and be used directly by production with the built-in recipes.
8. Remove obsolete full-frame per-detection preparation from the production pipeline once both depth consumers use the region path. Boundary adapters are not permission to materialize in the hot path.

### E. Migrate polar, producers, and rendering boundaries

1. Replace polar's direct global `mask.data[v_px, u_px]` access with region membership. Keep rounding, original beam-index order, full `ScanImageProjection.uv`, and compatibility wrappers intact.
2. Preserve lazy zero-or-one scan projection per batch. The trigger is the current usable-scan/non-`None`-mask behavior; do not skip a constructed empty selector if doing so would change existing miss precedence or records.
3. Keep independent selection for each detection. Preserve selected/merged beam records on success and failure. Keep `select_bbox_beams` separate: it refers to the original detection box for visualization, not the storage bounds of a tight mask. Do not assume a silhouette is always contained within that box.
4. Switch accepted bbox production directly to compact all-true region arrays. Switch accepted segmentation blobs to copied compact regions; preserve list alignment and all rejected `None` entries.
5. Keep box masking free of segmentation/color decoding. Preserve the silhouette/monocular shared-RGB behavior and exact matching unchanged.
6. Build the subscribed `mono8` mask-debug union by blitting each region into one full-size output. Do not materialize one full-size mask per detection. Preserve bytes, header, dimensions, encoding, and subscriber guards.
7. Adapt masked-RGB and overlay membership helpers to use the common geometry operations where regions are available. A received full-frame debug image can remain full-frame at the rendering boundary; do not introduce needless crop/materialize round trips. Preserve panel selection, bbox fallback visualization, missing-image behavior, and all ROS topic/message contracts.
8. Remove temporary migration shims from internal call paths. Retain only the explicitly tested compatibility facade, backed by the same geometry/reduction implementation.

### F. Validate, document, and stop

Run the gates below after each relevant boundary and again on the final state. Update the canonical mask/segmentation/estimator docs for local storage and global coordinates. Update `AI_CONTEXT.md` only for the new ownership/contract facts; no unrelated documentation sweep. Register new test files with CMake, rebuild the package, and rebuild Graphify using the repository helper.

## 5. Acceptance tests

| Area | Required proof |
|---|---|
| Region contract | Owned read-only compact payload; no full-model backing allocation; full-grid metadata; canonical empty; clamped/inverted/fractional-coordinate bbox parity; shape/bounds validation |
| Pixel geometry | Offset ROIs, image-edge touching, one-pixel regions, full-image masks, disconnected components, holes, negative/out-of-window queries, exact crop/materialize round trips |
| Tight masks | Foreground beyond original bbox is retained; rectangular-looking tight mask stays `TIGHT`; failed segmentation stays `None`, with no bbox fallback |
| Depth selection | Both precisions, finite/unlimited gates, zeros/NaN/inf/negative depths, both 2D recipes and supported 3D recipes; precomputed validity shared without recomputation |
| Numerical parity | Same foreground pixels/ordered points, representative global UV, optical XYZ, planar outputs, counts and miss reasons as pre-migration references; use tight justified float tolerances, not a tolerance broad enough to hide an offset bug |
| Legacy APIs | Existing helper signatures/return meaning and full-frame callback inputs remain valid; old and native entry points agree for equivalent inputs; precomputed callback identity preserved |
| Polar | Exact original beam-index parity, full `uv` shape/order, miss-stage parity, distinct overlapping masks, empty/non-`None` masks, bbox debug set, zero-or-one projection with RViz records on/off |
| Batch orchestration | No work for disabled paths; missing-depth/scan behavior; rejection/order parity; shared RGB early/late exact-hit tests still pass; no nearest-frame matching or persistent caches |
| Visualization | Byte/pixel-equivalent mask union and masked RGB; no per-detection full-mask materialization; no debug construction without subscribers; no measurement effect from enabling debug |
| Ownership/install | Import-direction and shared-default tests pass; new tests registered; rebuilt installed modules import; shared exploration and benchmark launches still resolve |

Avoid validating parity solely by comparing two wrappers that both call the same newly changed implementation. Retain independent deterministic reference expectations captured before migration, alongside geometry round-trip tests.

### Performance evidence

- Compare the same deterministic inputs before/after at current 640x480 and at a higher-resolution synthetic stress case, with few/many detections and small/large ROIs. Higher-resolution test data must not change camera configuration.
- Include production-representative small boxes below the oversized-box rejection threshold; full-frame mask cases can exercise the standalone core APIs.
- Report mask bytes, retained backing storage, shape/size of per-detection temporary arrays, and warm median/p95 CPU timings. Include bbox construction and tight-mask extent scan/copy; do not report only the cheaper downstream stage.
- Assert structurally that production does not materialize a full mask per detection or cast the full depth frame per Euclidean call. One shared full-frame depth-validity array and full-frame image outputs are allowed.
- Time shared batch preparation with both depth estimators enabled, not just independent standalone calls. Include debug off/on separately.
- No arbitrary speedup target or flaky timing assertion in unit tests. Report unchanged/slower cases honestly. Smaller mask storage is not proof of end-to-end latency improvement or a fix for dropped frames.

## 6. Validation commands and runtime limits

Use the repository's ROS-sourced perception environment. Rebuild after module changes so tests/runtime cannot silently exercise stale installed modules. Confirm imported module locations if the source and installed results disagree; do not assume a bare `PYTHONPATH` override can also find generated ROS messages.

```bash
cd /home/stefi/DyNAMO/DyNAMO-Ridgeback
source /opt/ros/jazzy/setup.bash
source install/setup.bash
env ROS_LOG_DIR=/tmp/dynamo_roi_ros_test_logs perception_venv/bin/python3 -m pytest -q \
  src/ridgeback_autonomy/test/test_mask.py \
  src/ridgeback_autonomy/test/test_intrinsics.py \
  src/ridgeback_autonomy/test/test_segmentation.py \
  src/ridgeback_autonomy/test/test_isolation_2d.py \
  src/ridgeback_autonomy/test/test_isolation_3d.py \
  src/ridgeback_autonomy/test/test_projective_ranging.py \
  src/ridgeback_autonomy/test/test_euclidean_reconstruction.py \
  src/ridgeback_autonomy/test/test_polar_profiling.py \
  src/ridgeback_autonomy/test/test_mask_measurement_node.py \
  src/ridgeback_autonomy/test/test_rendering.py
env ROS_LOG_DIR=/tmp/dynamo_roi_ros_test_logs perception_venv/bin/python3 -m pytest -q src/ridgeback_autonomy/test
colcon build --symlink-install --base-paths src --packages-select ridgeback_autonomy
source install/setup.bash
env ROS_LOG_DIR=/tmp/dynamo_roi_ros_test_logs perception_venv/bin/python3 -m pytest -q src/ridgeback_autonomy/test
colcon test --packages-select ridgeback_autonomy --event-handlers console_direct+
colcon test-result --verbose
ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py --show-args
ros2 launch ridgeback_autonomy target_distance_benchmark.launch.py --show-args
git diff --check
bash "$(git rev-parse --show-toplevel)/tools/rebuild_graphify"
```

Include newly added focused files in the focused command. On the unmodified baseline run tests before building only if the installed state is already current; otherwise rebuild first. At implementation milestones rebuild before interpreting installed-code test results. The final full suite and CMake/colcon results must include all new tests.

If an isolated simulator environment is available, do a short benchmark smoke with a visible target under each mask gate and verify actual measurements plus debug images. Check exploration through its shared factory/launch tests; a live exploration smoke is useful but is not an exploration-completion claim. Coordinate before launching or running cleanup if other tasks own active ROS/Gazebo processes; do not disrupt their runs for this refactor. Report unavailable live checks separately from completed deterministic parity proof.

Read the current [refactor validation record](/home/stefi/DyNAMO/DyNAMO-Ridgeback/docs/history/refactor_validation.md): it documents an unresolved depth-row coverage difference after the previous structural refactor. That is a pre-existing validation concern, not an established cause and not automatically fixed here. Do not tune gates, timing, synchronization, or scheduling to conceal it.

## 7. Ownership map

These are primary entry points, not permission to edit every file indiscriminately.

| File | Role in this change |
|---|---|
| [mask.py](/home/stefi/DyNAMO/DyNAMO-Ridgeback/src/ridgeback_autonomy/ridgeback_autonomy/perception/target_localization/core/mask.py) | Region primitive, constructors, membership, rendering/compatibility geometry |
| [mask_measurement_node.py](/home/stefi/DyNAMO/DyNAMO-Ridgeback/src/ridgeback_autonomy/ridgeback_autonomy/perception/target_localization/mask_measurement_node.py) | Producer handoff only; preserve ROS/model/synchronization lifecycle |
| [measurement_pipeline.py](/home/stefi/DyNAMO/DyNAMO-Ridgeback/src/ridgeback_autonomy/ridgeback_autonomy/perception/target_localization/measurement_pipeline.py) | Shared lazy preparation, native estimator entry points, debug union |
| [projective_ranging.py](/home/stefi/DyNAMO/DyNAMO-Ridgeback/src/ridgeback_autonomy/ridgeback_autonomy/perception/target_localization/core/projective_ranging.py) | Explicit 2D selection policy and shared ROI reduction |
| [euclidean_reconstruction.py](/home/stefi/DyNAMO/DyNAMO-Ridgeback/src/ridgeback_autonomy/ridgeback_autonomy/perception/target_localization/core/euclidean_reconstruction.py) | Explicit 3D filtering policy and shared geometry/reduction |
| [intrinsics.py](/home/stefi/DyNAMO/DyNAMO-Ridgeback/src/ridgeback_autonomy/ridgeback_autonomy/perception/target_localization/core/intrinsics.py) | Selected-depth conversion before float64 casting; preserve public geometry API |
| [isolation_2d.py](/home/stefi/DyNAMO/DyNAMO-Ridgeback/src/ridgeback_autonomy/ridgeback_autonomy/perception/target_localization/core/isolation_2d.py), [isolation_3d.py](/home/stefi/DyNAMO/DyNAMO-Ridgeback/src/ridgeback_autonomy/ridgeback_autonomy/perception/target_localization/core/isolation_3d.py) | Reuse algorithms/defaults; clarify local-array contracts only where needed |
| [polar_profiling.py](/home/stefi/DyNAMO/DyNAMO-Ridgeback/src/ridgeback_autonomy/ridgeback_autonomy/perception/target_localization/core/polar_profiling.py) | Origin-aware membership; preserve projection and selection semantics |
| [segmentation.py](/home/stefi/DyNAMO/DyNAMO-Ridgeback/src/ridgeback_autonomy/ridgeback_autonomy/perception/target_localization/core/segmentation.py), [rendering.py](/home/stefi/DyNAMO/DyNAMO-Ridgeback/src/ridgeback_autonomy/ridgeback_autonomy/perception/target_localization/core/rendering.py) | Respect existing model-output boundary; adapt rendering without wire-format changes |
| [tests](/home/stefi/DyNAMO/DyNAMO-Ridgeback/src/ridgeback_autonomy/test), [CMakeLists.txt](/home/stefi/DyNAMO/DyNAMO-Ridgeback/src/ridgeback_autonomy/CMakeLists.txt) | Baseline, parity/allocation coverage, test registration |
| [mask component](/home/stefi/DyNAMO/DyNAMO-Ridgeback/docs/localization/mask_component.md), [segmentation component](/home/stefi/DyNAMO/DyNAMO-Ridgeback/docs/localization/segmentation_component.md), estimator documents in the same directory | Update full-grid assumptions and explain precision policies |

## 8. Explicitly out of scope

AutoVision integration, multiprocessing/shared memory, GPU/model concurrency, worker scheduling, lock narrowing, exact-frame retries or buffer policies, camera resolution/alignment, detector/model changes, new isolation algorithms, thresholds/defaults, pointcloud-estimator changes, ROS message/topic changes, HUD/RViz layout changes, lifecycle/shutdown repairs, hardware validation, and broad benchmarking infrastructure.

If preserving a legacy contract or numerical parity genuinely requires changing one of those areas, stop that part and explain the conflict instead of expanding scope silently.

## 9. Completion report

Return: changed ownership/API summary; preserved versus intentionally new representation contracts; tests and exact commands; parity evidence; measured storage/timing results with limitations; installed build/launch checks; Graphify result; and any live validation not performed or pre-existing failures. Do not claim the latency backlog or depth-coverage concern solved merely because this refactor passes tests.

Stop after the native production path is in place, compatibility is confined to boundaries, existing behavior is proven, and validation/docs are complete. Do not start another latency issue.
