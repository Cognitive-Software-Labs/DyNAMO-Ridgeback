# Point-Cloud Provenance Test: Published vs. Deprojected

**Question:** euclidean reconstruction consumes an organized point cloud. That cloud can come from
two places — the **published** cloud (sim: gz `rgbd_camera` plugin; real:
realsense-ros pointcloud filter) or a **deprojected** cloud our own code builds
from the aligned depth frame. Is there any difference in *accuracy* or
*processing cost*? If not, should the published variant be kept at all?

> Resolved by §7: euclidean reconstruction deprojects masked depth pixels in
> code and never consumes a published cloud, so this question no longer gates
> the path — the material below is retained as the supporting evidence.

**Status:** script implemented at `~/tmp/cloud_provenance_test.py` (outside the
repo, never committed — delete after §6 is filled). Run with the sim + detector
up and the workspace sourced:

```bash
python3 ~/tmp/cloud_provenance_test.py --ros-args \
    -r __ns:=/r100_0001 -r /tf:=tf -r /tf_static:=tf_static \
    -p use_sim_time:=true \
    -p world:=target_distance_calibration \
    -p frames_target:=100
```

One run per G1 position; each writes `~/tmp/cloud_provenance_<label>.csv` and
prints column medians at exit. `pidstat` / `ros2 topic bw` / `delay` remain
manual, per §2. **Results collected 2026-07-11 — see §6. Decision taken — see
§7: Euclidean reconstruction uses the deprojected cloud only.** Script and CSVs kept in `~/tmp/`
for re-runs; delete once the decision is implemented.

---

## 1. Problem

Both clouds are deterministic functions of a depth map plus intrinsics, so the
published cloud carries **zero extra information**. The differences are all
engineering:

| Aspect | Published cloud | Deprojected cloud |
|---|---|---|
| Who computes | driver/plugin | our code (inverse pinhole) |
| Source depth (real) | **raw** depth, depth-optical frame | **aligned** depth (color grid) |
| Grid vs. mask | sim: color grid ✓; real: depth grid ✗ | color grid by construction ✓ |
| Organized | sim: yes; real: needs `ordered_pc` (not templated in `clearpath_config`) | by construction (H×W) |
| Depth sources | stereo only (Depth-Anything publishes no cloud) | any aligned depth frame |
| Bandwidth | full XYZ(RGB) cloud per frame | depth image only |
| Extra compute for us | none (parse only) | one vectorized deprojection per frame |

The architectural argument favors deprojection (grid-correct, source-flexible,
cheaper on the wire). This test verifies the argument empirically before the
published variant is dropped or demoted to a debug/reference role.

### Why the existing benchmark cannot answer this

The latest benchmark run (`artifacts/benchmarks/20260628_163046`, 70 trials) shows
`pointcloud` MAE 0.156 m vs. `sensor_depth` MAE 0.586 m — but those two
estimators use **different algorithms** (percentile-anchor reduction on the
cloud vs. focus-crop + Gaussian weighting on the depth image). The comparison
confounds *provenance* with *algorithm*. A clean test must hold the algorithm
fixed and vary only where the 3D points come from.

---

## 2. Test design

One throwaway script, run against the live sim, producing a CSV. Controlled
variables: same frames (stamp-synced), same ROI (same detection bbox), same
reduction function. Only the point source differs.

Three metric classes:

1. **Equivalence (cloud level).** Per synced frame, restrict both clouds to the
   detection ROI and compare point-by-point: max and RMS of
   `|published − deprojected|` per axis. In sim both derive from the same GPU
   Z-buffer, so the expected difference is float/encoding noise (≪ 1 mm). A
   larger systematic difference means the deprojection intrinsics are wrong —
   which is itself a finding (see §4, FoV caveat).
2. **Accuracy (estimate level).** Feed both point sets through the *identical*
   reduction (the existing percentile-anchor recipe from
   `compute_pointcloud_measurement`: 25th-percentile forward anchor, inliers
   −0.10/+0.35 m, nearest inlier, minus 0.25 m front offset) and compare each
   against ground truth from the Gazebo pose topic. Report per-frame error for
   both variants plus the paired difference.
3. **Processing.** Cost is accounted at three layers, because the published
   cloud is not free — *someone* computes it, it is just hidden in the driver:

   - **Producer (driver/plugin) CPU.** Measure the producing process with
     `pidstat -p <pid> 1` (or `top -b`) while toggling the work on and off:
     - sim: gz server CPU with vs. without a subscriber on `points`
       (gz computes lazily on subscription);
     - real: camera driver CPU with `pointcloud.enable` true vs. false, and
       separately `align_depth` true vs. false. Fair accounting: the published
       variant pays the pointcloud filter, the deprojected variant pays the
       align filter — and alignment is needed anyway for projective ranging and the mask
       work, so for the deprojected variant it is effectively a sunk cost.
   - **Wire.** Serialized message size per frame (`len(msg.data)`) for
     `points` vs. `depth/image`, `ros2 topic bw` on both as a rate
     cross-check, and `ros2 topic delay` (header stamp → arrival) to catch
     serialization/transport latency. Expected: PointCloud2 at 32 B/point
     (padded XYZRGB) → 640×480 ≈ 9.8 MB/frame ≈ 300 MB/s at 30 fps, vs. a
     640×480 depth image at ≈1.2 MB (32FC1) or ≈0.6 MB (16UC1) per frame —
     an 8–16× gap. Any higher-resolution profile scales both by pixel count.
   - **Consumer compute.** Per-frame wall time via `time.perf_counter`,
     reduction time excluded (identical for both variants). Three timings:
     1. parse `PointCloud2` → numpy (published variant),
     2. full-frame deprojection → numpy (naive deprojected variant),
     3. **masked-only deprojection with a cached ray table** — precompute the
        `(u − cx)/fx, (v − cy)/fy` grid once, then `XYZ = ray_table · Z` on
        only the masked pixels (~5–20 k instead of 307 k) — the form
        production code would actually use.

   Prediction to verify: naive full-frame deprojection (1–3 ms) loses to
   parsing (0.1–1 ms) on consumer compute alone, but masked-only + ray table
   is sub-millisecond, and the wire and producer layers dominate total system
   cost in the published variant's disfavor — decisively so on the real robot
   if compute is offboard.

**Scope: sim only.** The sim cloud is co-registered with RGB, so this test
cannot answer the real-robot grid question (published real cloud lives on the
depth grid; the mask lives on the color grid). The real-hardware comparison is
a separate session with its own prerequisites (`pointcloud.enable`,
`ordered_pc` passthrough, grid verification) — see
`docs/localization/object_localization_pipeline.md`.

---

## 3. Prerequisites

- Sim running with the sensor bridge up. As of writing, `gz sim` server runs
  but no camera topics are bridged — relaunch the stack
  (`target_distance_benchmark.launch.py` or `ridgeback_exploration.launch.py`)
  so these exist:
  - `/r100_0001/sensors/camera_0/color/image`
  - `/r100_0001/sensors/camera_0/depth/image`
  - `/r100_0001/sensors/camera_0/points`
  - detections topic from the running `target_detector_node` (script reuses live
    detections instead of loading its own model)
- A G1 spawned at a known pose (the benchmark runner's spawn machinery, or a
  manual `ros_gz_sim create` at a measured distance).
- Ground truth: `/world/<world>/pose/info` (the same source the benchmark
  runner uses), or a fixed known distance for a static scene.

### Intrinsics caveat (important)

Nothing in the repo subscribes to `camera_info` today; the estimators use FoV
values from `config/camera_config.json` (87°, the *real* D435), while the sim
camera renders with `horizontal_fov = 1.25 rad ≈ 71.6°` at 640×480. Deprojecting
with the config FoV would bake a systematic error into the deprojected variant
and invalidate the comparison. The script must therefore:

1. Prefer intrinsics from `sensors/camera_0/color/camera_info` if bridged
   (verify with `ros2 topic list | grep camera_info`);
2. else derive them from the sim URDF: `fx = fy = (W/2) / tan(HFoV/2)
   = 320 / tan(0.625) ≈ 442.1`, `cx, cy ≈ (W−1)/2, (H−1)/2`;
3. and cross-check by recovering intrinsics *from the published organized
   cloud itself* — for any valid point, `X/Z = (u − cx)/fx` — which
   self-calibrates the test and settles the gz center-pixel convention.

If (3) disagrees with (1)/(2), trust (3): the goal is to reproduce exactly what
the plugin computed.

---

## 4. Throwaway script plan

**Location:** outside the package, never committed — e.g.
`~/tmp/cloud_provenance_test.py`. Deleted after the numbers are in this doc.

**Shape:** a single rclpy node + `main()`, ~150 lines.

**Subscriptions** (all in namespace `r100_0001`):

| Topic | Type | Use |
|---|---|---|
| `sensors/camera_0/depth/image` | `sensor_msgs/Image` (32FC1) | deprojected variant input |
| `sensors/camera_0/points` | `sensor_msgs/PointCloud2` | published variant input |
| `sensors/camera_0/color/camera_info` | `sensor_msgs/CameraInfo` | intrinsics (if bridged) |
| detections topic (`TargetDetections`) | repo msg | shared ROI (bbox) |
| `/world/<world>/pose/info` | gz bridged pose array | ground truth |

**Sync:** match depth image, cloud, and detections on header stamp (exact match
works in sim — same render tick; keep small ring buffers keyed by
`(sec, nanosec)` like `benchmarking/alignment.py` does).

**Per synced frame:**

```text
1. bbox   <- detections (skip frame unless count == 1)
2. roi    <- focus crop of bbox (same crop the pointcloud estimator uses,
             so results are comparable with the existing benchmark)
3. A: parse PointCloud2 -> (H, W, 3) float32          [time this]
4. B: deproject depth   -> (H, W, 3) float32          [time this]
      X = (u - cx) / fx * Z ; Y = (v - cy) / fy * Z
5. equivalence: diff = A[roi] - B[roi] on valid pixels -> max / RMS per axis
6. reduction: identical percentile-anchor recipe on A[roi] and on B[roi]
   -> est_pub, est_dep   (reuse/replicate compute_pointcloud_measurement)
7. truth: planar distance from gz poses (robot yaw + 0.25 m front offset,
   same formula as the benchmark runner)
8. append CSV row:
   stamp, truth_m, est_pub_m, est_dep_m, err_pub_m, err_dep_m,
   cloud_diff_max_m, cloud_diff_rms_m,
   parse_ms, deproject_full_ms, deproject_masked_ms,
   points_msg_bytes, depth_msg_bytes,
   points_delay_ms, depth_delay_ms   # header stamp -> arrival
```

Alongside the script (not inside it): `pidstat` on the producer process and
`ros2 topic bw`/`delay` on both topics, per §2 layer 1 and 2 — run once per
scene, logged manually into §6.

**Run length:** ~100 synced frames per G1 position, 2–3 positions (e.g. 1.5 m,
3.5 m, 5.5 m forward) — enough for stable medians, minutes of wall time.

**Report:** medians of every column, printed at shutdown; CSV kept as raw
evidence. Numbers land in §6 of this doc.

Implementation notes:

- Parse the cloud with `sensor_msgs_py.point_cloud2` or a direct
  `np.frombuffer` on the msg buffer using the field offsets — the direct route
  is what the timing should measure (that is what production code would do).
- The script may import repo modules (`ridgeback_autonomy.perception.target_localization.core.*`,
  `geometry.py`) to guarantee the reduction is byte-identical to the existing
  estimator; if the function signature doesn't fit numpy arrays directly, a
  small adapter inside the script converts both variants to the same shape
  first, so any adaptation cost is shared.
- QoS: sensor-data (best-effort) to match the bridge.

---

## 5. Decision criteria

| Observation | Conclusion |
|---|---|
| cloud diff ≈ 0 **and** est_pub ≈ est_dep | published cloud adds nothing in sim → deprojected becomes canonical; published kept only as a debug/RViz reference and as the real-robot control case |
| deproject_ms ≪ frame period (expect 1–2 ms at 640×480) | no processing argument for the published variant |
| deproject_ms comparable to parse_ms | deprojection is not even a cost increase — drop the published variant from the benchmark matrix entirely |
| systematic cloud diff | intrinsics bug (FoV/center convention) — fix the deprojection, re-run; also flags the `camera_config.json` 87° vs. sim 71.6° mismatch as a live defect |
| est_dep clearly worse despite matching clouds | reduction is sensitive to something other than geometry (ordering, invalid-pixel handling) — investigate before trusting euclidean reconstruction plans |

Whatever the outcome, the real-robot question stays open: on hardware the two
variants read genuinely different depth (raw vs. aligned), so the sim result
only settles the *sim* rows of the benchmark matrix and validates the
deprojection code itself.

---

## 6. Results

Collected 2026-07-11. 100 synced frames per position at 1.5 / 3.5 / 5.5 m
forward (world `target_distance_calibration`, G1 at lateral 0, yaw π; 640×480).
Raw CSVs: `~/tmp/cloud_provenance_d{1p5,3p5,5p5}.csv`.

| Metric | Published | Deprojected |
|---|---|---|
| median abs error vs truth (m) | 0.090 / 0.168 / 0.166 (per position) | identical — paired diff 0.000 on all 300 frames |
| median cloud diff RMS (m) | ~1×10⁻⁷ (shared metric; max 4.2×10⁻⁷) | float32 noise; x-axis diff exactly 0 |
| median consumer cost (ms) | 0.72 (parse) | 0.87 (full) / 0.076 (masked + ray table) |
| median msg size (bytes/frame) | 7 372 800 (24 B/point padded XYZ) | 1 228 800 (32FC1) — 6.0× smaller |
| topic bandwidth (MB/s, observed) | 13–21 | 1.4–3.4 |
| topic delay, stamp→arrival (ms, sim time) | 18 | 14 |
| producer CPU delta (%) | ≈ 0 — gz server flat at ~175 % with the points bridge alive or killed; rendering dominates, cloud assembly not measurable | n/a in sim (no align step) |

Notes from the run:

- **The sim cloud is base-like (x fwd, y left, z up), not optical.** Confirmed
  empirically by the convention self-check (cloud X ≡ depth, not cloud Z). The
  frame_id still suggests an optical frame, so the TF reduction path returns
  nothing and production code takes the raw-cloud fallback on **every** frame
  (`used_tf = 0` across all 300). The fallback in `add_pointcloud_measurements`
  is therefore the *primary* sim path, not an edge case.
- **Intrinsics: no live defect.** Cloud-fitted fx = fy = 443.53,
  cx = 319.50, cy = 239.50 — matches both the URDF-derived value
  (320 / tan 0.625) and the bridged `camera_info` (443.5 / 320 / 240). The §3
  caveat about `camera_config.json` 87° remains relevant only if someone
  deprojects with the config FoV instead of `camera_info`.
- **`camera_info` IS bridged** in the current stack
  (`sensors/camera_0/color/camera_info` and `.../depth/camera_info`) — §3's
  "verify" is settled; a production deprojector should just subscribe to it.
- The abs-error-vs-truth values reflect the percentile-anchor reduction
  measuring the nearest G1 surface against model-origin ground truth — a
  property of the algorithm, shared bit-for-bit by both variants, and not a
  provenance effect. Scene is static and sim depth is noise-free, so each
  position converges to a single repeated estimate.
- `ros2 topic delay` is useless here (wall clock vs sim stamps); the script's
  own sim-time stamp→arrival delta is what the table reports.
- Producer accounting caveat: `parameter_bridge` subscribes on the gz side
  eagerly, so the cloud is computed and shipped whether or not any ROS node
  consumes it — the published variant's wire cost is paid continuously by
  merely launching the stack.

Verdict (accuracy): **equivalent.** Cloud difference is float-encoding noise
(≤ 4.2×10⁻⁷ m) and both variants produce bit-identical estimates through the
shared reduction on every frame. The published cloud adds zero information in
sim, as predicted.

Verdict (processing): **deprojection wins outright.** Masked deprojection with
a cached ray table (0.076 ms) is ~10× cheaper than parsing the published cloud
(0.72 ms); even naive full-frame deprojection (0.87 ms) is within noise of
parse cost. The wire layer favors the depth image 6× per frame, and the
producer layer showed no measurable savings from disabling the cloud — so per
§5 row 3, the published variant can be dropped from the sim benchmark matrix
entirely, kept only as a debug/RViz reference and as the real-robot control
case. The real-hardware comparison (raw vs aligned depth, grid mismatch)
remains open per §2 scope.

---

## 7. Decision (2026-07-11)

**Euclidean reconstruction uses the deprojected cloud only.** The published cloud is not a
pipeline input anywhere — it keeps exactly one role: optional RViz/debug
visualization.

Grounds (per §5 criteria, hit rows 1–3 simultaneously): accuracy equivalent
(bit-identical estimates, cloud diff = float noise), consumer compute favors
deprojection (~10× with masked ray table), wire cost 6× against the published
cloud, producer savings nil, and the two architectural facts the sim test
cannot change — the monocular depth source has no published cloud, and the
real-hardware published cloud lives on the wrong grid for mask indexing.

Consequences applied to the other docs:

- `docs/localization/euclidean_reconstruction.md` — deprojected is the sole input; the published-variant
  caveats collapse into a historical note pointing here.
- `docs/localization/object_localization_pipeline.md` — the organized point cloud is no longer
  listed as a pipeline input; "euclidean reconstruction has no input on hardware" is void (euclidean reconstruction
  needs only the aligned depth frame there).
- The `pointcloud.enable` / `ordered_pc` config gap on the real robot is
  **downgraded from blocker to irrelevant-for-the-paths** (matters only if
  someone wants the RViz debug cloud). The config gap that *does* matter on
  hardware is `align_depth.enable` (`docs/localization/aligned_depth.md` §2.1).
- The planned real-hardware published-vs-deprojected comparison is **cancelled**
  — with no production role for the published cloud there is nothing left to
  compare; hardware validation effort moves to the aligned-depth topic itself
  (grid check, hole rate, `align_depth` passthrough).
- The existing `pointcloud` estimator keeps consuming the published topic until
  euclidean reconstruction replaces it; its benchmark row is now understood as "deprojected-
  equivalent geometry + percentile-anchor reduction" per §6.
