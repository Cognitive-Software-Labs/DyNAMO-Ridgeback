# BENCHMARKING — Stationary hardware scene recording

Status: **agreed recording plan, 2026-10-27; recording not started**. This
document contains the recording procedure, measurement definitions and full
63-row scene menu for the stationary real-hardware benchmark capture.

## Agreed scope and result

Record raw sensor bags and independently measured G1 positions for later
projective ranging, euclidean reconstruction and polar profiling comparisons.
Pointcloud is excluded. **Do not start or run detection, segmentation, depth
inference or distance estimators for this capture session. Do not record their
output topics.** Existing application services are not stopped or reconfigured
by this plan; note any already-running workload in the session metadata.

The Ridgeback remains stationary and e-stopped. People position the single
crane-supported G1 with its feet touching the floor; no robot motion commands
are sent. Available props are three pole-like objects, one table, a panel taller
than the G1 and a cabinet that can block the LiDAR. Measure the actual usable
room dimensions rather than assuming a guaranteed 4 m working distance.

The intended coverage is **conditions represented by up to 63 of the 88 source
scenes, using adapted hardware arrangements**, not 63 exact reproductions.
Twenty multi-G1 scenes and five approximately 8–11 m scenes are excluded.
Retain distinct scene IDs, documented substitutions and skipped conditions.
One recording mapped to several simulator IDs remains one recording/trial.

The [fixed source YAML](../../src/ridgeback_autonomy/config/benchmark_scenarios_full.yaml)
owns simulator coordinates. This session collects inputs and labels; later
inference, parameter selection, scoring and benchmark-import tooling are outside
this recording task. Reuse completed hardware bring-up. There is no new
calibration campaign, simulator launch or separate initial pilot prerequisite.

## 1. Establish measurement uncertainty first

Perform the following with the stationary setup before collecting benchmark
bags. Write the raw readings, instrument resolution/accuracy where known,
method, photos and resulting bounds to `setup/measurement-method.json`.

1. Identify and photograph the **pelvis housing margins** used for the whole
   session: left/right and front/back. Project those margins vertically onto
   the floor with a plumb line or equivalent vertical projection. Find the
   midpoint in each direction and mark the resulting pelvis centre and body
   heading. Use consistent housing landmarks; exclude hands, thighs, feet,
   harness and crane. This is a defined geometric pelvis reference, not centre
   of mass or an estimated centre of the whole visible body.
2. Define the Ridgeback tape origin `F` as the centreline intersection with the
   floor projection of its foremost fixed body surface. Photograph that surface
   and mark `F`, the forward axis and a perpendicular lateral axis using
   accessible exterior edges and a straightedge/square. No measurement under
   the chassis or physical location of `base_link` is required. Use planar floor
   distances, not a sloping tape to a raised pelvis surface.
3. With the G1 at one usable position, repeat the complete margin projection,
   midpoint construction and x/y measurement **five times**, lifting/resetting
   the measuring tools between readings. Keep individual readings; use a second
   person for an independent repetition if available. Re-reading an unmoved
   tape is not an independent measurement. Record heading and posture.
4. Let the G1 settle and confirm it is visibly stationary before recording.
   Measure the projected pelvis midpoint at the initial placement and check it
   again only after the G1 has been moved or rotated. Reuse that measurement
   across prop changes and recordings at the same G1 pose; there are no routine
   before/after-clip pelvis checks. No separate 20-second sway measurement or
   continuous plumb-line monitoring is required. Record unquantified residual
   movement honestly rather than claiming a precise sway bound. Torso lean
   affects the position label only insofar as it shifts the pelvis midpoint.
5. Estimate separate per-axis contributions for projection/endpoint ambiguity,
   floor-mark width and alignment, instrument accuracy, repeated-measurement
   spread, and any bounded residual placement/movement. Record unquantified
   movement separately rather than inventing a numerical bound for it.
   Derive conservative `u_x` and `u_y` by
   summing plausible maximum contributions **without counting the same effect
   twice**. Repetition does not expose a consistently wrong landmark or datum.
   Label these working bounds, not statistical 95% confidence intervals.
6. Record `u_distance <= sqrt(u_x^2 + u_y^2)` as a conservative planar-distance
   bound for the physical front-mark measurements. This includes uncertainty
   in marking/alignment of `F`; it does not include an as-yet unknown conversion
   to a software coordinate origin.
   Do not promise a centimetre bound before measuring. If uncertainty is too
   large to distinguish the intended errors, improve the projection/marks or
   document the limitation before collecting the remaining scenes.

For a midpoint made from two margin coordinates, its uncertainty from those
margins is bounded by half the sum of their individual bounds. Include common
datum/alignment uncertainty separately. Save the full readings so the bound can
be audited later. Repeat or enlarge this uncertainty assessment if posture,
landmarks, instruments or measurement conditions change materially.

## 2. Establish the coordinate grid and capture setup

### Ground-truth coordinates

Keep original tape readings in frame `F`: `x_F` forward, `y_F` left, metres.
All real G1 and prop positions in the scene table are **direct tape targets
from this accessible front floor mark**, with `r_F = hypot(x_F, y_F)`. For
example, P01 means 2.70 m forward of the front mark, centred on its forward
line. No operator must identify a hidden robot-frame origin, measure beneath
the Ridgeback, or calculate a frame conversion to place the G1.

The earlier draft labelled the same numeric proposals as B-grid positions.
They are now explicitly proposed **front-mark placements**, not a numerical
conversion of the old draft: the unknown offset has not been guessed. Source
simulator coordinates remain unchanged, and actual hardware positions are
measured and logged independently.

Document the exterior surface and midpoint construction used for `F`, the
straightedge/square alignment method, photos and repeatability. If readily
accessible, also save a signed sensor-housing-to-front-mark measurement with
identified endpoints and uncertainty. This is supporting evidence for later
analysis, not a requirement to reach underneath the robot. A housing face is
not automatically the sensor optical/emission origin; the
[geometry reference](../robot/geometry.md) distinguishes those measurements.

### Software-coordinate correspondence is deferred

The current pipeline reports positions relative to `base_link` with a configured
forward offset; see
[vehicle-frame conversion](../../src/ridgeback_localization/ridgeback_localization/core/vehicle_frame.py).
The physical front mark is not assumed to equal that reporting origin. Save TF
and configuration as usual, but record `front_mark_to_estimator_origin` as
**unknown / deferred** unless supported by independent evidence. TF gives
relationships between software frames; it does not by itself establish the
physical front surface's position. The default 0.25 m is not a measurement of
that surface. Do not alter deployed TF or parameters during capture.

Locating `base_link` physically or solving this conversion is **not a recording
prerequisite**. Later scoring must establish the correspondence with supported
geometry or accessible reference measurements, include its uncertainty, and
convert either estimates or labels into one agreed frame. Until then, retain
the front-mark ground truth and explicitly withhold claims of precise absolute
error in the estimator frame. A fixed origin discrepancy can bias every scene;
it must not be hidden inside the pelvis/tape repeatability bound or fitted to
these estimator outputs and then treated as independent ground truth.

The accepted pelvis midpoint is not yet proven to coincide with either
simulator model origin. Preserve the landmark photos, posture and heading so
that relationship can be established before reporting cross-environment errors.
Do not substitute the nearest visible surface when a prop hides the pelvis.

### Session and recording preparation

- Take the exclusive robot/Intel/Thor resource window described in the priorities
  document. Preserve existing services, ROS domain, DDS configuration, camera
  profiles, TF and sensor mounts. Start/stop only session-owned recorder and
  bounded checking processes. No `cleanup.sh`, playback into the live graph,
  rebuild, install or application launch is part of this session.
- Create a **new UTC timestamped** session, for example
  `artifacts/hardware/<UTC>-distance-benchmark-raw/` on the recording host,
  with an archive under `/home/stefi/ScenesForDetectionTesting/<same-session>/`.
  Leave `20261025T123050Z-detector-trials/` unchanged.
- Observe the actual recording host, namespace, topic names, QoS and frame IDs;
  expected hardware identity is `r100_0160`, not an assumption. Read the relevant
  [camera/deployment](../physical/robot_local_deployment.md) and
  [Intel–Thor transport](../physical/intel_thor_transport.md) instructions before
  using the existing environment. Keep custom inference dependencies out of the
  raw recorder.
- Snapshot recording-host checkout revision/status, camera configuration,
  intrinsics, depth scale/alignment, active relevant workload, ROS environment,
  message definitions and recorder/QoS versions in `setup/` and `session.json`.
  Record actual values; missing information stays explicitly unknown.
- The read-only source archive is
  `/home/stefi/ScenesForDetectionTesting/20261025T123050Z-detector-trials/`.
  Its `tools/` contains the recording/logging/validation/transfer helpers;
  `setup/` contains recorder QoS and environment/configuration snapshots;
  `session.json`, `segments.jsonl`, `inventory.json` and `quality/` show the
  previous session structure. Its detector/estimator outputs were reference-only
  and are excluded from this session.
- Adapt copies of the previous session's `rec.py`, `validate.py`, `log.py` and
  inventory/transfer helpers **inside the new session**, without editing the old
  archive. Remove old scene defaults, hard-coded archive paths and inference
  subscriptions. Match validation thresholds to the observed sensor profile;
  do not blindly carry over a namespace, frame or configuration.
- Keep uncompressed MCAP with its `metadata.yaml`, original message headers and
  receive timestamps. Preserve transient-local static TF from every publisher
  in **each** bag. The old recorder's subscription readiness plus lead/tail
  approach can be reused; readiness means subscriptions, and saved messages
  must still pass validation.
- Check actual recording-host capacity and archive access. The previous archive
  averaged about 46 MB/s, about 1.1 GB per 20-second scene including margins.
  Allow roughly 70 GB for 63 attempts, plus headroom for retries and logs;
  derive the final budget from the first actual bag. Preserve local copies by
  default. Copy and checksum-verify completed bags; automatic deletion from the
  old offload helper is disabled for this session.

### Raw topics

Use the actual namespace in place of `<ns>`. These nine raw streams preserve
the previous archive's sensor inputs while excluding its three inference-output
topics. Front scan is the intended polar input; retain the other scans for
context and future inspection.

| Topic under `/<ns>/` | Message type | Purpose |
|---|---|---|
| `sensors/camera_0/color/image` | `sensor_msgs/msg/Image` | Original detector/segmentation input |
| `sensors/camera_0/color/camera_info` | `sensor_msgs/msg/CameraInfo` | Colour-grid intrinsics |
| `sensors/camera_0/depth/image` | `sensor_msgs/msg/Image` | Depth aligned to colour |
| `sensors/camera_0/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | Preserve native depth metadata; informational |
| `sensors/lidar2d_0/scan` | `sensor_msgs/msg/LaserScan` | Front LiDAR input |
| `sensors/lidar2d_1/scan` | `sensor_msgs/msg/LaserScan` | Rear LiDAR context |
| `sensors/scan` | `sensor_msgs/msg/LaserScan` | Merged scan context |
| `tf` | `tf2_msgs/msg/TFMessage` | Recorded dynamic transforms |
| `tf_static` | `tf2_msgs/msg/TFMessage` | Complete static sensor chains |

In the previous session, colour was `rgb8` 640×480 and aligned depth was
`16UC1` 640×480 on the colour optical frame. Its native depth CameraInfo carried
different intrinsics and frame: aligned-depth processing uses colour intrinsics.
Verify the actual new-session contract instead of treating those old readings
as a current live check. Do not subscribe to `detections/target/raw`,
`measurements/target/mask`, localization health or pointcloud topics.

## 3. Record the grouped scenes

Use the [63-row scene table below](#6-scene-menu-and-simulator-correspondence)
as the menu/order. It retains the agreed `Pxx-yy` IDs. `Pxx` changes require G1
translation; within a position block the action column identifies rotations.
Complete prop changes at a fixed heading before turning the G1. Recheck the
pelvis mark after every turn; a turn is not evidence of unchanged position.
For KEEP rows, reuse the current measured pelvis position without checking or
remeasuring it. Moving only props does not trigger a pelvis-position check.
The table specifies starting geometry, not ground truth. Keep the Ridgeback
fixed for the whole session. Measure feasible room depth/width and occupied
object extents before choosing marks. If a condition does not fit, adapt it
and record the change, or skip it without forcing a nominal coordinate.

For each distinct physical arrangement:

1. Log the row ID, position/heading ID, corresponding benchmark IDs and intended
   condition. Mark reused recordings explicitly; never count a mapped alias as
   a second independent trial. Clear movable props from a clean baseline.
2. At the initial placement and after moving or rotating the G1, measure/check
   its pelvis midpoint x/y, heading and posture before props hide it. Otherwise
   reuse the current placement record without a pelvis-position check. Remove
   measuring tools from the sensor view. Record which points are measured and
   which dimensions remain estimates.
3. Place the relevant props, preserving G1 position. Record named prop
   reference points, x/y, heading and dimensions; record actual surface gaps
   for near-contact cases. Photograph the arrangement and note crane/mast,
   background and any camera or LiDAR obstruction. A tall panel can hide a
   vertical half; it is not automatically a replacement for a low obstacle.
4. Verify the intended RGB/scan geometry with bounded raw-input checks, without
   running inference. For example, a cabinet must actually interrupt target
   bearings at the LiDAR plane; a table must have the intended open path. With
   no detector running, “inside box/mask” is a proposed projected-body overlap,
   not a certified detection-box or mask outcome.
5. Once people and tools are out of view and the G1 has settled, start a new
   bag named `NN_Pxx-yy_attemptK`. Wait for required subscriptions, retain a
   lead interval, mark the start, record **20 seconds settled**, mark the end,
   then retain a tail interval and stop only the recorder cleanly. Save exact
   UTC/receive-time boundaries. Take each attempt separately; no overwrites.
6. Validate and append the result before proceeding. Validate the **first actual
   scene immediately** to check helper adaptation and sensor matching; this is
   part of normal capture, not a separate inference/scoring pilot.

Baseline is one accepted 20-second recording per distinct arrangement, not an
unagreed multi-repeat experiment. Extra attempts address raw-data/placement
failures. The 20 seconds provide temporal samples, not independent placement
replications. Any later repetitions should preserve that distinction.

## 4. Validate and accept each attempt

Accept only when raw inputs are readable and complete, the ground-truth record
is complete within its stated bound, and the intended physical condition is
present. No detector/estimator success is required or assessed at this stage.

- Check clean recorder shutdown, MCAP readability, metadata, required types and
  nonempty streams. Preserve the exact command, exit status and recorder log.
- Check image dimensions, encoding, optical frame, timestamp continuity, frame
  rates and RGB/depth pairing over the settled interval. The previous validator
  used ≥29 Hz, ≤150 ms image-stamp gaps and ≥99% pairing for its 30 Hz profile;
  retain these if the same profile is verified, otherwise document the actual
  profile and justified criteria before recording.
- Check scan rates/gaps and **header-stamp matching to RGB**, not just receive
  rates. Count nearest front scans within the intended later processing
  tolerance (previous configuration: 50 ms); retain the gap distribution and
  unmatched stamps. Preserve mismatches for diagnosis rather than hiding them
  through frame selection.
- Check complete `base_link`→colour optical **and LiDAR** TF chains in every
  bag. The previous helper checked the camera chain; extend its recorded checks
  to the scan-to-camera relationship needed for polar profiling. This is an
  automated check of recorded transforms, not a physical measurement of
  `base_link` underneath the robot.
- Save raw previews near the start/middle/end and relevant scan evidence for
  clipping, occlusion and scan-blocking arrangements. Record intended versus
  observed condition without adopting historical simulator visibility fractions.
- Link the bag to the measured G1 placement/heading record and log prop positions
  and deviations. Update the pelvis measurement only after moving or rotating
  the G1; no per-clip position checks are required at an unchanged pose.
  Keep `raw_pass` and `scene_accepted` separate, including a reason for each
  rejection/skip. If several nominal rows become indistinguishable, record one
  physical case and list its aliases explicitly.

## 5. Save ground truth, archive and close the session

Reuse this directory structure from the previous session, adding structured
measurement records rather than estimated placement prose:

```text
<new-session>/
  session.json
  segments.jsonl
  inventory.json
  bags/NN_Pxx-yy_attemptK/{metadata.yaml,*.mcap}
  setup/                 # Method, repeated readings, front-mark photos, snapshots
  ground-truth/          # Versioned placement/heading and prop measurements/photos
  quality/               # Timing, recorder logs, validation, previews, checksums
  tools/                 # Session-specific helper/config snapshots
  ros-logs/
```

| Record | Required content |
|---|---|
| `session.json` | Schema version, UTC/session identity, host/revision/configuration, namespace/frames, raw topic contract, physical front-mark definition/photos, measurement method, software-origin correspondence explicitly unknown/deferred if unresolved, fixed room/crane conditions, storage paths |
| Placement record | Immutable `placement_id` including heading revision; pelvis landmarks/margins and original readings; measured `x_F,y_F,r_F`, yaw and posture; units; per-axis uncertainty/components; photographs; initial measurement or check after G1 movement/rotation; parent measurement-method/front-mark versions; no per-clip checks or converted coordinates required |
| Prop record | Prop ID/type/dimensions, physical reference definition, measured pose and uncertainty, relevant surface gap/scan obstruction, arrangement photo |
| `segments.jsonl` | Stable row/arrangement and bag IDs, attempt, linked benchmark IDs, ground-truth record/version, prop record IDs, settled interval, raw/scene status and reasons, deviations/aliases, quality report and archive-relative paths |
| `inventory.json` | Final attempts/status counts, represented/skipped benchmark IDs, distinct physical cases, file sizes/checksums, total duration, validation and archive completeness |

Reuse the same placement label across prop changes and recordings **until the
G1 is moved or rotated**, without checking the pelvis position between clips.
A translation or turn triggers a new placement/heading check record, even if
the nominal floor mark is the same. Keep original readings and append corrections
with version/reason instead of overwriting labels. Ground-truth files should
contain no model predictions.
Use relative paths within a session so archive transfer does not break links.

Copy closed bags and all matching metadata to the new archive. Verify complete
file inventories and SHA-256 hashes; retain failed attempts. Do not use the
old archive as an output destination or run its deletion helper unchanged.
At the end, close all session-owned processes, perform a bounded final raw-input
check, regenerate the inventory and list completed, adapted, merged and skipped
rows with reasons. Preserve interrupted/incomplete session status honestly.

Completion means every attempted bag has its validation result and measured
label, every menu row has an explicit disposition, and the archive copy is
verified. It does not mean the hardware bags have been imported into the
simulator benchmark, or that accuracy/throughput has been evaluated. Those
later activities must use the same source frames and clearly report centre
reference, uncertainty, geometry differences, coverage and misses; see
[benchmark semantics](../target_distance_benchmarking/semantics.md).

## 6. Scene menu and simulator correspondence

### How to read this table

- These are conditions represented by up to 63 source scenes using adapted
  hardware arrangements. The sole source for simulator poses is
  [benchmark_scenarios_full.yaml](../../src/ridgeback_autonomy/config/benchmark_scenarios_full.yaml).
  All source target and object x/y/yaw values below reproduce that YAML exactly.
  They are model-origin/world-coordinate inputs, not physical surface positions
  or already front-offset-adjusted scored distances.
- Real coordinates are **proposed tape targets from the physical front floor
  mark F**, in metres: forward x, lateral y positive left. G1 coordinates refer
  to the agreed projected pelvis midpoint. Measure directly from the front mark
  and its axes; no physical `base_link` location or frame conversion is needed.
  Real `r` is `hypot(x,y)` from F, not camera range or nearest-surface clearance.
  Numeric targets are retained as new front-mark proposals, not converted from
  the earlier B-grid draft using an assumed offset. Simulator poses are unchanged.
  Real yaw is radians about up; 3.14 faces the Ridgeback, 0 faces away, 4.70 is
  side-on. Each yaw describes the G1, not an inferred camera bearing.
- `Pxx-yy` preserves the previously discussed menu IDs. **PLACE** means translate
  G1 to a different floor mark; **TURN** means rotate and recheck its centre;
  **KEEP** means leave G1 position/heading unchanged and alter only props, reusing
  its measured placement without a pelvis-position check.
  Execute all rows for a heading before turning. This practical grouping has
  11 nominal floor marks; it is not a proven global movement minimum.
- Real object x/y values are **starting points** for a recorded physical
  reference (pole shaft centre; table/cabinet/panel footprint centre). Sizes,
  orientations and usable room width are not yet measured. Set object heading
  to achieve the stated condition and log it; do not silently treat different
  furniture origins or full-size substitutes as scaled simulator models.
  Measure real surface gaps for near-contact rows. Observe RGB and scan geometry
  without inference; detection-box/mask outcomes are only assessed later.
- P01 is the main 2.70 m mark. P02 at 3.20 m is conditional on room/crane
  clearance; if unavailable, use the P01 views where equivalent and label the
  lost range distinction. Edge x/y values are provisional and must be adjusted
  to observed clipping. A point fitting inside 3–4 m is not proof that the
  cabinet/table/crane footprint fits. Adapt or skip on site and log actual poses.
- Compared with the earlier scratch table, several background-object positions
  have been shortened into the nominal room envelope, and two far lateral
  controls brought inward. The affected rows explicitly flag changed spacing.
  Background separation and interference strength are not claimed identical to
  simulation. Other props still need footprint/visibility checks.
- A tall panel can hide a vertical half or most of the G1; use the cabinet/table
  where low-body hiding or scan blocking is needed. Edge-on panel splitting and
  table scan-through are conditional on their actual thickness/leg geometry.
  The three poles available exceed the two used simultaneously in this menu.
- One actual arrangement may represent several benchmark IDs. In particular,
  P02-03/P02-04 are the same proposed hardware scene: record once and map both.
  Mark any additional merges explicitly; never count an alias as an independent
  observation. Historical simulator visibility percentages are not current
  camera certifications and are deliberately omitted.

### Position blocks

| Block | Nominal G1 (x_F,y_F) m from front mark | Rows | Movement purpose |
|---|---|---:|---|
| P01 | (2.70, 0.00) | 49 | Main clutter block; group back, front, side and angled views |
| P02 | (3.20, 0.00) | 5 | Longest feasible range; conditional, with one shared arrangement |
| P03 | (2.00, +0.30) | 1 | Closer back-facing baseline |
| P04 | (1.30, 0.00) | 1 | Centred near clipping |
| P05 | (1.30, -0.65) | 1 | Lateral/rotated near clipping |
| P06 | (1.10, 0.00) | 1 | Closest centred clipping |
| P07 | (2.00, -1.25) | 1 | Mostly visible at right image edge |
| P08 | (2.00, -1.55) | 1 | Partly outside right image edge |
| P09 | (2.00, +1.25) | 1 | Mostly visible at left image edge |
| P10 | (2.00, +1.55) | 1 | Partly outside left image edge |
| P11 | (2.00, +1.80) | 1 | Sliver at left image edge |

### Scene rows

All real x/y/r values are measured from front mark F in metres; yaw values are
radians relative to its forward axis. `R` means the simulated
G1 model origin, `O` a simulated object origin. Simulator tuples are `(x,y,yaw)`.
The simulator runner derives scored ground truth from the actual spawned
Ridgeback and target poses; the YAML tuples alone are not published estimator
coordinates. The later physical-pelvis/model-origin correspondence remains to
be documented before cross-environment accuracy claims.

| Row / G1 action | Physical condition | Real G1 (x_F,y_F); yaw; r_F | Real props (x_F,y_F); r_F | Benchmark ID | Simulator target R (x,y,yaw) | Simulator objects O (model; x,y,yaw) | Adaptation / check |
|---|---|---|---|---|---|---|---|
| **P01-01 PLACE** | Unoccluded G1; back view | (2.70, +0.00); yaw=0.00; r=2.70 | none | `single_back_03` | (2.70, -0.45, 0.10) | none | Clear movable clutter; document fixed crane/background. |
| **P01-02 KEEP** | Pole ahead of G1 within projected body extent | (2.70, +0.00); yaw=0.00; r=2.70 | pole (2.11, +0.11), r=2.11 | `interfere_infront_03` | (2.70, -0.45, 0.10) | iv_pole (2.11, -0.34, 0.00) | Verify intended overlap; measure final prop pose. |
| **P01-03 TURN** | Unoccluded G1; facing view | (2.70, +0.00); yaw=3.14; r=2.70 | none | `single_facing_03` | (2.90, +0.00, 3.14) | none | Clear movable clutter; document fixed crane/background. |
| **P01-04 KEEP** | Background object on G1 bearing | (2.70, +0.00); yaw=3.14; r=2.70 | cabinet (3.45, +0.10); r=3.45 | `interfere_bearing_01` | (2.42, +0.34, 2.98) | hospital_bed (5.31, +0.79, 0.80) | Shorter spacing / reduced lateral offset; verify furniture footprint and intended effect. |
| **P01-05 KEEP** | Background object on G1 bearing | (2.70, +0.00); yaw=3.14; r=2.70 | cabinet (3.45, -0.10); r=3.45 | `interfere_bearing_02` | (2.68, -0.96, 3.47) | hospital_bed_high (5.41, -2.04, 1.53) | Shorter spacing / reduced lateral offset; verify furniture footprint and intended effect. |
| **P01-06 KEEP** | Object behind G1 | (2.70, +0.00); yaw=3.14; r=2.70 | cabinet (3.35, +0.10); r=3.35 | `interfere_behind_01` | (2.45, +0.21, 3.64) | hospital_bed_high (4.12, +0.47, 1.93) | Shorter spacing / reduced lateral offset; verify furniture footprint and intended effect. |
| **P01-07 KEEP** | Object behind G1 | (2.70, +0.00); yaw=3.14; r=2.70 | cabinet (3.35, -0.10); r=3.35 | `interfere_behind_02` | (3.21, -0.42, 3.51) | cabinet (4.69, -0.61, 3.02) | Shorter spacing / reduced lateral offset; verify furniture footprint and intended effect. |
| **P01-08 KEEP** | Object behind G1 | (2.70, +0.00); yaw=3.14; r=2.70 | pole (3.38, +0.01), r=3.38 | `interfere_behind_03` | (2.80, +0.05, 3.14) | iv_pole (3.50, +0.06, 0.00) | Verify intended overlap; measure final prop pose. |
| **P01-09 KEEP** | Object behind G1 | (2.70, +0.00); yaw=3.14; r=2.70 | pole (3.26, -0.08), r=3.27 | `interfere_behind_04` | (3.30, -0.40, 3.35) | iv_pole (3.99, -0.50, 0.00) | Verify intended overlap; measure final prop pose. |
| **P01-10 KEEP** | Object behind G1 | (2.70, +0.00); yaw=3.14; r=2.70 | pole (3.26, +0.06), r=3.26 | `interfere_behind_05` | (3.40, +0.30, 2.98) | iv_pole (4.10, +0.37, 0.00) | Verify intended overlap; measure final prop pose. |
| **P01-11 KEEP** | Object beside G1 | (2.70, +0.00); yaw=3.14; r=2.70 | table (2.64, +0.84), r=2.77 | `interfere_beside_01` | (3.65, -0.99, 3.39) | table (3.57, +0.14, 0.11) | Verify intended overlap; measure final prop pose. |
| **P01-12 KEEP** | Object beside G1 | (2.70, +0.00); yaw=3.14; r=2.70 | cabinet (2.69, -0.84), r=2.81 | `interfere_beside_02` | (3.81, +0.18, 2.90) | cabinet (3.79, -1.00, 1.75) | Verify intended overlap; measure final prop pose. |
| **P01-13 KEEP** | Clutter clear of target | (2.70, +0.00); yaw=3.14; r=2.70 | table (2.05, +1.39), r=2.47 | `interfere_control_01` | (4.30, -0.87, 3.46) | table_low (3.26, +1.34, 2.35) | Verify intended overlap; measure final prop pose. |
| **P01-14 KEEP** | Clutter clear of target | (2.70, +0.00); yaw=3.14; r=2.70 | pole (1.65, +1.00); r=1.93 | `interfere_control_02` | (3.84, -1.44, 3.41) | iv_pole (2.34, +1.42, 0.14) | Shorter spacing / reduced lateral offset; verify furniture footprint and intended effect. |
| **P01-15 KEEP** | Clutter clear of target | (2.70, +0.00); yaw=3.14; r=2.70 | cabinet (3.10, +0.95); r=3.24 | `interfere_control_03` | (2.83, -0.90, 3.50) | cabinet_tall (3.41, +1.78, 1.32) | Shorter spacing / reduced lateral offset; verify furniture footprint and intended effect. |
| **P01-16 KEEP** | Clutter clear of target | (2.70, +0.00); yaw=3.14; r=2.70 | pole (2.06, +0.35), r=2.09 | `interfere_control_04` | (2.80, +0.10, 3.15) | iv_pole (2.14, +0.46, 0.00) | Verify intended overlap; measure final prop pose. |
| **P01-17 KEEP** | Pole ahead of G1 within projected body extent | (2.70, +0.00); yaw=3.14; r=2.70 | pole (2.00, -0.04), r=2.00 | `interfere_infront_01` | (2.30, +0.10, 3.05) | iv_pole (1.70, +0.07, 0.00) | Verify intended overlap; measure final prop pose. |
| **P01-18 KEEP** | Pole ahead of G1 within projected body extent | (2.70, +0.00); yaw=3.14; r=2.70 | pole (2.14, +0.06), r=2.14 | `interfere_infront_02` | (2.90, -0.25, 3.28) | iv_pole (2.30, -0.19, 0.00) | Verify intended overlap; measure final prop pose. |
| **P01-19 KEEP** | Pole ahead of G1 within projected body extent | (2.70, +0.00); yaw=3.14; r=2.70 | pole (2.13, +0.04), r=2.13 | `interfere_infront_04` | (2.95, +0.40, 3.00) | iv_pole (2.33, +0.44, 0.00) | Verify intended overlap; measure final prop pose. |
| **P01-20 KEEP** | Two poles at different ranges/bearings | (2.70, +0.00); yaw=3.14; r=2.70 | pole (3.33, +0.00), r=3.33; pole (2.16, +0.13), r=2.16 | `interfere_pair_01` | (3.00, +0.00, 3.14) | iv_pole (3.70, +0.00, 0.00); iv_pole (2.40, +0.14, 0.00) | Verify intended overlap; measure final prop pose. |
| **P01-21 KEEP** | Two poles at different ranges/bearings | (2.70, +0.00); yaw=3.14; r=2.70 | pole (2.12, -0.06), r=2.12; pole (3.03, -0.16), r=3.03 | `interfere_pair_02` | (2.80, +0.25, 3.25) | iv_pole (2.20, +0.19, 0.00); iv_pole (3.14, +0.08, 0.00) | Verify intended overlap; measure final prop pose. |
| **P01-22 KEEP** | Two poles at different ranges/bearings | (2.70, +0.00); yaw=3.14; r=2.70 | pole (2.14, +0.13), r=2.15; pole (3.00, -0.20), r=3.00 | `interfere_pair_03` | (2.90, +0.00, 3.14) | iv_pole (2.30, +0.14, 0.00); iv_pole (3.22, -0.21, 0.00) | Verify intended overlap; measure final prop pose. |
| **P01-23 KEEP** | Two poles at different ranges/bearings | (2.70, +0.00); yaw=3.14; r=2.70 | pole (2.39, +0.03), r=2.39; pole (1.93, -0.04), r=1.93 | `interfere_pair_04` | (2.85, -0.25, 3.30) | iv_pole (2.52, -0.22, 0.00); iv_pole (2.04, -0.29, 0.00) | Verify intended overlap; measure final prop pose. |
| **P01-24 KEEP** | Two poles at different ranges/bearings | (2.70, +0.00); yaw=3.14; r=2.70 | pole (3.34, +0.05), r=3.34; pole (2.38, +0.12), r=2.38 | `interfere_pair_05` | (2.95, +0.20, 3.18) | iv_pole (3.65, +0.25, 0.00); iv_pole (2.60, +0.33, 0.00) | Verify intended overlap; measure final prop pose. |
| **P01-25 KEEP** | Pole close in range to G1 | (2.70, +0.00); yaw=3.14; r=2.70 | pole (2.37, -0.02), r=2.37 | `interfere_samerange_01` | (2.70, +0.15, 3.22) | iv_pole (2.37, +0.13, 0.00) | Verify intended overlap; measure final prop pose. |
| **P01-26 KEEP** | Pole close in range to G1 | (2.70, +0.00); yaw=3.14; r=2.70 | pole (2.41, +0.02), r=2.41 | `interfere_samerange_02` | (3.10, -0.20, 3.28) | iv_pole (2.77, -0.18, 0.00) | Verify intended overlap; measure final prop pose. |
| **P01-27 KEEP** | Pole close in range to G1 | (2.70, +0.00); yaw=3.14; r=2.70 | pole (2.37, +0.09), r=2.38 | `interfere_samerange_03` | (2.90, +0.35, 3.05) | iv_pole (2.55, +0.45, 0.00) | Verify intended overlap; measure final prop pose. |
| **P01-28 KEEP** | Hide most of G1 with panel/cabinet | (2.70, +0.00); yaw=3.14; r=2.70 | panel (1.75, -0.74), r=1.90 | `objocc_heavy_01` | (3.34, +0.95, 3.13) | privacy_curtain (2.17, +0.04, 2.67) | Verify intended overlap; measure final prop pose. |
| **P01-29 KEEP** | Hide most of G1 with panel/cabinet | (2.70, +0.00); yaw=3.14; r=2.70 | cabinet (1.85, +0.44), r=1.90 | `objocc_heavy_02` | (4.02, -1.49, 3.33) | cabinet (2.76, -0.84, 1.08) | Verify intended overlap; measure final prop pose. |
| **P01-30 KEEP** | Hide most of G1 with panel/cabinet | (2.70, +0.00); yaw=3.14; r=2.70 | cabinet (1.44, -0.10), r=1.45 | `objocc_heavy_03` | (4.17, +0.92, 3.24) | cabinet_tall (2.23, +0.76, 2.05) | Verify intended overlap; measure final prop pose. |
| **P01-31 KEEP** | Hide most of G1 with panel/cabinet | (2.70, +0.00); yaw=3.14; r=2.70 | panel (1.49, +0.58), r=1.60 | `objocc_heavy_04` | (4.12, -0.48, 3.17) | privacy_curtain (2.27, +0.40, 0.44) | Verify intended overlap; measure final prop pose. |
| **P01-32 KEEP** | Partial G1 occlusion with panel/cabinet/table | (2.70, +0.00); yaw=3.14; r=2.70 | cabinet (2.02, -0.26), r=2.04 | `objpartial_01` | (4.41, +0.31, 3.62) | hospital_bed (3.30, -0.12, 0.61) | Verify intended overlap; measure final prop pose. |
| **P01-33 KEEP** | Partial G1 occlusion with panel/cabinet/table | (2.70, +0.00); yaw=3.14; r=2.70 | cabinet (1.73, -0.86), r=1.93 | `objpartial_02` | (3.69, +1.61, 3.21) | hospital_bed_high (2.37, +0.44, 0.06) | Verify intended overlap; measure final prop pose. |
| **P01-34 KEEP** | Partial G1 occlusion with panel/cabinet/table | (2.70, +0.00); yaw=3.14; r=2.70 | panel (1.93, -0.64), r=2.03 | `objpartial_03` | (3.11, -0.34, 3.35) | privacy_curtain (2.22, -1.08, 0.16) | Verify intended overlap; measure final prop pose. |
| **P01-35 KEEP** | Partial G1 occlusion with panel/cabinet/table | (2.70, +0.00); yaw=3.14; r=2.70 | table (1.57, +0.26), r=1.60 | `objpartial_04` | (3.72, -0.92, 3.10) | table (2.17, -0.56, 0.09) | Verify intended overlap; measure final prop pose. |
| **P01-36 KEEP** | Partial G1 occlusion with panel/cabinet/table | (2.70, +0.00); yaw=3.14; r=2.70 | cabinet (2.02, -1.11), r=2.30 | `objpartial_05` | (3.32, +1.34, 3.15) | hospital_bed_high (2.48, -0.03, 2.19) | Verify intended overlap; measure final prop pose. |
| **P01-37 KEEP** | Partial G1 occlusion with panel/cabinet/table | (2.70, +0.00); yaw=3.14; r=2.70 | table (1.94, +0.25), r=1.96 | `objpartial_06` | (4.12, -1.27, 3.32) | table (2.96, -0.89, 2.38) | Verify intended overlap; measure final prop pose. |
| **P01-38 KEEP** | Cabinet/panel close ahead of G1 | (2.70, +0.00); yaw=3.14; r=2.70 | cabinet (1.97, +0.00), r=1.97 | `trap_contact_01` | (3.58, -0.69, 2.97) | hospital_bed_high (2.61, -0.69, 1.57) | Verify intended overlap; measure final prop pose. Measure actual surface gap; origin gap is not clearance. |
| **P01-39 KEEP** | Table close ahead of G1 | (2.70, +0.00); yaw=3.14; r=2.70 | table (2.07, +0.00), r=2.07 | `trap_contact_02` | (3.36, -0.93, 3.33) | table (2.57, -0.93, 1.57) | Verify intended overlap; measure final prop pose. Measure actual surface gap; origin gap is not clearance. |
| **P01-40 KEEP** | Table ahead and cabinet behind G1 | (2.70, +0.00); yaw=3.14; r=2.70 | table (1.77, +0.31); r=1.80; cabinet (3.35, -0.15); r=3.35 | `trap_layered_01` | (3.71, -1.27, 2.76) | table (2.43, -0.85, 0.00); cabinet (5.44, -1.72, 2.04) | Shorter spacing / reduced lateral offset; verify furniture footprint and intended effect. |
| **P01-41 KEEP** | Table ahead and cabinet behind; second arrangement | (2.70, +0.00); yaw=3.14; r=2.70 | table (1.73, +0.06); r=1.73; cabinet (3.30, +0.10); r=3.30 | `trap_layered_02` | (3.88, +0.43, 3.02) | table (2.49, +0.52, 0.00); cabinet (5.22, +0.67, 2.35) | Shorter spacing / reduced lateral offset; verify furniture footprint and intended effect. |
| **P01-42 KEEP** | Cabinet blocks LiDAR; camera sees upper G1 | (2.70, +0.00); yaw=3.14; r=2.70 | cabinet (0.95, +0.62), r=1.13 | `trap_lidar_bed` | (4.19, -1.39, 3.21) | hospital_bed (1.47, -0.43, 0.00) | Verify intended overlap; measure final prop pose. Cabinet must block scan but leave upper G1 visible. |
| **P01-43 KEEP** | Table allows scan through legs | (2.70, +0.00); yaw=3.14; r=2.70 | table (1.16, -0.31), r=1.20 | `trap_lidar_table` | (4.27, +0.80, 3.08) | table (1.83, +0.31, 0.00) | Verify intended overlap; measure final prop pose. Verify scan passes through actual table legs/apron. |
| **P01-44 KEEP** | Hide lower body; preserve upper-body view | (2.70, +0.00); yaw=3.14; r=2.70 | cabinet (1.84, -0.24), r=1.85 | `trap_rgb_01` | (3.85, +0.98, 3.46) | hospital_bed (2.62, +0.64, 0.00) | Verify intended overlap; measure final prop pose. |
| **P01-45 KEEP** | Hide lower body; second occlusion arrangement | (2.70, +0.00); yaw=3.14; r=2.70 | cabinet (1.82, -0.24), r=1.83 | `trap_rgb_02` | (3.70, +0.94, 3.37) | hospital_bed (2.49, +0.61, 0.00) | Verify intended overlap; measure final prop pose. |
| **P01-46 KEEP** | Panel edge splits silhouette | (2.70, +0.00); yaw=3.14; r=2.70 | panel (1.64, -0.04), r=1.64 | `trap_split_curtain` | (3.86, +0.17, 3.04) | privacy_curtain (2.35, +0.11, 1.57) | Verify intended overlap; measure final prop pose. Conditional on panel edge thickness. |
| **P01-47 KEEP** | Pole splits silhouette | (2.70, +0.00); yaw=3.14; r=2.70 | pole (1.57, +0.17), r=1.58 | `trap_split_pole` | (3.86, -0.48, 3.07) | iv_pole (2.25, -0.24, 0.00) | Verify intended overlap; measure final prop pose. |
| **P01-48 TURN** | Unoccluded G1; side view | (2.70, +0.00); yaw=4.70; r=2.70 | none | `single_side_01` | (2.42, -0.81, 5.00) | none | Clear movable clutter; document fixed crane/background. |
| **P01-49 TURN** | Unoccluded G1; free view | (2.70, +0.00); yaw=5.95; r=2.70 | none | `single_free_01` | (3.71, +1.18, 5.95) | none | Clear movable clutter; document fixed crane/background. |
| **P02-01 PLACE** | Unoccluded G1; back view | (3.20, +0.00); yaw=0.00; r=3.20 | none | `single_back_02` | (4.83, +1.42, 0.11) | none | Conditional room depth; original range shortened. |
| **P02-02 TURN** | Unoccluded G1; free view | (3.20, +0.00); yaw=2.70; r=3.20 | none | `single_free_02` | (5.17, +1.91, 2.67) | none | Conditional room depth; original range shortened. |
| **P02-03 TURN** | Unoccluded G1; facing view | (3.20, +0.00); yaw=3.14; r=3.20 | none | `single_facing_01` | (5.02, -1.11, 2.78) | none | Conditional room depth; original range shortened. |
| **P02-04 KEEP** | Unoccluded G1; facing view | (3.20, +0.00); yaw=3.14; r=3.20 | none | `single_facing_02` | (4.27, -0.14, 3.46) | none | Same real arrangement as P02-03; one bag can map both IDs. |
| **P02-05 TURN** | Unoccluded G1; side view | (3.20, +0.00); yaw=4.70; r=3.20 | none | `single_side_02` | (4.94, -1.41, 4.54) | none | Conditional room depth; original range shortened. |
| **P03-01 PLACE** | Unoccluded G1; back view | (2.00, +0.30); yaw=0.00; r=2.02 | none | `single_back_01` | (1.94, +0.32, 0.07) | none | Clear movable clutter; document fixed crane/background. |
| **P04-01 PLACE** | Near G1; verify camera clipping | (1.30, +0.00); yaw=3.30; r=1.30 | none | `near_clip_01` | (1.34, -0.03, 3.45) | none | Verify actual clipping; hardware field of view differs. |
| **P05-01 PLACE** | Near G1; verify camera clipping | (1.30, -0.65); yaw=0.65; r=1.45 | none | `near_clip_02` | (1.36, -0.67, 0.65) | none | Verify actual clipping; hardware field of view differs. |
| **P06-01 PLACE** | Near G1; verify camera clipping | (1.10, +0.00); yaw=3.30; r=1.10 | none | `near_clip_03` | (1.09, +0.03, 3.28) | none | Verify actual clipping; hardware field of view differs. |
| **P07-01 PLACE** | G1 near image edge but mostly visible | (2.00, -1.25); yaw=2.80; r=2.36 | none | `edge_mostly_in_01` | (4.39, -2.67, 2.74) | none | Adjust lateral position to actual clipping; verify usable width. |
| **P08-01 PLACE** | G1 partially outside image | (2.00, -1.55); yaw=2.80; r=2.53 | none | `edge_half_out_02` | (4.37, -3.08, 2.79) | none | Adjust lateral position to actual clipping; verify usable width. |
| **P09-01 PLACE** | G1 near image edge but mostly visible | (2.00, +1.25); yaw=2.80; r=2.36 | none | `edge_mostly_in_02` | (3.98, +2.38, 2.80) | none | Adjust lateral position to actual clipping; verify usable width. |
| **P10-01 PLACE** | G1 partially outside image | (2.00, +1.55); yaw=3.40; r=2.53 | none | `edge_half_out_01` | (4.30, +3.03, 3.48) | none | Adjust lateral position to actual clipping; verify usable width. |
| **P11-01 PLACE** | Only a sliver of G1 visible | (2.00, +1.80); yaw=3.40; r=2.69 | none | `edge_sliver_01` | (2.67, +2.05, 3.44) | none | Adjust lateral position to actual clipping; verify usable width. |

### Excluded source scenes

Multiple G1s (20): `multi_01`, `multi_02`, `multi_03`, `multi_04`, `twin_shoulder_01`, `twin_shoulder_02`, `twin_swap_01`, `twin_swap_02`, `interocc_moderate_01`, `interocc_moderate_02`, `interocc_sliver_01`, `interocc_sliver_02`, `interocc_neartotal_01`, `interocc_neartotal_02`, `chaos_01`, `chaos_02`, `chaos_03`, `chaos_04`, `interfere_multi_01`, `interfere_multi_02`.

Far range (5): `far_mid_01`, `far_mid_02`, `far_clamp_01`, `far_clamp_02`, `far_beyond_01`.

People are not substituted for a second G1 in this menu. A scene adapted so
strongly that its defining condition disappears should be logged as a different
physical condition or skipped, not certified by retaining a benchmark ID.
