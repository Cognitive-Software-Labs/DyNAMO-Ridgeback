# Benchmark evolution

Historical record of the target-distance benchmark's structural and semantic
changes. The current workflow, scenario contract, scoring semantics, and output
schema live in [target-distance benchmarking](../benchmarking/target_distance_benchmarking.md).
This page explains why older results and older descriptions can differ from the
current system; it is not a second runbook.

## Original fixed-grid benchmark

The original runner spawned one Unitree G1 per trial on a hardcoded
`5 forward x 3 lateral` grid. It expected exactly one detection and collapsed a
capture to scalar values. Frames with zero or multiple detections were excluded,
and selected estimators had to share a usable event. Those rules made a useful
single-target calibration tool, but they could not represent clutter,
inter-target occlusion, missed instances, or estimator-specific coverage.

The perception messages were already multi-instance: detections and measurement
arrays carried one element per box, and producers looped over all boxes. The
single-target assumption belonged mainly to benchmark scenario generation,
association, reduction, and reporting.

## Scenario-spec and multi-instance redesign

The redesign replaced the grid with declarative YAML scenes. Each scene names
one or more target poses and optional object occluders. The runner iterates
scenes and repeats, spawns the declared models, and computes one truth instance
per target.

The load-bearing rule is that truth participates only in benchmark operations:
computing reference distances and assigning sensor-produced estimates to the
instances against which they are scored. It never helps the detector or an
estimator separate, select, or localize targets.

The main implementation stages were:

| Stage | Commit | Durable change |
|---|---|---|
| Scenario loader and runner | `bbdcde8` | YAML `Scene`, `RobotSpec`, and `ObjectSpec`; scene-driven spawn and truth |
| Sensor-side association | `9d4a9ef` | Nearest planar estimate assignment with a sanity gate |
| Multi-instance scoring | `2b31cf6` | Per-instance rows, missed truth, and extra detections |
| Occluder assets | `9c0c376` | Hospital-bed and privacy-curtain models |
| Per-instance collages | `38298f2` | Matched, extra, and missed annotations |
| Scenario expansion | `c4d2048`, `cb0ffe9`, `de4c236` | Pose, yaw, object, and partial-occlusion coverage |

Current schema and module ownership are documented only in the benchmark
reference. Counts in old reports describe their exact scenario revision, not a
permanent size of the packaged suite.

## Scenario-set transition

The first scenario-spec suite retained the legacy grid as explicit scenes and
then expanded to a 30-scene multi-instance and occlusion set. The merged
scenario/status redesign was recorded against merge `20086ca`.

On 2026-08-24 the handwritten grid and older 30-scene suite were removed.
`benchmark_scenarios_full.yaml` became the packaged randomized suite, retaining
sampler seed `20260725` and later in-box clutter additions. The file currently
contains 88 scenes, but the YAML itself remains the authority; a future suite
change must not leave a supposedly timeless count in instructions.

A scenario-file change changes the sampled population. Results across different
scenario revisions are not an A/B comparison even when their estimator settings
match.

## Per-estimator usable events

On 2026-07-26 the all-estimator common-event gate was removed. Each estimator
began scoring on its own usable events, so one row's absence became that
estimator's miss instead of discarding the trial for every row. Zero-detection
trials became all-instance misses; skips were reserved for infrastructure
failures.

Numbers before and after this transition use different event populations and
are not directly comparable without reconstructing the former common-event
gate.

## Per-detection status transition

Before 2026-08-22, status histograms skipped frames unless the detection count
was exactly one. Multi-target scenes therefore contributed either nothing or
only frames where the detector had already lost an instance.

Status counting moved to every detected box. Mask estimators carry explicit
statuses, while pointcloud receives coarse OK/UNSET attribution from finiteness.
Trial CSVs gained `miss_reason`, and the shared dominant-reason rule made a
specific status outrank UNSET.

This transition alone did not create truthful per-ground-truth-instance failure
attribution: a missing value has no estimator position with which to associate
its reason. Later per-estimator planar association closed the safe-attribution
part of that boundary while leaving unmatched-box reasons at observation
granularity. The current contract is in the
[benchmark reference](../benchmarking/target_distance_benchmarking.md#miss-reasons-and-attribution-boundary).

## Depth acquisition transition

The original aligned-depth producer decoded and independently throttled the
stream before a downstream exact-stamp lookup. Under load, it discarded many
frames before the detector selected their stamps. On 2026-08-26 acquisition
moved into the mask measurement node: source frames are buffered undecoded and
only the detection-stamp frame is converted.

That removed the independent cadence gap; it did not claim that every exact
match now succeeds. Detailed measurements remain in
[aligned-depth coverage](aligned_depth_coverage.md).

## Historical validation evidence

Early redesign validation recorded these qualitative outcomes:

- single-target scenes reproduced the then-current baselines;
- inter-target occlusion scored the near target and marked the hidden target
  missed;
- a bed in front of the target exposed the legacy LiDAR row selecting the
  nearer occluder while mask-based rows stayed on the target or honestly
  missed it;
- the original 30-scene mask run reported roughly `0.055 m` MAE for projective
  ranging and euclidean reconstruction and `0.073 m` for polar profiling.

Those figures are retained as evidence that the redesign exercised its intended
cases, not as current baselines. They used an earlier estimator registry,
scenario population, and camera/render geometry. Software-rendered runs also
produced few usable frames in difficult scenes, and Gazebo pose queries
occasionally failed one trial.

## Camera-model comparability note

Runs recorded before 2026-08-31 used the repository's former D435 selection and
a hand-copied D435 static optical transform while the physical platform was
specified as D455. The repository now selects D455 explicitly, derives simulated
internal camera frames from the selected model, renders from the colour-frame
pose, and leaves hardware camera-internal TF to the RealSense driver.

The root cause and removed transform are recorded in
[operational incident history](operational_incidents.md#d435-static-camera-transform--removed-2026-08-31).
This note exists here only because the change creates a benchmark provenance
boundary: pre- and post-correction runs must not be presented as the same camera
geometry.

## Durable comparison rules

- Compare the same scenario YAML content and revision.
- Treat a new sampler seed or added scenes as coverage expansion, not a
  drop-in baseline.
- Preserve probe-specific repeat overrides.
- Record estimator registry, camera/render geometry, parameters, and commit.
- Treat scoring, status, or usable-event transitions as schema changes rather
  than unexplained shifts in accuracy or coverage.
- Use current `run.json`, trial CSVs, and scenario YAML as evidence; old branch,
  worktree, and launch commands are intentionally not retained as instructions.
