# Benchmark result-comparability transitions

Recorded dates: 2026-07-26, 2026-08-22, 2026-08-24, 2026-08-26, 2026-08-31

Tested revisions: unknown

Provenance: partial

This is dated evidence. Missing revisions or preserved worktree inputs limit
reproducibility; no new measurements were made during archive curation.

## Purpose

These dated transitions explain why historical populations and scores cannot
be compared directly. They are not the maintained schema or benchmark runbook.

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

## Scenario and scoring transitions

The scenario-loader/multi-instance series (`bbdcde82fdbc1e1d5045f05a9c4dc95333d64cf4` through `de4c23616ec93453d7ad7e4493bd89950e096da7`, merged
at `20086ca68dff120df67800d5e2754811669e9331`) replaced a single-target grid with YAML scenes and per-instance
truth, association, misses, and extras. Truth remained a scoring input; it was
not supplied to perception to separate targets.

On July 26, the all-estimator common-event gate was removed. Each estimator
scored its own usable observations; missing estimates no longer discarded the
event for every estimator. Zero detections became misses rather than skips.

On August 22, per-detection status counting replaced the exactly-one-detection
filter. Later per-estimator planar association made safe instance attribution
possible; unmatched-box reasons could not be assigned to an instance without
an estimate to associate.

On August 24, the handwritten grid and 30-scene suite were retired in favor
of the randomized suite (sampler seed 20260725). The record described 88 scenes
after clutter additions. Different scenario revisions sample different
populations and cannot constitute matched A/B evidence.

On August 26, raw depth buffering moved into the consumer, removing a second
sampling stage. Its measured context is in the
[depth-coverage investigation](aligned_depth_coverage.md).

## Historical validation evidence

Early redesign validation recorded these qualitative outcomes:

- single-target scenes reproduced the then-recorded baselines;
- inter-target occlusion scored the near target and marked the hidden target
  missed;
- a bed in front of the target exposed the legacy LiDAR row selecting the
  nearer occluder while mask-based rows stayed on the target or honestly
  missed it;
- the original 30-scene mask run reported roughly `0.055 m` MAE for projective
  ranging and euclidean reconstruction and `0.073 m` for polar profiling.

Those figures are retained as evidence that the redesign exercised its intended
cases, not as present-day baselines. They used an earlier estimator registry,
scenario population, and camera/render geometry. Software-rendered runs also
produced few usable frames in difficult scenes, and Gazebo pose queries
occasionally failed one trial.

## Camera-model comparability note

Runs recorded before 2026-08-31 used the repository's former D435 selection and
a hand-copied D435 static optical transform while the physical platform was
specified as D455. The August 31 correction selected D455 explicitly and derived simulated
internal camera frames from the selected model, renders from the colour-frame
pose, and leaves hardware camera-internal TF to the RealSense driver.

The root cause and removed transform are recorded in
[operational incident history](operational_incidents.md#d435-static-camera-transform--removed-2026-08-31).
This note exists here only because the change creates a benchmark provenance
boundary: pre- and post-correction runs must not be presented as the same camera
geometry.
