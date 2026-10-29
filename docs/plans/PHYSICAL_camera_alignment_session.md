# PHYSICAL — Camera and alignment robot session

Status: **ready, not run.** Prepared 2026-10-14 by Claude for one supervised
stationary session on `r100_0160`. It schedules the remaining camera and
camera–LiDAR alignment evidence of
[PHYSICAL — Robot measurements and stationary sensor validation](PHYSICAL_robot_measurements_and_validation.md),
which keeps owning scope and completion; this page fixes order, commands and the
numeric limits below **before** measurement. G1 cases are out of scope for this
session.

## Preconditions (every block)

- The base is stationary with the e-stop pressed for the whole block. No driving,
  controller or feedback trial runs at the same time; agree that with whoever is
  running them. A person standing at marks is the reason, not just data quality.
- No package updates are running (`/var/log/unattended-upgrades/`).
- Thor perception runs the accepted revision; `tools/intel_thor/check_link`
  passes on Intel; one quiet minute of `tools/camera_contract_check --backend
  hardware --duration 60` passes before the first capture.
- Each block writes under one `artifacts/hardware/<UTC>-<block>/` directory and
  records the git revision, profile, label and threshold.

## Blocks and limits

| Block | What | Pass limits (declared before measurement) | Time |
|---|---|---|---|
| A1 | Alignment overlay: upright board/box at ~1.5 m centre, ~3 m centre, ~2 m and ~0.6 m to the robot's left, then right | Board beams land on the board in every overlay; per placement, median aligned-depth residual of board beams within ±0.03 m; best sweep yaw within ±0.5°; no consistent sideways or vertical offset between placements | 25–35 min |
| A2 | Only if A1 fails: frame conventions, live intrinsics, TF ownership, measured offsets; correct, then repeat at a **new** placement | Same as A1 at the new placement; previous config kept for rollback | 20–40 min |
| B1 | Off-centre person at VGA: taped marks ~2 m ahead, ~0.6 m left and right; one mark ~1.5 m ahead, ~0.4 m left | Detections in ≥ 90 % of still frames; lateral sign correct; lateral within ±0.15 m of tape; distance within 10 % of tape for both estimators; empty-scene segment has no detections | 20–30 min |
| B3 | Estimator modes: box/silhouette × stereo/monocular at the ~2 m centre mark, plus empty scene | Finite plausible results in every valid case; correct status when invalid; monocular cases show real GPU inference; misses and estimator failures reported separately | 20–30 min |
| B4 | Thor resources during B1–B3 | Recorded only: power, temperature, per-stage latency; no limit is declared here | parallel |

Distances are from the camera front face to the target's facing surface, as in
the earlier smoke. Tape placement and a person's posture add several centimetres
of uncertainty; report it, do not tune to it. The detector setting stays
`full body person` at 0.26 and is recorded with every block.

## Commands

On Intel, ROS and `src/ridgeback_autonomy_hardware/config/intel_thor/dds_env.sh intel`
sourced:

```bash
tools/camera_lidar_overlay --output artifacts/hardware/<UTC>-alignment/<placement> --frames 5
tools/intel_thor/record_localization_run record --namespace r100_0160 --duration 600 --output <run>/
tools/intel_thor/record_localization_run mark --output <run>/ <mark>
tools/intel_thor/record_localization_run summarize --output <run>/
```

The session runs at 640x480 only. 1280x720 is not supported on the robot: it is not tested or developed further, and anyone who needs it must develop and qualify it first ([deferred gate](../BACKLOG.md#deferred-hardware-gates)).

## Results

Record pass, fail or not tested per block with evidence paths in the governing
plan and a dated archive record. A1 passing is a stationary alignment check for
those placements, not a calibration; the
[camera–LiDAR gate](../BACKLOG.md#camera-lidar-calibration) keeps its formal
uncertainty and motion criteria. If A1 fails and A2 cannot fix it, polar
profiling stays blocked while B1–B3 report normally.
