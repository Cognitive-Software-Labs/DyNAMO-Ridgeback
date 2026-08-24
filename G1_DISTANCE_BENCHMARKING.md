# G1 Distance Benchmarking

## Purpose

This note describes the current benchmark workflow for comparing G1 distance estimators in `ridgeback_autonomy`.

The benchmark keeps the same OWLv2 RGB detection front-end and compares different range-estimation backends on the same spawned target poses inside the `g1_distance_calibration` world.

## Supported Estimators

The public benchmark interface uses these estimator names:

- `rgb`
- `sensor_depth`
- `depth_anything`
- `pointcloud`
- `lidar`

Important naming note:

- the public benchmark API uses `depth_anything`
- the internal ROS message field is still `mono_depth_distance_m`
- the internal debug topic is still `debug/g1/camera/mono_depth`

## Launch Interface

Main entrypoint:

- [src/ridgeback_autonomy/launch/g1_distance_benchmark.launch.py](src/ridgeback_autonomy/launch/g1_distance_benchmark.launch.py)

Typical usage:

```bash
bash cleanup.sh

# Compare everything
ros2 launch ridgeback_autonomy g1_distance_benchmark.launch.py

# Single-estimator run
ros2 launch ridgeback_autonomy g1_distance_benchmark.launch.py estimators:=rgb

# Mixed comparison
ros2 launch ridgeback_autonomy g1_distance_benchmark.launch.py estimators:=rgb,pointcloud,lidar
```

Key public arguments:

- `estimators`
- `output_dir`
- `repeats`
- `settle_sec`
- `capture_sec`
- `color_topic`
- `depth_topic`
- `pointcloud_topic`
- `scan_topic`
- `base_frame`

## Benchmark Semantics

The benchmark now follows these rules:

- only positive spawned-target trials are generated
- the target stays in front of the robot for a fixed capture window, `10.0` seconds by default
- only successful single-target detections are used
- camera and LiDAR measurements are aligned on the same detection-event key:
  - `header.frame_id`
  - `header.stamp.sec`
  - `header.stamp.nanosec`
  - `count`
  - `bbox_xyxy`
- each estimator is scored on its OWN usable events; an estimator with zero
  usable events in an otherwise-live trial is a per-estimator miss, with its
  reason in the trial CSV's `miss_reason` column (run-level tallies in
  `run.json` → `reason_histogram`)
- a trial whose capture window yields NO detections at all still counts: every
  ground-truth instance is recorded as missed by every estimator
  (`all_instances_missed` in the log). A skipped trial means infrastructure
  failure only (spawn error, stream timeout)
- each selected estimator gets one final per-trial value, computed as the median over that trial’s aligned usable detections for that estimator

What is intentionally *not* scored:

- no-detection frames
- negative controls
- per-frame raw rows

This keeps the benchmark focused on distance quality once the shared detector has produced a usable target lock.

## Output Layout

Default output root:

- `<repo-root>/benchmark-results`

Each run creates:

- `<repo-root>/benchmark-results/<timestamp>_<scenario>[_<gate>][_<depth_source>]/`

  e.g. `20260822_202851_v2_silhouette_stereoscopic`. The timestamp leads so the
  directory sorts chronologically; the gate and depth source appear only when a
  mask (respectively depth-path) estimator actually ran.

Inside that run folder:

- `summary.md` — the readable report: accuracy, reliability, why boxes went
  unmeasured, and a per-scene table with one column per estimator. Start here.
- `<estimator>.csv` for each selected estimator
- `run.json` — the same run-level numbers for machines, plus provenance
- `images/<trial_id>.png`

`run.json` and the trial CSVs are the machine-readable sources (diffing runs,
feeding analysis); `summary.md` renders the same numbers for reading.

Pass `output_dir:=...` to the launch file to write the timestamped run folder somewhere else.

Each estimator CSV is trial-level only:

- one row per included trial
- same included-trial set across all selected estimators in that run

Each row contains:

- trial identity
- spawn pose
- ground-truth distance
- estimator name
- final per-trial estimate
- absolute error
- relative error
- usable aligned detection count
- shared collage image path

`run.json` holds the run-level numbers: a `run` object (label, start time,
commit, branch, `uncommitted_files`, scenario, scene/instance/trial counts), a
`parameters` object with every parameter the runner was launched with, and one
entry per estimator carrying the accuracy aggregates, a nested `missed`
breakdown, a nested `observations` group, and `reason_histogram` as a real
mapping. Field-by-field meanings:
[benchmark_v2_followups.md](object_localization_documentation/benchmark_v2_followups.md)
→ Output reference.

It replaced `comparison_summary.csv`, which duplicated most of `summary.md`
while being a poor machine format for this shape — JSON embedded in a cell, and
instance-level accuracy sharing one flat row with box-level counts.

`summary.md` renders the same numbers for reading, plus the provenance header —
start time, commit (flagged when the tree was dirty, since the hash alone will
not reproduce it), branch, and the full parameter table.

## Trial Images

Each included trial gets exactly one saved collage image:

- `images/<trial_id>.png`

The collage:

- uses one shared representative aligned detection event for the whole trial
- renders panels left-to-right in canonical order:
  - `rgb`
  - `sensor_depth`
  - `depth_anything`
  - `pointcloud`
  - `lidar`
- includes only the estimators selected for that run

Representative-frame selection:

- compute the per-trial median for each selected estimator (partial when an
  estimator produced nothing)
- rank candidates from the union of usable events: panel previews first, then
  most estimators present, then the sum of `abs(event_value - trial_median)`
  over the estimators the frame actually carries
- break ties by earliest timestamp; panels for estimators without a median
  show the dominant miss reason instead of a value

Every panel shows:

- the estimator name
- the current frame value
- the trial median
- the ground-truth distance
- the shared G1 bounding box

## Main Implementation Files

- [src/ridgeback_autonomy/ridgeback_autonomy/benchmarking/g1_distance_benchmark_runner_node.py](src/ridgeback_autonomy/ridgeback_autonomy/benchmarking/g1_distance_benchmark_runner_node.py)
- [src/ridgeback_autonomy/ridgeback_autonomy/benchmarking/estimators.py](src/ridgeback_autonomy/ridgeback_autonomy/benchmarking/estimators.py)
- [src/ridgeback_autonomy/ridgeback_autonomy/benchmarking/alignment.py](src/ridgeback_autonomy/ridgeback_autonomy/benchmarking/alignment.py)
- [src/ridgeback_autonomy/ridgeback_autonomy/benchmarking/reduction.py](src/ridgeback_autonomy/ridgeback_autonomy/benchmarking/reduction.py)
- [src/ridgeback_autonomy/ridgeback_autonomy/benchmarking/rendering.py](src/ridgeback_autonomy/ridgeback_autonomy/benchmarking/rendering.py)
- [src/ridgeback_autonomy/ridgeback_autonomy/benchmarking/summary.py](src/ridgeback_autonomy/ridgeback_autonomy/benchmarking/summary.py)
- [src/ridgeback_autonomy/ridgeback_autonomy/perception/g1_camera_measurement_node.py](src/ridgeback_autonomy/ridgeback_autonomy/perception/g1_camera_measurement_node.py)
- [src/ridgeback_autonomy/ridgeback_autonomy/perception/g1_lidar_measurement_node.py](src/ridgeback_autonomy/ridgeback_autonomy/perception/g1_lidar_measurement_node.py)

## Practical Notes

- `rgb`, `sensor_depth`, `depth_anything`, and `pointcloud` all come from the camera measurement node
- `lidar` stays in its own node
- when the benchmark launch receives a subset in `estimators`, only the required measurement nodes are launched
- when the camera node is launched for benchmarking, it is told which camera estimators to compute so unselected camera branches stay off
