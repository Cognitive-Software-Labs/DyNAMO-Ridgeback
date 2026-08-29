# G1 Distance Benchmarking

## Purpose

This note describes the current benchmark workflow for comparing G1 distance estimators in `ridgeback_autonomy`.

The benchmark keeps the same OWLv2 RGB detection front-end and compares different range-estimation backends on the same spawned target poses inside the `g1_distance_calibration` world.

## Supported Estimators

The public benchmark interface uses these estimator names:

- `pointcloud`
- `projective_ranging`
- `euclidean_reconstruction`
- `polar_profiling`

`pointcloud` reads the organized `PointCloud2` directly; the other three are the
mask-based localization paths documented under `object_localization_documentation/`.

## Launch Interface

Main entrypoint:

- [src/ridgeback_autonomy/launch/g1_distance_benchmark.launch.py](src/ridgeback_autonomy/launch/g1_distance_benchmark.launch.py)

Typical usage:

```bash
bash cleanup.sh

# Compare everything
ros2 launch ridgeback_autonomy g1_distance_benchmark.launch.py

# Single-estimator run
ros2 launch ridgeback_autonomy g1_distance_benchmark.launch.py estimators:=pointcloud

# Mixed comparison
ros2 launch ridgeback_autonomy g1_distance_benchmark.launch.py estimators:=pointcloud,polar_profiling
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
- point-cloud and mask measurements are aligned on the same detection-event key:
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

## Watching a run in RViz

The benchmark launches RViz with `sim/rviz/benchmark.rviz`. Four displays carry
the run, and each is switched on and off from the Displays panel while the run is
going — that is the only toggle mechanism; there are no parameters for it.

| Display | Shows |
|---|---|
| `G1 Estimates` | one coloured ring per estimator, at where it thinks the G1 is |
| `G1 Polar Rays` | the LiDAR beams behind the polar profiling estimate |
| `HUD` | top-right: every estimator's distance against the trial's ground truth |
| `Perception overlay` | docked under the 3D view: the color and mask panels |

The HUD colours each row to match that estimator's ring, from the one table in
`g1_estimate_viz_node`, so a reading and its ring cannot drift apart. Dark ring
colours (red, and Depth-Anything's purple) are lightened just enough to stay
readable on the HUD's dark panel, keeping the hue so the ring is still
recognisable.

The layout — overlay docked full-width beneath the 3D view, roughly 30% of the
height — is held by the `QMainWindow State` blob in `sim/rviz/benchmark.rviz`.
RViz hardcodes the left dock for every panel it creates and the config format has
no per-display dock-area key, so **that blob is the only thing placing it**. If it
is ever dropped, the overlay silently reverts to a narrow left-dock strip a few
tens of pixels tall.

`G1 Polar Rays` expands into three namespaces, each with its own checkbox:

- `polar/used` — the beams the estimate medians over, in amber
- `polar/dropped` — beams the mask selected but the range segmentation discarded,
  dim and **off by default**. Turn it on when diagnosing: without it, a frame
  where the estimator threw the robot away looks the same as one where nothing
  was there
- `polar/wedge` — the bearing span of the beams inside the detection box. Under
  `mask_gate:=box` this matches the rays exactly; under `silhouette` it is wider,
  and the gap is what the segmentation removed

A box that does not span the scan plane's image row contains no beams, so it
draws no wedge. That is the correct picture for a target fully occluded at scan
height, not a bug — the `objocc_*` scenes hit it routinely.

The rings cover all four estimators. The three mask-based ones are the reason
the palette runs to cyan, blue and violet, against the point cloud's amber: in a
clean scene every ring lands within centimetres of the others.

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
- `video/run.mp4` — the RViz window for the whole run

`run.json` and the trial CSVs are the machine-readable sources (diffing runs,
feeding analysis); `summary.md` renders the same numbers for reading.

## Run video

Every run records the RViz window to `video/run.mp4` (H.264, 10 fps). The
collages freeze one representative frame per trial; the video keeps the motion
around it, which is where a mask collapsing or an estimator latching onto an
occluder actually shows.

Recording is best-effort and never costs a trial: a missing `ffmpeg`, a missing
`xdotool`, or an RViz window that never appears logs a warning and the run
continues. Turn it off with `record_video:=false`; `record_fps`, `record_max_sec`
and `record_window_class` are also parameters.

It captures the RViz window by id, so it is unaffected by what else is on screen
— but that relies on a compositing X server. Without compositing, an occluded
window records whatever is drawn over it.

Because the perception overlay is published into RViz (see below), that one
window holds the whole picture.

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
  - `pointcloud`
  - `projective_ranging`
  - `euclidean_reconstruction`
  - `polar_profiling`
- includes only the estimators selected for that run
- draws every panel on the same color frame: no estimator has imagery of its
  own, so the panels differ in their annotations, not their source

Representative-frame selection:

- compute the per-trial median for each selected estimator (partial when an
  estimator produced nothing)
- rank candidates from the union of usable events: frames with a color preview
  first, then most estimators present, then the sum of
  `abs(event_value - trial_median)` over the estimators the frame actually
  carries
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
- [src/ridgeback_autonomy/ridgeback_autonomy/perception/g1_pointcloud_measurement_node.py](src/ridgeback_autonomy/ridgeback_autonomy/perception/g1_pointcloud_measurement_node.py)
- [src/ridgeback_autonomy/ridgeback_autonomy/perception/g1_mask_measurement_node.py](src/ridgeback_autonomy/ridgeback_autonomy/perception/g1_mask_measurement_node.py)

## Practical Notes

- `pointcloud` comes from `g1_pointcloud_measurement_node`; the three mask rows come from `g1_mask_measurement_node`
- when the benchmark launch receives a subset in `estimators`, only the required measurement nodes are launched
- each node is told which of its own rows to compute, so an unselected branch never runs
