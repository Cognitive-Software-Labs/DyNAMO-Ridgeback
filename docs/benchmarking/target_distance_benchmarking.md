# Target-distance benchmarking

## Purpose and scope

The benchmark compares independent target-localization estimators on fixed
spawned target instances in `target_distance_calibration`. Unitree G1 is the
current scenario asset; reusable perception and benchmark APIs use target names.
It measures simulator ground-truth accuracy, misses/coverage, and run provenance.
It is not hardware validation and never substitutes one estimator for another.

## Supported estimators and axes

Public estimator keys, in canonical order:

- `pointcloud` — organized `PointCloud2` input;
- `projective_ranging` — mask plus aligned depth, image-domain reduction;
- `euclidean_reconstruction` — mask plus aligned depth, point-domain reduction;
- `polar_profiling` — mask plus projected planar scan.

`estimators:=all` is the default. A subset launches only required measurement
nodes and disables unselected branches. Mask estimators expose `mask_gate`
(`box` default, or `silhouette`). Depth paths additionally expose `depth_source`
and registered `isolation_2d`/`isolation_3d` recipes; silhouette depth paths
bypass rect isolation. Polar is independent of depth source; pointcloud has its
own source. Output names include only axes that actually executed.

The root README owns complete build/run instructions. Main launch:
[`target_distance_benchmark.launch.py`](../../src/ridgeback_autonomy/launch/target_distance_benchmark.launch.py).

```bash
bash cleanup.sh
ros2 launch ridgeback_autonomy target_distance_benchmark.launch.py
ros2 launch ridgeback_autonomy target_distance_benchmark.launch.py \
  estimators:=projective_ranging,polar_profiling mask_gate:=silhouette
```

## Scenario and comparison discipline

The packaged full set is `config/benchmark_scenarios_full.yaml` (88 scenes,
20 multi-robot in the recorded 2026-08-24 transition; verify current source
before reporting counts). It includes occlusion, clutter, traps, and probe
scenes; the [gallery](benchmark_scenarios_v2_gallery.html) is a visual reference.

- Compare runs against the exact same scenario YAML and revision. Regenerating
  with a new seed expands coverage; it is not an A/B baseline.
- A scene can override repeats. `repeats:=1` still allows a probe with
  `repeats_override: 8` to collect its intended within-pose sample.
- Record current parameters and code provenance; an older scene/model/render
  configuration is not directly interchangeable with a current run.
- Never run `cleanup.sh` between persistent sweep configurations. A sweep owns
  one simulator/RViz/detector environment and restarts the per-config layer.

### Scenario schema

Scenario YAML is declarative. A file contains optional defaults and a non-empty
`scenes` list. Every scene has a unique id and at least one target; objects and
per-scene repeat overrides are optional:

```yaml
defaults:
  robot_yaw_rad: 3.141592653589793
scenes:
  - id: example
    repeats_override: 2
    robots:
      - {x: 3.0, y: 0.4, yaw: 3.14}
    objects:
      - {model: hospital_bed, x: 2.0, y: 0.3, yaw: 1.57}
```

Coordinates are world-frame planar poses: `x` is forward, `y` is lateral
(positive left), and `yaw` is radians about +Z. The runner pins spawned models
to the floor. `benchmarking/scenarios.py` is the authoritative parser and
validation contract; scenario-file comments record generation-specific design
and certification details.

## Event, observation, and instance semantics

One **event** is one aligned measurement frame keyed by frame id, exact stamp,
detection count, and bounding boxes. Pointcloud and mask messages with that key
merge. A multi-robot frame is still one event.

One **observation** is one detected box. Status histograms count observations,
so a two-box event contributes two statuses per estimator. A statusless
pointcloud value gets coarse OK/UNSET inferred from finiteness.

One **instance row** is a ground-truth target for one trial and estimator.
Association uses per-estimator planar positions, so every estimator scores on
its own usable events. Each final trial estimate is the median of that
estimator's associated values. Consequences:

- An estimator can miss while another scores; there is no common-event gate.
- A zero-detection trial records every instance as missed for every estimator.
- `detector_miss`, `gate_miss`, and `no_value` are scored outcomes. A skipped
  trial means infrastructure failure (for example spawn/stream failure), not a
  perception miss.
- No-detection frames and per-frame raw estimates are not themselves accuracy rows.

## Miss reasons and attribution boundary

Mask measurement messages carry a status per detection and estimator.
`compute_status_histogram` counts every box; a specific failure outranks UNSET
when selecting a dominant explanation. Trial CSV `miss_reason` is populated only
for `no_value`; the other miss categories are already named by `outcome`.

Attribution never borrows another estimator's position or assumes detection
order. In a multi-instance trial, a box status reaches an instance row only when
that same estimator's locator associates the box to that ground-truth instance.
Statuses on unmatched boxes remain observation-level evidence. The one exception
is an event containing exactly one target and one box, where identity is
unambiguous even without an estimator locator. When a `no_value` row has no safe
association, `miss_reason` is blank and the report renders `no_value (reason
unknown)` rather than copying the trial's dominant reason onto one or more
unproven identities.

`scored` rows need no reason; `gate_miss` and `detector_miss` are already explicit
outcomes. The collage remains a trial-level evidence view and may show a dominant
estimator reason; it is not an instance attribution source.

Accuracy/miss fields are per instance. `observations` and `reason_histogram` are
per detected box. Do not add those granularities together or interpret coverage
as accuracy. The authoritative enum is `common/miss_reason.py`; the pipeline
[glossary](../target_localization/target_localization_pipeline.md)
explains stages.

## Outputs and provenance

Default root: `<repo>/artifacts/benchmarks`. Each timestamped run contains:

- `summary.md` — human-readable accuracy, reliability, provenance, and scenes;
- `run.json` — machine-readable source of run parameters and aggregates;
- one trial-level CSV per selected estimator, including misses;
- `images/<trial_id>.png` collages;
- `video/run.mp4`, best-effort RViz capture when recording succeeds.

The `run` object records label/start, commit/branch, numeric
`uncommitted_files`, absolute scenario path, and included/skipped counts.
`parameters` records node parameters. Each `estimators` entry records stable key
and display name; row/scored counts; absolute/relative error aggregates;
detector/gate/no-value misses; extra detections; observation totals/OK/coverage;
and `reason_histogram`. A nonzero `uncommitted_files` means the commit alone does
not reproduce the run.

CSV rows include trial/scene/instance and spawn truth; estimator/outcome/reason;
estimate and absolute/relative error; usable events and captured frames; and
collage path. Read `run.json` for run-level statistics and CSVs for instance rows.

## Representative images and video

One shared representative event supplies every panel in a trial collage.
Selection prefers an available preview, then most selected-estimator values,
then proximity to available trial medians; timestamp breaks ties. Missing rows
show their dominant reason. Panels follow canonical estimator order and annotate
the same color frame, target box, event value, trial median, and truth.

RViz shows estimator rings, polar used/dropped/wedge namespaces, the truth/error
HUD, and the perception overlay. Displays are toggled in RViz, not via runtime
parameters. The video records that RViz window best-effort; absence of a video
does not fail a trial. Collages are evidence aids, not machine-readable truth.

## Sweep outputs and implementation ownership

`target_benchmark_sweep` validates YAML, starts one persistent environment,
runs one restartable config at a time, resumes configs only from valid `run.json`,
records real-time factor plus sweep/scenario SHA-256 provenance, and emits
`sweep.json` plus `summary.md`. Do not replace process restarts with runtime
estimator mutation: selected rows determine node existence.

For model-throughput investigations, `depth_match_debug:=true` also emits a
bounded mask-worker timing summary. Each distribution retains the first five
calls as cold evidence and reports warm count/mean/P50/P95/P99 plus the all-time
maximum.
Stages cover RGB preparation, SlimSAM load/inference, stereo or monocular depth,
mask-region preparation, estimator reduction, CUDA synchronization, total worker
time, dequeue age and detection-receipt-to-publication age. The switch remains
off by default and does not change matching, queueing, estimator, or publication
semantics.

Key ownership:

- `alignment.py`: event merging, previews, per-detection status access;
- `association.py`/`scoring.py`: per-estimator instance association/outcomes;
- `reduction.py`: usable-event and status-histogram reduction;
- `trial_results.py`: instance rows and annotations;
- `summary.py`: reports; `rendering.py`: collages;
- `target_distance_benchmark_runner_node.py`: ROS orchestration and lifecycle;
- `sweep.py`, `target_benchmark_sweep.py`, `sweep_report.py`: sweep domain,
supervisor, and reporting.

## Offline projective replay

Replay V1 is the measurement-accuracy path for box-gated stereoscopic
`projective_ranging` parameter sweeps. The live runner counts an exact number
of raw detector batches per trial, then performs a bounded exact-stamp drain.
It stores raw depth ROIs and camera context, including invalid values and
explicit missing matches; it never stores isolated foreground pixels or final
measurements.

The offline executor loads one trial per worker task and applies every variant
while that trial's arrays are resident. It reuses the live capture reduction,
association, scoring, trial-row, JSON/CSV, and sweep-report code. Per-config
keys that do not affect projective ranging are rejected; live-only sweep
defaults are removed from variant provenance.

A dataset is loadable only when every scenario trial produced a payload and the
manifest reports zero skipped trials. A bounded drain may legitimately retain
`NO_DEPTH_FRAME` evidence, but an infrastructure failure that skips the trial
marks the complete capture `incomplete`. `replay.json` records dataset and sweep
hashes, evaluator Git provenance, worker count, and evaluation wall time. The
root `summary.md` compares variants over the same frozen trials.

The root README owns the experimental capture and replay commands. Replay is
not authoritative for detector behavior, ROS delivery, timing, throughput, or
final integration, and the full-scenario performance gate remains outstanding.

Architecture guards prohibit reusable benchmark modules from importing the
runner node and prohibit perception/exploration from importing benchmarking.
Historical scenario/status transitions are retained in
[benchmark history](../history/benchmark_evolution.md).
