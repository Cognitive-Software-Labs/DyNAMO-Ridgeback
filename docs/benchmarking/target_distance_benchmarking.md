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

## Local configuration UI

`target_benchmark_configurator` is a local-only browser UI for building a
benchmark job and running it. It binds only to `127.0.0.1`, chooses an available
port by default, and puts an unguessable token in the URL and every API request.
Use `--no-open` to print the URL for a headless or remote session.

The UI imports and exports the canonical replay-job format, then validates the
generated document through `parse_job`, exactly as the replay command does.
Importing and immediately exporting an unchanged job is byte-stable.

The server writes the job itself, to
`artifacts/benchmark-jobs/benchmark-<profile>-<run id>.yaml` with absolute paths
throughout, and the rendered command names that file. Every operator-supplied
path is anchored on the workspace root once, by `paths.anchored_path`, so what
the page reports about an input is what the run resolves. Browser download of a
job cannot do this: the page never learns the download directory, while a job's
relative paths resolve against the job file's own parent. The reversal and its
evidence are in
[running benchmarks from the configurator GUI](../history/benchmark_gui_direct_run.md).

The three offline profiles — legacy `measurement` replay, frozen `mask-output`
comparison, and `mask-model` materialization — start, cancel, and tail their log
from the page. They route through one command and import no ROS runtime, so they
cannot orphan a simulator. Run records live under
`artifacts/configurator/runs/<run_id>/`; runs outlive the GUI and the tab
re-attaches from disk. Concurrent runs are refused server-side, because two
replays contend for the worker counts the operator chose. Cancellation signals
the process group and escalates SIGINT → SIGTERM → SIGKILL.

`live-system` sweeps stay terminal-only: `target_benchmark_sweep` runs
`run_preflight_cleanup`, whose catch-all would `kill -9` the page. The UI shows
a disabled Start with that reason. See
[troubleshooting](../troubleshooting.md).

The browser is a client of the ROS-free replay-profile capability contract;
the service is the authority for profile and axis decisions. The GUI validates
canonical replay jobs and their typed artifacts before running, so it cannot
misrepresent a live or box-gated run as a frozen-mask experiment.

The combined installed-package suite passed with 758 tests after the
configurator landed, as recorded in
[layered replay implementation validation](../history/layered_replay_implementation_validation.md).
That record contains no canonical browser screenshot or visual-acceptance
artifact; the functional contract is verified, while visual proof is not
claimed.

## Legacy measurement replay

The legacy measurement profile is the measurement-accuracy path for box-gated stereoscopic
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

The root README owns the capture and replay commands. Replay is
not authoritative for detector behavior, ROS delivery, timing, throughput, or
final integration. The 2026-09-09 clean full-scenario run selected five batches,
proved live/offline parity across 135 rows, and cleared the performance gate by
about 50x; the complete evidence is in
[the validation history](../history/offline_measurement_replay_validation.md).

## Layered replay profiles

`benchmarking/replay_profiles.py` is the ROS-free authority for the four stable
profile IDs, question mappings, mutable/frozen stages, legal axes, types,
ranges, compatible estimators, supported claims, and limitations. The command
`target_replay_describe --json` exposes that exact contract to CLIs and the
configurator. Invalid upstream knobs use structured errors; for example,
`segmentation_model` under `mask-output` recommends `mask-model`.

The offline artifact graph uses a version-2 typed envelope while the compact
`schema_version: 1` dataset continues to load unchanged:

```text
sensor-capture (exact detections + RGB + depth + calibration + TF)
    ├── box mask-cache ───────────┐
    └── SlimSAM mask-cache ───────┴── measurement variants
```

Each typed manifest records `format: dynamo-replay`, manifest/payload versions,
a content-derived artifact ID, completeness, payload hashes, producer signature,
Git/dependency/model provenance, and trial/event/detection counts. A mask cache
also pins its sensor parent by artifact ID and exact manifest SHA-256. Loading a
cache without that parent, with a different parent, with a changed manifest, or
with a changed payload fails before evaluation.

Sensor payloads contain one losslessly compressed NumPy file per trial. Every
raw batch remains present, including empty detections. RGB is exact `uint8` on
the detection grid; aligned depth is exact float32 metres with NaN/Inf/zero
semantics retained. Missing RGB, depth, intrinsics, or transforms remain
explicit `null` evidence. Ground truth is stored only in trial scoring metadata
and is never passed into a mask producer.

Mask caches store one index-aligned outcome per parent detection. `None` is an
explicit producer miss and differs from a valid empty `MaskRegion`. Regions
retain origin, full-grid size, `rect|tight` precision, shape, and bit-packed
boolean pixels. Tight regions use their actual nonzero extent, which may extend
outside the padded prompt box. Changing any producer parameter or provenance
creates a separately identified immutable cache.

`target_replay_materialize_masks` loads one model once and streams all parent
trials through it; the box producer needs no model. A canonical `mask-model` job
may name several materializations. Its default is one model worker, and parallel
model workers require distinct explicit devices so a parameter grid cannot load
many copies onto one GPU accidentally.

`target_replay_benchmark` validates a job and all lineage before work. CPU
workers load one trial plus its selected caches and evaluate every compatible
projective/Euclidean numeric variant while the arrays are resident. Both depth
rows call the live `fill_path_measurements` kernel and therefore share one
prepared depth region per detection. Process-map ordering plus ordered reduction
makes sequential and parallel result rows byte-stable. Offline jobs accept only
the selected profile's axes in both sweep defaults and individual configs; use
a replay-specific sweep rather than carrying ignored live-only defaults into a
job. The public output directory appears only after the entire job succeeds;
interrupted staging directories are not accepted as results.

Offline reports carry the profile boundary. `measurement` and `mask-output`
cannot support model/runtime claims. `mask-model` may retain isolated producer
duration as diagnostics, but only `live-system` can establish ROS delivery,
end-to-end latency, throughput, GPU contention, simulator real-time factor, or
integration behavior. Implementation evidence and the remaining live gates are
recorded in
[the layered replay validation note](../history/layered_replay_implementation_validation.md).

Architecture guards prohibit reusable benchmark modules from importing the
runner node and prohibit perception/exploration from importing benchmarking.
Historical scenario/status transitions are retained in
[benchmark history](../history/benchmark_evolution.md).
