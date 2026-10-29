# Benchmark outputs and provenance

What a run writes, what each field means, and which module owns it. Start at
[running benchmarks](running_benchmarks.md) if you are choosing how to launch.

## Outputs and provenance

Default root: `<repo>/artifacts/benchmarks`. Each timestamped run contains:

- `summary.md` — human-readable accuracy, reliability, provenance, and scenes;
- `run.json` — machine-readable source of run parameters and aggregates;
- one trial-level CSV per selected estimator, including misses;
- `images/<trial_id>.png` collages;
- `video/run.mp4`, best-effort RViz capture when recording succeeds.

Start with `summary.md`; `run.json` carries the same run-level numbers for
machines.

The `run` object records label/start, commit/branch, numeric
`uncommitted_files`, absolute scenario path, and included/skipped counts.
`parameters` records node parameters. Each `estimators` entry records stable key
and display name; row/scored counts; absolute/relative error aggregates;
detector/gate/no-value misses; extra detections; observation totals/OK/coverage;
and `reason_histogram`. A nonzero `uncommitted_files` means the commit alone does
not reproduce the run.

The run also names its simulator:
- `run.backend`, `run.world` (the public name) and `run.native_world`.
- `run.simulation`:
  - the backend and both world names;
  - the normalized backend-only `settings`, and the camera profile;
  - the adapter package and version;
  - the simulator name and version (and, for Isaac, its interpreter);
  - the scene-control service namespace and its reported features;
  - the asset catalog's schema, path and SHA-256, with every resolved resource;
  - a readiness timeline: services available, first clock, clock advancing,
    and first spawn.
- `parameters` repeats `backend` and `backend_settings`.
- `summary.md` names the backend, the world (with its native name when it
  differs) and the catalog.

CSV rows include trial/scene/instance and spawn truth; estimator/outcome/reason;
estimate and absolute/relative error; usable events and captured frames; and
collage path. Read `run.json` for run-level statistics and CSVs for instance rows.

An estimator dropped from a variant is named with its reason in `run.json` and in
the summary, rather than being silently absent — a table that simply lacks a row
reads as one that scored nothing.

### Run folder and file names

A run folder is named
`<timestamp>_<scenario>[_<gate>][_<depth_source>]` under `output_dir`, for
example `20260822_202851_full_silhouette_stereoscopic`. The timestamp leads so
folders sort chronologically. The packaged `benchmark_scenarios_full.yaml`
shortens to `full`; any other scenario keeps its file stem. The gate appears
only when a mask estimator ran, and the depth source only when a depth-path
estimator ran. `run_dir_name` replaces the whole name; a sweep uses the
configuration name. `benchmarking/naming.py` owns these rules.

Mask-row CSV names fold in the config axes that executed: the mask gate, the
aligned-depth source, the path, and on the box gate the foreground recipe
(`isolation_2d` for projective ranging, `isolation_3d` for Euclidean
reconstruction):

| Row | Example CSV |
|---|---|
| Box gate, stereoscopic depth, projective ranging | `box_gated_stereoscopic_projective_ranging_nearest_mode_histogram.csv` |
| Box gate, stereoscopic depth, Euclidean reconstruction | `box_gated_stereoscopic_euclidean_reconstruction_height_crop_nearest_mode_band.csv` |
| Silhouette gate, no recipe token (the tight branches run none) | `silhouette_gated_stereoscopic_projective_ranging.csv` |
| Polar profiling, gate only | `box_gated_polar_profiling.csv` |
| Point cloud | `pointcloud.csv` |

### Child ROS logs

Benchmark entrypoints point each child's `ROS_LOG_DIR` at its own output:
`<run>/logs/ros/` for a standalone runner, `<sweep>/environment/logs/ros/` for a
sweep's environment, and `<sweep>/<config>/logs/ros/` for each configuration.
An explicit `ROS_LOG_DIR` always wins. `output_dir:=...`, or a sweep's
`defaults.output_dir`, moves the run or sweep root.

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

## Sweep outputs

`target_benchmark_sweep` validates YAML, starts one persistent environment,
runs one restartable config at a time, resumes configs only from valid `run.json`
without a recorded launch or configuration-shutdown failure,
records real-time factor plus sweep/scenario SHA-256 provenance, and emits
`sweep.json` plus `summary.md`. `sweep.json` carries the same `simulation`
record as each run. Its resume signature includes the backend, the normalized
backend settings, the native world, and the catalog bytes, so a sweep never
resumes onto a different simulator or catalog. Do not replace process restarts with runtime
estimator mutation: selected rows determine node existence.

One timestamped sweep folder holds the manifest, the comparison report, and one
ordinary run folder per configuration:

```text
artifacts/benchmarks/20260828_141530_baseline/
  sweep.json
  summary.md
  pointcloud/
    run.json  summary.md  pointcloud.csv  images/  video/run.mp4
  mask_projective_ranging/
    ...
```

The sweep `summary.md` has one row per `(config, estimator)`, sorted by mean
absolute error. It also records each configuration's wall time and a real-time
factor sampled just before that configuration started. RTF is diagnostic only,
but it helps separate configuration effects from observation-coverage drift as a
long-lived simulator slows down. The [sweep mechanism](sweeps.md) owns how the
folder is produced and resumed.

Each configuration records `output_valid`, `launch_returncode`, and `shutdown`.
A valid `run.json` remains available after a process or shutdown failure, but
does not make that configuration successful or eligible for automatic reuse
after a recorded nonzero launch status, escalation, incomplete session cleanup,
or cleanup error. Retrying preserves those measurements in a sibling
`.incomplete_*` directory. Older records without lifecycle fields retain their
existing compatibility behavior; absence is not new lifecycle proof.

`lifecycle.groups` records the final owned-group stop results; `lifecycle.errors`
retains cleanup failures and `lifecycle.clean` describes that final cleanup.
Each stop result records leader status, signals, recorder-specific signals,
escalation, and whether living group members remain. A configuration can fail
before a later clean final cleanup, so inspect both its status and the lifecycle.
The human-readable sweep summary surfaces shutdown failures alongside retained
comparison rows. Neither saved measurements nor launcher exit zero establishes
that every child stopped cleanly.

For model-throughput investigations, `depth_match_debug:=true` also emits a
bounded mask-worker timing summary. Each distribution retains the first five
calls as cold evidence and reports warm count/mean/P50/P95/P99 plus the all-time
maximum.
Stages cover RGB preparation, SlimSAM load/inference, stereo or monocular depth,
mask-region preparation, estimator reduction, CUDA synchronization, total worker
time, dequeue age and detection-receipt-to-publication age. The switch remains
off by default and does not change matching, queueing, estimator, or publication
semantics.

Offline replay writes its own root beside these. `target_replay_benchmark`
stages the whole job and publishes it by atomic rename, so a public output
directory appears only after the entire job succeeded; an interrupted staging
directory is never accepted as a result. It contains `results/` and `job.json`.
`target_offline_replay_benchmark` writes its results at the output root instead.
Both record `replay.json` with dataset and sweep hashes, evaluator Git
provenance, worker count, and evaluation wall time, plus a root `summary.md` and
`sweep.json` comparing variants over the same frozen trials. Each variant gets
the normal CSV, `run.json` and `summary.md` set.

## Renaming a result

The [configurator](configurator.md)'s Results panel can rename artifacts and run
outputs. The name is load-bearing in three cases and rename is refused with the
reason:

- a **sweep config subdirectory**, because a sweep addresses it as
  `<sweep>/<config name>` and both resume and reporting would stop finding it;
- a **staging directory**, because it is an interrupted writer's scratch space
  and not a published result;
- anything that is **not a benchmark artifact or run output**.

Renaming a **sweep** is allowed and rewrites `configs[].arguments.output_dir` in
that sweep's `sweep.json` to the new directory. The sweep writes that key as the
sweep directory itself for every config, so the rewrite restates what the key
already means and also repairs entries left stale by an earlier move.

Renaming an **artifact** is allowed. Artifact lineage is content-addressed and
survives it, but saved jobs name their inputs by path, so any job referencing the
old name is reported.

## Module ownership

- `alignment.py`: event merging, previews, per-detection status access;
- `association.py`/`scoring.py`: per-estimator instance association/outcomes;
- `reduction.py`: usable-event and status-histogram reduction;
- `trial_results.py`: instance rows and annotations;
- `summary.py`: reports; `rendering.py`: collages;
- `backends.py`: the registered simulators, their adapter packages and
  backend-only settings, and refusal of `hardware`;
- `asset_catalog.py`: logical-name-to-resource catalogs and native worlds;
- `scene_control.py`: the simulator-neutral `simulation_interfaces` client
  (bounded calls, ownership of `bench_*` names, readiness, rollback);
- `simulation.py`: pose normalization and ground-truth instances;
- `target_distance_benchmark_runner_node.py`: ROS orchestration and lifecycle;
- `sweep.py`, `target_benchmark_sweep.py`, `sweep_report.py`: sweep domain,
supervisor, and reporting.

Architecture guards prohibit reusable benchmark modules from importing the
runner node and prohibit perception/exploration from importing benchmarking.
