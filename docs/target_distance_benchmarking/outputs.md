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

An estimator dropped from a variant is named with its reason in `run.json` and in
the summary, rather than being silently absent — a table that simply lacks a row
reads as one that scored nothing.

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

Offline replay writes its own root beside these. `target_replay_benchmark`
stages the whole job and publishes it by atomic rename, so a public output
directory appears only after the entire job succeeded; an interrupted staging
directory is never accepted as a result. It contains `results/` and `job.json`.
`target_offline_replay_benchmark` writes its results at the output root instead.
Both record `replay.json` with dataset and sweep hashes, evaluator Git
provenance, worker count, and evaluation wall time, and a root `summary.md`
comparing variants over the same frozen trials.

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
- `target_distance_benchmark_runner_node.py`: ROS orchestration and lifecycle;
- `sweep.py`, `target_benchmark_sweep.py`, `sweep_report.py`: sweep domain,
supervisor, and reporting.

Architecture guards prohibit reusable benchmark modules from importing the
runner node and prohibit perception/exploration from importing benchmarking.
