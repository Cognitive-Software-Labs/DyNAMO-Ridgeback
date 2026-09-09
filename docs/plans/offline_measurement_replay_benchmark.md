# Plan: offline replay benchmark for measurement strategies

Status: **COMPLETE.** Replay V1 captures detector and sensor inputs once, then
evaluates any number of projective-ranging configurations against those
identical inputs. The clean 2026-09-09 full-scenario run selected five batches,
proved the 109-trial live/offline contract, and measured about a 50x end-to-end
speedup. It does not replace the live benchmark and does not authorize a
production measurement-default change.

Prepared 2026-09-09 against `79ccb8fc`.

### Validation evidence

The clean run at commit `b5a22c15` captured all 109 trials and 135 instances in
`501.118 s`, with no skipped trials and exactly five raw batches per payload.
The dataset occupies 11 MB and its NumPy payload entries compress 2.91:1. The
offline baseline matched all live outcomes and miss reasons; across 112 numeric
estimates its maximum live/offline difference was `4.04e-7 m`, consistent with
the live message's float32 transport. All 15 output CSVs were byte-identical at
1, 2, 4, 8, 16, and 32 workers. Sixteen workers was the measured knee on the
32-thread host at `0.53 s` end to end (`0.331 s` median evaluator time over five
runs). Capture plus evaluation was about 8 minutes 22 seconds, roughly 50x
faster than the prior 6.99-hour live-sweep estimate. See
[`docs/history/offline_measurement_replay_validation.md`](../history/offline_measurement_replay_validation.md)
for hashes, batch convergence, scaling, memory, limitations, and artifact paths.

## Decision to make

Can measurement recipes and numeric parameter variants be evaluated from one
versioned capture, with live-result parity and a material wall-time reduction,
instead of rerunning Gazebo and OWLv2 once per configuration?

The target architecture is:

```text
one live capture
       |
       v
versioned replay dataset
       |
       +-- baseline parameters
       +-- candidate 1
       +-- candidate 2
       +-- ...
       `-- candidate N
               |
               v
       paired accuracy report
```

The existing live benchmark remains authoritative for detector behaviour, ROS
delivery, latency, throughput, and final integration. Replay is authoritative
only for measurement behaviour over its frozen inputs.

## Why this is worth testing

The live runner currently spends a fixed 10 seconds capturing and 2 seconds
settling in every trial. The supervisor estimates 15.4 seconds total per trial,
so fixed waits account for about 78% of wall time. The packaged projective
parameter sweep expands to 15 configurations x 109 trials and currently
estimates 6.99 hours.

Those 15 configurations rerun the same simulator, detector, scenario poses, and
sensor path even though only a small measurement recipe or numeric constant
changes. A replay benchmark would make every candidate consume exactly the same
detections and sensor samples, improving experimental pairing while removing
repeated simulator and detector work.

## V1 scope and stop condition

Implement only the narrow case that motivated the plan:

- `projective_ranging`;
- `box` mask gating;
- stereoscopic aligned depth;
- `isolation_2d` recipe and numeric parameter variants;
- the existing scenario, association, scoring, and report contracts;
- an arbitrary number of variants from one sweep specification.

Defer detector/model comparisons, SlimSAM and Depth-Anything changes, polar
profiling, organized point-cloud replay, ROS throughput/concurrency measurement,
and per-variant RViz/video reproduction.

Stop V1 when live/offline parity is proven and the current 15-configuration
projective sweep is at least 5x faster end to end. If replay cannot clear that
bar without weakening the benchmark contract, do not expand it to other paths.

## Phase 0: freeze the comparison contract

Before implementation, record:

- the source commit and clean-tree state;
- the scenario bytes and SHA-256;
- the live parameter sweep bytes and SHA-256;
- the current live runner's per-instance CSV, miss-reason, association, and
  summary contracts;
- one compact scenario selection containing a single target, multiple targets,
  a detector miss, foreground occlusion, a far-range case, and background
  interference.

Ground truth remains scoring-only. It must never be supplied to a measurement
recipe or used to choose which events enter the dataset.

## Phase 1: define the replay dataset

Create a versioned dataset containing, per trial and raw detection batch:

- scenario, repeat, and trial identity;
- ground-truth target poses for later scoring;
- detection header and exact source stamp;
- image dimensions;
- empty detection batches as well as successful detections;
- labels, scores, and bounding boxes;
- raw aligned-depth ROI for each frozen box, including invalid samples;
- camera intrinsics;
- depth-source usable range;
- camera-to-base transform;
- source commit, capture parameters, scenario hash, and file hashes.

Store canonical depth values in metres but preserve invalid values and the
source's declared usable range so depth validity and working-range parameters
remain replayable. Do not store already-isolated foreground pixels or final
measurements: doing so would bake the strategy under test into the dataset.

Use one compressed payload per trial plus a human-readable manifest. One trial
is the worker and storage locality boundary: a worker should load its arrays
once and apply every variant to them. Avoid adding a storage dependency until
the measured dataset size demonstrates that the standard-library/NumPy format
is inadequate.

Colour previews are optional in V1. Accuracy and miss contracts must not depend
on generating a collage for every offline variant.

## Phase 2: capture by evidence rather than elapsed seconds

Add an offline-dataset capture mode to the live runner:

1. Spawn the scene and discard messages belonging to the preceding scene.
2. Count fresh raw detection batches, including `count == 0` batches.
3. Capture exactly `capture_batches` input opportunities.
4. Allow a bounded drain period for depth inputs corresponding to those stamps.
5. Record absent matches explicitly rather than waiting until success.
6. Retain `capture_timeout_sec` as the crash/stall safety bound.

Do not stop after one successful estimate or after every target has been found.
Either rule would bias the benchmark in favour of unreliable estimators and
would never terminate cleanly on intentional detector-miss scenes.

Keep fixed `capture_sec` mode for detector, throughput, transport, and
concurrency experiments. Select the V1 `capture_batches` default with a small
controlled comparison rather than guessing it.

## Phase 3: implement the offline executor

Add one command that accepts a replay dataset, a variant sweep, an output
directory, and a worker count. It must:

- reuse the existing sweep parsing and parameter validation where applicable;
- reject parameters that do not affect the selected measurement strategy;
- load one trial per worker task;
- evaluate every variant within that worker;
- parallelize independent trials across processes;
- preserve deterministic variant and trial ordering;
- avoid importing or starting ROS, Gazebo, OWLv2, or GPU-backed models.

Parallelize by trial chunks, not by launching one process per variant. Each
worker should keep one trial's depth arrays hot and run all variants over them;
this avoids repeatedly loading the same arrays and avoids multiplying memory or
model state by the number of candidates.

Within a worker, cache only intermediates whose inputs and semantics are shared
by a declared variant group. Cache keys must contain every parameter and code
version that can change the intermediate. An optimization that changes results
or hides a live parameter is a failed parity check, not an acceptable speedup.

## Phase 4: reuse scoring and reporting

Reuse the existing benchmark domain modules for:

- ground-truth association;
- per-instance outcomes and miss reasons;
- trial medians;
- CSV generation;
- `run.json` generation;
- per-variant summaries;
- cross-variant sweep reporting.

Do not create a second definition of accuracy, coverage, detector miss, gate
miss, or no-value. Add replay-specific provenance to the existing result shape:

- replay dataset ID and content hash;
- dataset schema version;
- evaluation commit and dirty count;
- complete variant parameters;
- shared-preprocessing/cache version;
- worker count and execution mode;
- per-trial differences from the named baseline.

All comparisons are paired by trial and event because every candidate consumes
the same captured evidence. Reports must not describe the variants as
independent benchmark runs.

## Phase 5: parity and regression proof

Before interpreting replay results:

1. Capture one small live dataset while running the matching live estimator.
2. Evaluate the live configuration offline from those exact inputs.
3. Require identical instance outcomes and miss reasons.
4. Require numerically equivalent per-event and per-trial measurements.
5. Prove multi-instance association parity.
6. Prove empty detections remain detector misses.
7. Prove sequential and parallel replay produce identical ordered outputs.
8. Prove changing one parameter affects only its registered strategy.

Cover malformed/incomplete datasets, missing depth matches, invalid depth
samples, unknown schema versions, irrelevant parameters, duplicate variant
names, interrupted output, and dataset-hash mismatch. An incomplete variant
must not leave a result that the resume logic accepts as valid.

## Phase 6: performance evaluation

Measure, rather than infer:

- total dataset size and compression ratio;
- one-time live capture time;
- sequential replay wall time;
- parallel replay wall time across representative worker counts;
- peak resident memory;
- time per trial and per variant;
- scaling as variants increase while the dataset stays fixed.

Run the current 15 projective configurations through both paths. The replay
path passes only if its default variant preserves the live contract and total
capture-plus-evaluation time is at least 5x faster than the current 6.99-hour
estimate. Record the worker-count knee instead of defaulting to every available
CPU when memory bandwidth or payload duplication stops scaling.

## Phase 7: final live confirmation

Replay selects candidates; it does not authorize a default change. For any
candidate proposed for production:

- run the current default and finalist through the live benchmark;
- confirm the accuracy, assignment, and miss-reason conclusions;
- check transport, timing, and integration behaviour;
- record both replay-dataset and live-run provenance;
- treat any disagreement as a replay-boundary bug until explained.

## Later extensions

Only after V1 passes its stop condition, consider:

- euclidean reconstruction from the same raw depth ROIs;
- shared cached SlimSAM masks;
- shared cached Depth-Anything output;
- polar replay with scans and time-resolved extrinsics;
- organized point-cloud replay;
- vectorized evaluation of compatible parameter grids.

Each extension needs its own live/offline parity fixture. Model outputs may be
cached when measurement parameters are under test, but a model must be rerun
when the experiment changes that model or its preprocessing.

## Deliverables

- Versioned replay schema and manifest documentation.
- Live capture mode with batch quota, drain bound, and hard timeout.
- Pure offline executor with bounded process-level parallelism.
- Reused live scoring/report outputs plus replay provenance.
- Unit, parity, corruption, determinism, and resume tests.
- One dated history report containing parity evidence, dataset size, scaling,
  wall-time comparison, limitations, and the decision on later extensions.
- Updated benchmark reference and public usage instructions only after the
  parity and performance gates pass.
