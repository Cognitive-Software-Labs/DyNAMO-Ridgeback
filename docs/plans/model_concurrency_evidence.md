# Plan: establish whether model concurrency is a real bottleneck

Status: **READY FOR MEASUREMENT.** The default-off timing instrumentation and
the fourteen-configuration sweep contract are implemented; the controlled run
and decision remain open. This is not authorization to integrate AutoVision or
change the production execution architecture. The plan was prepared 2026-09-03
against `cb8061a` after the exact-depth transport investigation selected
CycloneDDS.

## Decision to make

Determine, on the current stack, whether `mask_gate:=silhouette` plus
`depth_source:=monocular` fails to keep up because SlimSAM and Depth-Anything
execute sequentially in the mask worker. If it does, measure which concurrency
architecture is most likely to help. If it does not, close the hypothesis with
evidence instead of adding multiprocessing.

Answer these separately:

1. Does the heavy path drop or replace detection batches in steady state?
2. What portion of service time belongs to SlimSAM, Depth-Anything, shared RGB
   preparation, depth reduction, and the remaining pipeline?
3. Is the combined cost additive, or is there extra GPU/memory contention?
4. Is cold model startup a separate operational problem from warm throughput?
5. Would concurrency reduce the critical path on the actual GPU, or merely add
   IPC, copies, duplicate runtime state and VRAM pressure?
6. Does AutoVision's actual multiprocessing/`rs_ipc` implementation outperform
   Python shared memory, ROS 2/CycloneDDS, and the current in-process path for
   DyNAMO's real payload and scheduling semantics?

## Scope and boundaries

- Primary workload: exactly `projective_ranging,euclidean_reconstruction` so
  both depth rows exercise the same produced depth without point-cloud or polar
  work adding noise.
- Primary formal evidence comes from the simulator benchmark with ground truth.
  Exploration is a final qualitative smoke only.
- Keep exact-stamp semantics, D455 geometry, detector settings, estimator
  recipes, mask/depth gates, scenario bytes, scoring and CycloneDDS fixed.
- Do not reintroduce a separate always-on depth producer. Experimental model
  workers must be detection-keyed and return results for the requested stamp.
- Use `spawn`, never `fork`, for CUDA-owning Python processes.
- Do not add an unbounded queue. Record queue capacity, drop/backpressure policy,
  and every discarded stamp.
- Do not interpret GPU utilization alone as useful overlap. Completion rate,
  end-to-end latency, drops, accuracy, RTF, RSS/PSS and VRAM decide.
- Do not modify production execution architecture in this assignment. The
  default-off timing diagnostic and reproducible sweep configuration are
  preparation and should be committed; concurrency prototypes belong in an
  isolated worktree and must not be committed. If a candidate wins, produce a
  separate implementation handoff.
- Preserve unrelated processes and changes. Use a dedicated ROS domain and
  unique `/tmp/dynamo-model-concurrency-*` artifact roots. Inspect ownership
  before cleanup and never terminate another agent's run.
- Record exact commands and environments. Commit and validate every preparation
  file before starting Gazebo; do not push unless explicitly requested.

## Phase 0: freeze provenance and host conditions

Before instrumentation or a run, record:

- repository HEAD/status and installed-workspace provenance;
- scenario path and SHA-256;
- ROS distribution, RMW implementation, ROS domain and relevant environment;
- Python, PyTorch, Transformers, CUDA, driver and model checkpoint revisions;
- CPU model/count, RAM, GPU model/VRAM, power mode and clocks where available;
- baseline host CPU/GPU load and active simulator/model processes;
- simulation real-time factor and whether RViz/video recording are enabled.

Use the same packaged examples scenario for the screening and decision matrix:
five scenes, eight instances, 2 s settle and 10 s capture. Use the same scenario
bytes for every cell. Run on an otherwise idle host first; load sensitivity is
a later, explicitly separate axis.

The formal simulator comparisons must be driven by
`target_benchmark_sweep` YAML, not by a series of independent launch commands.
The sweep is the experiment contract: it keeps Gazebo, the D455 camera, TF,
Ridgeback and OWLv2 alive while restarting only the per-configuration layer.
Store the ordered configurations, shared defaults and repeat counts in the YAML
so the run can be validated, resumed and reproduced without reconstructing
shell history.

## Phase 1: add default-off stage timing

The existing diagnostics already expose detection receipt/replacement, dequeue
age, total worker time and lock timing. They do not provide an unthrottled
distribution for each model stage. Add the smallest default-off diagnostic that
records constant-space count/mean/max plus bounded samples or histogram bins for:

1. detection receipt to dequeue;
2. RGB decode/preparation;
3. SlimSAM forward plus post-processing;
4. Depth-Anything input conversion, forward, tensor transfer and resize;
5. mask/depth preparation and both estimator reductions;
6. full worker service time;
7. detection stamp/receipt to measurement publication;
8. pending-slot replacements, completed batches and per-stamp outcomes.

CUDA launches are asynchronous. Synchronize the relevant device immediately
before and after model-stage measurements, or use CUDA events with an explicit
synchronize before reading them. Report synchronization overhead and keep it
identical across compared cells.

Externally sample at a fixed low rate:

- GPU utilization, memory, power and temperature;
- per-process CPU, RSS/PSS and context switches;
- host load and memory pressure;
- simulation RTF and wall duration.

Separate **cold** and **warm** behaviour. Record node start, model-load duration,
first detection, first valid publication and first five batches. Exclude cold
loads from steady-state percentiles, but report them as an independent result.

Add focused tests proving diagnostics are bounded, default-off, do not alter
the output/status contract, and count one completed/replaced batch correctly.
Run the A/A comparison as the first two configurations in the same persistent
sweep, with the only changed YAML key being `depth_match_debug: false` versus
`true`. If the probe materially changes throughput or replacements, treat the
remaining instrumented matrix as observational and reduce the probe before a
follow-up run.

## Phase 2: current-architecture factorial benchmark

Run these four cells with identical estimator selection and benchmark settings:

| Cell | Mask gate | Depth source | Mask-node model work |
|---|---|---|---|
| A: floor | `box` | `stereoscopic` | neither model |
| B: segmentation | `silhouette` | `stereoscopic` | SlimSAM only |
| C: monocular | `box` | `monocular` | Depth-Anything only |
| D: combined | `silhouette` | `monocular` | SlimSAM then Depth-Anything |

Use the installed `config/benchmark_sweep_model_concurrency.yaml` as the
experiment contract. It begins with the combined-path diagnostic A/A pair and
then contains the twelve matrix configurations below. Its shared `defaults`
pin the examples scenario, three scene repeats,
`estimators: projective_ranging,euclidean_reconstruction`, capture/settle
settings and every non-factor setting. Every config explicitly pins
`mask_gate`, `depth_source`, and the diagnostic state, so inheritance cannot
make the experimental axes ambiguous. The supervisor records the sweep and
scenario SHA-256 values in `sweep.json`.

Encode three process-level replications as twelve uniquely named configs in
this exact rotated order:

```text
replicate 1: A, B, C, D
replicate 2: D, C, B, A
replicate 3: B, D, A, C
```

The YAML list order is the execution order. The A/A pair runs first; the matrix
rotation distributes warm-up and thermal drift instead of always favouring the
same cell. A replicate is a fresh per-config process; `repeats: 3` inside each
config supplies the repeated scenes. Do not replace this with twelve separately
started environments.

Procedure:

1. Finish the diagnostic, YAML, tests, documentation and installed-workspace
   build. Commit every changed file and require `git status --porcelain` to be
   empty before any command is allowed to start Gazebo.
2. Validate the installed YAML and its resolved trial/time estimate with
   `target_benchmark_sweep <yaml> --dry-run`.
3. Run the complete fourteen-config YAML once through
   `target_benchmark_sweep`. Let the supervisor execute configurations
   sequentially against its one persistent environment and use its normal
   resume behavior after an interruption.
4. Require each per-config process to terminate and verify that its model memory
   is released before the supervisor starts the next config. Never run
   `cleanup.sh` between configurations; it would kill the controlled persistent
   environment.
5. Do not edit, regenerate or reorder the YAML during a run. A changed YAML or
   scenario hash starts a new experiment rather than resuming the old one.
6. Keep diagnostics, video/RViz state and external sampling identical. If video
   is disabled for timing, disable it everywhere and run one separate visual
   integrity smoke.

For every cell report warm median/P95/P99 and worst-case values for every stage,
observed detection inter-arrival distribution, batch completion ratio, pending
replacements, dequeue age trend, end-to-end publication age, trial coverage and
miss reasons. Also report scored instances, MAE/median/P95 error, RTF, wall time,
CPU, RAM and VRAM.

Compute the interaction term rather than merely comparing D to A:

```text
SlimSAM contribution       = B - A
Depth-Anything contribution = C - A
Expected additive D         = A + (B - A) + (C - A)
Observed interaction        = D - expected additive D
```

Use the same latency statistic for every term. A positive interaction suggests
resource contention or combined-path overhead; a near-zero interaction means
the sequential cost is simply additive.

## Phase 3: decide whether a concurrency problem exists

Use observed cadence, not only the current nominal 10 FPS limit. Declare a current warm
throughput problem only if repeated D runs show one or more of:

- pending replacements or a materially lower completed-batch ratio than B/C;
- service time at or above the observed arrival interval, with dequeue age
  increasing rather than returning to baseline;
- P95 end-to-end publication age crossing the project's display/usefulness
  window;
- CUDA OOM, memory thrash, repeated model reloads, or RTF degradation that is
  specific to D;
- brief benchmark trials losing scored outcomes because D cannot produce a
  timely value.

Distinguish four conclusions:

1. **No current problem:** D keeps up without replacements or accumulating age.
2. **Cold-start only:** first-use model loading loses work, warm execution keeps up.
3. **Warm sequential overload:** the sum of model stages exceeds sustainable
   cadence without a large interaction term.
4. **Resource interaction:** D is materially worse than the additive prediction.

If conclusion 1 holds, stop. Record the evidence in history and move this plan
to `docs/do_not_try_again/` with the conditions that would justify reopening it.
Do not prototype AutoVision.

## Phase 4: candidate experiments, only after overload is proven

First capture a deterministic replay corpus of actual RGB frames, detection
boxes/stamps and expected masks/depth outputs from D. Keep it outside Git with a
manifest and checksums. Use it to compare candidates without Gazebo variance.

Test at least these alternatives; add evidence-driven candidates not listed:

| Candidate | Question answered |
|---|---|
| Current sequential in-process path | Reference latency, memory and outputs |
| Same-process threads plus CUDA streams/events | Can the two GPU workloads overlap without IPC or duplicated processes? |
| Two spawned Python model workers using `multiprocessing.shared_memory` | Does process isolation/overlap beat serialization and duplicated CUDA context cost? |
| Separate ROS 2 nodes over CycloneDDS | Does the architecture already used by DyNAMO provide sufficient isolation and operational semantics? |
| AutoVision's pinned multiprocessing + actual `rs_ipc` code | Does its shared-memory transport and policy outperform the simpler alternatives on this payload? |
| Staged cross-frame pipeline | Can SlimSAM for one stamp overlap Depth-Anything for another while keeping bounded, explicit latest/ordered semantics? |

For AutoVision, inspect the current repository and pinned `rs_ipc` revision
again rather than relying on the earlier snapshot. Record license, build/runtime
dependencies, supported payload ownership, blocking/latest-frame semantics,
failure handling and cleanup. Benchmark the real code; do not substitute a
home-grown queue and label it AutoVision.

Every candidate must:

- load each model once and use the same checkpoints/device/precision;
- warm up identically, synchronize GPU timing and run enough samples for stable
  median/P95/P99;
- use the same bounded input policy and record every dropped/superseded stamp;
- measure enqueue, transfer/copy, queue wait, model, return-transfer and total
  completion latency separately;
- record parent and child RSS/PSS, total VRAM and CUDA context overhead;
- detect stale/out-of-order results, worker crashes and clean shutdown;
- compare output masks/depth arrays to the sequential replay within the existing
  numerical contract;
- never pair results with a different stamp.

Do not infer benefit from transport microbenchmarks alone. A faster RGB handoff
cannot justify a model-process split if GPU execution serializes or duplicated
contexts consume unacceptable VRAM.

## Phase 5: live validation of finalists

Take at most the two replay candidates that materially outperform sequential D
and exercise them as temporary live prototypes. Define a second checked-in or
artifact-pinned sweep YAML containing current sequential D and each finalist as
uniquely named, rotated replications. Run that YAML once through the same
persistent-environment supervisor; do not compare finalists using unrelated
manual launches. Keep the Phase 2 D settings and scenario identical except for
the explicitly named execution architecture.

Success requires all of:

- a reproducible reduction in pending replacements/completion loss when the
  baseline exhibits them;
- P95 end-to-end latency improvement larger than both 10% and twice measured
  A/A run variation;
- no worse scored-instance outcome and no material MAE/coverage regression;
- exact stamp identity for every consumed result;
- bounded memory/queue growth, no OOM, no stale/out-of-order publication and
  clean worker shutdown;
- no material RTF regression hidden by a perception-only speedup.

If no candidate clears those gates, recommend keeping the current architecture
and name the actual limiting resource. If AutoVision clears them, produce a
separate compatibility-safe implementation plan covering ownership, packaging,
license attribution, failure recovery, rollback and production tests. Do not
integrate it in this assignment.

## Phase 6: verification and report

Run focused instrumentation/model/mask/launch/benchmark tests, then the complete
package suite. Build the installed workspace and verify all four launch cells.
Run `git diff --check`; rebuild Graphify after any temporary code work before
capturing source-structure evidence. Remove all temporary product changes from
the main worktree before delivery.

Deliver one dated history document containing:

- provenance, commands, sweep-YAML path and hash, scenario hash and artifact
  paths;
- cold versus warm results;
- the four-cell stage/throughput/accuracy/resource table;
- A/A variance and factorial interaction calculation;
- the evidence-based conclusion category from Phase 3;
- candidate replay and live results, if Phase 4 was reached;
- which hypotheses were rejected and why;
- exact remaining uncertainty and recommended next action.

Only add a concurrency item to `docs/BACKLOG.md` if the current live D path
demonstrably fails its throughput/latency contract. Exploration may follow as a
visual operational smoke, but it cannot supply accuracy evidence.

Stop when the evidence supports one of: no current problem, a precisely scoped
cold-start issue, a proven warm bottleneck with a winning candidate, or a proven
bottleneck with no acceptable candidate.
