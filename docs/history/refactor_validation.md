# Target-localization refactor validation

2026-08-31. Structural implementation is complete. Runtime accuracy and exploration
checks passed. The depth-row coverage item is now closed: it was a pre-existing
blocking TF fallback, not a refactor regression — see "Resolution of the coverage
item" below. A smaller exact-stamp depth-availability loss remains open.

## Committed implementation

- `1354265`: generic target names, role-based module ownership, shared launch and
  topic contracts, one-way package dependencies.
- `0b8c10f`: mask synchronization and batch processing separated from ROS orchestration.
- `616c604`: visualization readings, HUD, markers, and style separated from the node.
- `74a8597`: simulator geometry, trial results, preview alignment, and reduction
  separated from benchmark runner orchestration.
- `95f4bd3`: single-owner shared tuning defaults, gate tokens, front offset, and
  integer timestamp helpers; stronger ownership and import tests.

Algorithm-specific defaults remain independent even when numerically equal.
The pointcloud and mask processes remain separate; no model or process was added.
Exact depth/color matching, batch-local RGB sharing, and zero-or-one scan projection
per batch are preserved by the existing behavior tests.

## Automated checks

- ROS-sourced direct pytest: **541 passed**.
- Colcon: all **36 registered test groups passed**, **541 individual tests**,
  zero errors/failures/skips. Three previously unregistered test files are now
  included; a test ensures future files cannot be omitted accidentally.
- Package build passed. All five public launches passed `--show-args`.
- `git diff --check` passed.
- Graphify rebuilt: 2,832 nodes, 5,147 edges, 152 communities.
- Architecture guards cover absolute and relative imports, prohibit
  `perception -> benchmarking` and `exploration -> benchmarking`, keep reusable
  modules below node orchestration, and enforce shared topic/default ownership.

## Benchmark evidence

Same examples scenario: five scenes, eight target instances, one repeat,
2-second settle, 10-second capture, all four estimators, box gate, stereoscopic
depth, default isolation recipes, unlimited mask-depth gate, video enabled.
All three runs included five trials and skipped none.

Baseline values below were captured during the pre-refactor run on 2026-08-30.
Its temporary artifacts are no longer present on the host. Both post-refactor
runs were inspected directly on 2026-08-31 and recorded clean-tree provenance.

| Estimator | Scored, all runs | Baseline MAE (m) | Post-refactor MAE (m), both runs | Baseline coverage | `74a8597` coverage | `95f4bd3` coverage |
|---|---:|---:|---:|---:|---:|---:|
| Pointcloud | 6/8 | 0.129655 | 0.129655 | 97.61% | 97.63% | 97.62% |
| Projective ranging | 6/8 | 0.055221 | 0.055221 | 45.02% | 40.71% | 40.08% |
| Euclidean reconstruction | 6/8 | 0.056944 | 0.056944 | 45.02% | 40.71% | 40.08% |
| Polar profiling | 5/8 | 0.081697 | 0.081697 | 38.65% | 37.94% | 38.49% |

Scored-instance outcomes are unchanged: two detector misses for each estimator,
plus one sparse-ray/no-value miss for polar profiling; no extra detections.
Final-run depth rows had 101 valid observations, 15 `NO_DEPTH_FRAME`, and 136
`UNSET` out of 252 observations. These counters show unavailable depth inputs,
but do not establish why coverage differs from the baseline. Do not describe the
two lower post-refactor results as proven harmless variance or performance parity.
A controlled before/after timing investigation is the remaining check; no gate,
matching tolerance, scheduling, or numerical tuning was changed to conceal it.

### Resolution of the coverage item (2026-08-31)

The open coverage item above is closed. Three findings must be kept apart.

**1. Refactor parity.** Matched three-repeat reruns of the old build `ebf3a02`
(44.12%) and refactored `f16afd0` (45.68%) did not reproduce a fixed refactor
regression; the earlier 40.71% / 40.08% single-repeat figures were not a stable
signal. This is absence of a reproduced regression, not proof of performance
equivalence for all workloads.

**2. The real defect was pre-existing and in TF.** `lookup_transform_components`
gave every candidate frame the full 0.5 s timeout, so the unresolvable configured
frame `r100_0001/robot/base_link` was waited on before the already-buffered
`base_link` on every call. That throttled the mask worker below its input rate,
and its latest-wins pending slot discarded the overflow as `UNSET`. Root cause
and fix are recorded in [operational incident history](operational_incidents.md#base-frame-fallback-latency--fixed-2026-08-31); `test_tf_utils` guards it.

**3. Measured effect of the fix**, same examples scenario, three repeats, normal
namespaced `base_frame` default — no `base_frame:=base_link` workaround:

| Estimator | Coverage before | Coverage after | `UNSET` before | `UNSET` after |
|---|---:|---:|---:|---:|
| Pointcloud | 97.62% | 99.60% | — | 3 |
| Projective ranging | 45.68% | 90.23% | 396 | 0 |
| Euclidean reconstruction | 45.68% | 90.23% | 396 | 0 |
| Polar profiling | 38.49% | 83.40% | — | 0 |

Both depth rows went from 344 to 674 valid observations of 747. `UNSET` is zero:
every detected box now reaches a mask result. Polar's remaining 124 misses are
all `TOO_FEW_RAYS_SELECTED`, a real sparse-ray outcome rather than a dropped
batch.

Accuracy is unchanged at six decimals, and scored outcomes are identical —
18/24, 18/24, 18/24, 15/24, six detector misses each plus three polar no-value
results, no extra detections, 15 trials with none skipped:

| Estimator | MAE (m) before | MAE (m) after |
|---|---:|---:|
| Pointcloud | 0.129655 | 0.129655 |
| Projective ranging | 0.055221 | 0.055221 |
| Euclidean reconstruction | 0.056944 | 0.056944 |
| Polar profiling | 0.081697 | 0.081697 |

**4. Residual loss, still open.** Both depth rows report `NO_DEPTH_FRAME` ×73 of
747 (9.77%). That is the separate exact-stamp depth-availability gap, unaffected
by this fix; the earlier direct-frame probe showed 7.32% of a smaller
single-repeat sample. It needs correlated generation/arrival/lookup
instrumentation before any change. Do not close it by switching to nearest-frame
depth matching or by widening the tolerance.

Provenance: run
`/tmp/dynamo-tf-fix-benchmark/20260831_134214_examples_box_stereoscopic`,
executed from an isolated worktree at `f16afd0` plus the TF fix, with its own
build and install tree, `ROS_DOMAIN_ID=42`. Isolation was necessary because
unrelated concurrent work was live in the primary working tree; the worktree's
four uncommitted files were the fix, its test, the test registration, and a venv
symlink. The host was loaded by those concurrent sessions (one `ekf_node`
update-rate warning, load average ~6). Load depresses throughput, so it cannot
have inflated this coverage result.

Post-refactor artifact directories (temporary, not checked in):

```text
/tmp/dynamo-post-refactor-benchmark-74a8597/20260831_121827_examples_box_stereoscopic
/tmp/dynamo-final-benchmark-95f4bd3/20260831_123422_examples_box_stereoscopic
```

Each contains `run.json`, summary, four CSVs, four scene collages, and an RViz
video. The fully occluded scene correctly has no collage. Final-run collage
inspection confirmed all estimator panels used the exact source frame (0.0 ms
delta); its video was readable and 69.3 seconds long.

## Exploration evidence and caveats

Two `mock_hospital` / `explore_lite` runs exercised the renamed stack. Samples
received detections, both measurement streams, HUD, markers, camera overlay, map,
costmap, and odometry. The first run travelled 4.60 m and 4.31 m in separate
30-second samples; the repeat travelled 4.10 m. RViz inspection confirmed the map,
velocity/coverage panel, four estimator columns, and camera overlay. No target was
in view during those samples, so they verify the empty-detection path; non-empty
estimator measurements are covered by the benchmark. The user visually approved
the repeat. This was a smoke test, not a full exploration-completion test.

The repeat had one collision-monitor heartbeat timeout; Nav2 automatically reset,
reactivated, and resumed motion. Shutdown also exposed simulator/RViz termination
delays and a mask-worker publisher-context race on Ctrl-C. The mask-worker shutdown
path and subscriber-count query already exist in baseline `ebf3a02`; this refactor
does not address lifecycle behavior. Runtime checks used only targeted shutdown
of processes belonging to these runs, not a machine-wide cleanup.
