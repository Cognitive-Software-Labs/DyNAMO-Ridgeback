# Refactor coverage investigation — August 30–31, 2026

Recorded dates: 2026-08-30, 2026-08-31

Tested revisions: `ebf3a02982f19925cd1a37d1479f437f02a2f123`, `f16afd0c70b3ef259c978d36949c8fae9fd4fec5`, `74a8597e9a7ee76b4848e5aa9b4d2553fa1fcaf5`, `95f4bd3db97c5799c6b8e16e878243e90416a940`

Provenance: partial

This is dated evidence. Missing revisions or preserved worktree inputs limit
reproducibility; no new measurements were made during archive curation.

## Conclusion

The initial post-refactor single-run coverage drop did not reproduce as a fixed
refactor regression. A separate pre-existing TF fallback wait throttled mask
processing; correcting it improved measured coverage without changing scored
outcomes or six-decimal aggregate accuracy. Exact-depth delivery was a separate
investigation, not resolved by this refactor.

## Benchmark evidence

Same examples scenario: five scenes, eight target instances, one repeat,
2-second settle, 10-second capture, all four estimators, box gate, stereoscopic
depth, default isolation recipes, unlimited mask-depth gate, video enabled.
All three runs included five trials and skipped none.

Baseline values below were captured during the pre-refactor run on 2026-08-30.
Its temporary artifacts are no longer present on the host. Both post-refactor
runs were inspected directly on 2026-08-31 and recorded clean-tree provenance.

| Estimator | Scored, all runs | Baseline MAE (m) | Post-refactor MAE (m), both runs | Baseline coverage | `74a8597e9a7ee76b4848e5aa9b4d2553fa1fcaf5` coverage | `95f4bd3db97c5799c6b8e16e878243e90416a940` coverage |
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
A controlled before/after timing investigation was the remaining check at this
stage; no gate,
matching tolerance, scheduling, or numerical tuning was changed to conceal it.

### Resolution of the coverage item (2026-08-31)

The open coverage item above is closed. Three findings must be kept apart.

**1. Refactor parity.** Matched three-repeat reruns of the old build `ebf3a02982f19925cd1a37d1479f437f02a2f123`
(44.12%) and refactored `f16afd0c70b3ef259c978d36949c8fae9fd4fec5` (45.68%) did not reproduce a fixed refactor
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
every detected box at that stage reaches a mask result. Polar's remaining 124 misses are
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

**4. Residual loss at this stage, closed 2026-09-01.** Both depth rows reported
`NO_DEPTH_FRAME` ×73 of 747 (9.77%). The later correlated transport
investigation located subscriber-specific image delivery loss under Fast DDS;
the matched CycloneDDS run delivered 706/706 observations with no
`NO_DEPTH_FRAME`. Exact matching and its zero-tolerance contract did not change.

Provenance: run
`/tmp/dynamo-tf-fix-benchmark/20260831_134214_examples_box_stereoscopic`,
executed from an isolated worktree at `f16afd0c70b3ef259c978d36949c8fae9fd4fec5` plus the TF fix, with its own
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
path and subscriber-count query already exist in baseline `ebf3a02982f19925cd1a37d1479f437f02a2f123`; this refactor
does not address lifecycle behavior. Runtime checks used only targeted shutdown
of processes belonging to these runs, not a machine-wide cleanup.
