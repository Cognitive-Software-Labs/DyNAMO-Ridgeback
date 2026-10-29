# BENCHMARKING — Live configurator sweeps

Status: **cleanup ownership done (2026-10-14); live start, conflicts and
supervision remain.** The
[benchmarking backlog](../target_distance_benchmarking/BACKLOG.md#live-sweeps-from-the-configurator)
owns this feature and its priority. This plan covers the remaining live-run implementation, not the completed
[offline run infrastructure](../target_distance_benchmarking/configurator.md).

## Cleanup ownership (done)

The UI still disables Start for `live-system`, and
`RunSupervisor.ensure_startable` still rejects it server-side, but preflight
cleanup no longer kills the configurator. [`cleanup.sh`](../../cleanup.sh) now
applies one ownership filter to every kill pass and to the leftover check. It
skips cleanup's ancestors, the GUI, the configurator and offline replay CLIs, and
processes started from another colcon workspace, and it matches node names as
whole path components. The
[troubleshooting entry](../troubleshooting.md#what-cleanupsh-leaves-alone)
describes the behaviour; `test_cleanup_ownership.py` checks it on sentinel
processes in a dry run (`CLEANUP_DRY_RUN=1`), which kills nothing.

Not covered: two runs from the **same** checkout are not isolated from each
other, so preflight still stops a concurrent terminal-started sweep's nodes. That
belongs to conflict detection below. Do not enable Start by bypassing preflight
cleanup.

## Remaining implementation

### Dry-run estimate and execution environment

Reuse `target_benchmark_sweep --dry-run` for validation and duration estimates;
it must not start a simulator or terminate processes. Use the same canonical
job/sweep parser as the CLI rather than recreating validation in browser code.

Before Start, show the renderer for the environment the child will actually
inherit, along with the recorded qualification status and covered conditions.
A GPU renderer alone does not establish
[environment qualification](../target_distance_benchmarking/running_benchmarks.md#qualified-execution-environments).
Keep unknown or unqualified conditions explicit. Apply that workflow gate to
tuning and result claims without inventing a second qualification policy.

### Conflicting runs

Extend the existing server-side single-run guard to identify genuinely
conflicting terminal-started runs and stale processes. Report the conflicting
process and resource, with an actionable resolution. The mere presence of a
`gz sim` process, especially a retained GUI, does not establish a conflict.
Recheck at launch so two near-simultaneous requests cannot bypass the guard.

### Start, progress, cancellation, and reconnection

Reuse existing persistent run records and process-group supervision. Locate the
actual created or resumed sweep directory from the supervisor's output; do not
recompute its timestamp. Read per-configuration progress and failures from
`sweep.json` and link to the existing results artifacts.

A tab close, browser restart, or configurator restart must not lose ownership
of the live sweep. Reattach only after checking process identity. Cancellation
must stop the owned live-run processes and finish recording their state, while
preserving unrelated processes. Removing the browser and server live-start
restrictions is the final step after these paths are verified.

## Acceptance checks

- Dry run returns validation and estimates without launching or killing anything.
- Preflight preserves the configurator and unrelated sentinel processes while
  removing only the identified stale benchmark processes; both broad install
  matches and simulator/node matches are covered.
- Real conflicts, including terminal-started runs, are identified before launch;
  unrelated simulator GUIs do not cause a blanket refusal.
- The displayed renderer describes the child environment, and qualification
  status is not inferred merely from a successful GL probe.
- Created and resumed sweeps reconnect to their actual output directory; progress
  and failures agree with `sweep.json`.
- Cancellation, tab close, browser restart, and server restart work during a
  representative live sweep, with no orphaned owned processes or stale PID attach.
- Existing offline start/cancel/reattach behavior still passes its focused checks.

Use focused process tests with harmless owned subprocesses before a live smoke.
Then validate the lifecycle in the qualified environment. Keep benchmark inputs,
measurement defaults, profile contracts, and scoring unchanged. Follow the
[installed-file build rules](../project/conventions.md#generated-state).
Record evidence, update the configurator reference and run guide, and close the
owning backlog item when the acceptance checks pass.

## Archived evidence

- [GUI execution decision](../../archive/engineering/benchmark_gui_direct_run.md)
