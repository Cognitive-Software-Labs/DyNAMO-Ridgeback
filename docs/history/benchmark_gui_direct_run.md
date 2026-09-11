# Running benchmarks from the configurator GUI

Dated 2026-09-11, implemented on branch `feat/benchmark-gui` against `18eacb4`.
This records why the configurator stopped being export-only, what was measured
while building it, and what deliberately did not land.

Supersedes the "not a process supervisor" decision taken in the retired
benchmark-configuration-GUI plan. The current contract lives in
[target-distance benchmarking](../benchmarking/target_distance_benchmarking.md).

## Why the exported command had to go

The exported command did not work as emitted, for any profile, and no
browser-side fix existed.

`render_job` rendered `command_argv(f'./{filename}', job)` while the browser
downloaded the YAML to its own download directory. Every relative path *inside*
a job resolves against **the job file's own parent**, not the running shell:

```python
def _relative_to_job(job_path: Path, value: str) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (job_path.parent / path).resolve()
```

So a job in `~/Downloads` failed three ways: the `./` prefix required the
shell's working directory to be the download directory, relative inputs resolved
under `~/Downloads`, and a relative `output_dir` wrote results there.

`<a download>` has no directory control. `showSaveFilePicker` is available on
`127.0.0.1` because loopback is a secure context, but its handle exposes `.name`
and no path, so the page can move the file and still never tell the server where
it went. The write had to move server-side, and once the server owned the write
it may as well own the run.

A second, independent inconsistency: `inspect_path` resolved against the
**server process cwd** while execution resolved against the **job directory**,
so the GUI could report "109 trials, 545 events" for a path the run then could
not find. Both anchors are now the workspace root, via
`benchmarking/paths.py::anchored_path`.

## What landed

- **Server-side job write.** `artifacts/benchmark-jobs/benchmark-<profile>-<run
  id>.yaml`, absolute paths throughout, and the rendered command names that file.
  The timestamped name replaces a fixed per-profile one that silently overwrote
  the previous job.
- **Run records** under `artifacts/configurator/runs/<run_id>/` holding
  `run.json`, `run.log`, and a `returncode` file written by the child's wrapper
  shell. Runs outlive the GUI; the browser tab is a viewer that re-attaches from
  disk on load.
- **Start, cancel, and a live log tail** for `measurement`, `mask-output`, and
  `mask-model`. Those three route through one command and import no ROS runtime,
  so they cannot orphan a simulator.
- **Results browser and rename**, with per-kind rename safety.

## Two defects found by building it

**The recorded PID is the wrapper shell, and the wrapper dies first.** The run is
spawned as `bash -c '<command>; echo $? > returncode'` so the exit code survives
the GUI exiting. On cancellation the whole process group is signalled, and that
wrapper dies on the first SIGINT while the benchmark it started may ignore the
signal or take minutes to shut down. Escalation keyed on the wrapper's liveness
therefore stopped after SIGINT and left the real process running, unescalated.
Cancellation now watches the process **group** (`os.killpg(pgid, 0)`); PID-level,
cmdline-verified liveness is still what re-attach uses, because PIDs are reused
across a GUI restart and `os.kill(pid, 0)` would report a stranger's process as
the operator's run.

**`os.altsep` is `None` on Linux.** A rename-name guard written as
`(os.altsep or '') in candidate` tests `'' in candidate`, which is always true,
so every rename was refused. Both are covered by tests now.

## What did not land, and why

- **`live-system` Start (Phase 2 of the plan).** Blocked, not deferred:
  `cleanup.sh`'s catch-all `kill_matches "lib/ridgeback_autonomy/"` is a
  `pgrep -f` substring match, and the configurator's installed copy lives at
  `install/ridgeback_autonomy/lib/ridgeback_autonomy/target_benchmark_configurator`,
  so it matches. `target_benchmark_sweep` calls `run_preflight_cleanup` before
  Gazebo starts, so a GUI-launched sweep would `kill -9` the GUI mid-click. The
  UI states this reason next to a disabled Start rather than hiding the button.
  See [`docs/troubleshooting.md`](../troubleshooting.md).
- **Working around it with `--skip-preflight-cleanup`.** That flag produces the
  two-`gz sim` failure where every trial dies with "Target pose not present".
- **Concurrent runs.** Refused server-side, not merely disabled in the UI. Two
  offline replays are technically safe but contend for the worker counts the
  operator chose, which silently invalidates any timing read off them.
- **A progress bar for offline replay.** Offline replay publishes atomically at
  the end, so the only honest states are running / succeeded / failed plus the
  log. Live sweeps already write `sweep.json` after every config, which is what
  Phase 2 should poll.
- **Naming a run before it starts.** Rename-after-the-fact is the low-risk half.

## Rename safety is not uniform

Measured against the 118 directories in `artifacts/benchmarks/` on this branch.

| Target | Verdict | Why |
|---|---|---|
| Offline replay output | Allowed | Provenance records the job source path, its sha256 and git state — never its own location |
| Live trial output | Allowed | `run.label` is a generated timestamp already independent of the directory name; `smoke_post_cleanup/run.json` carries label `20260909_202703`, so divergence is an existing legal state |
| Typed artifact | Allowed, with a warning | Child lineage is `{artifact_id, manifest_sha256}`, content-addressed and path-free. But job YAMLs reference artifacts **by path**, so the warning names the saved jobs that would break |
| Sweep directory | Allowed, with a rewrite | `sweep.json` stores `configs[].arguments.output_dir` as the old absolute sweep path; rename rewrites every entry |
| Config subdirectory | Refused | Resume and reporting address it as `os.path.join(sweep_dir, config.name)` |
| Staging directory | Refused | `.<name>.partial-<pid>-<uuid>` is an interrupted writer's directory, not a result |

The sweep rewrite assigns the new path unconditionally rather than matching the
old one, because the stored value already went stale for pre-relocation sweeps:
`20260828_152011_baseline` still recorded
`/home/stefi/DyNAMO/DyNAMO-Ridgeback/benchmark-results/20260828_152011_baseline`
after the tree moved to `artifacts/benchmarks/`. Reassigning repairs those too.

The browser lists top-level entries and descends one level into directories that
are neither, because operators group sweeps (`smoke/20260828_144853_smoke`).
Config subdirectories are not listed — a sweep's configs are its own business —
so the refusal above is reachable only through the API.

## Verification

- 815 tests pass from the repo root with `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash` sourced, up from 789; 26 of the additions cover this work.
- End-to-end in the real UI: a `measurement` job over `v1_dataset` (5 trials, 25
  events) supplied with **relative** input and output paths started, settled
  `succeeded` with exit code 0, and wrote `results/`. Re-running it was refused
  with a field error before anything spawned. Renaming its output through the
  browser worked, and the `v1_dataset` row correctly warned that one saved job
  referenced the old path.
- Cancellation escalation is covered by a test whose child installs `trap '' INT`
  — `SIG_IGN` is inherited, so the run can only end once cancellation escalates
  past SIGINT. A live cancel was not observed: every offline replay small enough
  to drive by hand finished before the signal landed.
