# The local benchmark configurator

`target_benchmark_configurator` is a local-only browser UI for building a
benchmark job and running it. The root README owns the command and the operator
walkthrough; this page owns the contract. Start at
[running benchmarks](running_benchmarks.md) to decide whether the GUI is the
right way in at all.

## Surface

It binds only to `127.0.0.1`, chooses an available port by default, and puts an
unguessable token in the URL and every API request. Use `--no-open` to print the
URL for a headless or remote session.

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

## Authoring a live sweep

A `live-system` job may either name an existing sweep YAML or author one. The
sweep-path field decides: name a file and that file runs exactly as written,
leave it blank and the variant cards below author a sweep instead. A small
defaults block above them holds the two axes a live sweep states once rather
than per config — `scenario` and `repeats` — because a sweep authored without
them silently inherits the packaged 88-scene set at five repeats. Browser inputs
yield text, so the authored document is typed before it is written; the file
sits beside hand-written sweeps and has to read like them.

Authoring changes the job contract: `parse_job` accepts an inline sweep document
for `live-system`, where it previously demanded a path. The inline form exists so
the page can validate on every keystroke without writing files. Rendering
materializes it through `configurator_runs.write_sweep_file` to
`artifacts/benchmark-jobs/sweep-<name>-<stamp>.yaml`, and the job then names that
written path, so a saved job still resolves itself and `target_benchmark_sweep`
still opens a file. A live job whose sweep was never written to disk cannot
produce a command at all, rather than emitting one that names nothing.

## Running from the page

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

The Results panel lists typed artifacts and run outputs and can rename them;
[outputs](outputs.md) owns the rename rules and their refusals.

## Estimator reachability

The browser is a client of the ROS-free replay-profile capability contract;
the service is the authority for profile and axis decisions. The GUI validates
canonical replay jobs and their typed artifacts before running, so it cannot
misrepresent a live or box-gated run as a frozen-mask experiment.

Each profile offers exactly the estimators it can evaluate: every offline
profile offers the three mask rows, and `live-system` adds the point cloud. The
estimator axis itself names every estimator the registry publishes, and the
rendered choices narrow to the profile's own set — narrowing the axis instead
would refuse a hand-written job naming a supported estimator, not merely omit it
from a menu. Selecting an estimator a profile cannot evaluate reports
`incompatible_estimator`, naming the estimator and the nearest profile that does
run it, rather than the type failure it used to report.

The menu narrows a second time once evidence is named, because two files of the
same kind differ in what they can feed. The page reads
`unavailable_estimators` off the validation response — not a separate request,
which could narrow the menu for a different file than the one being validated —
and states every narrowing beside the evidence line: *"Polar Profiling
unavailable: this sensor capture is payload version 1, written before the LiDAR
scan was recorded."* An estimator's settings disappear with it, so a polar band
is never offered where it would not change a number.

A draft that already names a refused estimator keeps showing it. Removing it
silently is exactly what the refusal exists to prevent, so the value stays, its
knobs go, and the job is refused with the reason before the run rather than
after every trial has loaded.

## Validation status

The combined installed-package suite passed with 758 tests after the
configurator landed, as recorded in
[layered replay implementation validation](../history/layered_replay_implementation_validation.md).
That record contains no canonical browser screenshot or visual-acceptance
artifact; the functional contract is verified, while visual proof is not
claimed.
