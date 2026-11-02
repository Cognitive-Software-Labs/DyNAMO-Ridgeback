# Unattended Gazebo and Isaac qualification campaign

The workstation-specific helper in
[`tools/benchmark/qualification/campaign.py`](../../tools/benchmark/qualification/campaign.py)
prepares and supervises the agreed two-repetition campaign. It reuses the existing
sweep owner rather than launching individual measurement nodes itself.

It is an execution and evidence-collection helper. A completed job does not
automatically close the [qualification backlog](BACKLOG.md).

## Fixed campaign

Four separate sweeps run sequentially: Gazebo repetition 1, Isaac repetition 1,
Gazebo repetition 2, Isaac repetition 2. Every sweep starts a new simulator and
executes box/stereo, silhouette/stereo, box/monocular and silhouette/monocular.
This gives 16 full benchmarks and two independent simulator sessions per condition.

Each condition selects all 88 full-benchmark scenes, with `repeats: 1` preserving
the packaged overrides: 109 trials. The furnished lab, 640×480 profile, current
model/estimator defaults, 2-second settling and 10-second wall-clock captures are
fixed. Projective ranging, Euclidean reconstruction and polar profiling run together;
pointcloud is excluded. GUI, RViz/overlays, video and diagnostics stay enabled.

Both repetitions record typed sensor parents so capture overhead and input
handling are matched between repetitions. Automatic replay uses the first
repetition's eight parents; the second set is retained as evidence. Every
simulator/condition receives its own capture directory. Repetition 2 never resumes
or reuses repetition 1. Automatic retries or replacement runs are disabled.

## Capture readiness is mandatory

The existing quota-based, stereoscopic-only typed recorder is insufficient for
this campaign. Both first-event context retention and timed monocular capture
must be established before a full run begins. The helper does not implement
those recorder changes.

Before the full matrix, two short capture-check configurations run in each
simulator: box/stereo for 4 seconds and box/monocular for 6 seconds, on `multi_04`.
The actual capture-window durations must follow those settings within 0.5 seconds.
Each must provide live measurement batches, correct depth-source identity,
640×480 dimensions and complete RGB/depth/calibration/transforms/scan for every
nonempty detection batch. Empty detections remain legitimate events; explicit
unused missing channels in those batches are counted rather than hidden.

The unequal durations distinguish timed capture from the old five-batch quota.
These checks are functional readiness checks, not additional full repetitions or
a replacement timing study. They run before spending time on the 16 full runs.

If the recorder fix introduces launch arguments, put only those established
arguments in
[`capture_arguments.yaml`](../../tools/benchmark/qualification/capture_arguments.yaml)
**before preparing the final bundle**. They are applied to capture-enabled
configurations. A pending bundle generated before that change must be regenerated;
do not alter a running campaign. With current recorder behavior the readiness
checks should fail; successful execution has not yet been demonstrated.

## Preparation and launch

From the workspace root, with ROS and the installed workspace sourced:

```bash
python3 tools/benchmark/qualification/campaign.py prepare \
  artifacts/benchmarks/qualification_prepared
python3 tools/benchmark/qualification/campaign.py check \
  artifacts/benchmarks/qualification_prepared
```

Preparation and `check` do not start a simulator, load a model or contact hardware.
`check` validates actual scene selection and trial expansion, not the sweep CLI's
historical fixed time estimate. The generated `inputs/` contains the four full
sweeps and two short capture-check sweeps. The prepared `REPORT.md` is an explicit
pending report.

After capture integration is verified and the workstation is available:

```bash
python3 tools/benchmark/qualification/campaign.py start \
  artifacts/benchmarks/qualification_prepared
```

This starts a transient user systemd service. No simulator starts during ordinary
preparation. `start` is the separate action that begins execution, including the
short readiness checks. It does not wait for another chat or start merely because
that chat finishes. The existing graphical session must remain logged in;
the launcher forwards its display context and uses `tools/gpu-run`. The campaign
process holds a logind automatic-idle inhibitor through the workstation's existing
`python3-dbus` package; this prevents automatic idle suspend, without changing
system power policy or blocking a deliberate manual suspend. The chat application can be
closed after launch. No OpenAI calls or periodic agent supervision are involved.

The service name and exact launch command are saved in `service.json`. To inspect
or stop it, use `systemctl --user status UNIT.service` or
`systemctl --user stop UNIT.service`, substituting the recorded unit. Stop requests
reach the campaign owner first, allowing ordered shutdown. The script never
invokes broad `cleanup.sh` or signals unrelated processes by name.

## Deadlines, ownership and failure behavior

The job budget is 12 hours minus the timing study's rounded 428 seconds. The full
live blocks share a maximum of nine hours, with a 2 h 15 min cap per block. A
configuration is admitted only with a 35-minute remaining allowance; its runtime
timeout is at most 40 minutes and cannot extend the block deadline. Three minutes
are reserved for final cleanup. These are execution limits, not scientific margins
or a guarantee that all work will finish.

The component timing projection did not establish sustained full-window raw
recording costs. Recording all 16 runs adds work; the nine-hour allocation is a
hard cap rather than a claim that the original 25% projection margin survives.
If observed costs do not fit, the campaign stops with the incomplete matrix
reported instead of changing capture settings between repetitions.

The helper holds a local campaign lock, checks for existing simulator/perception
jobs and active external CUDA work, uses a separate ROS domain/Gazebo partition,
and tracks only its descendants. Linux child-subreaper ownership retains SDK
helpers that create new sessions or outlive a launch leader. The original sweep
owner performs normal teardown; the outer deadline guard cleans up owned leftovers
and retains any escalation as a failure.

Before each condition and after its result, experimental file hashes are checked.
Sources, installed executable copies, scene/background inputs and model-cache
references are preserved; sources and installed entrypoints must agree. No rebuild
or model download is performed automatically. Source changes, a failed condition,
unexpected trial accounting, absent required replay inputs, a non-timed window,
missing warm model work, unplayable video or unclean shutdown stop the campaign.
Expected detector misses and estimator no-value outcomes are not infrastructure
failures.

## Automatic results

`REPORT.md` is regenerated on normal completion and failure, including a service
post-stop hook. `campaign.json` retains operation commands, exit status, elapsed
time, ownership and errors. Per-stage guard files contain run checks, captured
event counts and final model-stage diagnostic summaries. Original sweep manifests,
CSV rows, images, videos and ROS/console logs remain intact.

After all live blocks, eight first-repetition sensor parents are processed using
the corresponding box or real SlimSAM mask producer, then all three estimators
are evaluated with one numeric worker. Trial/instance CSV keys, outcomes, reasons,
counts, truth and estimates are compared with live results. The technical numeric
parity tolerance is 0.0001 absolute (0.1 mm for metre-valued columns), with zero
relative tolerance. This is separate from scientific repeatability margins.

Numeric evaluation is repeated against the same immutable parent/cache, requiring
exact CSV equality for the compared numeric/outcome columns. No second model
materialization is done merely for that determinism check. `comparisons.json`
summarizes the two independent live repetitions without inventing scientific
acceptance thresholds.

Trial-level parity is narrower than full per-event parity: the current saved live
reports do not prove every event's numeric output and association identical. Full
per-event comparison, declared repeatability margins, camera/LiDAR visibility and
occlusion auditing, and deliberate cancellation/pending-request qualification
remain explicit pending items. The helper never upgrades those claims to passed.
Three-replication remote/physical-seat equivalence is outside this two-repetition
campaign.

No full overnight campaign has yet been demonstrated. Preparation and software
tests establish the orchestration and rejection paths; capture readiness and
actual overnight outcomes must supply the remaining runtime evidence.
