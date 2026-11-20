# Unattended qualification and Isaac repeatability

The workstation-specific helper in
[`tools/benchmark/qualification/campaign.py`](../../tools/benchmark/qualification/campaign.py)
prepares and supervises the agreed two-repetition campaign. It reuses the existing
sweep owner rather than launching individual measurement nodes itself.

It is an execution and evidence-collection helper. A completed job does not
automatically close the [qualification backlog](BACKLOG.md).

## Isaac-only repeatability

`prepare-isaac BUNDLE` prepares two fresh Isaac sessions and only the Isaac
readiness check. No Gazebo check, run or imported result is included. Each
session runs all four mask/depth conditions and the full 109-trial scenario
expansion. It uses deterministic PathTracing, AA 0, ideal depth and the same
fixed 10-second windows, five-batch samples, model defaults and recording
workload in both repetitions. The existing deterministic render policy and
[monocular startup gate](../target_localization/aligned_depth.md) apply.

```bash
python3 tools/benchmark/qualification/campaign.py prepare-isaac BUNDLE
python3 tools/benchmark/qualification/campaign.py check BUNDLE
```

These commands prepare and validate inputs without launching anything. The
matrix check rejects different settings between repetitions and imports are
disabled. Preparation records source, installed-entrypoint, input and model-ref
hashes in `prepared_files.json`; launch preflight rejects drift instead of
silently updating the tested implementation. Prepare a fresh reviewed bundle
after an intentional change. Both condition guards require sampled live records;
monocular captures additionally require successful producer readiness on a
camera frame preceding every sampled capture timestamp. This checks startup
independently of whether a target was detected.

`comparisons.json` preserves each matched trial/instance pair, both outcomes and
miss reasons, both estimates and absolute/relative deltas. `REPORT.md` gives
median and maximum deltas. Matching misses have no invented numerical delta.
Compare scene and target identities within each mask/depth/estimator condition;
do not pair neighbouring rows or different targets. The optional offline phase
replays the four first-session condition samples and starts no simulator; it is
separate from the live repeatability check.

### Receiving-agent heartbeat

The execution helper does not itself wake a chat. Before launching an Isaac-only
bundle, the receiving agent must register a periodic heartbeat in its own chat,
verify delivery and record `enabled: true`, `delivery_verified: true`, the real
`automation_id` and `target_chat_id` in `monitor_registration.json`. Preparation
leaves these fields disabled and empty. Both `start` and live preflight reject
an unregistered heartbeat. A file containing an ID is a registration receipt,
not proof of delivery; the agent must perform that check through its automation
provider. If the provider is unavailable, keep the job prepared.

The heartbeat runs the read-only health check every two minutes:

```bash
python3 tools/benchmark/qualification/repeatability_watch.py BUNDLE
```

It returns JSON and exits nonzero for campaign/guard failures, an inactive owned
unit, a stale host sampler, unreadable comparisons, or more than five minutes
without a completed capture window. Diagnostic log chatter does not reset that
progress clock. Each stage gets a fresh startup allowance. Outcome/miss-reason
changes and distance deltas above 20 mm or 1% request agent review. Those are
operational alert triggers, not agreed scientific equivalence margins; all
deltas remain in the report.

The agent stays quiet while healthy state is unchanged, wakes on a new failure,
stall or review condition, and reports completion once. On a problem it preserves
the bundle and inspects the guard, stage log, service result and host samples;
it stops only the recorded owned unit when necessary. No automatic restart,
replacement run, estimator change or cross-user process cleanup is permitted.
Deactivate the heartbeat after handling the terminal result. The receiving
agent's handoff supplies the exact bundle and heartbeat prompt.

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

Every condition of both repetitions stores a bounded sample of its live
windows, so capture overhead and input handling are matched between
repetitions. The later offline phase replays the first repetition's eight
samples; the second set is retained as evidence. Every simulator/condition
receives its own capture directory. Repetition 2 never resumes or reuses
repetition 1. Automatic retries or replacement runs are disabled.

## Sampled-window capture

[`capture_arguments.yaml`](../../tools/benchmark/qualification/capture_arguments.yaml)
applies `sensor_capture_mode: sampled_window` and `capture_batches: 5` to every
configuration, checks included. Each trial keeps its 10-second wall-clock live
window and scores it exactly as an uncaptured run would. The runner stores only
the first five raw batches delivered inside that window, empty ones included,
with their exact RGB, float32 depth, calibration, transforms and scan; a bounded
drain after the deadline may complete only those batches. The
[replay profile reference](profiles.md#sampled-window-capture) owns the recorder
contract, including the sampled live reference written beside each run's
full-window CSVs and the depth-source record that freezes monocular depth with
the usable ceiling the live node declared.

A batch the live mask node never measured has no live record and keeps its
depth explicitly absent. The node keeps only its newest pending batch. The
monocular readiness gate moves cold initialization before capture; it cannot
recover any later replaced work. Legacy qualification evidence retains those
gaps as inconclusive replay trials. The Isaac-only repeatability guards reject
unmeasured sampled batches rather than accepting that legacy population.

## Readiness checks

Before the full matrix, three short configurations run in each simulator on
`multi_04`: box/stereo for 4 seconds, box/monocular and silhouette/monocular for
6 seconds. A simulator whose full-run conditions are all imported has nothing
left for its check to guard, so that check is skipped and recorded in
`skipped_checks`. Each check must show:

- a measurement window within 0.5 seconds of its setting, timed apart from the
  sample drain, and more live raw batches than the five it stored;
- the full five-batch sample, the expected depth source, and 640×480 grids;
- complete RGB, calibration, transforms and scan for every non-empty sampled
  batch, and depth wherever the live node produced it, each frame carrying the
  live source's record (a finite ceiling for monocular, none for stereo);
- a hashed sampled live reference whose stamps equal the stored events and
  whose live records measured the stored boxes.

Empty detections remain legitimate events; unused missing channels in them are
counted rather than hidden. The full runs apply the same guards except the
beyond-the-sample comparison. These are functional readiness checks, not
additional full repetitions or a replacement timing study.

## Preparation, probe and launch

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

A readiness probe runs on its own separately prepared bundle. It executes only
the two check sweeps and then replays their samples offline exactly as the later
offline phase would, so it starts simulators but never a full block. The probe
bundle ends as `probe-complete-not-a-campaign` and can never be launched:

```bash
tools/gpu-run /usr/bin/python3 tools/benchmark/qualification/campaign.py probe \
  artifacts/benchmarks/qualification_probe
```

After a successful probe and with the workstation available, launch the live
phase from a freshly prepared bundle:

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

### Imported conditions and shared-GPU operation

Two optional, pre-launch changes to a prepared bundle are recorded in
`campaign.json` and `REPORT.md`:

```bash
python3 tools/benchmark/qualification/campaign.py import BUNDLE \
  --from EARLIER_BUNDLE --stage gz_r1 --conditions box_stereo,silhouette_stereo
python3 tools/benchmark/qualification/campaign.py allow-external-gpu BUNDLE
```

`import` adopts completed full-run conditions of an earlier bundle instead of
rerunning them. The earlier stage must have recorded the condition as successful
and validated; it is revalidated with the current evidence checks, its
experimental settings must equal this bundle's apart from output paths, and its
run, CSV, sampled-reference and capture hashes are recorded. Inventory refuses
the campaign's evidence if any of them later changes. A stage's remaining
conditions run in their own fresh simulator session, so an imported condition
does not share a session with them; a stage with every condition imported is
skipped and its input is kept under `inputs_imported/`. Workstation
time of the earlier run is charged through the carried phase accounting, not
again.

`allow-external-gpu` changes only the external CUDA rule: another user's CUDA
process above 5% SM is recorded at every stage boundary rather than stopping the
campaign. Another simulator/perception job and low disk still stop it. Whatever
the policy, the live phase samples GPU users and memory every 10 seconds into
`host_samples.jsonl`, and the report states for each stage how many samples saw
external GPU work and the lowest available memory. The sampler records memory;
it does not stop the campaign under memory pressure.

## Deadlines, ownership and failure behavior

The job budget is 12 hours minus the timing study's rounded 428 seconds. The full
live blocks share a maximum of nine hours, divided equally: 2 h 15 min per block
for the four-block campaign and 4 h 30 min for the two-block Isaac repeatability
campaign. In the four-block campaign a configuration is admitted only with a
35-minute remaining allowance and its runtime timeout is at most 40 minutes. Isaac
repeatability uses 45 and 55 minutes. Deterministic PathTracing trials settle at
about 14.3 seconds, near the 13.4 of realtime mode, so those two limits are
headroom rather than a measured requirement. Neither timeout can extend the block
deadline. A readiness stage has a 15-minute cap and admits a
configuration with a 2-minute allowance, leaving room for a cold simulator start
before the first condition. Every stage stops admitting 90 seconds before its
deadline, and three minutes are reserved for final cleanup. These are execution
limits, not scientific margins or a guarantee that all work will finish.

The component timing projection did not establish sustained full-window raw
recording costs. Recording all 16 runs adds work; the nine-hour allocation is a
hard cap rather than a claim that the original 25% projection margin survives.
If observed costs do not fit, the campaign stops with the incomplete matrix
reported instead of changing capture settings between repetitions.

The helper holds a local campaign lock, checks for existing simulator/perception
jobs and active external CUDA work (stopping or recording, as above), uses a
separate ROS domain/Gazebo partition, and tracks only its descendants. Linux child-subreaper ownership retains SDK
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

## Live and offline phases

The live phase (`start`, or `run` in the foreground) runs the checks and the
four blocks and stops. On success its status is `live-complete-offline-pending`;
`offline_inputs.json` inventories every sample by artifact ID, manifest hash,
sampled-reference hash and size, and `REPORT.md` names the next command:

```bash
python3 tools/benchmark/qualification/campaign.py start-offline \
  artifacts/benchmarks/qualification_prepared
```

`start-offline` (or `offline` in the foreground) accepts only that status. It
verifies that the implementation, inputs and inventoried evidence are unchanged
and then runs only mask materialization and numeric replay; it never starts a
simulator or repeats a live condition. Each phase's elapsed workstation time is
persisted, and the offline phase receives only what remains of the same
allocation, never a fresh one. Neither phase can be run twice on a bundle.

## Automatic results

`REPORT.md` is regenerated on normal completion and failure, including a service
post-stop hook. `campaign.json` retains operation commands, exit status, elapsed
time per phase, remaining budget, ownership and errors. Per-stage guard files
contain run checks, window and sample counts, measurement and drain durations
and final model-stage diagnostic summaries. Original sweep manifests, full-window
CSV rows, sampled references, images, videos and ROS/console logs remain intact.

The offline phase processes the eight first-repetition samples using the
corresponding box or real SlimSAM mask producer, then evaluates all three
estimators with one numeric worker over the frozen depth. Event identity is
checked first: stored stamps against the sampled stamps, payload hashes, and
each live record against its raw boxes. Two numeric comparisons follow:

- **Per event:** every sampled batch the live node measured is replayed with the
  exact variant arguments of the numeric replay and compared, box by box, with
  its own live record: every estimator's status, distance, forward and lateral.
  Batches without a live record are counted, not compared. Polar differences on
  an event whose live row used another scan than the frozen one are counted
  separately as attribution; they still fail.
- **Per trial:** trial/instance CSV keys, outcomes, reasons, counts, truth and
  estimates are compared with the **sampled live reference**, over trials whose
  every sampled batch has a live record; the others are listed. The full-window
  CSVs are kept for repeatability and never used for parity.

A dataset with nothing compared is reported inconclusive, never passed. The
technical numeric parity tolerance is 0.0001 absolute (0.1 mm for metre-valued
columns), with zero relative tolerance. This is separate from scientific
repeatability margins. Any mismatch stops the offline phase.

Numeric evaluation is repeated against the same immutable parent/cache, requiring
exact CSV equality for the compared numeric/outcome columns. No second model
materialization is done merely for that determinism check. `comparisons.json`
summarizes the two independent live repetitions without inventing scientific
acceptance thresholds.

Parity covers the stored sample only, not every batch of the live window.
Declared repeatability margins, camera/LiDAR visibility and occlusion auditing,
and deliberate cancellation/pending-request qualification remain explicit
pending items. The
helper never upgrades those claims to passed. Frozen-depth replay qualifies the
estimators over predicted monocular depth; it does not evaluate another
monocular model or show that a second inference would be identical.
Three-replication remote/physical-seat equivalence is outside this two-repetition
campaign.

## Readiness status

The 2026-11-03 probe at
`artifacts/benchmarks/sampled_capture_probe_20261103_live_scan_fix/` passed with
status `probe-complete-not-a-campaign` and no errors. Sampled capture now
retains the exact scan selected live, including when a closer scan arrives later.

- **Gazebo: ready for live collection.** All three short-condition guards,
  per-event replay comparisons and fixed-input numeric checks passed. Twelve of
  fifteen stored batches had live results; the three unmeasured box/monocular
  batches remain explicit, and that trial's aggregate parity is inconclusive.
- **Isaac: ready for live collection.** All three short-condition guards,
  per-event replay comparisons and fixed-input numeric checks passed. All
  fifteen stored batches had live results, including matching polar profiling.

Across both simulators, the largest compared coordinate difference was below
5×10⁻⁷ m; the existing 0.0001 tolerance was unchanged. Measurement windows were
within 0.6 ms of their settings. Both simulator sessions shut down cleanly,
without outer escalation or owned leftovers. The complete probe took 176.6 s.

These checks establish capture mechanics and replay on one scene per condition.
The full live runs and deferred offline phase must supply the remaining evidence;
readiness is not a completed scientific qualification verdict. Charge measured
preparation probes to the launch bundle's prior-phase budget so they do not
receive another allocation on launch.

## Archived evidence

- [Full-matrix Isaac repeatability pair, 2026-11-06](../../archive/engineering/2026-11-06-isaac-full-repeatability.md)
- [Sampled capture and live-scan parity, 2026-11-03](../../archive/engineering/sampled_capture_live_scan_parity_20261103.md)
- [Qualification campaign stopped by systemd-oomd, 2026-11-03](../../archive/engineering/2026-11-03-qualification-oom.md)
