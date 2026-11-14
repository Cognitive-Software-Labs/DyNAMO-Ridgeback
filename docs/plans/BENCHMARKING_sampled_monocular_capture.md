# BENCHMARKING — Sampled monocular capture for the overnight qualification

Status: **implemented; readiness probes passed in both simulators** (2026-11-03).
The recorder now saves the scan actually used live, resolving the Isaac polar
parity failure. Full live qualification and the deferred offline phase remain. The
[helper reference](../target_distance_benchmarking/unattended_qualification.md#readiness-status)
records the results. Launch remains with the campaign owner.
Prepared on **2026-11-03, Europe/Berlin**, against `a6c255a` on `dev`.
Recheck HEAD, installed copies and workstation ownership before implementing.

## Outcome and latest user decisions

Make tonight's live qualification supply reusable stereo and monocular inputs,
so the following offline session needs no additional simulation recordings.
Complete sensor recordings are unnecessary. Preserve the agreed live experiment:

- 16 full benchmarks: four mask/depth conditions × two simulators × two
  independent repetitions, ordered `gz_r1`, `isaac_r1`, `gz_r2`, `isaac_r2`.
- Each block has box/stereo, silhouette/stereo, box/monocular and
  silhouette/monocular in one persistent simulator session. Each repetition
  starts a fresh session; no automatic retries or replacement runs.
- All 88 furnished-lab scenes and their existing repeat overrides: 109 trials
  per condition. Keep background placement and all model/estimator settings.
- Existing readiness/warmup, 2-second settling, **10-second wall-clock live
  measurement windows**, 640×480 camera, default OWLv2/SlimSAM/Depth Anything
  settings, and projective ranging, Euclidean reconstruction and polar profiling.
- Visualization, overlays/RViz, video and diagnostics remain on. No pointcloud
  estimator, new detector, model tuning or background changes.
- Save a bounded sample in **every trial of every condition**, using the existing
  five-batch default. Sample both repetitions to match their recording workload;
  use the eight first-repetition datasets for the initial offline checks/sweeps.
  Retain repetition 2 as evidence. Do not select only successful detections.
- Defer the full offline processing phase to the following session. Small
  readiness/parity probes before launch are necessary to establish that the
  saved inputs will be usable.

This supersedes the campaign's earlier requirement to record every event of
every 10-second window. It does **not** shorten the live windows to five batches.
It qualifies estimator replay using frozen predicted monocular depth; it does
not evaluate alternative monocular models or prove a second inference identical.

## Read first and coordinate

Read [project context](../project/PROJECT_CONTEXT.md),
[conventions](../project/conventions.md),
[documentation ownership](../project/documentation.md), the
[current helper reference](../target_distance_benchmarking/unattended_qualification.md)
and [layered replay profiles](../target_distance_benchmarking/profiles.md).
The helper reference describes the existing implementation; this plan owns the
proposed change until it is implemented.

You are not alone in the checkout. Preserve other agents' changes and the
locally excluded thesis-priorities plan. Coordinate edits to the qualification
helper with the campaign owner. Keep one workstation experiment at a time.
Do not launch a competing overnight campaign, change host power policy, touch
hardware, clean unrelated processes, or commit/push unrelated files.

## Verified starting points

- `37331c7` fixed retention of RGB/camera-info/scan context arriving before a
  capture window. Keep that fix. Its saved two-scene captures have ten complete
  events per backend in
  `artifacts/benchmarks/capture_first_event_fix_20261102/`.
- `a6c255a` added the helper under `tools/benchmark/qualification/`.
  It currently enables sensor capture in all 16 conditions, requires timed
  capture checks, then automatically performs offline processing.
- The runner rejects `sensor_capture_dir` with `depth_source:=monocular`.
  Its recording branch stops on `capture_batches`, ignores `capture_sec`, and
  restricts the live events to the recorded stamps. Removing only the monocular
  rejection would therefore change the live experiment and still lose its
  source-range contract.
- The mask node already publishes the exact aligned depth used for its live
  measurements through its debug-depth publisher, including monocular output.
  Live monocular measurement itself is already implemented.
- Recorded events currently set `depth_usable_max_m` to `None` for stereo.
  The monocular source derives a finite usable ceiling from the **loaded**
  checkpoint. Layered numeric replay already consumes this event field.
- The existing helper compares replay CSVs with the full live CSVs. That is
  valid only for identical event populations; bounded sampling requires a
  separate live reference for the sampled events.

## Implementation sequence

### 1. Separate live measurement duration from the saved sample

Add an explicit opt-in sampled-window mode for typed sensor capture. Reuse
`capture_batches` as the storage bound and `capture_sec` as the live measurement
duration; a new name such as `sensor_capture_mode:=sampled_window` is acceptable.
Preserve existing quota-mode defaults and legacy `replay_dataset_dir` behavior.

For sampled-window mode:

1. Open the normal wall-clock measurement window and admit the first five unique
   raw detection batches delivered during it, including empty batches.
2. Once the sample quota is reached, continue live measurement for the full
   duration. Stop adding raw batches to storage, not to live scoring. Keep
   buffering only the context needed by the selected sample.
3. At the measurement deadline, freeze the live event population and the sampled
   batch identities. Use a separate bounded drain for pending sample depth,
   context and live results. Drain callbacks may complete already admitted
   batches; they must not add new scored batches or extend the measurement
   window. Do not move the scene before that drain finishes.
4. Preserve the full-window live results for repeatability. **Do not apply** the
   current `restrict_events_to_stamps` operation to the main live results in
   sampled-window mode. Create a separate sampled live-event collection.
5. Report measurement duration, drain duration, full live batch count, requested
   sample count, actual sample count and exact sampled stamps. If five batches
   were not available, preserve the actual count and report insufficient sample
   coverage; never extend the live window or manufacture batches to fill it.

Update quota-dependent completion predicates and buffer bounds as well as the
main loop. Retain exact RGB/depth stamps, current camera-info matching rules and
the existing bounded nearest-scan policy. Do not replace missing inputs with
another frame. Keep empty detections and explicit missing channels visible.

This is bounded sampling during an existing timed trial, not complete timed raw
recording. Export measurement timing separately from total method/drain time.
The helper's observer currently reads `active_trial_id` after the runner clears
it: capture the identity **before** invoking the method and test that lifecycle.

### 2. Preserve the monocular depth contract

Enable the new mode for both depth sources and both masks. Store the exact
float32 metre-valued depth grid used by the live mask node, together with its
matching RGB, raw detections, calibration, camera transforms, scan/scan transform,
trial identity and background provenance. Reuse the existing typed writer and
aligned-depth publication; do not run another Depth Anything inference in the
recorder or reconstruct depth from videos.

Deliver the live source identity and usable ceiling to the recorder through a
small, explicit metadata contract. Inspect existing transport before selecting
the narrowest change. A new metadata publication or interface field is acceptable
if needed, but it must identify the source/model revision and correspond to the
depth that was actually produced. Preserve the model's declared maximum and the
policy deriving the usable ceiling in provenance. Do not hardcode a guessed
ceiling, infer stereo from missing metadata, or load a second model to obtain it.

For each usable monocular depth frame, record the finite usable ceiling obtained
from the loaded source. Preserve stereo's existing semantics. Before model
availability, retain explicit missing-depth/unknown metadata and matching live
outcomes; do not label them successful complete frames. Ensure numeric replay
uses the saved source ceiling together with the configured working depth gate,
as the live path does. Frozen monocular depth replay must not load Depth Anything.

### 3. Save a matching live reference for the sample

Persist the actual live measurement records for every sampled stamp, including
detection identity/order, per-estimator forward/lateral/distance values, statuses
and miss reasons. Preserve raw detections separately from post-mask outcomes.
Use the existing artifact/event serialization where possible, with hashes and
documented lineage. Keep existing artifact readers compatible.

Build a separate sampled trial/instance reference using the existing scoring
and association functions on **only those live records**. Keep the full-window
CSV unchanged. Record the sampled stamp list explicitly so offline results can
be checked for event identity, batch/detection counts, outcomes and association
before comparing numerical values. Do not derive the live reference by rerunning
the offline estimator; that would not be independent parity evidence.

For the initial matching-settings check, use the helper's existing technical
absolute tolerance of `1e-4`, relative tolerance zero, and exact identity/status
checks. This is not a scientific repeatability margin. Repeated numeric replay
of the same immutable depth/mask cache should retain its existing exact check.

### 4. Integrate with the unattended campaign

- Set sampled-window mode and the same five-batch limit in all 16 conditions.
  Keep separate capture directories and all 109 trial entries per dataset.
- Adapt guards to check 10-second **measurement** duration independently of
  sample count and drain time. Require usable source metadata, sample coverage,
  required inputs and an independent sampled live reference. Preserve the
  existing accounting, detector identity, background, video, diagnostics,
  rendering and shutdown checks.
- Reuse the short 4-second stereo / 6-second monocular checks in both simulators.
  Exercise silhouette+monocular in the same short sessions as well. A saved
  sample should remain bounded as live measurement continues beyond the quota.
- Split live collection from offline execution through an explicit phase/action.
  Overnight execution stops after the four live blocks with a truthful state
  such as `live-complete-offline-pending`, an input inventory and next command.
  A later offline-only invocation consumes those same immutable artifacts;
  it must not start a simulator or repeat a live condition.
- Tomorrow's parity compares against the **sampled live reference**, not the
  full-window CSV. Use actual box/SlimSAM materialization as configured and then
  frozen-depth numeric replay. Retain full-window CSVs for live repeatability.
- Retain ownership, fail-fast behavior, file freezing, service shutdown and
  cleanup. Budget the short checks and the later offline phase within the same
  12-hour workstation allocation; 428 seconds were already used by the original
  timing study. Persist budget/deadline accounting across phases. Do not silently
  give the later invocation another 12 hours or promise a completion time.
- Regenerate a fresh pending bundle after integration. Keep the old
  `artifacts/benchmarks/qualification_prepared_20261102/` as preparation evidence.
  Freeze implementation and inputs before launch; offline work must not edit
  the implementation underneath an active live campaign.

## Likely code ownership

| Area | Starting files / symbols |
|---|---|
| Runner, recording and event membership | `src/ridgeback_autonomy/ridgeback_autonomy/benchmarking/target_distance_benchmark_runner_node.py`: initialization, `on_replay_raw_detections`, `capture_measurement_window`, `replay_capture_is_complete`, `write_replay_trial` |
| Live depth and source metadata | `src/ridgeback_localization/ridgeback_localization/mask_measurement_node.py`: `depth_for_batch`, `effective_depth_max`, `publish_aligned_depth_debug`; `core/depth_sources.py` |
| Launch/schema forwarding | `src/ridgeback_autonomy/launch/target_benchmark_config.launch.py`, public benchmark launch and the actual sweep argument allowlist under `benchmarking/` |
| Artifact/reference compatibility | `benchmarking/replay_artifacts.py`, `layered_replay.py`, `replay.py`, `trial_results.py`; inspect existing serializers before adding a new format |
| Unattended integration | `tools/benchmark/qualification/{campaign.py,evidence.py,sweep_entry.py,capture_arguments.yaml}`, `observer/sitecustomize.py`, four sweep YAMLs |
| Focused verification | `src/ridgeback_autonomy/test/test_layered_replay.py`, relevant localization tests and `tools/benchmark/qualification/test_campaign.py` |
| Implemented documentation | `docs/target_distance_benchmarking/unattended_qualification.md`, replay profile reference and root README where public arguments/commands change |

Paths abbreviated with `benchmarking/` refer to
`src/ridgeback_autonomy/ridgeback_autonomy/benchmarking/`. Change interfaces only
when the source-metadata contract requires it; rebuild their dependents then.

## Verification and acceptance

1. Add focused regressions proving that reaching five samples does not end a
   10-second live window, later live batches remain in its scores, and drain
   completions cannot enlarge either population. Test shortfalls, empty batches,
   delayed depth/context, exact identities, correct observer trial IDs and
   unchanged legacy quota behavior.
2. Verify monocular depth and its finite source ceiling survive artifact
   round-trip and affect replay's range filtering exactly as live. Cover both
   mask paths, explicit missing evidence and existing stereo/old-artifact cases.
3. Test that the sampled live reference contains only stored stamps, differs
   appropriately from a deliberately larger full-window result, and detects
   altered estimates/statuses/association. Test deferred offline execution uses
   existing artifacts, preserves budget and cannot spawn a live stage.
4. Run focused suites, including `test_layered_replay.py` and the helper tests.
   Run `tools/check_dependencies` before building. Rebuild changed packages and
   any interface dependents; verify the **installed executable copies**, not
   just symlinked Python modules. Use writable logs under `artifacts/`.
5. On an available workstation, run the bounded checks in both Gazebo and Isaac
   with the campaign's models/background/display settings. Check actual stored
   arrays and metadata; run matching box and real-SlimSAM replay on the tiny
   samples, including monocular. Confirm zero skipped trials, clean teardown and
   no owned leftovers. Measure actual capture bytes and overhead during these
   probes; do not launch a full benchmark to estimate them.
6. Run documentation-link checks, `git diff --check` and the required
   `bash tools/rebuild_graphify` after code changes. Update the helper reference
   to the demonstrated behavior, retaining remaining scientific qualification
   limits and separate Gazebo/Isaac verdicts.

Before campaign launch, deliver the implementation diff, exact commands/results,
small capture artifact paths/hashes, per-backend parity and shutdown results,
measured probe costs, and the regenerated bundle's live/offline commands. Report
any failed or inconclusive gate explicitly; do not weaken a guard to get a pass.

Stop at launch readiness and hand back to the campaign owner. This task is not
the parameter-tuning campaign, a claim that all scientific qualification gates
have passed, or the broader three-replication remote/physical-seat comparison.
