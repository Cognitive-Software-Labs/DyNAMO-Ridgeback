# BENCHMARKING — Isaac repeatability: repair, run and recover (Codex)

Status: complete on 2026-11-06. Live collection finished with no execution error; results are in the full-matrix record under Archived evidence. The text below is the original instruction, kept for reference.

Updated by Codex on 2026-11-06, Europe/Berlin. Preparation base revision:
`cc86eef5a5c708be2a31aac64db29a91225f4023`, plus dirty changes preserved in
the evidence bundles. Recheck the current checkout before changing anything.

## Instruction to the receiving agent

The user explicitly authorizes you to **fix the readiness-budget problem, start
a fresh Isaac-only repeatability run, and, if something goes wrong, diagnose it,
fix it and restart using a fresh bundle**. Carry this through to completed live
collection and a repeatability report. Do not stop merely to propose the known
fix or ask again for launch/restart permission. This authorization supersedes
older preparation-only and no-replacement instructions in the previous handoff,
failed bundle, heartbeat prompt and terminal alert. It does not authorize blind
retries: establish the cause and validate each repair before restarting.

This document is a handoff; its author has not launched a replacement. If a new
run is already active when you take over, adopt its supervision rather than
launching a duplicate. Never modify source/configuration while your experimental
run is active: stop its exact owned unit and preserve output before a repair.

Workspace: `/home/stefi/DyNAMO/DyNAMO-Ridgeback`.
Do **only Isaac**. Gazebo is accepted and must not be rerun or imported. No
hardware, detector comparison, parameter tuning or unrelated cleanup is in scope.
The optional offline replay phase is not part of this live repeatability task.

## Known failure and evidence

Failed attempt:
`artifacts/benchmarks/isaac_repeatability_prepared_20261105T210614Z/`.
Start with `FAILURE_DIAGNOSIS.md`, `campaign.json`, `check_isaac.log`,
`check_isaac_guard.json`, `failure_service_journal.log` and `host_samples.jsonl`.

The readiness sweep started on 2026-11-05 at 23:50 Europe/Berlin and stopped
at 23:53 after 136.5 seconds. No full `isaac_r1` or `isaac_r2` run began.
Both box/stereo and box/monocular checks passed. Each stored five batches with
complete required inputs and no missing live measurements. The monocular
warmup gate passed. Silhouette/monocular was never launched: its manifest had
already been marked running before command construction refused admission.

Verified mechanism:

- `tools/benchmark/qualification/campaign.py::run_stages` gives a readiness
  stage a 240-second cap, then passes `stage_end - 90` as
  `QUALIFICATION_STOP_AT` to `sweep_entry.py`.
- `tools/benchmark/qualification/sweep_entry.py` requires another 30 seconds
  at configuration admission. Thus all three checks must start before roughly
  120 seconds, including simulator startup, RTF sampling, scene work, recording
  and previous condition shutdown. The third admission arrived too late.
- The log says `Insufficient remaining budget to admit another configuration`.
  This was not a sensor-result validation failure. Shutdown was clean, with no
  escalation or owned leftovers. Fourteen host samples showed no external-busy
  CUDA samples and at least about 42.25 GiB available RAM.

At terminal handling the campaign unit was failed with MainPID=0 and both
monitor units were inactive. Recheck rather than assuming these remain true:

- Campaign: `ridgeback-qualification-isaac-repeatability-prepared-20261105T210614Z.service`
- Monitor: `isaac-repeatability-monitor-20261105t210614z.timer` and matching `.service`

Keep this failed bundle intact. Never change its frozen inputs or overwrite its
logs/status to make it reusable.

## Repair and validate

Use the existing campaign helper; do not build a parallel supervisor. Correct
its readiness stage/admission budget so the entire three-configuration check
has a realistic startup, model-load, capture and cleanup allowance. A bounded
10-minute readiness-stage cap is a reasonable candidate; verify its interaction
with the 90-second reserve, 30-second admission allowance and overall deadline.
Keep a real bound and cleanup reserve. Do not change measurement durations,
recorder quotas, estimators or guards to make the check pass.

Add a focused regression test using a simulated clock/workload: the third
configuration must still be admitted at the elapsed time that caused this
failure, while a genuinely exhausted budget must still reject admission. Run:

```bash
cd /home/stefi/DyNAMO/DyNAMO-Ridgeback
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m pytest -q tools/benchmark/qualification/test_campaign.py \
  tools/benchmark/qualification/test_isaac_repeatability.py
```

The prior preparation had 39 passing helper tests; they did not cover this
realistic readiness admission timing. Preserve unrelated dirty work. If changing
installed node scripts, check dependencies, rebuild the affected package and
verify actual executable copies under `install/*/lib/*`, not site-packages
symlinks. Helper-only changes run directly from `tools/`. Update the owning
[execution reference](../../target_distance_benchmarking/unattended_qualification.md)
for changed limits and rebuild graphify after code changes.

## Fixed experiment

Prepare a new uniquely named bundle using `campaign.py prepare-isaac NEW_BUNDLE`,
then `campaign.py check NEW_BUNDLE`. Do not launch the failed bundle or adopt
its partial checks as fresh repetitions. Preserve new source snapshots, patch,
model references, inputs and hashes before execution. Review drift against the
old preparation; never silently refresh active/finalized frozen hashes.

Two independent fresh Isaac sessions each run four conditions: box/stereo,
silhouette/stereo, box/monocular and silhouette/monocular. Each condition uses:

- all 88 scenes, expanded by packaged overrides to 109 trials;
- one within-scene repeat, 2-second settle, fixed 10-second wall-time windows;
- five stored raw batches per window, independent of full-window scoring;
- 640×480, furnished lab with props, OWLv2 at threshold 0.30 and a 10 FPS ceiling;
- projective ranging, Euclidean reconstruction and polar profiling; no pointcloud;
- deterministic PathTracing, AA 0, ideal depth, GUI, video, overlays and diagnostics.

Keep the selected result-cache disablement, AA override correction and real-frame
monocular readiness gate. Keep many-light sampling and estimator parameters.
The objective is approximately matching final outputs; RGB byte identity is not
an acceptance requirement. The prior small pair on `interfere_control_01` and
`chaos_03` had matching outcomes/reasons, maximum distance delta 4.732 mm and no
missing depth/live records among 356 monocular batches. That does not qualify
the full scene/mask matrix. Evidence is linked below.

## Start with working failure wakeups

Check host GPU, RAM, disk, graphical session and existing simulator/perception
processes. Sandbox `nvidia-smi` may falsely fail; use approved host access.
Never stop another user's work. The previous working GUI was `DISPLAY=:12.0`,
UID 1009, runtime `/run/user/1009` and bus `unix:path=/run/user/1009/bus`; verify
these values. The existing helper uses a campaign lock and isolated ROS domain,
and skips global cleanup. Respect its admission policy and record contention.

The earlier monitor worked: a local user-systemd timer ran the bundle's
`notify_monitor.py` every two minutes and queued a message using:

```text
codex queue --thread RECEIVING_CHAT_ID --message MESSAGE
```

No dedicated automation tool was available, so do not block on finding that
specific tool if the tested local timer/CLI route works. Reuse/adapt the previous
`notify_monitor.py`, `HEARTBEAT_PROMPT.txt`, `monitor_delivery_test.json` and
`monitor_registration.json` as examples. Point the new monitor at the **new
bundle and your receiving chat**, verify a benign queue delivery, arm a uniquely
named timer, and record its real unit/chat IDs and verified-delivery booleans.
Never copy the old disabled receipt as proof of registration.

Update the new wakeup prompt to reflect this recovery authorization: on a
failure, diagnose, stop only the owned run if needed, preserve it, repair,
validate, create a new bundle, rearm monitoring and restart. The old monitor
prompt says not to restart and is superseded by the user's instruction above.
Stay quiet on unchanged healthy status, notify only on changed problems/review
or completion, and disable each terminal attempt's old timer before rearming.
Keep a recovery ledger linking attempt, cause, repair, validation and replacement
bundle, including cumulative time consumed; respect execution limits rather than
silently resetting an exceeded campaign budget.

Once monitoring and preflight are valid, use the existing launch command:

```text
python3 tools/benchmark/qualification/campaign.py start NEW_BUNDLE
```

Verify that the monitor actually fires and that **all three readiness guards
pass**, including silhouette/monocular, before claiming full collection started.
Then supervise both full sessions. Repeated failure requires a revised diagnosis,
not another identical attempt. If a real external blocker or necessary change
outside this scope prevents progress, explain it to the user.

## Recovery and final report

On an alert, read `campaign.json`, the current guard and stage log, `REPORT.md`,
`service.json`, host samples and journal. A stale progress clock or an unhealthy
owned unit warrants investigation. Stop only the unit recorded for this attempt;
allow ordered rollback/recording finalization. Do not use global cleanup or kill
by process name. Repair infrastructure/model-readiness issues at their owning
layer; do not tune estimator accuracy or relax evidence checks.

If code or experimental settings change after either full repetition, collect a
new matched pair under one frozen implementation. Never combine repetitions
from different implementations to claim repeatability. An outcome or numerical
review alert alone is not permission to alter an estimator to reduce variation.

For results, match condition, estimator, `trial_id` and `instance_index` in the
full-window CSVs. Retain misses and reasons. Report per-target deltas and median/
maximum differences, changed outcomes/reasons, worst scenes, capture counts,
readiness/sample completeness and host load. Use `comparisons.json` pairs and
`REPORT.md`; sampled replay parity is a separate population. The monitor's 20 mm
or 1% alerts are operational review triggers, not agreed scientific equivalence
margins. Do not silently declare a scientific pass or confuse repeatability
with absolute accuracy. Report completion once and stop monitoring.

Restarting Codex leaves the user service running while the GUI session remains
logged in. A new agent must inspect and adopt an active run before doing anything
else. A workstation reboot interrupts the simulator/transient timer: preserve
that attempt and start a fresh reviewed matched pair under this authorization;
do not treat the interrupted process as reproducibly resumable.

## Archived evidence

- [Full-matrix Isaac repeatability pair](../../../archive/engineering/2026-11-06-isaac-full-repeatability.md)
- [Monocular readiness and two-scene Isaac repeatability](../../../archive/engineering/2026-11-05-monocular-readiness-and-isaac-repeatability.md)
