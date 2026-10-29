# r100_0160 D-lane baseline, D2 preparation and the reconnected deadman run

Recorded dates: 2026-10-22
Tested revisions: unknown
Provenance: partial

This record covers the 2026-10-22 driving work on `r100_0160`: the read-only
baseline snapshot and stationary health runs, the D3 and D8 preparation state,
the one D2 deadman trial under a temporarily reinstalled test interlock, the
restoration of the original-manual guard, and the import of earlier runs into
the unified D-lane evidence schema. It was extracted from the consolidated
driving reference at the
[documentation snapshot](https://github.com/Cognitive-Software-Labs/DyNAMO-Ridgeback/blob/077692ab033fd3a86d4849bbbbdb443a9f0c2125/docs/physical/driving_readiness.md).

Provenance gaps: the baseline snapshot records source and installed
configuration hashes, but the source text names no repository commit, so no
tested revision is claimed. Firewall rules could not be listed without root in
the snapshot. The snapshot, health batches and trial recordings live only on
the robot host under `artifacts/hardware/`. The test-interlock installation
directory for the deadman trial is not named in the source text.

## Baseline and stationary health

The guard service used `--original-manual-speeds`. A read-only stationary batch
passed with the physical stop pressed; both platform and guard services were
active, and the local clock gate passed. Those observations did not authorize a
moving stop trial at original manual speeds.

The live identity snapshot from `driving_qualification.py snapshot` is under
`d-lane-baseline-20261022-live/`. It records the manual-speed guard, the
installed field driver and the feedback plugin. A contemporaneous read-only
stationary health run passed under `stationary-batch-20261022T094439Z-e52fb0/`.
Because the snapshot could not list firewall rules without root, each fault
runner kept its own privileged exact-rule preflight and cleanup check.

## D3 and D8 state

The passive D3 silent-loss recorder was prepared with its link-drop and
1.5-second neutral-report requirements. Earlier captures had not demonstrated a
link drop and did not pass D3. The live test needs one operator beside the
physical stop and another carrying the still-held controller away; with only
one operator available it was deferred. No D8 stopping distance had been
measured by this date.

## D2 deadman trial

Before the trial the temporary 5 cm/s test interlock was installed and verified
with the physical stop pressed. The `d2-deadman-reconnected-20261022` trial
reached one bounded forward permit. The first true permit was at 20.005 seconds
and the last nonzero motor command at 21.022 seconds. The joystick trace showed
L1 plus forward input through 23.423 seconds, with no sample in which L1 was
released while forward remained held. The recorder therefore classified the
deadman-specific command-chain result as inconclusive: the one-second budget,
not a deadman release, ended motion. Its final reports show the physical stop
pressed, zero motor commands and stationary raw odometry. The operator confirmed
the robot was stationary with the red stop pressed. This is not an isolated
deadman stop; a qualifying trial must capture button release with the forward
stick still held, before the independent permit expires.

The temporary test-interlock installation was then restored from its saved
backup. The guard again ran `--original-manual-speeds`; both guard and platform
reached active/running, the clock gate passed, and a fresh stationary health
batch passed with the physical stop pressed. No further moving D2 trial was
performed in original-manual-speed mode.

An alternative D2 physical-stop recorder was prepared in
`tools/hardware/commissioning_forward_pulse.py` for the restored manual guard:
a ramped 0.05 m/s forward command for at most two seconds, with the operator
pressing the physical stop during measured movement. Its parser and
classification checks passed offline; it had not been run on hardware by the
end of this record.

## Import into the evidence schema

The established D1 six-direction runs, the one D2 command-loss run, the three
D3 deliberate disconnect/reconnect cycles, the passing D4 handover and the
stationary D5 field-driver acceptance were imported into the
`driving_qualification.py` schema. The import preserved their original limits:
D2 still needed three isolated repetitions per mechanism, D3 still needed
genuine radio degradation, and D4 still needed regression on the final
installed configuration. The source runs are in
[the direction and deadman record](2026-10-14-direction-and-deadman-trials.md),
[the reconnect-cycle record](2026-10-12-controller-input-loss-and-reconnect-cycles.md#repeated-stationary-cycles)
and [the D4 record](2026-10-21-original-manual-and-d4-trials.md#d4-handover-with-the-motor-ready-gate);
the D5 acceptance is owned by the
[wheel-feedback reference at the documentation snapshot](https://github.com/Cognitive-Software-Labs/DyNAMO-Ridgeback/blob/077692ab033fd3a86d4849bbbbdb443a9f0c2125/docs/physical/drive_feedback_diagnostics.md).
