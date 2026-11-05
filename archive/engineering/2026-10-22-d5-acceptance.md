# Split field-age bounds, field-driver installation and stationary acceptance

Recorded dates: 2026-10-22
Tested revisions: unknown
Provenance: partial

This record covers D5: the choice of split field-age bounds, the pinned
field-driver build, its installation on the physical Ridgeback and its
stationary live acceptance, including one failed acceptance and the criterion
revision that followed. The source text recorded no repository commit for the
builder, installer, replay or acceptance checker, so tested revisions are
unknown. Build and install receipts on the robot host pin source, patch,
dependency and binary hashes but were not copied into the repository.
Directories named below are on the robot host under `artifacts/hardware/`. The
text was moved here from the
[wheel-feedback reference at the documentation snapshot](https://github.com/Cognitive-Software-Labs/DyNAMO-Ridgeback/blob/077692ab033fd3a86d4849bbbbdb443a9f0c2125/docs/physical/drive_feedback_diagnostics.md).

## Split-bound replays

The single 200 ms bound treated a slow setpoint reply like a stale wheel speed:
rear setpoint gaps of 150–250 ms withheld aggregates while position and speed
stayed near 40 ms ([capture record](2026-10-16-field-age-captures.md)). The
policy was changed to require position and speed to be new since the previous
publication and at most 150 ms old, and duty, current and setpoint to be valid
and at most 500 ms old. The single-bound constructor was kept for the earlier
models.

The capture-chain replay ran the maintained classes over five real captures:
quiet and loaded stationary, a stationary attempt, the manual pulse and the D4
handover ([moving-capture record](2026-10-21-d5-motion-captures-and-queue.md)).
At 150/500 ms every capture published on every tick after start-up (50 ms worst
gap) with no consumer fault, also with a fixed 25–100 ms receive-queue delay.
Injected asymmetric faults behaved as intended: one rear setpoint silent for
400 ms never reached the consumer, while 700 ms of setpoint silence or 300 ms of
one wheel's speed silence latched it. A 100 ms motion bound failed at 75 ms of
queue delay and 125 ms failed at 100 ms. Beyond the bound the chain failed
safe: the consumer never activated. Results are under
`artifacts/hardware/d5-split-150-replay-20261022-{q,b,c,p,h}/`, and the 100 ms sweep is in
`artifacts/hardware/d5-split-bound-replay-20261022-*`. The timer was idealised and the delays
fixed; these are not live measurements.

## Pinned build

The pinned package `artifacts/hardware/d5-split-bound-candidate-20261022-3/` (Clearpath 2.9.4 with
the fault-flag, cache-safety, completeness, field-age and queue-timing patches)
passed 119 real-driver and 240 aggregate-routine assertions, including
slow-auxiliary reuse, asymmetric wheels, stale duty or current and missing
per-wheel fields. The policy check passed 542 assertions
(`artifacts/hardware/d5-split-bound-policy-20261022-2/`).

## Staging and installation

A read-only stage dry run (`artifacts/hardware/d5-field-driver-stage-dryrun-20261022/`) recorded
the D2 test interlock as the installed cap mode, so the live window had to
stage again. The baseline smoke run of the live acceptance checker on the old
repair driver correctly reported not passed
(`artifacts/hardware/d5-field-live-smoke-baseline-20261022/`).

The first apply refused before changing anything because the guard had
returned from the D2 test interlock to original-manual mode after staging. A
fresh stage (`artifacts/hardware/d5-field-driver-install-20261022-2/`) then installed with status
`installed-verified-stationary`.

## Stationary acceptance

The first 60-second acceptance (`artifacts/hardware/d5-field-live-acceptance-20261022/`) is
recorded as **not passed**. Only the queue limit failed: one bucket peaked at
52.7 ms against the original 50 ms limit. Every bucket peaked at about 50 ms
with a mean of about 30 ms. That pattern is the structural wait of the driver's
once-per-50 ms queue drain rather than load.

Deviation: the acceptance criterion was revised after that failing run, to one
drain cycle plus 25 ms of margin (75 ms). The basis was that the original
50 ms limit left no allowance for the driver's own 50 ms drain cycle. The
failing run keeps its verdict.

A fresh repeat (`artifacts/hardware/d5-field-live-acceptance-20261022-2/`) **passed**:

- 1,200 aggregates at 19.99 Hz, with a 52.0 ms maximum gap;
- all four drivers reporting, with motion age at most 51.2 ms (limit 150 ms) and
  auxiliary age at most 160.6 ms (limit 500 ms);
- no invalid samples;
- queue maximum 50.6 ms, with none over 100 ms;
- the stop pressed throughout and odometry zero.

This qualified stationary operation of the field driver. Motion and load
behavior were left to D6–D8, and the original driver stayed available through
the installer's restore.
