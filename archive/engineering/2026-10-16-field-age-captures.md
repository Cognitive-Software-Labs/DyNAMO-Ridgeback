# Single-bound field-age candidate and stationary field captures

Recorded dates: 2026-10-16
Tested revisions: unknown
Provenance: partial

This record covers the first per-field age driver candidate, which used one
200 ms bound for every field, and two passive 30-second stationary CAN captures
used to judge that bound. The single bound is **superseded** by the split
150 ms motion and 500 ms auxiliary bounds in
[the acceptance record](2026-10-22-d5-acceptance.md). The source text recorded
no repository commit for the builder, replays or captures, so tested revisions
are unknown. Candidate receipts on the robot host pin source and patch hashes
but were not copied into the repository. Directories named below are on the
robot host under `artifacts/hardware/`. The text was moved here from the
[wheel-feedback reference at the documentation snapshot](https://github.com/Cognitive-Software-Labs/DyNAMO-Ridgeback/blob/077692ab033fd3a86d4849bbbbdb443a9f0c2125/docs/physical/drive_feedback_diagnostics.md).

## Candidate build

`clearpath_puma_field_age.patch` and its receipt policy were layered after the
two installed repair patches and the completeness candidate. The live driver
was unchanged and the default Puma builder and installer still used only the
two repair patches. The field-age builder emitted `candidate.json`, not a
deployment `SOURCE.json`, and performed no installation or service change.

The candidate first used **200 ms for experiments**, following the earlier
motion-field replay model in
[the repair record](2026-10-12-puma-repair-rates-and-freshness-replays.md); no
production bound was selected. The complete package and its checks passed
under `artifacts/hardware/20261016-puma-field-age-candidate-3/`; standalone policy checks are under
`artifacts/hardware/20261016-field-age-policy/`. Loaded and under-motion captures, actual ROS and
receive-thread timing, consumer interaction and a supervised pinned deployment
were all open at this point.

## Quiet stationary capture

A fresh passive 30-second stationary `vcan0` capture (directory
`artifacts/hardware/20261016-one-hour-field-capture/`, despite its name) recorded 14,813 relevant
received fields. Its 20 Hz timer replay projected 19.97–20.00 Hz at 200 ms, but
wheel 3's setpoint field had a 199.83 ms receive gap, leaving almost no margin
for callback or scheduler delay at that limit. Position and speed field gaps
stayed below 40 ms.

The payload-validated replay (`replay-invalidating.json`) found zero malformed
relevant RX frames and produced the same projection. The replay was extended
to reject wrong payload widths and to model a malformed recognized field as
invalidating the candidate cache; its test injects a bad speed frame and checks
the resulting publication gap. It remained an offline timer model: it did not
execute the ROS queue, measure a loaded callback delay or establish physical
wheel synchronization.

An ideal-timer sensitivity replay (`replay-age-sensitivity.json`) projected
19.93–19.97 Hz with a 150 ms bound, but its worst publication silence was
100 ms, compared with 50 ms at 200 ms. These were experimental bounds only: a
stationary capture cannot establish the delay distribution under motion or
load, or a safe production bound relative to the installed 250 ms consumer
timeout. The result supported evaluating slow setpoint fields and fast motion
fields separately.

## Concurrent-workload capture

A second passive 30-second capture, `artifacts/hardware/d5-passive-20261016T104846Z/`, ran while
Thor's failure observer was active. All 14,794 relevant RX frames had the
expected payload widths. Wheel 3's setpoint receive gap reached
**250.073 ms**, while its position and speed gaps stayed near 40 ms. At the
experimental 200 ms bound the timer replay projected 19.93–19.97 Hz and a worst
100 ms aggregate-output silence. `replay-complete.json` carries all twenty
per-wheel field gap rows and 150–250 ms experimental age-limit sweeps in one
repeatable output. The installed driver and the wheels were neither changed
nor moved.

This was concurrent-workload evidence, not a controlled CPU load or
moving-wheel qualification. It showed that the production bound had to come
from the actual field-delay distribution and consumer behavior, not from the
earlier 199.83 ms stationary maximum alone.
