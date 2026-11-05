# r100_0160 D2 physical-stop capture attempts

Recorded dates: 2026-10-14, 2026-10-16
Tested revisions: unknown
Provenance: partial

This record covers the attempts to capture a D2 physical e-stop action during
bounded forward motion on the build installed on `r100_0160` on 2026-10-14 and 2026-10-16. The moving attempts
used the observer's bounded-permit trial with its physical-stop action under the
installed test-permit interlock (0.05 m/s, one second per stop cycle); the
source text does not record each run's exact flags. None qualified the physical stop; together they explain the operator-sequencing
rules in the current qualification procedure. It was extracted from the
consolidated driving reference at the
[documentation snapshot](https://github.com/Cognitive-Software-Labs/DyNAMO-Ridgeback/blob/077692ab033fd3a86d4849bbbbdb443a9f0c2125/docs/physical/driving_readiness.md).

Provenance gaps: the source text names no repository commit for the observer,
whose prompts and cross-axis bound changed between attempts as described
below. Raw recordings and operator terminal logs live only on the robot host
under `artifacts/hardware/`. Times are host receive times in seconds from
recording start.

## Attempts

| Run | Outcome | Cause |
|---|---|---|
| `d2-physical-stop-20261014T141512Z` | Aborted before a permit | Forward input arrived while every firmware stop report was still asserted |
| `d2-physical-stop-retry-20261014T141651Z` | Stayed in preflight | All stop reports asserted; the operator had kept the stop pressed in both attempts |
| `d2-physical-stop-20261016T101524Z` | Aborted before a permit | 22 asserted stop reports, 549 false permits; forward input without a recorded release |
| `d2-neutral-stop-report-20261016T102329Z` | Stationary reporting check passed | Release at 9.909 s, re-press at 25.851 s |
| `d2-physical-stop-retry-20261016T102502Z` | Aborted during motion | −0.00616 rad/s yaw exceeded the 0.005 rad/s bound |
| `d2-physical-stop-final-*`, first run | Ended without a result | No release or true permit recorded |
| `d2-physical-stop-final-*`, `20261016T102741Z` | Aborted during motion | −0.00616 rad/s yaw, same bound |
| `d2-physical-stop-adjusted-20261016T102950Z` | Aborted before motion | 25 asserted stop reports, 623 false permits; forward input without a recorded release |
| `d2-physical-stop-two-step-20261016T103208Z` | Inconclusive | Stop pressed after the budget had already ended motion |
| `d2-physical-stop-at-motion-20261016T103421Z` | Inconclusive | Stop pressed after the budget had already ended motion |

In every attempt limited and wheel commands were zero outside permitted motion,
and the operator confirmed the robot stationary with the red stop pressed
afterward.

## Blocked starts and the reporting check

The 2026-10-14 attempts were safely blocked starts: no true test permit,
nonzero limited command or nonzero wheel command was sent. The operator
explained afterward that they had kept the physical e-stop pressed during both.
They were not evidence of a reporting fault. A moving e-stop test requires
intentionally releasing the physical stop at `RECORDING READY`, waiting for the
observer to report `PHYSICAL RELEASE RECORDED`, then commanding the bounded
movement before pressing the stop.

On 2026-10-16 the operator reported physically releasing the red stop at
`RECORDING READY` in `d2-physical-stop-20261016T101524Z`, which conflicted with
every recorded firmware stop state. The neutral, stationary
`d2-neutral-stop-report-20261016T102329Z` check therefore recorded a full
physical-stop release and re-engagement with the software test lock still
asserted. In its operator window the firmware stop changed from pressed to
released at 9.909 seconds and back to pressed at 25.851 seconds, with no test
permits or joystick movement requests. The earlier all-pressed trace therefore
does not establish a broken stop-reporting path. The stop report arrives roughly
once per second and may lag the physical release.

## Cross-axis aborts

`d2-physical-stop-retry-20261016T102502Z` did record the physical release and
start true one-shot permits. Forward limited commands reached the installed
0.05 m/s cap. At 24.322 seconds the limited command included −0.00616 rad/s yaw,
exceeding the observer's forward-only 0.005 rad/s cross-axis bound, and it
aborted before recording a physical-stop action. The joystick's yaw axis had
drifted to −0.0123 while forward remained held. The operator observed only
forward motion. This was a bounded abort, not evidence of a failed physical
e-stop.

The second `d2-physical-stop-final-*` run, at `20261016T102741Z`, recorded the
release and one true permit before the same −0.00616 rad/s yaw component
triggered the same abort. After these two aborts the observer's incidental-yaw
command bound was changed from 0.005 to 0.01 rad/s (10% of the installed yaw
cap); the 0.05 m/s forward cap, zero lateral bound, wheel odometry bounds and
independent permit timeout were unchanged. This was an offline-verified
recorder adjustment.

## Prompt sequencing

The adjusted observer run `d2-physical-stop-adjusted-20261016T102950Z` refused
to arm because forward joystick input arrived without a recorded physical
release; the operator terminal shows the same abort message. The observer
prompt was then changed to present only the physical-release action at
`RECORDING READY`, with controller instructions appearing only after the
firmware reports `PHYSICAL RELEASE RECORDED`. This changed operator sequencing,
not the release gate, installed speed limits or physical-stop behaviour.

The two-step run `d2-physical-stop-two-step-20261016T103208Z` reached forward
motion with a recorded physical release and bounded permits. The physical-stop
press was reported at 21.781 seconds, approximately 4.12 seconds after the first
permit and well after the independent guard budget ended at 18.659 seconds. The
operator observed the robot stop by itself before pressing the red stop. The
next prompt was changed to ask for the red stop as soon as movement starts,
rather than waiting for a terminal prompt that can arrive too late for a human
response.

The at-motion-prompt run `d2-physical-stop-at-motion-20261016T103421Z` again
recorded physical release, joystick forward, true permits and a later
physical-stop press. The first permit was at 21.504 seconds; the independent
guard budget ended at 22.504 seconds; the reported stop press was at 28.493
seconds. The action therefore occurred after the automatic stop, and the
command-chain classifier returned inconclusive. The one-second permit budget was
not enlarged to make a human-timed test pass.

## Result

No attempt in this series qualified physical braking or the e-stop command
chain under motion on the build installed at that time. A qualifying trial must capture the
stop action while nonzero motor commands are still present, with the operator
able to act within that window. The `20261009-physical-estop-stop-4` observation
in [the 2026-10-09 record](2026-10-09-estop-qos-and-disconnect-trials.md#physical-e-stop-transitions)
retained its documented limits.
