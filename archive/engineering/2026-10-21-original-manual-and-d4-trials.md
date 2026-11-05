# r100_0160 original-manual guard profile and D4 handover trials

Recorded dates: 2026-10-16, 2026-10-21
Tested revisions: unknown
Provenance: partial

This record covers the isolated check of the deployed mux policy, the
installation of the guard's original-manual speed profile, the isolated and live
D4 source-handover trials, the D5 manual pulses that exposed commanding under a
motor fault, and the temporary test-lock installations and restorations of that
day on `r100_0160`. It was extracted from the consolidated driving reference at
the [documentation snapshot](https://github.com/Cognitive-Software-Labs/DyNAMO-Ridgeback/blob/077692ab033fd3a86d4849bbbbdb443a9f0c2125/docs/physical/driving_readiness.md).

Provenance gaps: the source text names no repository commit for the guard,
installer or recorders. The isolated runs preserve source and policy snapshots,
executable hashes and results in their evidence directories; the live runs
preserve traces and receipts. All of these live only on the robot host under
`artifacts/hardware/`.

## Deployed mux policy in isolation

On 2026-10-16 the isolated mux check, given the copied robot policy with every
input and lock topic replaced, passed the arbitration and suppression checks
for external, interactive-marker, joystick and RC priorities (1, 8, 10 and 12),
each of its three locks (255, 254 and 253), stamped-message preservation, input
expiry and return to the lowest-priority source. The exact source and
configuration hashes and the result are under
`artifacts/hardware/20261016-deployed-mux-isolated/`. This advanced D4's
software-policy check; live takeover and its physical response remained open.

## Original manual speed profile

On 2026-10-21 the operator requested ordinary manual driving at the original
joystick speed range. The guard's explicit `--original-manual-speeds` mode was
staged under `artifacts/hardware/20261021-original-manual-preview/`; its
isolated domain-213 mock-controller run passed original boosted diagonals,
source silence, the stop relay, stale firmware data and release without replay.
This did not establish physical stopping at the original speeds.

The profile was **installed and verified stationary on 2026-10-21** as an
upgrade from `artifacts/hardware/20261009-test-interlock-install`, recorded in
`artifacts/hardware/original-manual-install-20261021T121340Z/`. The live guard
used `--original-manual-speeds`, with no test-permit requirement. The
post-install probe confirmed the pressed stop, stationary odometry, active
mecanum controller, guarded command routing and the reliable stop relay. The
sensor and desktop service process IDs were unchanged. Physical driving at the
restored speed range was not qualified by this stationary install.

## Isolated D4 handover

The mux-to-guard handover check, run against the 2026-10-16 deployed-policy
snapshot with the current guard in its original manual speed profile, passed 17
checks: all four priority levels over continuously publishing lower sources,
each descending handover, guard zero during the expiry gap, all-source silence,
firmware-stop relay suppression, release without replay, fresh-input
resumption and exact timestamp/frame/velocity preservation. Original manual
diagonal boost values (1.2 m/s on each planar axis and 1.5 rad/s yaw) passed
unchanged in this synthetic test. After the last higher-priority input, guard
zero arrived in 121–130 ms and the lower-priority stream resumed in 526–527 ms,
consistent with the 0.5-second mux timeout. Logs, per-message traces,
policy/source snapshots, executable hash and the result are under
`artifacts/hardware/20261021-d4-mux-guard-handover-final/`. These are host
software timings, not physical stopping times; the run neither changed the live
speed profile nor tested wheels, radio behaviour or operator response.

## Live D4 handover attempts

The live bounded-source handover recorder was prepared the same day. Its
read-only preflight passed with the physical stop pressed, stationary odometry,
neutral controller and original-manual guard installed
(`artifacts/hardware/d4-live-preflight-20261021-3/`). The first preflight
correctly rejected an existing `twist_marker_server/cmd_vel` publisher, so the
trial used the unoccupied `cmd_vel` and `rc_teleop/cmd_vel` inputs at 0.025 and
0.045 m/s requested forward speeds. These are synthetic inputs to the live mux,
guard and platform, not a test of the physical RC controller.

| Run | Result |
|---|---|
| `artifacts/hardware/d4-live-handover-20261021/` | Timed out waiting for the physical stop-release report; no movement command published |
| `artifacts/hardware/d4-live-handover-20261021-2/` | Both sources published; the high-priority source took over, the lower-priority source returned 0.526 s after the last high request, and no sustained nonzero command followed silence. Raw-odometry forward peak 0.00178 m/s. Timed out before the operator re-engaged the stop, so `aborted` |
| `artifacts/hardware/d4-live-handover-20261021-4/` | No stop release, no nonzero source publication, no nonzero guard command, zero raw odometry; no live motion evidence |
| `artifacts/hardware/d4-live-handover-20261021-5/` | Lower-priority source returned 0.525 s after the final high request; final stop recorded pressed; raw odometry zero throughout and the operator reported the robot stayed still |

After run 2, a read-only probe confirmed the physical stop pressed, the robot
stationary and the controller active, and the operator requested one fresh
supervised repeat. Run 5 passed live mux and guard switching, but the robot
did not move, so physical wheel response during handover was still
unverified. A bounded, joystick-free manual-mode pulse was the next check of
the drive path.

## D5 manual pulses and the motor-ready gate

The one-second 0.05 m/s pulse under `artifacts/hardware/d5-manual-pulse-20261021/`
reached outgoing CAN speed-set frames, but the robot stayed still and raw
odometry showed only a tiny delayed response. Motor status reported fault bit 8
on all four controllers during the command, and incoming setpoint fields stayed
zero. The fault cleared several seconds after the physical stop was released,
after the pulse had ended. The bounded pulse and D4 handover tools were then
changed to require fresh, stable zero-fault speed-mode reports after stop
release before sending any nonzero command; the change first passed offline
tests and was then exercised live.

The next one-second pulse under `artifacts/hardware/d5-manual-pulse-20261021-2/`
waited 6.78 s for all four motor faults to clear after stop release. The
operator observed forward motion and a self-stop; raw odometry peaked at
0.0511 m/s and the final physical stop was recorded pressed. The earlier
no-motion result was therefore consistent with commanding while the motors
still reported fault 8, rather than a broken mux-to-wheel command route.

## D4 handover with the motor-ready gate

The passive-CAN-wrapped repeat under `artifacts/hardware/d4-handover-can-20261021/`
sent no nonzero source command: the operator kept the physical stop pressed,
and all observed stop reports remained pressed until the recorder timed out.
Its 32,739 valid CAN field receipts describe stationary feedback only and cannot
qualify the live handover.

The run under `artifacts/hardware/d4-handover-can-20261021-6/` recorded the
physical stop release and waited 7.23 s for four clear motor statuses. The live
mux accepted the 0.025 m/s lower-priority source, then the 0.045 m/s
higher-priority source; the lower source returned 0.525 s after the last high
request. Raw forward odometry peaked at 0.0465 m/s. The operator confirmed
forward-only movement and a self-stop before pressing the red stop; the final
pressed-stop report was saved. The passive CAN capture had 0 malformed relevant
frames. This qualified one bounded live source handover and return; actual
radio-controller takeover, loaded recurrence and physical stopping distance
remained separate checks.

## Temporary test-lock installations

For D4/D5 testing later that day, the one-second 0.05 m/s test-interlock mode
was temporarily installed in
`artifacts/hardware/d4-d5-test-interlock-live-20261021-2/`. The three D5 capture
attempts did not arm moving trials, and the exact saved pre-test manual-speed
files and service activity were restored from that run. After restoration the
guard and platform were active; a read-only ROS probe confirmed the pressed
physical stop, stationary raw odometry, active mecanum controller, guarded
routing and reliable mux-stop relay. This restoration was not a physical-speed
or D5 moving qualification.

A later attempt to stage a joystick-independent D5 pulse stopped before any
motion: the platform's clock startup gate waited for a fresh chrony reference,
while the installer had a shorter 35-second service-start timeout. The recovery
restored the manual-mode files; after a fresh NTP burst both services became
active and the stationary stop/routing probe passed. The installer was then
changed to require a fresh clock reference before disrupting the platform and
to queue restore startup without imposing that short timeout on the clock gate.
The failed attempt is under `artifacts/hardware/d5-pulse-test-lock-20261021/`;
no pulse capture or moving qualification resulted.

The corrected retry installed the 0.05 m/s test lock under
`artifacts/hardware/d5-pulse-test-lock-20261021-2/`. One joystick-independent
half-second forward pulse completed under that lock and captured passive CAN;
the detailed D5 evidence and limits were owned by the
[wheel-feedback reference at the documentation snapshot](https://github.com/Cognitive-Software-Labs/DyNAMO-Ridgeback/blob/077692ab033fd3a86d4849bbbbdb443a9f0c2125/docs/physical/drive_feedback_diagnostics.md).
The exact pre-test manual-speed files were restored afterward. Both platform and
guard were active, and a read-only ROS probe again confirmed the pressed
physical stop, stationary raw odometry, active mecanum controller and guarded
routing. The restore receipt initially said `platform-starting` because start
was queued behind the clock gate; the subsequent live probe, rather than that
intermediate label, established the active postcondition.
