# r100_0160 bounded direction trials and D2 deadman and command-loss runs

Recorded dates: 2026-10-14, 2026-10-16
Tested revisions: unknown
Provenance: partial

This record covers the D1 bounded floor trial in each of the six principal
directions, the stick-mapping checks they required, the D2 deadman attempts and
the input-sequence diagnosis that followed, and the D2 command-source-loss trial
on `r100_0160`. All moving trials ran under the installed test-permit
interlock (0.05 m/s planar magnitude, 0.1 rad/s yaw, one second per stop cycle)
with an operator beside the robot and clear floor confirmed. It was extracted
from the consolidated driving reference at the
[documentation snapshot](https://github.com/Cognitive-Software-Labs/DyNAMO-Ridgeback/blob/077692ab033fd3a86d4849bbbbdb443a9f0c2125/docs/physical/driving_readiness.md).

Provenance gaps: the source text names no repository commit for the observer,
pulse helper or controller diagnostic that ran, and the observer gained mapping
and prompt changes during the day as described below. Raw samples and analysis
summaries live only on the robot host under `artifacts/hardware/`. All times
within a run are host receive times in seconds from recording start.

## Preparation

Before these runs, the observer's `--stop-action deadman` and `--stop-action
e-stop` forward modes and the pulse helper's interlocked command-loss path had
passed offline tests only. The observer's `--stop-action` modes keep true
permits flowing after the requested action and mark the result inconclusive
unless the action, not the one-second budget, ended motion.

## Controller input under the stop

After the local clock gate eventually passed and the platform reached
active/running, `controller-input-20261014T090714Z` recorded 991 joystick
samples. Of these, 797 had non-neutral axes or buttons, including forward input
and driving-enable button 4, followed by 87 neutral samples. All 19 firmware
stop reports and 399 software-stop reports were asserted. All 380 limited
commands, 381 motor commands and 381 raw odometry samples were finite and zero.
This established fresh ROS controller input and neutral return under the stop.
The input spanned about 10.7 seconds rather than the requested one-second
action. The preceding `controller-check-20261014T090349Z` contained only neutral
input; the operator confirmed they had only run the command for that attempt.

## Direction trials

Each passing run recorded twenty nonzero wheel-command batches with the
expected sign pattern followed by zeros, a final asserted firmware stop, and
operator confirmation of the requested movement and an automatic stop before
the operator pressed the physical stop.

| Run | Limited command | Nonzero span | First zero motor batch | Raw odometry |
|---|---|---|---|---|
| `forward-permit-20261014T090919Z` | forward only, capped at 0.05 m/s | about 0.997 s | followed the bounded permit | forward motion, stationary after |
| `backward-permit-20261014T095109Z` | x in −0.05..0 m/s, zero lateral and yaw | 0.995 s | 32.2 ms after the last nonzero limited command | backward during motion, zero late tail |
| `left-permit-20261014T104511Z` | y in 0..0.05 m/s, zero x and yaw | 0.959 s | 54.0 ms after the last nonzero limited command | positive lateral, near-zero late tail |
| `right-permit-20261014T104838Z` | y in −0.05..0 m/s, zero x and yaw | 0.991 s | 42.7 ms after the last nonzero limited command | negative lateral, zero late tail |
| `turn-left-permit-20261014T105128Z` | zero planar, yaw in 0..0.1 rad/s | 0.976 s | 40.2 ms after the last nonzero limited command | encoder yaw reached 0.102 rad/s, zero late tail |
| `turn-right-permit-20261014T110845Z` | zero planar, yaw in −0.1..0 rad/s | 0.955 s | 50.1 ms after the last nonzero motor batch | reached −0.1024 rad/s, zero through the final two seconds |

The forward run was the first floor trial with the observer's bounded permit;
its operator confirmed stationary with the stop engaged afterward. Lateral
wheel batches had alternating signs, with opposite signs for left and right;
turn-right had the opposite wheel signs to turn-left. All six bounded direction
signs then had one floor trial with operator confirmation. None of these runs
measured physical stopping distance or qualified feedback faults, silent radio
loss or unrestricted driving.

### Left: input mismatch and mapping check

The first left attempt (`left-permit-20261014T095533Z`) never issued a permit
and aborted on the 60-second sequence deadline. Raw Joy axis 0 ranged −1..1
while axis 3 (the configured lateral input) remained zero. The combiner
requested yaw up to ±0.5 rad/s and no lateral velocity, so the recorder
correctly refused the left-specific permit. All limited output remained zero;
the physical stop was re-engaged and the operator confirmed stationary with it
pressed. This was an unarmed direction/input mismatch, not a failed sideways
motor response.

The stationary observer gained `--mapping-axis 3` to observe the stick-to-Joy
mapping with the stop pressed. The first explicit mapping runs ended before
stick input: one had driving-enable held and another released the physical
stop. Both retained zero wheel commands and odometry. Mapping mode was then
changed to wait, within its sixty-second deadline, while driving-enable is held.
`stick-mapping-20261014T104342Z` captured isolated positive axis-3 movement
(range 0..1), followed by neutral return, with the physical stop asserted in all
ten firmware reports. All limited commands, wheel commands and raw odometry
remained finite and zero. The operator action was the instructed
right-thumbstick-left movement, supporting the expected positive lateral input
mapping.

### Turn-right: input mismatches and mapping check

Two turn-right attempts at `turn-right-permit-20261014T110150Z` and
`turn-right-permit-20261014T110247Z` did not arm a permit or move the robot. Both
recorded zero limited yaw, zero wheel commands and no odometry yaw above
0.01 rad/s. The first saw right-stick lateral input (axis 3 negative), while the
second saw positive yaw-stick input matching turn-left; neither contained the
negative axis-0 input required for turn-right. The second observer reached its
60-second deadline and ended with Ctrl-C, so its result is an aborted
control-input attempt. The operator confirmed the robot stationary with the
physical e-stop pressed and red lights flashing. The mapping mode then accepted
`--mapping-axis 0`.

`yaw-right-mapping-20261014T110632Z` aborted because the physical stop was
released during the stationary mapping check. After re-engaging it,
`yaw-right-mapping-20261014T110707Z` recorded only axis 0, with values
−1.0..−0.183, a neutral return, and the expected negative yaw sign. It created no
command publisher, observed no stop release and detected no movement command.
The bounded trial observer was then changed to end with an explicit stop
instruction if no valid direction input arrives within 15 seconds after
physical-stop release.

## D2 deadman attempts

The first D2 deadman retry (`d2-deadman-20261014T113700Z`) is **inconclusive**.
The first true permit was sent at 24.491 seconds, the software lock reasserted at
25.478 seconds, and the last nonzero motor command arrived at 25.493 seconds.
Joystick input remained forward with driving-enable held until about 27.0
seconds; the recorded button release at 27.066 seconds coincided with centring
the stick. The guard's one-second budget therefore explains the stop before the
release, although the operator observed that the robot stopped and confirmed it
was later stationary with the physical stop pressed. The D2 classifier returned
`stop-action-command-chain-inconclusive`.

The repeat `d2-deadman-retry-20261014T114331Z` also remained inconclusive. Its
first true permit arrived at 19.035 seconds, the test lock reasserted at 20.012
seconds and the last nonzero motor command arrived at 20.027 seconds. ROS
joystick samples kept reporting L1/button 4 held through forward input until
the stick was centred at about 21.404 seconds; the first sample with that
button released arrived at 21.450 seconds. The operator reports releasing the
button first and observing the robot stop, but the recorded input transitions
cannot distinguish that action from the earlier budget stop.

A stationary physical/virtual controller snapshot at
`d2-button-path-20261014T114618Z` sampled both devices at approximately 50 ms
intervals under the pressed physical stop. Both reported the same Linux L1 key
310 state: held from about 6.023 to 14.002 seconds, released at 14.052 seconds.
Forward axis 1 returned near neutral before that release, so this run also did
not record L1 release while forward remained held. The operator reports using
the top-left L1 bumper. The diagnostic's printed instruction had said to
"release and centre" together, conflicting with the separate-release
instruction given to the operator, so a dedicated `--separate-release` mode was
added.

The first `--separate-release` attempt at
`d2-separate-release-20261014T115127Z` reached L1-plus-forward only at 18.269
seconds of a fixed 20-second recording, so it ended before the requested release
interval. Both devices agreed; the result is a timing-limited capture, not a
demonstrated button failure. The diagnostic was changed to wait up to 60
seconds for the first held input and then record up to 20 more seconds.

The corrected stationary run `d2-separate-release-fixed-20261014T141043Z`
verified the physical stop and zero odometry before and after sampling. Both
devices matched throughout. In the first bout L1 became held at 16.213 seconds
and forward axis 1 crossed the chosen displacement threshold at 17.568 seconds;
the stick returned toward neutral at 21.483 seconds, while L1 was released only
at 24.646 seconds. A second L1 press began at 29.565 seconds, with forward
displacement at 30.017 seconds, and L1 remained held until the 20-second
post-input window ended. No interval with L1 released and the forward stick
still displaced was captured. This supported an incomplete action sequence, not
a physical-to-virtual lag or a button defect. The guidance at the time was not
to repeat moving deadman trials solely to resolve this ambiguity; the
2026-10-09 observed stop remained the best deadman evidence, with its
protocol deviations.

The later stationary `d2-release-action-20261016T101033Z` run resolved the
input-sequence ambiguity. The physical and managed virtual controllers both
reported L1 plus forward, then L1 released while the forward stick remained
displaced for about four seconds (80 samples each). Their first held and
separate-release transitions agreed within the recorder's sampling interval.
Independent before/after probes found the physical stop engaged, active
controller routing and stationary raw odometry. This verified the L1 input path
and the operator action under the stop; it did not establish a moving deadman
stop on the current build.

## D2 command-source loss

The supervised trial `d2-command-loss-20261014T141323Z` used the installed test
interlock and the pulse helper's forward-only 0.05 m/s interlocked path. It sent
ten nonzero ramped source commands over approximately 0.5 seconds, then
deliberately sent no source command for one second while true test-permit
heartbeats continued. The first zero motor command arrived 54.5 ms after the
recorded start of publishing silence, with no later nonzero motor command. The
source-silence stop preceded the guard's independent one-second budget; raw
odometry was stationary during the silent interval and at the end. The physical
stop was reported pressed at completion. The operator confirmed brief forward
motion and automatic stopping before pressing it. This qualified the low-speed
**command-source timeout** in this configuration, subject to host receive-time
and single-trial limits; it is not a measured physical stopping distance or
radio-loss qualification.

The physical-stop attempts made the same day are in
[the physical-stop capture record](2026-10-16-physical-stop-captures.md).
