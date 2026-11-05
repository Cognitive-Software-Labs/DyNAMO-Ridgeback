# r100_0160 e-stop QoS repair and stationary disconnect trials

Recorded dates: 2026-10-06, 2026-10-09
Tested revisions: unknown
Provenance: partial

This record covers the discovery and repair of the firmware e-stop QoS mismatch
at the mux, the first recorded physical e-stop transitions and neutral release,
an observed low-speed deadman stop, and stationary wireless-controller
disconnect trials on `r100_0160`. It was extracted from the consolidated
driving reference at the
[documentation snapshot](https://github.com/Cognitive-Software-Labs/DyNAMO-Ridgeback/blob/077692ab033fd3a86d4849bbbbdb443a9f0c2125/docs/physical/driving_readiness.md).

Provenance gaps: the source text names no repository commit for the guard,
relay, observer or installer that ran. Installed file hashes live in the
installation receipts. Raw recordings, controller journals and analyses live
only on the robot host under `artifacts/hardware/`. The date of the
disconnect check that exposed the QoS mismatch is not recorded; the repair
candidate's mock evidence carries the `20261006` prefix.

## QoS mismatch discovery

A stationary wireless-disconnect check exposed an incompatible e-stop QoS pair:
`proton_ros2` offers **BEST_EFFORT**, while the installed `twist_mux` lock
subscription requests **RELIABLE**. The mux forwarded a small joystick command
while every captured firmware stop report was asserted. The originally
installed guard received the BEST_EFFORT stop data and kept its limited output
and wheel commands at zero; the observer aborted before a disconnection was
captured. The earlier graph audit had preserved this QoS mismatch, but its
interpretation missed it. The auditor was then extended to flag known
reliability/durability delivery blockers.

## Repair candidate checks

Before deployment, thirteen isolated loopback checks passed with the actual
installed mux and mecanum controller and mock wheels, including
BEST_EFFORT-to-RELIABLE stop delivery, asserted-lock suppression, release
without replay and stale firmware stop suppression during ongoing fresh
commands. Evidence lives under `artifacts/hardware/20261006-stop-qos-mock/`; the
tested candidate is under `20261006-stop-qos-staged/`.

## Relay installation

The repair was **installed and verified stationary on 2026-10-09** as an
upgrade of the capped installation. All four owned file hashes matched the
maintained source and installation record. The post-install probe confirmed a
single reliable guard publisher on `platform/emergency_stop_mux`, the mux's
reliable subscription to that relay, no mux subscription to the original
firmware stop topic, asserted relay data and the active controller subscribed
exclusively through the guard. The sensor and VNC service process IDs were
unchanged; the camera service was already inactive. Installation evidence lives
under `artifacts/hardware/20261009-stop-qos-install/`.

On the 2026-10-09 boot, the original guard failed twice because the DDS
profile's required `wlp3s0` interface was not yet available, then started
successfully. The installed QoS repair retained the same unit ordering.

An independent 19-second subscriptions-only recording under
`artifacts/hardware/20261009-stop-qos-stationary/` captured 19 asserted firmware
stop reports and 380 samples each of raw odometry, limited commands and wheel
commands, all zero. All captured motor feedback speeds were zero. This verified
stationary software state; physical controller disconnection, stop transitions
and stopping during motion remained separate tests.

A subsequent stationary controller trial under
`artifacts/hardware/20261009-wireless-disconnect-stationary-2/` captured live
joystick requests while all 31 firmware stop reports were asserted. The mux
forwarded no commands; limited commands, wheel commands, raw odometry and motor
speed feedback remained zero. Deadman and stick input continued until the
recording ended. The operator reported starting the recorder and operating the
joystick, without switching the controller off. This demonstrated
asserted-stop suppression; physical controller disconnection remained untested.

## Physical e-stop transitions

The `20261009-physical-estop-stop-2` observer run captured an asserted firmware
stop while the joystick still requested forward. Immediately before the
asserted report, the limited command was 0.05 m/s. The first limited command
after it was zero; the next captured wheel commands and raw odometry were also
zero. All subsequent captured samples stayed zero. This established software
suppression and zero reported wheel motion across the captured stop transition;
the physical operator observation was still required separately. Only 0.43
seconds of recording remained after the asserted report, so this run does not
establish a sustained stop or safe release without replay. It included multiple
movement bouts and exceeded the requested single one-second movement. Receive
timestamps do not measure latency from the physical button press. Raw recording
and interpretation: `artifacts/hardware/20261009-physical-estop-stop-2/`.

The `20261009-physical-estop-stop-3` recording captured nearly five seconds of
asserted-stop state with 101 zero limited commands, 100 zero wheel commands and
100 zero raw odometry samples. However, the forward request ended at 17.66
seconds and the last nonzero raw odometry was at 17.82 seconds; a later firmware
report still indicated released at 18.56 seconds, before reporting pressed at
19.56 seconds. This run demonstrates a sustained stationary tail but does not
isolate physical e-stop inhibition during motion. The operator's action order
was not clarified.

The `20261009-physical-estop-stop-4` run captured the stop asserted while the
joystick driving-enable button and forward axis remained active. The
immediately preceding limited command was 0.05 m/s; its first post-report
sample was zero. All 62 subsequent limited commands, 61 wheel commands and 61
raw odometry samples stayed zero across a 3.05-second tail. This isolates
asserted-stop suppression from controller centring. Raw wheel motion had
already subsided before the periodic asserted report arrived, consistent with
earlier physical inhibition, but the physical button timestamp was not recorded
and its latency cannot be derived. The operator confirmed that pressing the
physical red stop stopped the robot while forward remained held, and that the
robot was then stationary with the physical stop pressed. Physical stopping is
therefore observed for this trial; it does not qualify release or recovery.
The first movement command preceded the asserted report by 6.96 seconds,
exceeding the requested one-second exposure; this interval includes an unknown
reporting delay. This is a procedure deviation, not a compliant one-second
trial. No safe-release/replay test was performed.

## Neutral release

The `20261009-neutral-estop-release` live recording (observer mode
`e-stop-release`) captured 13.93 seconds between released and re-engaged
firmware reports. No raw mux command arrived; all 339 limited-command samples,
337 wheel-command samples and 337 raw odometry samples during the operator
phase were finite and exactly zero. The final two firmware reports asserted the
stop. The operator confirmed the robot remained still. Neutral release is
observed for this trial; held-input release and wireless reconnection remained
separate gates.

## Observed low-speed deadman stop

A trial recorded under `artifacts/hardware/20261009-deadman-stop-3/` captured
release of the driving-enable button while forward stick input remained
nonzero. Combiner and mux publication ceased; the guard zero and zero wheel
commands arrived about 0.115 seconds after release, and raw odometry settled
within 0.005 per component about 0.209 seconds after release. Commands and raw
odometry stayed stopped until another button press approximately six seconds
later. The operator confirmed that the first movement stopped by itself before
the physical e-stop was pressed. This established an observed low-speed deadman
stop with the installed caps and repaired stop relay.

The first forward bout lasted about 2.6 seconds rather than the requested one
second. A second movement started near the end; recording ended with forward
commands, nonzero raw odometry and released stop data. The entire capture is
therefore not a clean single-pulse pass. The operator subsequently confirmed
stationary, e-stop pressed and flashing red lights; an independent live probe
received three asserted-stop reports and 57 stationary raw-odometry samples,
with the capped controller and stop relay routing intact. Moving controller
disconnection, physical e-stop during motion, takeover, abrupt-start behaviour
and cold boot remained separate open gates.

## Stationary controller disconnection

On 2026-10-09 the third stationary trial
(`artifacts/hardware/20261009-wireless-disconnect-stationary-3/`, observer mode
`disconnect-stationary`) captured held joystick input followed by a `ds4drv`
journal disconnection at 11:39:03 UTC. The joystick then reset all axes and
buttons to neutral, and the combiner sent no commands after that reset. The
recording retained approximately 2.43 seconds of neutral input after reset; its
20-second tail starts at first input, not at physical disconnection. All 27
firmware stop reports were asserted, the mux forwarded no commands, and limited
commands, wheel commands, raw odometry and motor feedback speeds remained zero.
This verified propagation of a deliberate controller power-off while stopped;
it does not measure physical power-off-to-reset latency, sustained radio-loss
handling, reconnection or moving-disconnect stopping.

An earlier claim that the PS+Triangle shortcut disconnects the controller was
corrected by configuration parsing: its binding is defined under
`profile:kbmouse`, but the deployed driving controller has neither that profile
nor a named binding set enabled, and global bindings are empty. The stationary
tests did record disconnections, but their mechanism is unproven.

The `20261009-shortcut-held-disconnect` recording captured driving-enable,
forward input and PS/Triangle held until 40.658 seconds, followed by neutral
input at 40.688 seconds. The driver logged disconnected at 13:17:50 UTC. All 40
stop reports were asserted; no mux commands arrived, and 964 limited commands,
781 raw odometry samples and 800 wheel commands were zero. This established
held-input clearing across the recorded stationary disconnection, without
proving that the shortcut caused it or qualifying stopping in motion.

The `20261009-auto-held-disconnect-2` stationary trial verified the observer's
device-specific automatic disconnect (`--disconnect-device`): the disconnect
command returned zero with success output, the controller driver logged
disconnected at 2026-10-09 13:27:21 UTC, and subsequent Bluetooth inspection
reported `Connected: no`. Held joystick input was last received at 4.646
seconds and neutral input at 4.665 seconds; these receive times are not a
physical link-loss latency measurement. The recording retained twenty seconds
of neutral input afterward. All 24 firmware stop reports were asserted; no mux
commands arrived, and all 493 samples each of limited commands, wheel commands
and raw odometry were finite and exactly zero. This qualified the captured
deliberate disconnect/reset while the physical stop was held.

An aborted auto-disconnect trial on 2026-10-09 received its first raw odometry
only at 4.05 seconds, leaving twelve samples before rejection; a subsequent
passive stationary probe completed. Late wheel-feedback discovery is an observed
startup condition, not proof of missing motor traffic or a successful
disconnect test. The observer then gained its six-second discovery extension.

## Attempted moving disconnect

The attempted `20261009-moving-disconnect` run failed before arming: all
captured firmware stop reports indicated released and the observer timed out
waiting for an initially pressed stop. Nonzero mux requests spanned roughly
11.45–51.08 seconds of recording during preflight, exceeding the requested
one-second exposure. No disconnect process was launched; Bluetooth remained
connected. The operator subsequently confirmed the robot stationary with the
physical stop pressed. This is not evidence of failed stopping after a link
loss, because no link loss was induced. The automatic moving variant was then
disabled until a preflight motion interlock was prepared and tested, and the
observer was changed to abort immediately on commands or wheel motion before
READY; that warning alone is not a physical command interlock. No further
moving trial was authorized by this run. The interlocked follow-up is in
[the 2026-10-12 interlock record](2026-10-12-interlock-and-moving-disconnect-trials.md).
