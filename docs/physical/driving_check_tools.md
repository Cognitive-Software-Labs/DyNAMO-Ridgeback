# Ridgeback driving check and recorder tools

This page owns what each driving check under `tools/hardware/` measures, which
flags change its behaviour and what its result can and cannot establish. The
[driving operator runbook](driving_operator_runbook.md) owns operator
commands; the [qualification procedure](driving_qualification_procedure.md) owns
how results become D-lane verdicts.

Common rules:

- Each tool writes to a fresh `--run-dir` or output directory under
  `artifacts/hardware/`; the observer refuses an existing directory.
- A successful recorder exit is not a successful stop test. Results are
  recorded as awaiting analysis and operator observation unless the tool says
  otherwise; physical observations are recorded separately from software
  measurements.
- No tool resets or enables the base, releases or re-engages the physical stop,
  or reconnects a controller. Recorders cannot stop the robot; the operator's
  physical stop is the only physical inhibitor.
- Receive timestamps are host callback times. They do not measure physical
  button latency, braking time or stopping distance.

## Isolated software checks

These run private ROS graphs and never address a base controller on the robot.

| Tool | Graph | What it checks |
|---|---|---|
| [`check_driving_mux.py`](../../tools/hardware/check_driving_mux.py) | New mux, domain 213, loopback DDS, private namespace | Stamped-message preservation, source priorities, stop-lock suppression and release, silence after input expiry, priority handoff |
| [`check_driving_handover.py`](../../tools/hardware/check_driving_handover.py) | Private mux and current guard in original-manual mode, domain 214, loopback DDS | 17 checks: all four priorities over continuously publishing lower sources, each descending handover, guard zero during the expiry gap, all-source silence, firmware-stop relay suppression, release without replay, fresh-input resumption, exact timestamp/frame/velocity preservation |
| [`check_commissioning_pipeline.py`](../../tools/hardware/check_commissioning_pipeline.py) | Installed mecanum controller with `mock_components/GenericSystem` wheels, domain 213, private namespace; never loads a physical driver | 13 checks: routing, boost and diagonal caps, preserved stamps, expiry, asserted-stop suppression, BEST_EFFORT-to-RELIABLE stop delivery, release without replay, stale firmware stop during fresh commands; `--original-manual-speeds` checks that profile |
| [`check_test_interlock.py`](../../tools/hardware/check_test_interlock.py) | Simulated topics only, domain 214, no hardware nodes | 7 checks: no-permit blocking, capped permitted forwarding, heartbeat loss, maximum budget expiry, refusal to rearm with false/true alone, blocked physical release, new permit after an asserted-stop cycle |

`check_driving_mux.py --config` reads an installed YAML policy but replaces every
input and lock topic before running the isolated mux. `check_driving_handover.py`
requires such a policy snapshot (`--config`) and replaces every source and lock
endpoint. Neither contacts deployed processes. They are software behaviour
tests, not e-stop or motor tests.

## Graph and parameter auditor

[`audit_driving_readiness.py`](../../tools/hardware/audit_driving_readiness.py)
uses the robot-local namespace, domain 0 and the deployed CycloneDDS profile
`/etc/clearpath/cyclonedds.xml`. A bounded worker queries graph endpoints and
only `ListParameters`/`GetParameters`. It creates no data subscriptions or
command publishers, changes no parameters and never reports physical readiness
as a pass. It flags known reliability/durability delivery blockers; absence of
those flags is not a full QoS compatibility pass. Missing or unreadable
parameters remain evidence gaps.

## Live handover recorder

[`check_live_driving_handover.py`](../../tools/hardware/check_live_driving_handover.py)
drives the **live** mux, guard and platform with two synthetic forward sources:
0.025 m/s on `cmd_vel` (priority 1) and 0.045 m/s on `rc_teleop/cmd_vel`
(priority 12). It does not test the physical RC controller. `--preflight-only`
is read-only: it requires the physical stop pressed, stationary odometry, a
neutral controller and the original-manual guard, and rejects an input topic
that already has a publisher. A motion run additionally needs
`--operator-confirmed` and `--bounded-motion-confirmed`, records the physical
stop release, and waits for fresh, stable zero-fault speed-mode reports from all
four motors before either source publishes. The operator presses the physical
stop at the end; a run that times out waiting for that is `aborted`.

## Forward-pulse helper

[`commissioning_forward_pulse.py`](../../tools/hardware/commissioning_forward_pulse.py)
publishes its own bounded command on the external mux input. It defaults to
passive observation. Execution (`--execute --operator-confirmed`) requires a
released, fresh firmware stop and verifies installed guard files, exclusive
output ownership, controller routing and activity, stationary raw odometry and
an uncontested external mux input. After release it waits for fresh zero-fault
speed-mode motor reports before any nonzero command. Missing stop, odometry or
guard data, competing nonzero commands or unexpected odometry abort the pulse.
Samples and an operator-observation placeholder are saved separately; a
stationary final odometry value cannot establish actual braking.
Its conservative abort thresholds are diagnostic bounds, not certified physical
stopping limits. It performs no service changes or firmware reset/enable
actions. Command values are serialized as floats.

| Option | Effect |
|---|---|
| default | Forward 0.05 m/s for at most one monotonic second, then explicit zeros |
| `--axis backward\|left\|right\|turn-left\|turn-right` | One other principal direction; turning peaks at 0.1 rad/s |
| `--ramp` | 250 ms rise and fall within the one-second pulse; peak and abort limits unchanged |
| `--stop-mode timeout` | Ends publication at the pulse end and observes one second of silence before cleanup zeros; with `--ramp`, the profile only ramps up so silence begins at the peak |
| `--test-interlock` | Verifies the installed test lock, starts with the stop pressed and a neutral controller, prompts for release, publishes one half-second forward source, then stops publishing while the test permit stays fresh |
| `--original-manual-speeds` | Verifies the installed original-manual guard and requires stop release for one one-second 0.05 m/s forward pulse |
| `--stop-mode physical-e-stop --physical-stop-confirmed` | Original-manual guard only, executed and ramped forward: up to two seconds at 0.05 m/s, the operator presses the physical stop during measured movement, the source stays alive briefly after the stop, and limited and motor commands must be zero while the stop is pressed |

Timeout mode is a command-publication-loss test; it does not prove joystick
disconnect or a physical e-stop under motion. In the interlocked command-loss
path the guard's fixed budget still ends motion by one second; the result is
inconclusive if no nonzero wheel command preceded silence, if zero arrives only
at the independent budget, or if a nonzero command resumes after the first zero.
In physical-e-stop mode the stop action time is the first motor-status sample
showing the e-stop fault (8) on part of the wheel set, or the firmware stop
report if no such sample exists; a fault not followed by a firmware stop report
within 1.2 s aborts. The trial is classified unqualified unless the command
chain is isolated. None of these modes measures braking distance.

## Physical-control observer

[`observe_driving_stop.py`](../../tools/hardware/observe_driving_stop.py)
checks graph and file ownership and records joystick messages, mux and guard
outputs, wheel commands and feedback, raw and filtered odometry, motor status,
diagnostics, the firmware stop and the relay. The operator performs the
physical action. Only the permit modes publish anything: test permits.

| `--test` mode | Requires | Records |
|---|---|---|
| `stationary` | — | Passive stationary state; with `--test-interlock` also the software lock |
| `stationary --mapping-axis 0\|3` | Test interlock, operator confirmation, connected controller | One stick action within 60 s, then one second of neutral; prints the changed motion axes |
| `disconnect-stationary` | Physical stop engaged | Held deadman and forward input, then its reset when the controller disconnects; 60 s input-start window, 20 s tail from first input |
| `e-stop-release` | Pressed stop, fresh neutral input | Ten seconds after the released stop report, then re-engagement with two asserted reports; unexpected commands, wheel motion, stale or non-neutral controller input and premature re-engagement abort |
| `interlock-held` | Test interlock | Held forward input with the physical stop released, five seconds of observation, neutral controls and two final asserted reports; sends no commands or permits, rejects existing or new permit publishers and requires the software lock asserted throughout |
| `interlock-permit` | Test interlock, `--permit-motion-confirmed` | One bounded trial; `--direction` and `--stop-action` below |
| `interlock-disconnect` | Test interlock, `--permit-motion-confirmed`, `--moving-disconnect-confirmed`, one connected `--disconnect-device` | One-shot disconnect half a second after the first nonzero mux command during permitted forward input |
| `interlock-reconnect` | Test interlock, `--reconnect-device <controller-MAC>`, controller initially disconnected | Ten seconds disconnected, a manual PS reconnect with centred controls, thirty more seconds, then the device connected and fresh neutral input with the stop asserted; sends no commands, permits or connect requests |
| `interlock-feedback-fault` | Test interlock, wheels raised, `--feedback-fault-confirmed` | One invalid feedback batch during measured motion; owned by the [motor-link fault reference](motor_link_fault_injection.md#fault-paths) |

The un-interlocked `deadman`, `e-stop` and `disconnect` modes record an
operator's deadman release, physical stop or controller power-off during manual
driving on the default capped installation; they publish no permits.
Automatic moving disconnection outside the interlock is disabled:
`--disconnect-device <MAC>` is refused with `--test disconnect` and otherwise
supports only `disconnect-stationary` and `interlock-disconnect`.

With `disconnect-stationary`, `--disconnect-device <MAC>` needs operator
confirmation, checks that this device is connected before recording, and
disconnects it on the first held input, retaining the command output and exit
status. Command failure or a timeout aborts the recording. Successful command
exit alone is not proof of disconnection; the
[controller input page](controller_input.md#disconnection-behaviour) lists what
to verify. This deliberate disconnect is separate from silent radio loss.

Before recording, the observer compares the installed guard files with the
default-caps rendering, or with the test-interlock rendering when
`--test-interlock` is given, and refuses any difference. It therefore does not
run against the original-manual installation.

**Preflight and aborts.** Every mode first checks fresh firmware stop, odometry
and output observations and a stationary robot. If fewer than twenty raw
odometry samples arrive in the initial window, it waits up to six more seconds
for wheel-feedback discovery, then still rejects insufficient data, nonzero
preflight wheel motion and stale or released stop state. Commands or wheel
motion before READY abort immediately; that warning is not a physical command
interlock. Stationary modes abort on physical stop release, software unlock,
stale observations, nonzero wheel commands or odometry, unexpected downstream
commands and competing permit publishers. Mapping mode waits, within its
60-second deadline, while driving-enable is held and ignores those samples.
Stale-observation errors name the topic, observed age and threshold (0.25 s for
raw odometry).

**Permit trials.** Initial preflight requires a pressed physical stop, neutral
controls, no permit publishers, stationary wheels and an asserted software
lock. The operator releases the stop at `RECORDING READY`; controller
instructions appear only after the firmware reports `PHYSICAL RELEASE
RECORDED`. Only then, on held input in the selected direction, does the
recorder publish fresh permits, for at most 1.25 seconds; the guard enforces its
own one-second budget, and the extra tail checks that heartbeats cannot extend
motion. Without valid direction input within 15 seconds of release, the trial
ends with an explicit stop instruction. Every exit attempts a false permit.

- `--direction forward|backward|left|right|turn-left|turn-right` selects only the
  recorder's expected input, command and odometry component and sign. It
  rejects mixed-axis input before granting a permit. Fault and disconnect modes
  stay forward-only. No installed guard file changes.
- Command acceptance: the selected component within the 0.05 m/s or 0.1 rad/s
  cap, other planar components zero, and incidental yaw up to 0.01 rad/s
  (10% of the yaw cap), or 0.02 rad/s for the deadman stop action.
- Raw-odometry bounds: the selected planar component between −0.01 and
  0.075 m/s (yaw −0.02 to 0.15 rad/s), other planar components within 0.02 m/s
  and yaw within 0.05 rad/s.
- In the default `--stop-action budget`, controller release or an asserted stop
  cancels the trial permanently.
- `--stop-action deadman` keeps true permits flowing after driving-enable is
  released, so a deadman stop is distinguishable from permit cancellation. It
  records a release only while the forward stick stays displaced; centring the
  stick alone does not count.
- `--stop-action e-stop` prompts for the physical stop as soon as movement
  starts and records its assertion. The physical stop still asserts the guard
  lock.

**Moving disconnect.** In `interlock-disconnect`, the disconnect request time,
command output and return status are preserved. Joystick reset alone does not
cancel permits: true heartbeats continue within the same 1.25-second window
while the guard caps permission to one second. A stale joystick stream after the
requested disconnect is expected. `bluetoothctl` runs with closed stdin and a
one-second CLI timeout; its exit code is supporting evidence only. After the
request the recorder records through the bounded permit and exits only with a
fresh asserted physical stop, zero limited and wheel commands and stationary raw
odometry, then queries the controller's actual Bluetooth state; an unconfirmed
disconnect is marked failed. It never reconnects.

## Controller tools

The experimental
[`check_controller_cycles.py`](../../tools/hardware/check_controller_cycles.py)
combines up to three (`--cycles 1..3`, default 3) deliberate disconnect and
neutral-reconnect cycles in one stationary session. It requires the installed
test interlock, a confirmed pressed physical stop and guarded routing, then
continuously checks stop and lock freshness, zero limited and four-wheel
commands and stationary raw odometry. Each cycle confirms held joystick input
and its combiner source, disconnects only the configured controller, prompts
for a manual PS reconnect with neutral controls, verifies restored held input,
then a sustained neutral return and a quiet source. It never publishes commands
or permits and never restarts services. Snapshots, Bluetooth query results and
input messages preserve a failed phase as well as completed cycles.

[`observe_silent_controller_loss.py`](../../tools/hardware/observe_silent_controller_loss.py)
is the passive D3 silent-loss recorder. It requires an initially connected
controller, held forward input and active teleop source commands, a later
Bluetooth link drop, fresh neutral and zero reports within 1.5 seconds of the
last held joystick report, and zero mux and guard commands throughout a
pressed-stop trial. It logs when the held command was last seen at each stage
(`joy_teleop/joy`, `joy_teleop/cmd_vel`, `joy_teleop/cmd_vel_combined`,
`platform/cmd_vel`, `platform/cmd_vel_limited`). One operator stays beside the
physical stop while a second carries the still-held controller away.

### Controller-input diagnostic

[`diagnose_controller_input.py`](../../tools/hardware/diagnose_controller_input.py)
snapshots only controller-driver and ROS joystick process thread wait
channels, stacks and syscalls, input-device file handles, input inventory,
service state and the controller journal. It neither reads input-device data
nor grabs devices, sends signals, restarts services, writes Bluetooth settings
or publishes ROS commands. Root is needed to inspect the root-owned driver;
missing permissions are recorded as inspection errors. It is a diagnostic tool,
not a readiness or recovery pass.

With `--sample-states --operator-confirmed`, the physical stop pressed and the
exact temporary interlock installed, it verifies stationary stop and routing
before and after a 20-second operator input window. It compares cached
absolute-axis and button state of the kernel physical controller and the managed
virtual controller through read-only input ioctls, never consuming event
queues, grabbing devices or altering mapping. It requires exactly one physical
device with the configured MAC and one managed virtual device; ambiguity aborts.
Inspect `input-states.jsonl` to distinguish physical-report reception from
virtual-device forwarding. `--separate-release` waits up to 60 seconds for the
first held input, then records up to 20 more seconds, prompting for L1 release
while the forward stick stays displaced and recording whether both devices show
that release.

## Qualification evidence

[`driving_qualification.py`](../../tools/hardware/driving_qualification.py)
joins recorder results with physical observations through its `snapshot`,
`finalize` and `matrix` subcommands. The
[qualification procedure](driving_qualification_procedure.md#evidence-and-verdicts)
owns its rules.

## Archived evidence

- [Commissioning guard installation and first low-speed trials](../../archive/engineering/2026-10-06-driving-guard-and-deadman-trials.md)
  — first pipeline checks and the pulse helper's serialization fix.
- [E-stop QoS repair and stationary disconnect trials](../../archive/engineering/2026-10-09-estop-qos-and-disconnect-trials.md)
- [Interlock, bounded-permit and moving-disconnect trials](../../archive/engineering/2026-10-12-interlock-and-moving-disconnect-trials.md)
  — permit-trial acceptance changes and the disconnect-client correction.
- [Controller input loss, recovery and reconnect cycles](../../archive/engineering/2026-10-12-controller-input-loss-and-reconnect-cycles.md)
- [Original-manual profile and D4 handover trials](../../archive/engineering/2026-10-21-original-manual-and-d4-trials.md)
  — isolated and live handover runs and the motor-readiness gate.
