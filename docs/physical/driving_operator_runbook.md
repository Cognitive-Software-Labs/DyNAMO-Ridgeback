# Ridgeback driving operator runbook

This runbook holds the operator commands for driving checks on `r100_0160`:
read-only audits, commissioning-guard installation and rollback, the
physical-control observer, the forward-pulse helper, the live handover
recorder, controller checks and qualification evidence. It does not restate
what each tool measures or how a result becomes a verdict:

- [Driving router](driving_readiness.md#installed-state): the installed guard
  mode and its receipt.
- [Commissioning guard](commissioning_guard.md): speed modes, admission and
  the test-permit interlock.
- [Check and recorder tools](driving_check_tools.md): what each mode records,
  its flags, acceptance bounds and aborts.
- [Qualification procedure](driving_qualification_procedure.md): operator
  sequencing, stop-action classification and D-lane acceptance.
- [Controller input](controller_input.md) and
  [stop mechanisms](estop_stop_semantics.md).

## Before every session

- Work on Intel from the robot checkout with ROS sourced:

  ```bash
  cd /home/robot/DyNAMO-Ridgeback
  source /opt/ros/jazzy/setup.bash
  ```

- Every installer and recorder writes to a fresh directory below
  `artifacts/hardware/`, written here as `<run-dir>`. The tools refuse an
  existing directory. Keep failed and aborted runs.
- Start with the physical stop pressed, the controller neutral (sticks centred,
  L1 and R1 released), an operator beside the robot, a clear floor and the stop
  within reach. Recorders cannot stop the robot: press the physical stop
  immediately for any unexpected movement.
- Run one live robot action at a time. Each moving trial needs its own
  approval.
- Check the installed guard mode in the
  [installed state](driving_readiness.md#installed-state) before choosing a
  tool. The observer and the interlocked recorders accept only the default-caps
  or the test-interlock installation and refuse the original-manual profile.
  The forward-pulse helper checks the mode its flags name, and the live
  handover recorder needs the original-manual profile. To run interlocked
  checks from the original-manual profile,
  install the test interlock first and restore it afterwards
  ([guard installation](#guard-installation-and-rollback)).
- A completed recording is not a pass. Record what physically happened and join
  it with the recorder result through
  [qualification evidence](#qualification-evidence).

## Read-only and isolated checks

These collect graph and parameter evidence and exercise a private mux. They send
no commands to the robot and restart no services; neither establishes physical
driving readiness.

```bash
export ROS_LOG_DIR="$PWD/artifacts/hardware/<run-dir>/ros"
python3 tools/hardware/audit_driving_readiness.py \
  --output "$PWD/artifacts/hardware/<run-dir>/audit.json"
# Synthetic inputs confined to loopback, domain 213 and a private namespace:
python3 tools/hardware/check_driving_mux.py
```

The auditor selects the deployed domain 0 and `/etc/clearpath/cyclonedds.xml`
itself and labels missing evidence. The other isolated checks and their private
graphs are listed under
[isolated software checks](driving_check_tools.md#isolated-software-checks).

## Guard installation and rollback

The [installer](../../tools/hardware/install_commissioning_caps.py) owns the
four guard files and every mode change; the
[guard reference](commissioning_guard.md#installation-and-rollback) owns what it
verifies. Apply and restore need sudo, the robot stationary with the physical
stop pressed, controls centred and an operator beside it. The installer
observes that state, restarts only the platform and guard, sends no motion
commands and requests no camera, sensor or desktop restart. Without `--apply`
it only renders the files under `<run-dir>/staged/` for review.

First installation on a fresh host installs the default commissioning caps,
0.05 m/s planar magnitude and 0.1 rad/s yaw:

```bash
sudo python3 tools/hardware/install_commissioning_caps.py \
  --apply --operator-confirmed \
  --run-dir "$PWD/artifacts/hardware/<run-dir>"
```

A mode change or upgrade names the receipt whose recorded hashes match the
installed files; the current receipt is listed in the
[installed state](driving_readiness.md#installed-state). Add `--test-interlock`
for the temporary test lock, `--original-manual-speeds` for the original manual
profile, or neither for the default caps:

```bash
sudo python3 tools/hardware/install_commissioning_caps.py \
  --apply --operator-confirmed --test-interlock \
  --upgrade-from-run-dir "$PWD/artifacts/hardware/<receipt-matching-installed-files>" \
  --run-dir "$PWD/artifacts/hardware/<run-dir>"
```

Every installation verifies capped routing, the single guard publisher, the
reliable stop relay and the mux remap. Rollback restores the files saved by one
installation; run it from that installation's own receipt with the physical
stop still pressed:

```bash
sudo python3 tools/hardware/install_commissioning_caps.py \
  --restore --operator-confirmed \
  --run-dir "$PWD/artifacts/hardware/<run-dir-of-the-install-to-undo>"
```

- Restoring an upgrade or mode change returns the previous installation, for
  example the original-manual profile after a test-interlock session.
- Restoring a first installation removes the caps. It is not permission to
  drive.
- An unsuccessful apply leaves the platform stopped. An explicitly requested
  rollback uses the same receipt.
- Restore queues the platform start behind its clock gate. Confirm the active
  platform with a live probe, such as the
  [fast stationary batch](stationary_repairs_runbook.md#fast-stationary-health-batch),
  rather than an intermediate receipt label.

The test interlock blocks ordinary driving without a fresh test permit; it is
not a manual-driving mode. Inspect a new installation with the
[stationary observer](#stationary-observer-checks).

## Stationary observer checks

[`observe_driving_stop.py`](../../tools/hardware/observe_driving_stop.py)
publishes no commands in these modes, and the physical stop stays pressed
throughout. The commands show `--test-interlock`, which the `interlock-*` modes
always need. On a default-caps installation, omit it from the `stationary`,
`disconnect-stationary` and `e-stop-release` modes.

Passive state, including the software lock:

```bash
python3 tools/hardware/observe_driving_stop.py \
  --test stationary --test-interlock \
  --run-dir "$PWD/artifacts/hardware/<run-dir>"
```

**Stick mapping.** It needs the test interlock even on a default-caps
installation. Add `--operator-confirmed --mapping-axis 3` to check the
right stick (sideways), or `--mapping-axis 0` for the left stick (turning).
Keep driving-enable released. At `MAPPING READY`, move only the selected stick
sideways for one second and let go. The recorder waits up to 60 seconds, prints
when input arrives, then requires one second of neutral input. It ignores input
while driving-enable is held and waits for its release. Releasing the physical
stop or any wheel command or motion aborts immediately. A mapping mismatch is
not permission to try movement.

**Disconnect propagation.** For a wireless controller, with the stop pressed for
the whole check:

```bash
python3 tools/hardware/observe_driving_stop.py \
  --test disconnect-stationary --test-interlock --operator-confirmed \
  --run-dir "$PWD/artifacts/hardware/<run-dir>"
```

Hold the deadman and a small forward input, switch the controller off while
holding them, then release its controls. The observer waits up to 60 seconds
for input and records 20 more seconds; it aborts if the stop reports released
or a nonzero mux command appears. Reconnect afterwards with the stop pressed and
controls centred.

For a reproducible device-specific disconnect, reconnect the controller first,
then add `--disconnect-device <controller-MAC>`. At `READY`, hold driving-enable
and a small forward input until the recorder disconnects the device, then
release. Do not reconnect until the run is analysed. The command's exit status
is supporting evidence only; verify the result as described under
[disconnection behaviour](controller_input.md#disconnection-behaviour).

**Neutral reconnection.** Start with the controller disconnected:

```bash
python3 tools/hardware/observe_driving_stop.py \
  --test interlock-reconnect --test-interlock --operator-confirmed \
  --reconnect-device <controller-MAC> \
  --run-dir "$PWD/artifacts/hardware/<run-dir>"
```

After ten seconds of disconnected state the recorder prompts for a manual PS
press with every driving control centred and released. It records thirty more
seconds, then checks the connection, fresh neutral input and the asserted stop.
Preserve the driver journal for the actual disconnect and reconnect times. This
does not cover silent radio loss, reconnection with the stop released or a
multi-minute outage.

## Stop release without motion

Both checks release the physical stop while the robot must stay still. They
send no commands or permits.

**Neutral release:**

```bash
python3 tools/hardware/observe_driving_stop.py \
  --test e-stop-release --test-interlock --operator-confirmed \
  --run-dir "$PWD/artifacts/hardware/<run-dir>"
```

At `RECORDING READY`, release only the physical stop and leave the controller
untouched. The observer records ten seconds after the released report, then
prompts you to press the stop again and waits for two asserted reports. Held
input during release and wireless reconnection are separate checks.

**Held input against the software lock:**

```bash
python3 tools/hardware/observe_driving_stop.py \
  --test interlock-held --test-interlock --operator-confirmed \
  --run-dir "$PWD/artifacts/hardware/<run-dir>"
```

At `READY`, release only the physical stop with controls centred. After
`PHYSICAL RELEASE RECORDED`, gently hold driving-enable and forward for at most
one second, release the controls and press the stop again. The robot must stay
still throughout.

## Bounded moving trials

Moving trials run under the installed test interlock: 0.05 m/s planar, 0.1 rad/s
yaw and at most one second of motion per physical stop cycle. Follow the
[operator sequencing](driving_qualification_procedure.md#operator-sequencing):
release only the stop at `RECORDING READY`, touch the controller only after
`PHYSICAL RELEASE RECORDED`, make one movement in the selected direction, and
press the stop at the prompt. Neutral means axes 0, 1, 3 and 4 centred and both
driving-enable buttons released; unused axes may rest at nonzero values.

**Bounded permit:**

```bash
python3 tools/hardware/observe_driving_stop.py \
  --test interlock-permit --test-interlock --operator-confirmed \
  --permit-motion-confirmed \
  --run-dir "$PWD/artifacts/hardware/<run-dir>"
```

Hold driving-enable and the direction until the stop prompt, about 1.25 seconds
after first input; the guard must block commands after at most one second. At
the prompt, release the controls and press the stop. Press it at once if the
robot is still moving at the prompt.

- `--direction backward|left|right|turn-left|turn-right` selects one other
  approved direction per run. Right-stick horizontal moves sideways and
  left-stick horizontal turns; keep the other stick centred. The flag changes
  only the recorder's expected direction, not the guard.
- `--stop-action deadman` (forward only): release L1 as soon as movement is
  felt while the stick stays displaced, then centre the stick.
- `--stop-action e-stop` (forward only): press the physical stop as soon as
  movement starts, while still holding the controls.

The command and odometry acceptance bounds, including the incidental yaw
tolerance, are listed under
[permit trials](driving_check_tools.md#physical-control-observer).

**Moving controller disconnect.** Separately approved, after bounded-permit
stopping is qualified:

```bash
python3 tools/hardware/observe_driving_stop.py \
  --test interlock-disconnect --test-interlock --operator-confirmed \
  --permit-motion-confirmed --moving-disconnect-confirmed \
  --disconnect-device <controller-MAC> \
  --run-dir "$PWD/artifacts/hardware/<run-dir>"
```

The recorder checks the installed interlock and the connected device before
`READY`. After the release is recorded, gently hold forward and driving-enable.
It requests one device-specific Bluetooth disconnect half a second after the
first nonzero mux command. Keep the controls held until the disconnect, then
release them, press the stop at the prompt and do not reconnect. A stop only at
the one-second budget boundary does not qualify disconnect handling. Review
driver logs, transport status, input reset, wheel commands and the physical
observation. The un-interlocked `--test disconnect --disconnect-device` stays
refused, and a deliberate disconnect does not simulate silent radio loss.

**Raised-wheel feedback fault.** A qualified helper first secures all four
wheels clear of the floor on rated stands:

```bash
python3 tools/hardware/observe_driving_stop.py \
  --test interlock-feedback-fault --test-interlock --operator-confirmed \
  --permit-motion-confirmed --feedback-fault-confirmed \
  --run-dir "$PWD/artifacts/hardware/<run-dir>"
```

After the release is recorded, gently hold driving-enable and forward until the
stop prompt. After measured forward wheel motion the recorder publishes one
empty feedback batch; observe whether the wheels stop by themselves before
pressing the stop. This does not interrupt CAN or measure loaded stopping
distance. Restore the controller afterwards with the
[stationary recovery](puma_feedback_runbook.md#stationary-feedback-interruption-checks),
stop still pressed. The raised-wheel D6 and D7 link-fault commands are in
[motor-link fault injection](motor_link_fault_injection.md#raised-wheel-trial-tool).

**Un-interlocked stop observations.** On the default-caps installation only,
`--test deadman`, `--test disconnect` or `--test e-stop` with
`--operator-confirmed` record an operator's stop action during capped manual
driving and publish no permits. The observer waits up to 60 seconds for the
first nonzero mux command and records ten more seconds. Gently request forward
for at most one second and perform the selected action. For deadman release,
release the button first, then centre the stick: centring alone tests a zero
command. Re-engage the stop afterwards and reconnect a controller only with the
stop engaged.

## Forward-pulse helper

[`commissioning_forward_pulse.py`](../../tools/hardware/commissioning_forward_pulse.py)
publishes its own bounded command on the external mux input. An operator
confirms a clear level floor, nobody in the path and the stop within reach;
leave the joystick and RC controls untouched. Run the passive default first,
with the stop pressed:

```bash
python3 tools/hardware/commissioning_forward_pulse.py \
  --run-dir "$PWD/artifacts/hardware/<run-dir>"
```

After it succeeds, the operator releases the stop and approves one movement:

```bash
python3 tools/hardware/commissioning_forward_pulse.py \
  --execute --operator-confirmed \
  --run-dir "$PWD/artifacts/hardware/<run-dir>"
```

After five seconds of passive checks it requests 0.05 m/s forward for at most
one second, then sends zeros; the nominal travel is 5 cm. Press the stop for
unexpected movement, re-engage it after the test, and record the observed
direction and stop separately from odometry.

- The default mode verifies the default-caps installation;
  `--test-interlock` and `--original-manual-speeds` verify the other modes and
  restrict the motion they allow.
- `--axis backward|left|right|turn-left|turn-right` selects one approved
  direction per run; turning peaks at 0.1 rad/s.
- `--ramp` adds a 250 ms rise and fall inside the same one-second pulse.
- `--ramp --stop-mode timeout` ends publication at the peak and observes one
  second of silence: publication loss, not a joystick disconnect.
- `--stop-mode physical-e-stop --physical-stop-confirmed` runs only on the
  original-manual guard, forward and ramped.

The [forward-pulse reference](driving_check_tools.md#forward-pulse-helper)
owns each mode's checks. Do not repeat a motion abort merely to obtain a pass:
preserve the recording and examine the [wheel feedback](drive_feedback_diagnostics.md)
first.

## Live handover recorder

The D4 recorder runs on the installed original-manual guard and drives the live
mux from two synthetic forward sources at 0.025 and 0.045 m/s. Leave the
controller untouched. Run the read-only preflight with the stop pressed, then
the motion run in a fresh directory:

```bash
python3 tools/hardware/check_live_driving_handover.py \
  --preflight-only --operator-confirmed \
  --run-dir "$PWD/artifacts/hardware/<run-dir>"
python3 tools/hardware/check_live_driving_handover.py \
  --operator-confirmed --bounded-motion-confirmed \
  --run-dir "$PWD/artifacts/hardware/<run-dir>"
```

Release the stop when prompted and press it at the end; a run that times out
waiting for that press is recorded as aborted. The
[live handover reference](driving_check_tools.md#live-handover-recorder) owns
its gates.

## Controller checks and recovery

The configured controller MAC is listed in `/home/robot/controllers.txt`; the
deployed controller on `r100_0160` is `AC:36:1B:89:A1:0A`.

**Input diagnostic.** For a controller that is Bluetooth-connected but delivers
no physical reports or ROS input, keep the stop pressed and collect the
read-only diagnostic. Root is needed to inspect the driver process:

```bash
sudo python3 tools/hardware/diagnose_controller_input.py \
  --run-dir "$PWD/artifacts/hardware/<run-dir>"
```

With the test interlock installed, add `--sample-states --operator-confirmed`
to compare cached physical and virtual device states over a 20-second input
window, and `--separate-release` to prompt for an L1 release while the stick
stays displaced. Inspect `input-states.jsonl` and any recorded permission
errors before claiming recovery. The
[diagnostic reference](driving_check_tools.md#controller-input-diagnostic) owns
what it reads.

**Connected-but-no-input recovery.** Keep the stop pressed and the software test
lock active where installed. Preserve the diagnostic first, then disconnect only
the identified controller:

```bash
bluetoothctl --timeout 2 disconnect <controller-MAC>
bluetoothctl --timeout 1 info <controller-MAC>
```

Confirm `Connected: no`, then press PS once with the controls centred and both
driving-enable buttons released. Verify actual input and its return to neutral
with a fresh [stick-mapping recording](#stationary-observer-checks). Bluetooth
connection or zero ROS messages alone do not show live input. The
[failure reference](controller_input.md#connected-but-no-input-failure) owns
the symptoms and the unresolved cause.

**Reconnect cycles.** With the test interlock installed, the stop pressed and
exactly one configured controller:

```bash
python3 tools/hardware/check_controller_cycles.py --operator-confirmed \
  --cycles 3 --run-dir "$PWD/artifacts/hardware/<run-dir>"
```

Follow the prompts: hold input, let the recorder disconnect the controller,
reconnect with PS and neutral controls, hold input again, then return to
neutral.

**Silent radio loss (D3).** One operator stays beside the robot with the stop
pressed; a second holds L1 and forward and walks away with the controller until
its link drops, keeping both held:

```bash
python3 tools/hardware/observe_silent_controller_loss.py --operator-confirmed \
  --run-dir "$PWD/artifacts/hardware/<run-dir>"
```

The recording lasts 120 seconds by default (`--duration`). Installation and
rollback of the controller autoconnect helper are in the
[desktop and controller setup repair](stationary_repairs_runbook.md#desktop-and-controller-setup-repair).

## Qualification evidence

[`driving_qualification.py`](../../tools/hardware/driving_qualification.py)
joins recorder results with physical observations and never changes robot
state. Outputs must be fresh paths below `artifacts/hardware/`.

```bash
python3 tools/hardware/driving_qualification.py snapshot \
  --output "$PWD/artifacts/hardware/<snapshot-dir>/baseline.json"
python3 tools/hardware/driving_qualification.py finalize \
  --run-dir "$PWD/artifacts/hardware/<recorder-run-dir>" \
  --lane D2 --mechanism deadman --verdict passed \
  --observation "<what the operator saw>" \
  --baseline "$PWD/artifacts/hardware/<snapshot-dir>/baseline.json"
python3 tools/hardware/driving_qualification.py matrix \
  --output "$PWD/artifacts/hardware/<matrix-dir>/matrix.json"
```

`finalize` also takes `--video <file>`, `--recovery <text>`,
`--final-mode <text>` and repeated `--condition NAME=VALUE` and
`--measurement NAME=VALUE`. Values `true` and `false` become booleans and
numbers become floats; a D4 run counts toward the matrix only with
`--condition final_configuration=true`. The verdict is one of `passed`,
`failed`, `inconclusive` or `aborted`. The matrix counts these mechanism names:

| Lane | `--mechanism` values |
|---|---|
| D1 | `forward`, `backward`, `left`, `right`, `turn-left`, `turn-right` |
| D2, D8 | `command-loss`, `deadman`, `physical-e-stop` |
| D3 | `silent-radio-loss`, and `reconnect-cycle` with `--measurement cycles=<n>` |
| D5 | `stationary-field-freshness` |
| D6 | `feedback-loss` |
| D7 | `command-link-loss` |

The [evidence rules](driving_qualification_procedure.md#evidence-and-verdicts)
own what each lane needs, and the
[stopping-distance procedure](driving_qualification_procedure.md#stopping-distance-measurement)
lists the D8 conditions and measurements.
