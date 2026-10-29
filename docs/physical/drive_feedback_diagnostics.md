# Ridgeback wheel feedback and diagnostic qualification

<!-- doc-budget: 908 lines; reason: split pending into current-state pages and archive records -->

The [driving qualification](driving_readiness.md) remains open. Command caps
bound requested velocities; they do not establish synchronized wheel response,
physical stopping distance or truthful diagnostic decoding.

## Fault flags

The installed `clearpath_motor_msgs/PumaStatus` defines masks 1, 2, 4 and 8 for
current, temperature, bus voltage and bridge-driver faults. The matching
Clearpath 2.9.4 Puma node uses those masks as bit-shift positions when decoding
its human-readable diagnostics. For example, shifting fault 8 by mask 8 yields
zero, so `Bridge Driver Fault: False` contradicts the raw fault field. Use the
raw mask until corrected; this reporting defect does not establish the reason
for a motor fault.

The maintained [diagnostic-only patch](../../patches/clearpath_puma_fault_flags.patch)
tests each mask with a bitwise AND instead. It was **installed and verified
stationary on 2026-10-12** using the corrected pinned build described below.
The [offline verifier](../../tools/hardware/verify_puma_fault_patch.py) checks the
downloaded 2.9.4 source hash, applies the patch to a temporary tree, and compiles
the extracted corrected expression against all 256 fault values and four
masks. An isolated full package build with that patch also completed; neither
check installs or qualifies the rebuilt driver on the robot. Control, gains,
firmware and diagnostic rate targets are unchanged.

The isolated build exposed an additional upstream bounds warning: analog-input
access uses field index 15 in a status-cache array of length 15. The protocol
index mask permits values 0–15, while several other cache arrays are smaller
and also lack bounds checks. This is a source-level memory-access defect,
not established as the cause of any observed motion transient. The original candidate
was not suitable for deployment. The corrected cache patch and verified full
build are described below; live deployment remains pending. The build,
source archive hash and warning are preserved under
`artifacts/hardware/20261006-puma-diagnostic-build/`.

Source ownership is the official
[2.9.4 node](https://github.com/clearpathrobotics/clearpath_robot/blob/2.9.4/clearpath_motor_drivers/puma_motor_driver/src/multi_puma_node.cpp)
and [label definitions](https://github.com/clearpathrobotics/clearpath_robot/blob/2.9.4/clearpath_motor_drivers/puma_motor_driver/include/puma_motor_driver/multi_puma_node.hpp).
The inspected node source hash is
`d156ed91b28d5526de64c625cad7555b7728f54c7c1e020c9b341bbabccda89b`.
Before a future package upgrade or fresh-host reconstruction, re-evaluate that
patch against the selected source; do not overwrite apt executables or source a
mutable checkout into the live platform service. Any rebuilt deployment needs
its own pinned install, provenance, stationary verification and rollback.

## Corrected pinned repair build

The October 9 candidate was withdrawn before deployment: its receipt listed
patches, but the source used for compilation still contained the upstream code.
`git apply` silently skipped paths because the exported source lived below this
repository's ignored artifact directory. The compiler's array-bounds warning
and unchanged source hashes exposed this build error. The isolated sanitizer
harness had applied its patch in a separate temporary tree and remained valid;
it did not establish patch application to the full compiled package.

The corrected builder applies patches directly to the exported tree with
`patch`, checks forward application and reverse applicability, and repeats
applied-patch verification before staging. A regression test reproduces the
old silent skip inside an ignored nested Git tree. The installer rejects old
candidates without verified patch-application provenance before service actions.
The maintained cache-safety patch sizes all protocol-indexed caches to sixteen
entries, reads fixed-point values through aligned `memcpy`, selects exact API
classes and zero-pads validated short payloads. It does not change gains,
firmware, diagnostic rate targets or per-wheel freshness policy.

The corrected 2026-10-12 package build completed with no compiler stderr;
1,013 cache/message checks, sixteen alignment checks and 1,024 fault-mask cases
passed. The old upstream harness still reproduces the bounds/alignment defects.
The new source/binary checksums, sanitizer outputs and build logs are under
`artifacts/hardware/20261012-puma-repair-candidate/`. Its deployment was installed and verified stationary on 2026-10-12, with
receipt under `20261012-puma-repair-install/`. The root-pinned package
and three owned helper/drop-ins preserve whichever exact capped/test-interlocked
installation was verified at staging; changed caps or dependencies block apply.
The DDS interface-readiness helper is included but actual cold-boot recovery
remains unqualified. Public commands and rollback live in the root README.

The 2026-10-12 stationary capture (`20261012-stationary-wheel-feedback`) received
380 aggregate feedback samples at approximately 20 Hz, with a maximum receive
gap of 51.9 ms. Each message included all four wheel identities, with zero
reported speeds and zero output voltages throughout. Raw motor status fault
was 8, while installed human-readable bridge-fault fields reported false,
reconfirming the decoding inconsistency. This does not identify a hardware
fault cause, prove per-wheel cache freshness or qualify driver recovery.
The frequency warning remains a separate request/reply and freshness investigation.

## Installed repair verification

The running driver executable is now under the root-owned prefix
`/opt/dynamo-puma/76ff6b99a1e35cc56e27e07302aa98f94ebb85f809348e25eaab04a50ef49856/`.
Its full package manifest and three installed helper/drop-in hashes match the
verified stage. The existing test-interlocked caps remain byte-identical to
their baseline. Only platform and guard were restarted; installation recorded
no sensor, camera or VNC service changes. Rollback uses the same installation
receipt through the maintained installer; it restores the previous capped
platform without modifying the test-lock files.

The independent `20261012-puma-repair-postinstall` recording received 380
aggregate feedback messages at approximately 20 Hz; every wheel identity was
present and every speed was zero. All 19 physical-stop and 396 software-lock
reports were asserted. Human-readable bridge-driver fault is now true,
consistent with raw mask 8 while physically stopped. This demonstrates correct
mask decoding, not the cause of that status or a new physical motor fault.
Diagnostics still measured approximately 20 Hz for the front and rear-left
wheels and 17.8 Hz for rear-right against the unchanged 40 Hz target. The cache
freshness, rate asymmetry, sudden-start behavior, reconnect handling and actual
cold-boot gates remain open. No movement was performed for this verification.

## Rates and freshness

Three different rates must be distinguished:

- The Puma node timer defaults to 20 Hz when `frequency` is absent from the
  deployed configuration; it limits aggregated feedback publication.
- Individual fields arrive through CAN and are cached independently. An
  aggregate message timestamp does not prove each wheel/field was just updated.
- The hardcoded 40 Hz diagnostic target applies to selected CAN message classes
  counted by `Driver::processMessage`, not the ROS aggregate publication rate.
  In particular, status-class speed fields and speed-control setpoint replies
  take different paths through this counter.

The [2.9.4 driver](https://github.com/clearpathrobotics/clearpath_robot/blob/2.9.4/clearpath_motor_drivers/puma_motor_driver/src/driver.cpp)
and the node source above own these semantics. The diagnostic target must not
be changed to 20 merely because the ROS topic arrives at 20 Hz. Measure the
counted CAN classes, request/reply origin, per-device loss and cache freshness
before choosing a fix. A five-second passive vcan capture and a ten-second ROS
capture are preserved under `artifacts/hardware/20261006-driving-readiness/`;
they show unequal per-device setpoint traffic and do not close that investigation.

The node's aggregate readiness check ORs received-field bits across all wheels.
It can therefore pass without every field being newly received for every wheel.
Reading cached speeds also clears their received flags. This is a source-level
freshness limitation; it does not by itself prove the cause of a live transient.

The paired passive capture `20261012-wheel-can-rate` distinguishes actual
speed-state traffic from the diagnostic counter after the patched deployment.
Over 14.976 seconds, all four devices received 384 status-speed frames each
(about 25.64 Hz), with maximum receive gaps at most 40.1 ms. The diagnostic's
counted incoming control classes consisted of speed-setpoint API `0x0480`:
300 replies per front wheel, 295 rear-left and 245 rear-right, about 20.03,
19.70 and 16.36 Hz respectively. Their maximum receive gaps were about 51.5,
100.4 and 200.5 ms. Outgoing speed-setpoint commands and empty setpoint queries
were each about 20.03 Hz for every device. Counts use the full capture duration;
RX/TX distinguish host incoming frames from locally transmitted traffic.

This establishes incoming setpoint-response asymmetry despite equal outgoing
traffic in a stationary sample; it does not establish the loss location or prove
that physical speed feedback stalled. The separate speed-state stream stayed
steady for every wheel. The 40 Hz diagnostic target is still not automatically
changed to 20: reply/query/command semantics and firmware expectations must be
reconciled first. Aggregate ROS freshness also cannot establish each cached
field's age; the OR-across-wheels readiness limitation remains open. These
interface receive timestamps are not physical wire timestamps and the test
qualifies no under-motion latency or physical wheel response.

## Offline freshness candidate and deployment boundary

The separate `clearpath_puma_freshness.patch` remains **offline and not
installed**. It requires every wheel's feedback fields instead of OR-ing flags
across wheels, resets status-field completeness for each wheel, and runs
consuming diagnostic getters on a copied driver cache so diagnostics do not
clear publication flags. This establishes receipt since consumption, not an
absolute field-age bound or simultaneous physical wheel response.

The offline verifier executes the actual extracted node/diagnostic routine
bodies with inert per-wheel caches. The current source falsely accepted twenty
single missing-feedback cases, fifteen missing-status cases and a distributed
partial-field batch; diagnostics also consumed feedback availability. The
candidate rejected all such incomplete batches and preserved diagnostic reads.
Complete batches remained consumable once. Source/patch hashes and results
are preserved under `20261012-puma-freshness-offline/routine-tests-3`.
The full candidate package compiled with empty compiler stderr; it was neither
installed nor included in the default two-patch builder/deployment path.

A flag-only timing model replayed the passive `20261012-wheel-can-rate` capture
at a 20 Hz publish timer over ten phases, assuming diagnostics every second and
excluding two seconds of warmup. Baseline projected output was 18.96–19.04 Hz
with forty-three batches per phase containing unreceived wheel fields. Candidate
output was 16.41–16.49 Hz with none of those batches, but its maximum projected
publication gap increased to 200 ms. Maximum modeled cached-field age decreased
from about 197 ms to 97 ms. These projections are not measurements of a live
patched driver; the model includes no physical wheel timing, transport queue,
full driver state machine or hardware-consumer behavior. A synthetic missing
wheel reply correctly produces a long candidate publication gap rather than
borrowing other wheels' freshness.

An additional **offline-only policy model** requires each wheel's position and
speed to have arrived since the previous publication, while bounding the age
of every cached field to 200 ms. Replaying the same stationary capture over ten
timer phases projected 19.96–20.04 Hz, with a 50 ms maximum publication gap
and 196.8 ms maximum cached-field age. Synthetic loss of one wheel's speed
produced a 400 ms publication gap; loss near the end of a trace is also counted
as silence. This is a scheduling model in
`tools/hardware/replay_puma_freshness.py`, with output under
`artifacts/hardware/20261013-motion-field-replay-2.json`; no driver patch or
robot configuration was changed. The observed maximum is only about 3 ms below
the proposed age limit, and the capture was short and stationary. A longer,
under-motion capture and actual driver/consumer validation are required before
selecting or deploying this policy.

A second passive capture on 2026-10-13 lasted 29.982 seconds with the physical
stop pressed. All four wheels had 769 speed replies each, with maximum receive
gaps of about 40 ms; rear-right setpoint replies numbered 523, with a maximum
gap of about 151 ms. The replay projected 19.98–20.01 Hz and a 50 ms maximum
gap for the motion-field model, with maximum cached-field age about 150 ms.
The all-fields-new model projected 12.72–17.33 Hz across timer phases. The
capture and replay are under `artifacts/hardware/20261013-wheel-can-30s/`.
`candump -L -x` on this host wrote compact `CANID#DATA R/T` lines, so the
replayer now accepts both that format and the earlier verbose log format.
These are host-observed stationary traffic and timer projections, not measured
motion response or evidence that the candidate is safe to install.

Therefore do not deploy the freshness candidate merely because its routine
checks pass. Verify the Ridgeback hardware consumer's handling of absent/stale
aggregate feedback and choose explicit age bounds and diagnostics, preserving
the incoming response asymmetry as evidence. Keep firmware/control rates,
command caps and the live installed two-patch driver unchanged until that
consumer/deployment gate is resolved. The controller reconnect investigation
and cold-boot checks are independent of this candidate.

## Bounded field-age driver candidate

The additional `clearpath_puma_field_age.patch` and
[receipt policy](../../tools/hardware/puma_field_freshness.hpp) implement an
**offline, speed-mode-only candidate**, layered after the two deployed safety
patches and the flag-completeness candidate. The live driver is unchanged.
Every wheel must have new position and speed since the last publication, and
all five feedback fields must have valid payload widths and bounded monotonic
receipt ages. Duty/current use two-byte payloads; position/speed/speed-setpoint
use four. Invalid or regressing receipt metadata cannot renew readiness. Slow
auxiliary fields may be reused within the explicit age bound instead of forcing
all five fields to arrive again on every 20 Hz output tick.

The receive callback timestamps each frame before queueing. The queue carries
that timestamp into the decoder; dequeuing late or stamping a new ROS aggregate
does not make an old field fresh. The queue's empty check and pop share its
mutex. All four wheels are checked before building an aggregate, and new-motion
flags are consumed only after the publish call returns. Diagnostics inspect
the cache without consuming those flags and expose the oldest cached field
age; −1 denotes missing/invalid data. This measures age since the host receive
callback, not the MCU or physical CAN-wire sample time.

[The offline builder](../../tools/hardware/stage_puma_field_candidate.py)
reconstructs pinned Clearpath 2.9.4 from the verified archive, applies the
maintained patch chain, copies the policy header and builds into its evidence
directory. It executes only a checker using real driver cache/decoder methods
with null transport/node handles; the motor node is compiled but never
started. Separate extracted-node tests exercise missing-wheel fields, failed
publication, flag consumption, and a queued frame aged beyond the bound.
The complete package and these checks passed under
`artifacts/hardware/20261016-puma-field-age-candidate-3/`; standalone policy
checks are under `20261016-field-age-policy/`.

The candidate first used **200 ms for experiments**, following the earlier
replay model; the split bounds below supersede it. A production bound is not selected. Loaded/under-motion captures,
actual ROS/receive-thread timing, consumer interaction and a supervised pinned
deployment remain open. The default Puma builder/installer still uses only the
two deployed safety patches. This builder emits `candidate.json`, not a
deployment `SOURCE.json`, and performs no installation or service changes.

A fresh, passive 30-second stationary `vcan0` capture on October 16
(`artifacts/hardware/20261016-one-hour-field-capture/`) recorded 14,813 relevant
received fields. Its 20 Hz timer replay projected 19.97–20.00 Hz for the
200 ms candidate, but wheel 3's setpoint field had a 199.83 ms receive gap.
The payload-validated replay in `replay-invalidating.json` found zero malformed
relevant RX frames in that capture and produced the same projection. The replay
now rejects wrong payload widths and models a malformed recognized field as
invalidating the candidate cache; its test injects a bad speed frame and
verifies the resulting publication gap. It remains an offline timer model: it
does not execute the ROS queue, measure a loaded callback delay, or establish
physical wheel synchronization.
This leaves almost no margin for callback or scheduler delay at the proposed
limit. Position and speed field gaps stayed below 40 ms in this capture.
An ideal-timer sensitivity replay (`replay-age-sensitivity.json`) also projected
19.93–19.97 Hz with a 150 ms bound, but its worst publication silence was
100 ms rather than 50 ms at 200 ms. These are experimental bounds only; the
stationary capture cannot establish the delay distribution under motion/load
or a safe production bound relative to the installed 250 ms consumer timeout.
The experiment supports separate evaluation of slow setpoint and fast motion
fields; it does not justify selecting or deploying a production age bound.

A second passive 30-second capture on October 16
(`artifacts/hardware/d5-passive-20261016T104846Z/`) ran while Thor's failure
observer was active. All 14,794 relevant RX frames had expected payload
widths. Wheel 3's setpoint receive gap reached **250.073 ms**, while its
position and speed gaps stayed near 40 ms. At the experimental 200 ms bound,
the timer replay projected 19.93–19.97 Hz and a worst 100 ms aggregate-output
silence; the installed driver and physical wheels were not changed or moved
by this capture. `replay-complete.json` now carries all twenty per-wheel field
gap rows and 150–250 ms experimental age-limit sweeps in one repeatable output.
This is concurrent-workload evidence, not a controlled CPU
load or moving-wheel qualification. It reinforces that the production bound
must be chosen from the actual field-delay distribution and consumer behavior,
not the earlier 199.83 ms stationary maximum alone.

`tools/hardware/puma_policy_chain_check.cpp` exercises the maintained
driver-field and consumer policies together without ROS or CAN. A synthetic
slow setpoint with one withheld aggregate tick leaves the 250 ms consumer
healthy; a long single-wheel speed gap suppresses aggregates, trips and
latches the consumer, and requires explicit activation after fresh data. This
verifies policy composition only. A moving capture uses
`tools/hardware/capture_wheel_field_trial.py`: it starts passive `candump`,
then runs the existing bounded forward observer and preserves its evidence
alongside CAN timing. Its wrapper never certifies a physical stop from terminal
exit alone. The wrapper requires actual CAN frames before starting the observer
and cancels the observer if the CAN capture process exits during the trial.
No candidate driver deployment follows automatically.

The same capture wrapper accepts `--source pulse` for a joystick-independent
screening capture. It runs the existing guarded forward-pulse tool under the
temporary test interlock: the physical e-stop must be reported released, the
controller must remain neutral, and one ramped forward command lasts at most
half a second at 0.05 m/s before source silence. The operator remains beside
the robot with the physical stop reachable and presses it after the pulse.
The wrapper starts passive CAN capture first and cancels the pulse if capture
dies. A half-second of motion can confirm that moving field receipts exist; it
cannot establish a production field-age bound under sustained motion or load.
The same wrapper also accepts `--source handover` to capture the live D4
two-source forward handover under the installed original-manual mode. Its
requests remain 0.025 and 0.045 m/s, with the physical stop and guard active;
the wrapper only adds passive CAN recording. This path requires a separate
supervised run and operator confirmation of actual motion and stopping.
For the installed original-manual guard, `--source manual-pulse` selects a
one-second, ramped 0.05 m/s forward diagnostic. It requires a fresh pressed
physical stop and neutral controller before release, checks the installed
manual-mode files and live routing, records motor commands and wheel feedback,
then sends zeros and waits for the red stop to be pressed again. It is useful
when the shorter test-interlocked pulse or low-speed source handover reports
zero raw odometry. The CAN wrapper still needs an operator to confirm visible
motion and stopping; a completed trace alone does not establish travel.
The first such run under `artifacts/hardware/d5-manual-pulse-20261021/`
confirmed the one-second 0.05 m/s request reached the mux, guard, wheel-command
topic and outgoing CAN speed-set frames. The operator saw no movement. During
that command, all four incoming motor setpoint fields remained zero and their
fault fields reported bit 8 (`FAULT_BRIDGE_DRIVER` in PumaStatus). The same
bit cleared in the passive CAN recording several seconds after physical-stop
release, then returned after the stop was pressed. A separate read-only ROS
status sample with the stop pressed showed all four motors in speed mode with
fault 8. These observations indicate the pulse was sent before the motor
controllers reported ready; they do not establish a motor defect or a moving
D5 field-age bound. The bounded pulse and handover recorders now wait for all
four speed-mode motor statuses to report zero faults for one second after stop
release before sending motion. They abort without commanding if readiness does
not arrive within twenty seconds.
The repeated pulse under `artifacts/hardware/d5-manual-pulse-20261021-2/`
waited 6.78 s after physical-stop release for stable zero-fault speed-mode
status, then sent the same one-second 0.05 m/s request. The operator saw a
forward movement and self-stop before pressing the red stop. Raw odometry
peaked at 0.0511 m/s; the final stop was recorded pressed. The aligned
[motion-window analyzer](../../tools/hardware/analyze_puma_motion_window.py)
found 466 valid relevant CAN receipts and zero malformed receipts during the
0.959 s nonzero-command window. All twenty wheel/field groups were present;
the largest field silence including window edges was 150.2 ms for one rear
wheel's setpoint. Nineteen ROS feedback aggregates arrived during that window,
with a maximum 50.4 ms gap. All sampled motor statuses were fault-free in
speed mode. This establishes one short moving capture with a working
motor-readiness sequence. It does not measure internal queue age, loaded or
sustained motion, or justify the experimental 200 ms field-age bound.
The successful two-source D4 floor handover under
`artifacts/hardware/d4-handover-can-20261021-6/` gives a second short moving
window. The same analyzer aligned its ROS command stamps to the passive CAN
host-receive times: 835 valid relevant field receipts, zero malformed,
all twenty wheel/field groups present over 1.695 s of nonzero limited output.
The largest per-field silence including window edges was 150.3 ms, and all
three sampled motor statuses in that interval were fault-free in speed mode.
Raw forward odometry peaked at 0.0465 m/s. The operator observed forward-only
motion and a self-stop, with the final physical stop recorded pressed. This
supports short moving-frame delivery through a live priority switch; the
handover recorder did not sample ROS feedback aggregates or motor command
messages, and neither capture measures internal queue delay or loaded
recurrence.

The [capture-chain replay](../../tools/hardware/check_puma_capture_chain.py)
also runs the maintained C++ field-age and consumer policy classes against two
real 30-second passive CAN recordings. At the experimental 200 ms limit, its
50 ms timer phases produced 599–600 aggregates in the quiet capture and 598 in
the concurrent-workload capture, with maximum gaps of 50 and 100 ms and no
consumer fault. Removing one wheel's speed receipts from each recording caused
the consumer to fault and stay latched. The saved results are
`artifacts/hardware/d5-policy-replay-20261021-a6/` and `-b6/`.
This executes the policy classes over observed receipt timing, but the timer is
idealized and the robot did not use the candidate driver. Injecting a fixed
125 ms processing delay into the loaded capture reduced output to 512–587
aggregates and produced a 250 ms maximum gap, exactly at the consumer timeout;
at 175 ms delay no phase produced an initial aggregate. This is a
modelled delay, not a measured ROS queue delay. A moving/load capture and
controlled ROS receive-queue run still determine whether 200 ms has enough
margin for deployment.

Three supervised D5 capture attempts on 2026-10-21 preserved real CAN traffic
under `artifacts/hardware/d5-moving-capture-20261021/`, `-2/`, and `-3/`, but
none armed a movement publisher: the first saw no valid forward input within
15 seconds of stop release; in the latter two the physical stop remained
pressed when forward input arrived. They therefore add stationary timing
evidence only. The third capture had 25,282 valid relevant field receipts and
no invalid receipts. Its maintained-policy replay at the proposed 200 ms
bound produced at least 1,024 aggregates over the sampled 50 ms timer phases,
with a maximum 100 ms publication gap and no modelled consumer fault; removing
one wheel's speed receipts latched the consumer fault. Evidence is in
`artifacts/hardware/d5-attempt-3-policy-replay-20261021/`. Neither this replay
nor the operator's stationary trials qualify feedback timing under motion or
justify deploying the candidate bound.

On 2026-10-21 the joystick-independent pulse wrapper completed one guarded
half-second command and passive CAN capture under
`artifacts/hardware/d5-pulse-capture-20261021/`. The pulse recorder reported
the physical stop released before command publication, a 0.05 m/s requested
peak, zero motor commands about 98 ms after source silence, no subsequent
nonzero command, final stationary odometry, and the physical stop pressed.
All twenty wheel/field groups had valid CAN receipts in the aligned half-second
command window (247 relevant receipts, no malformed receipts); a one-second
window before and after also contained all groups. The window report is under
`artifacts/hardware/d5-pulse-motion-window-20261021/`. In the full capture,
the maintained-policy replay at a 200 ms field-age bound produced at least
500 aggregates over the timer phases, with a maximum modeled gap of 150 ms
and no modeled consumer fault; its result is under
`artifacts/hardware/d5-pulse-policy-replay-20261021/`. ROS feedback first
showed only a small wheel-speed response after the command window, while raw
odometry during the command was zero. Physical travel still requires the
operator's observation. This brief pulse does not establish sustained
under-motion timing, actual ROS receive-queue delay, or a production bound.

A separate [read-only ROS delivery recorder](../../tools/hardware/measure_feedback_ros_delivery.py)
observed the **installed** driver's aggregate output for 30 seconds with the
physical stop reported pressed and raw odometry zero. Its 601 aggregates had
0.31 ms median, 0.40 ms 95th percentile and 5.06 ms maximum elapsed time
between the driver's aggregate-generation header stamp and this observer's
callback. The longest observer arrival gap was 55.0 ms, with no regressing
stamps. Evidence is under `artifacts/hardware/d5-ros-delivery-20261021/`.
This checks DDS delivery to an independent stationary observer, not the
driver's internal CAN callback queue, the controller-manager subscriber,
heavy-load behavior or physical wheel synchronization. Those paths still need
measurement before choosing a production field-age bound.

The next offline candidate adds
[five-second internal queue timing](../../patches/clearpath_puma_queue_timing.patch)
to the field-age driver. It counts CAN callback-to-dequeue age, mean and maximum
age, and receipts over 100 ms without altering feedback eligibility. Its pinned
package compiled and 101 driver plus 238 aggregate-routine assertions passed
under `artifacts/hardware/d5-queue-timing-candidate-20261021-2/`. The first
build in `d5-queue-timing-candidate-20261021/` compiled the package but its
extracted test harness lacked the new diagnostic members; that harness was
corrected before the passing build. Neither candidate was installed. Queue
timing remains **unmeasured on live CAN** until an instrumented driver is
separately reviewed, installed and observed under the pressed stop and load.
An additional 240-assertion extracted-node run checked that the five-second
diagnostic bucket resets after reporting; it is under
`artifacts/hardware/d5-queue-timing-aggregate-recheck-20261021/`.

A smaller [diagnostic-only queue patch](../../patches/clearpath_puma_queue_diagnostic.patch)
was prepared from the pinned 2.9.4 source and the two installed Puma repair
patches. It timestamps each CAN callback, records callback-to-dequeue age and
queue depth, and logs five-second buckets without adding the experimental
field-age eligibility policy. Its [offline builder](../../tools/hardware/build_puma_queue_diagnostic.py)
compiled the node under `artifacts/hardware/d5-queue-diagnostic-only-20261022/`;
the saved receipt pins the source, patch, dependency and binary hashes. The
node was not installed or started. This instrumentation still needs a separate
runtime review and stationary observation before its measurements can inform
any production field-age limit; callback age also excludes earlier kernel and
network delay.

## Split field-age bounds and pinned deployment (D5)

The single 200 ms bound treated a slow setpoint reply like a stale wheel
speed: rear setpoint gaps of 150–250 ms withheld aggregates, while position and
speed stayed near 40 ms. The policy now has **two bounds**. Position and speed
must be new since the previous publication and at most **150 ms** old; duty,
current and setpoint must be valid and at most **500 ms** old. The consumer only
checks the auxiliary fields for finiteness, and every field still invalidates on
a malformed payload. Diagnostics report the oldest motion and auxiliary ages
per driver. The single-bound constructor remains for the earlier models.

[The capture-chain replay](../../tools/hardware/check_puma_capture_chain.py)
runs the maintained classes over five real captures: quiet and loaded
stationary, a stationary attempt, the manual pulse and the D4 handover. At
150/500 ms every capture published on every tick after start-up (50 ms worst
gap) with no consumer fault, also with a fixed 25–100 ms receive-queue delay.
Injected asymmetric faults behaved as intended: one rear setpoint silent for
400 ms never reached the consumer, while 700 ms of setpoint silence or 300 ms of
one wheel's speed silence latched it. A motion bound of 100 ms failed at 75 ms of
queue delay and 125 ms failed at 100 ms. Beyond the bound the chain fails safe:
the consumer never activates. Results are under
`artifacts/hardware/d5-split-150-replay-20261022-{q,b,c,p,h}/`; the 100 ms sweep
is in `d5-split-bound-replay-20261022-*`. The idealised timer and fixed delays
are not live measurements.

The pinned package `artifacts/hardware/d5-split-bound-candidate-20261022-3/`
(Clearpath 2.9.4 plus the fault-flag, cache, completeness, field-age and
queue-timing patches) passed 119 real-driver and 240 aggregate-routine
assertions, including slow-auxiliary reuse, asymmetric wheels, stale duty or
current and missing per-wheel fields. The policy check passed 542 assertions
(`d5-split-bound-policy-20261022-2/`). The queue-timing patch logs
`DYNAMO_PUMA_QUEUE_TIME` buckets, which measure the live callback-to-dequeue
delay that the 150 ms bound must cover.

[The field-driver installer](../../tools/hardware/install_puma_field_driver.py)
handles deployment.
- **Stage** verifies the candidate against the current maintained inputs, its
  dependencies and binary hash. It requires the running driver to be the pinned
  repair named by the feedback-plugin drop-in. It renders a later drop-in,
  `97-dynamo-puma-field-driver.conf`, which copies that ExecStart and swaps only
  the driver prefix to the root-owned `/opt/dynamo-puma-field/<manifest-hash>`.
- **Apply** requires sudo, a pressed physical stop and the clock start margin.
  It restarts only `clearpath-platform.service`, then verifies the stop, stationary
  odometry, the active controller, guard routing, the new driver path and the
  unchanged plugin and independent services.
- **On failure**, it removes its drop-in, restarts the original driver and
  verifies it.
- **`--restore`** does the same on request and keeps the prefix as evidence.

A read-only stage dry run is under
`d5-field-driver-stage-dryrun-20261022/`; it recorded the D2 test interlock then
installed, so the live window must stage again.

Live acceptance:
1. With the stop pressed, run
   [`check_puma_field_driver_live.py`](../../tools/hardware/check_puma_field_driver_live.py)
   for 60 seconds. It passes only if all of the following hold:
   - the field driver is running;
   - aggregates arrive at 19 Hz or more, with no gap over 100 ms;
   - all four drivers report valid ages within 150/500 ms;
   - every queue bucket is at most 75 ms and none is over 100 ms (the 75 ms
     acceptance bound is one 50 ms drain cycle plus 25 ms scheduling margin);
   - the stop stays pressed and odometry stays stationary.
2. Its baseline smoke run on the old driver correctly reported "not passed"
   (`d5-field-live-smoke-baseline-20261022/`).
3. If the check fails, or the driver is not kept, run the installer's `--restore`.

**Installed and accepted stationary on 2026-10-22.** The first apply refused
before changing anything, because the guard had returned from the D2 test
interlock to original-manual mode after staging. A fresh stage
(`d5-field-driver-install-20261022-2/`) installed with status
`installed-verified-stationary`.

The first 60-second acceptance (`d5-field-live-acceptance-20261022/`) is
recorded as **not passed**. Only the queue limit failed: one bucket peaked at
52.7 ms against the original 50 ms limit. Every bucket peaked at about 50 ms
with a mean of about 30 ms. That is the structural wait of the driver's
once-per-50 ms queue drain, not load, so the limit was revised to one drain
cycle plus 25 ms (75 ms).

A fresh repeat (`d5-field-live-acceptance-20261022-2/`) then **passed**:
- 1,200 aggregates at 19.99 Hz, with a 52.0 ms maximum gap;
- all four drivers reporting, with motion age at most 51.2 ms (limit 150) and
  auxiliary age at most 160.6 ms (limit 500);
- no invalid samples;
- queue maximum 50.6 ms, with none over 100 ms;
- the stop pressed throughout and odometry zero.

This qualifies stationary operation of the field driver. Motion and load
behaviour are left to D6–D8. The original driver stays available through
`--restore`.

## Hardware consumer missing-feedback behavior

The installed `ros-jazzy-clearpath-hardware-interfaces` package is
`2.9.4-1noble.20260724.050450`. Its Puma hardware consumer is separate from the
newly pinned motor-driver package. Matching 2.9.4 source updates joint states
only when a new aggregate feedback message arrives; otherwise it keeps previous
speed/travel values. `read()` still returns success, and `write()` continues
publishing the current requested wheel commands. Neither path checks feedback
age. Installed library read/write disassembly is consistent with that behavior;
its SHA-256 is `2c7af1dad48dc0e8f0f71f90a471617f291a8204208f3d224e7c35533ef4a3b7`.

An offline harness executes the actual `read`, `write`, joint-update and
command-building method bodies against inert feedback/command sinks. After one
complete nonzero feedback batch, one hundred simulated 50 ms cycles without
feedback retained old velocity, returned success and emitted four hundred
nonzero wheel-command values. It also reproduced partial batches mixing new
and old states, an unknown joint name inserting a map entry that overwrites
joint index zero, and nonfinite speed accepted without a read error. These
source-level cases do not establish their occurrence on the live robot; no live
feedback loss or malformed feedback was injected. Source/binary provenance,
installed disassembly and results are under
`artifacts/hardware/20261012-feedback-consumer-offline/`.

This closes the consumer inspection, not driving qualification. Before deploying
the offline freshness candidate, qualify the consumer policy described below
for monotonic feedback age, complete/unique expected wheel identities, finite
fields and inhibition of nonzero commands on invalid/stale feedback.
Do not invent fresh zero velocity as a measurement of physically stopped wheels.
Choose timeout and recovery semantics from observed gaps and isolated integration
tests, then qualify them under the commissioning interlock. No such consumer
repair is deployed. The existing command-age timeout and one-second test permit
limit address command delivery and test exposure; they do not validate motor
feedback freshness. Keep the new per-wheel completeness patch offline until
this consumer boundary is resolved.

## Offline feedback-consumer safety candidate

`patches/clearpath_puma_feedback_consumer.patch` modifies the matching official
Clearpath 2.9.4 hardware consumer. Its shared policy has one maintained source,
`tools/hardware/puma_feedback_safety.hpp`; the offline builder copies it into
the candidate package. The later stage-5 version is installed as described
below; this section describes its pre-installation qualification. It is
separate from the fault-mask/cache-memory repair in the motor driver.

The candidate accepts only a complete batch of configured joint names, each
unique, with unique valid CAN device IDs and finite speed, travel, duty, current
and setpoint fields. It binds name/device associations on activation and rejects
changes during that activation. Initial CAN IDs are learned from feedback,
not independently checked against the robot declaration. All measurements remain
real feedback; inhibition never fabricates zero measured velocity.

Callback receipts use a monotonic clock, backdated by nonnegative ROS header
age at delivery so time already spent in transport consumes the same budget.
A provisional 250 ms effective age limit
is checked independently by both read and write; a newly arriving packet cannot
erase a preceding expired interval. ROS header timestamps must be present,
no more than 250 ms old or 5 ms in the future, checked at receipt and consumption.
Header checks use ROS time and therefore still require clock behavior to be
qualified. This does not establish freshness of every cached motor field: that
requires the separate driver-completeness repair and further CAN investigation.

The consumer collects available middleware input up to 32 times per read,
validating every delivered batch. A single `spin_some` collection can leave
25 Hz input accumulating behind a 20 Hz read loop. Inactive reads drain input
and send zeros without falsely failing startup.

The transport preserves up to 32 queued batches so a good packet cannot overwrite
an invalid body before validation. Queue overflow and invalid header evidence
cause inhibition. Invalid or expired feedback produces an explicit zero command,
returns a hardware error and latches the fault. Later good feedback does not
resume driving. Explicit lifecycle activation requires a fresh, complete,
stationary batch (wheel speed magnitude at most 0.001 rad/s), waits at most five
seconds, and otherwise fails with zero commands. Deactivation and the error callback also send zeros and disable active reads/writes;
inactive writes remain zero until explicit successful activation.
These are software requests; delivery and physical braking are not established
by offline tests.

The offline checks execute the actual policy and extracted transport, read,
write, activation and deactivation bodies with injected clocks and inert ROS.
They cover startup without feedback, malformed batches, message overwrite,
overflow, identity changes, stale feedback, command inhibition, latched recovery
and stationary activation. A full package compilation verifies the ROS API
boundary. A direct compiled-plugin fixture also passes real ROS transport
checks for stationary activation, healthy commands, inhibition when feedback
ceases, retained measurements, latched recovery and deactivation. It uses fake
wheels on loopback-only domain 214, verifies the loaded library identity, and
does not start the motor driver. This fixture sets joint storage directly; it
does not exercise controller-manager initialization or error transitions. The maintained builder applies the patch with forward/reverse checks,
records dependency versions, source/library hashes and input snapshots, and
rejects inputs changed during the build. Its public command is in the README.
It never installs, opens CAN or joins the live ROS graph.

The corrected candidate passes a controller-manager test on loopback-only
ROS domain 215. The test initializes from a fake URDF, activates the real plugin
and a forward-command controller, maintains 25 Hz fake feedback with a 20 Hz
controller, then stops and restores feedback while continuing to request
commands. It verifies zero-command delivery, controller deactivation, absence of
automatic restart, explicit stationary hardware/controller recovery without old
command replay, new-command acceptance and orderly deactivation. The controller
becomes inactive and the hardware unconfigured after the error; recovery
requires explicit configuration and activation.

Earlier offline candidates failed this gate: accepting already aged input
without spending that age from the monotonic budget extended inhibition toward
500 ms, and collecting only one middleware sample per read accumulated a backlog.
The maintained source corrects both. Artifact candidates `reproducible-2` through
`reproducible-4` are superseded; `reproducible-5` passed. These were offline
qualification checks; the later stationary live test is described below.
The controller manager still logs an interface-mode-switch error when it removes
interfaces before controller deactivation, although the queried controller is
inactive and fake motor commands remain zero. Explicit recovery succeeded;
that framework log is not evidence of an installed robot fault or qualified
physical recovery.

On 2026-10-13 the [plugin-only stager](../../tools/hardware/stage_puma_plugin_prefix.py)
created an overlay from the verified `reproducible-5` candidate. It copies the
Puma library and package discovery metadata, then links the sibling hardware
plugins and package executables to their apt files. The first isolated
controller-manager test exposed a missing `package.xml` in the manually
assembled prefix and failed before activation. Adding the manifest corrected
plugin discovery. The second fake-wheel test loaded the staged library by
its exact path and passed all 19 checks, including zero commands about
250 ms after feedback stopped, controller deactivation and explicit neutral
recovery. Evidence is under `20261013-puma-plugin-prefix-manager-check-2/`;
the stager reproduced the verified library and metadata under
`20261013-puma-feedback-plugin-stage-reproduced/`.

The maintained [installer](../../tools/hardware/install_puma_feedback_plugin.py)
stages a root-owned Puma plugin prefix and one later platform drop-in. It
preserves the installed auxiliary launch chain and test interlock, checks
rollback ownership, and compares the library mapped by the real controller
manager after restart.

The first live install attempt on 2026-10-13 used a minimal prefix that omitted
the package's executable directory. Its fake-wheel manager test passed because
it exercised only the hardware plugin, but the real Clearpath launch failed
when resolving `clearpath_hardware_interfaces` executables. The installer
removed its drop-in and restarted the original platform. Its receipt remained
at `applying` after the sudo process ended; the separate
`20261013-puma-feedback-install-stage-3/recovery-audit.json` independently
verified the original apt plugin loaded, platform/guard active, stationary
physical stop, unchanged camera/sensor/desktop services and absent drop-in.
The unused first pinned prefix remains on disk as evidence and is not selected.

The revised stager carries symlinks to the six original apt executables alongside
the new Puma library and companion plugin descriptors. The corrected package
under `20261013-puma-feedback-install-stage-5/` uses a hash of its full manifest
for a distinct root prefix. It passed fake-wheel controller-manager checks,
launch argument parsing, package lookup and all six executable-resolution
checks. Physical stopping, missing CAN feedback under movement and live recovery
still need commissioning evidence.

The stage-5 install initially stopped before service changes because the local
clock reference was older than the platform's 180-second gate. After bounding
the chrony source poll to 128 seconds and obtaining a fresh NTP sample, the
same stage installed on 2026-10-13. Its receipt reports
`installed-verified-stationary`: the running controller manager mapped the
exact pinned `libpuma_hardware.so`, the physical stop was engaged, raw wheel
odometry was stationary, the platform controller and speed-guard routing were
active, and the installer found the camera, sensors and desktop services
unchanged. This establishes stationary startup and plugin selection. The
missing-feedback stopping and recovery behavior observed with fake wheels
still needs a controlled physical test before driving readiness is complete.

On 2026-10-13, with the physical e-stop engaged, a guarded live test paused
the actual `multi_puma_node` process for 0.8 seconds. An independent systemd
timer and the observer both resumed it; its PID and the controller-manager PID
were unchanged. ROS wheel feedback disappeared for the measured outage. Two
zero wheel-command messages arrived 0.290 seconds after the pause began, and
the controller manager logged a hardware read error and deactivated the
platform controller. Raw output is under
`artifacts/hardware/20261013-live-feedback-outage/`; the checker labelled it
inconclusive because its zero-command window started at 0.300 seconds, after
those messages. The separate timestamped interpretation under
`artifacts/hardware/20261013-live-feedback-outage-analysis/` retains that raw
status and explains the corrected 0.250–0.500-second window. This establishes
the installed plugin's response to real ROS feedback interruption while
physically stopped. It does not prove braking while moving, electrical CAN
loss, or how quickly the motors receive zero after the driver resumes.
With the physical stop still engaged, only `clearpath-platform.service` was
restarted. A fresh three-second recovery capture then verified real feedback,
zero wheel commands, stationary odometry, the active platform controller and
the same pinned plugin. Evidence is under
`artifacts/hardware/20261013-feedback-outage-recovery/`. This is a supervised
stationary recovery, not a moving recovery or repeated-fault qualification.

## Remaining moving and motor-link qualification

The live 0.8-second pause stopped the entire `multi_puma_node`. That process
both publishes wheel feedback and consumes wheel commands. A moving repeat
would confound the result: the motor driver itself could stop consuming
commands independently of the plugin's feedback timeout. It cannot establish
physical braking from feedback protection.

A valid moving software-fault check must keep the pinned Puma driver and its
command delivery running while withholding or invalidating only feedback to
the hardware consumer. Under the installed commissioning test interlock, begin
from a fresh pressed physical stop, neutral controller, active platform and
verified fresh wheel feedback. Use one bounded forward permit at the existing
5 cm/s cap, inject the fault only after measured forward motion begins, and
record the limited command, motor command, raw wheel odometry, fault time,
controller lifecycle state and physical stopping distance. A persistent
injection needs an independent cleanup watchdog; the prepared invalid-batch
check publishes one batch and its one-shot permit expires even if the test
terminal closes. Require
zero motor commands, controller fault latching, measured stopping and a
supervised pressed-stop recovery before calling that check passed. A malformed
ROS feedback message would test the consumer's invalid-batch path, not actual
missing per-wheel CAN frames. No such moving fault was run by the 2026-10-13
stationary test.

Two raised-wheel attempts on 2026-10-13 did not reach fault injection. In the
second, the physical stop was released, but all recorded Joy messages remained
neutral; the configured Bluetooth controller was subsequently confirmed
disconnected. The recorder now checks that physical controller link before
arming any bounded moving trial. Neither attempt is evidence of moving fault
response.

The virtual `vcan0` receiver in the Clearpath launch reads the shared virtual
bus and republishes ROS telemetry on `vcan0/rx`; pausing that receiver does
not isolate the Puma driver's SocketCAN input. Do not present that as a motor
link interruption. A genuine lost motor-CAN path can also block outgoing zero
commands, so its physical stop depends on motor-controller behavior beyond
this ROS plugin. Qualify that failure only with all wheels securely supported
by rated service equipment, a qualified operator at the physical stop, and a
bounded command whose loss path and automatic recovery are independently
controlled. Capture physical wheel rotation as well as host CAN frames and ROS
topics. No floor-driving CAN-disconnect trial is authorized by these stationary
results.

### Prepared raised-wheel link-fault procedure (D6/D7)

[`check_raised_wheel_link_fault.py`](../../tools/hardware/check_raised_wheel_link_fault.py)
prepares both trials and has not been run. Preconditions:
- the D5 field driver is installed and accepted;
- all wheels are clear on rated stands;
- a qualified operator is at the physical stop;
- the window is exclusive.

The unchanged guarded pulse tool is the only motion source; the injector never
publishes. After the motor command has been forward for 350 ms and every wheel's
feedback shows motion, it inserts one firewall rule on the MCU CAN-over-UDP bridge
(UDP 11412) for at most 1.5 s:

- `--fault feedback` (D6) drops MCU→PC only. The driver keeps running and sending
  commands, so the hardware plugin's missing-feedback path is isolated.
  - **Automatic criterion:** feedback disappears after the small in-flight
    allowance; zero motor commands arrive within 0.5 s and none later; the
    upstream limited source remains nonzero after that response; the controller
    leaves active and does not restart; fresh feedback after the block shows a
    sustained stop for at least 0.25 s; the stop is pressed at the end.
  - Missing feedback prevents ROS from observing the physical wheels during the
    outage. A pass therefore also requires independently recorded video and the
    supervised recovery; the automatic result alone is inconclusive.
- `--fault command-link` (D7) drops PC→MCU only. Outgoing zeros cannot reach the
  motors, while feedback still arrives and measures the wheels.
  - **Automatic criterion (provisional):** feedback remains fresh with no gap
    over 150 ms; every finite four-wheel sample reaches and sustains at or below
    0.05 rad/s for 0.25 s within the provisional 0.5 s watchdog limit while the
    source remains nonzero through the stop; no movement resumes after the rule
    clears.
  - A longer result is a motor-controller watchdog finding, not a reason to
    lengthen the block. Record the wheels on video as well.

Before any motion, a target-less counting rule must see at least 20 bridge
packets per second in the chosen direction, or the test refuses. The installed
drop rule's own packet counter must also increase; a zero counter makes the
result inconclusive. Malformed/nonfinite or non-four-wheel feedback never counts
as stopped. A transient
systemd unit started before the fault removes the exact rule after the block
plus 250 ms, as soon as the injector process disappears, or at its 10-minute
runtime cap through `ExecStopPost`. Tests run that cleanup loop against a fake
iptables.

After either trial the platform is treated as latched. `--recover` requires the
stop pressed and no remaining fault rule, restarts only the platform, and runs
the read-only `check_live_feedback_outage.py --verify-recovery`. That checker and
the stationary ingress checker accept both the pinned repair and the field driver.

The D5 split-bound field driver described above is installed and passed its
stationary live acceptance. D6 and D7 must preserve its 150 ms motion-field,
500 ms auxiliary-field and 75 ms queue bounds in their captured evidence; that
stationary pass does not itself qualify moving or fault behavior.

The 2026-10-13 stationary incoming-link trial used the maintained
`check_motor_can_ingress.py` checker. With the physical stop pressed, it added
an INPUT-only UDP rule for MCU `192.168.131.2` to PC `192.168.131.1:11412`;
an independent systemd timer was armed to remove the rule. Baseline contained
65 aggregate feedback samples over about 3.2 seconds. After the rule was
inserted, one already in-flight aggregate sample arrived; no later aggregate
feedback appears in the captured blocked interval. The controller-manager
journal reports its read-cycle fault and controller deactivation about 290 ms
after rule insertion. Eight motor-command messages captured before the
observer stopped were all zero. The observer's **raw status is failed** because
it treated the resulting stale odometry as an observation failure at about
280 ms; do not relabel that raw run as a pass. Its finally block removed the
exact firewall rule (`manual_rollback_rc=0`, final rule check absent). Evidence
is in `artifacts/hardware/20261013-stationary-can-ingress/` and the platform
journal around 11:16:30 UTC. The checker now tolerates odometry becoming stale
after a fault and requires restored feedback before returning an observed
status. A same-day repeat with the corrected checker returned
`stationary-can-ingress-observed`: 64 baseline feedback samples, two samples
already in flight during the 0.8-second block, and 39 samples after removal.
The driver PID stayed the same and the exact firewall rule was absent afterward.
All observed motor commands and raw wheel velocities were zero. Command and
odometry streams did not resume after incoming feedback returned, so restored
transport alone did not restore active control. The repeat evidence is under
`artifacts/hardware/20261013-stationary-can-ingress-repeat/`.

Under the still-pressed physical stop, one platform restart and the existing
read-only recovery checker then passed: pinned driver and plugin loaded, active
controller, fresh real feedback, zero commands and stationary odometry. The
recovery result is in `artifacts/hardware/20261013-can-ingress-recovery/`.
After the repeat, a second supervised platform restart passed the same
stationary recovery checks, with evidence under
`artifacts/hardware/20261013-can-ingress-repeat-recovery/`.
The firewall test models full incoming motor-link loss at the PC boundary,
not one missing wheel field, a broken CAN conductor, failed outgoing zero
delivery or physical stopping under motion.

## Observed startup transients

The 2026-10-06 guarded step-pulse sequence completed backward, left, right and
turn-left requests with final stationary raw odometry. Physical direction and
stopping observations for those four cases are still required. Turn-right
aborted twice, and a later repeat aborted during backward. The test stopped
and sent zeros when reported motion outside the requested axis exceeded its
conservative 0.02 m/s planar threshold.

The turn-right retry recorded a controller reference containing only yaw.
One wheel-feedback message reported the two left wheels moving while both
right wheels remained at their previous zero readings; the next message
reported all four moving in the requested turn direction. The controller used
those mixed readings to produce the brief forward odometry. This establishes
the software calculation, not whether the physical wheels responded at
different times or their feedback was delivered at different times. Do not
raise the abort threshold or treat the transient as harmless without evidence.

The helper's optional 250 ms rise/fall profile stays within the original peak
and one-second duration. It changes the requested acceleration, allowing a
controlled comparison to the abrupt step. It does not repair CAN freshness or
qualify emergency stopping. Capture controller references, individual wheel
feedback, raw/filtered odometry and operator observations together on future
trials. Evidence is under `artifacts/hardware/20261006-axis-pulses/`,
`20261006-turn-right-retry/` and `20261006-axis-pulses-repeat/`.

One gently ramped turn-right trial and a subsequent five-direction ramped
sequence completed under the **same abort thresholds**. The operator confirmed
all requested directions and clean stops. Cross-axis readings were much smaller
than in the step trials. The ramp changes commanded acceleration; this supports
using it for subsequent commissioning, not concluding that the cache/physical
timing issue is fixed. Evidence is under `20261006-turn-right-ramp/` and
`20261006-axis-pulses-ramp/` within the same hardware artifact root.
