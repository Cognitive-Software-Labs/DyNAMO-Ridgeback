# Field-age evidence from short moving captures, replays and queue-timing builds

Recorded dates: 2026-10-21, 2026-10-22
Tested revisions: unknown
Provenance: partial

This record covers the D5 evidence gathered while the single 200 ms field-age
bound was still the candidate: guarded pulses and a source handover with
passive CAN capture, policy replays over those recordings, a ROS delivery
measurement, and the queue-timing builds. During all of it the robot ran the
pinned repair driver, not a field-age driver. The 200 ms bound is
**superseded** by the split bounds in
[the acceptance record](2026-10-22-d5-acceptance.md). The source text recorded
no repository commit for the capture wrapper, analyzer, replays or builders, so
tested revisions are unknown. Directories named below are on the robot host
under `artifacts/hardware/` and were not copied into the repository. The text
was moved here from the
[wheel-feedback reference at the documentation snapshot](https://github.com/Cognitive-Software-Labs/DyNAMO-Ridgeback/blob/077692ab033fd3a86d4849bbbbdb443a9f0c2125/docs/physical/drive_feedback_diagnostics.md).

## Manual pulse before the motor-readiness gate

The first one-second 0.05 m/s manual pulse (`artifacts/hardware/d5-manual-pulse-20261021/`)
reached the mux, guard, wheel-command topic and outgoing CAN speed-set frames,
but the operator saw no movement. During the command all four incoming motor
setpoint fields stayed zero and their fault fields reported bit 8
(`FAULT_BRIDGE_DRIVER` in `PumaStatus`). The same bit cleared in the passive CAN
recording several seconds after physical-stop release and returned after the
stop was pressed. A separate read-only ROS status sample with the stop pressed
showed all four motors in speed mode with fault 8. This indicates the pulse was
sent before the motor controllers reported ready; it does not establish a motor
defect or a moving field-age bound. The pulse and handover recorders were
changed to wait for all four speed-mode motor statuses to report zero faults
for one second after stop release, and to abort without commanding if that
does not happen within twenty seconds.

## Manual pulse with the readiness gate

The repeat (`artifacts/hardware/d5-manual-pulse-20261021-2/`) waited 6.78 s after stop release for
stable zero-fault speed-mode status, then sent the same one-second 0.05 m/s
request. The operator saw forward movement and a self-stop before pressing the
red stop. Raw odometry peaked at 0.0511 m/s, and the final stop was recorded
pressed. The motion-window analyzer found 466 valid relevant CAN receipts and
zero malformed receipts in the 0.959 s nonzero-command window. All twenty
wheel/field groups were present; the largest field silence, including window
edges, was 150.2 ms for one rear wheel's setpoint. Nineteen ROS feedback
aggregates arrived in the window, with a maximum 50.4 ms gap, and every sampled
motor status was fault-free in speed mode. This is one short moving capture with
a working readiness sequence; it does not measure internal queue age, loaded or
sustained motion, or justify the 200 ms bound.

## D4 handover capture

The successful two-source D4 floor handover (`artifacts/hardware/d4-handover-can-20261021-6/`)
gave a second short moving window. The analyzer aligned its ROS command stamps
with passive CAN host-receive times: 835 valid relevant field receipts, zero
malformed, all twenty wheel/field groups present over 1.695 s of nonzero
limited output. The largest per-field silence including window edges was
150.3 ms, and all three sampled motor statuses were fault-free in speed mode.
Raw forward odometry peaked at 0.0465 m/s. The operator observed forward-only
motion and a self-stop, and the final physical stop was recorded pressed. The
handover recorder sampled neither ROS feedback aggregates nor motor command
messages, and neither moving capture measures internal queue delay or loaded
recurrence.

## Capture-chain replays at 200 ms

The capture-chain replay ran the maintained C++ field-age and consumer policy
classes against two real 30-second passive recordings from 2026-10-16
([field-age capture record](2026-10-16-field-age-captures.md)). At 200 ms its
50 ms timer phases produced 599–600 aggregates in the quiet capture and 598 in
the concurrent-workload capture, with maximum gaps of 50 and 100 ms and no
consumer fault. Removing one wheel's speed receipts made the consumer fault and
stay latched. Results are in `artifacts/hardware/d5-policy-replay-20261021-a6/` and
`artifacts/hardware/d5-policy-replay-20261021-b6/`. A fixed 125 ms processing delay injected into
the loaded capture reduced output to 512–587 aggregates with a 250 ms maximum
gap, exactly the consumer timeout; at 175 ms no phase produced an initial
aggregate. The timer was idealised and the delays modelled, not measured.

## Attempts that did not move

Three supervised capture attempts (`artifacts/hardware/d5-moving-capture-20261021/`, `-2/` and
`-3/`) preserved real CAN traffic but never armed a movement publisher: the
first saw no valid forward input within 15 seconds of stop release, and in the
other two the physical stop was still pressed when forward input arrived. They
add stationary timing evidence only. The third had 25,282 valid relevant field
receipts and no invalid ones. Its maintained-policy replay at 200 ms produced
at least 1,024 aggregates over the sampled 50 ms phases, with a 100 ms maximum
gap and no modelled fault; removing one wheel's speed receipts latched the
consumer (`artifacts/hardware/d5-attempt-3-policy-replay-20261021/`).

## Joystick-independent pulse

The pulse wrapper completed one guarded half-second command with passive CAN
capture (`artifacts/hardware/d5-pulse-capture-20261021/`). The pulse recorder reported the stop
released before publication, a 0.05 m/s requested peak, zero motor commands
about 98 ms after source silence, no later nonzero command, final stationary
odometry and the stop pressed. All twenty wheel/field groups had valid receipts
in the aligned half-second window (247 relevant receipts, none malformed), and
also in one-second windows before and after (`artifacts/hardware/d5-pulse-motion-window-20261021/`).
The maintained-policy replay at 200 ms over the full capture produced at least
500 aggregates with a 150 ms maximum modelled gap and no modelled fault
(`artifacts/hardware/d5-pulse-policy-replay-20261021/`). ROS feedback showed only a small
wheel-speed response after the command window, and raw odometry was zero
during the command. The source text records no operator observation of travel
for this pulse, so physical movement is unconfirmed.

## ROS delivery to an independent observer

A read-only recorder observed the installed repair driver's aggregate output
for 30 seconds with the stop pressed and raw odometry zero. Its 601 aggregates
had 0.31 ms median, 0.40 ms 95th-percentile and 5.06 ms maximum delay between
the driver's header stamp and the observer callback. The longest arrival gap
was 55.0 ms, and no stamp regressed (`artifacts/hardware/d5-ros-delivery-20261021/`). This checks
DDS delivery to a stationary observer, not the driver's internal CAN callback
queue, the controller-manager subscriber, heavy load or physical wheel
synchronization.

## Queue-timing builds

The queue-timing patch added five-second callback-to-dequeue statistics (mean,
maximum and receipts over 100 ms) to the field-age driver without changing
eligibility. The first build (`artifacts/hardware/d5-queue-timing-candidate-20261021/`) compiled
the package, but its extracted test harness lacked the new diagnostic members;
after the harness was corrected, the pinned package compiled and 101 driver and
238 aggregate-routine assertions passed (`artifacts/hardware/d5-queue-timing-candidate-20261021-2/`).
A further 240-assertion extracted-node run checked that the diagnostic bucket
resets after reporting (`artifacts/hardware/d5-queue-timing-aggregate-recheck-20261021/`). Neither
build was installed; queue timing was unmeasured on live CAN until the
field-age driver carrying this patch was installed.

A smaller diagnostic-only queue patch, built on the pinned 2.9.4 source with
only the two repair patches, timestamps each callback and logs callback-to-
dequeue age and queue depth in five-second buckets without field-age
eligibility. Its offline builder compiled the node under
`artifacts/hardware/d5-queue-diagnostic-only-20261022/`, with a receipt pinning source, patch,
dependency and binary hashes. It was never installed or started. Callback age
excludes earlier kernel and network delay.
