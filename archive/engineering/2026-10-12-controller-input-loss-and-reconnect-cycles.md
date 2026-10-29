# r100_0160 controller input loss, recovery and reconnect cycles

Recorded dates: 2026-10-12, 2026-10-16
Tested revisions: unknown
Provenance: partial

This record covers the wireless controller's failure to deliver input after a
deliberate moving disconnect, its diagnosis and manual recovery, the
installation of the non-restarting autoconnect helper, and the stationary
reconnect cycles that followed on `r100_0160`. Every recording was made with the
physical stop pressed and the test-permit interlock installed. It was extracted
from the consolidated driving reference at the
[documentation snapshot](https://github.com/Cognitive-Software-Labs/DyNAMO-Ridgeback/blob/077692ab033fd3a86d4849bbbbdb443a9f0c2125/docs/physical/driving_readiness.md).

Provenance gaps: the source text names no repository commit for the observer,
diagnostic or setup installer that ran. The setup receipt and the cycle run
preserve tested source snapshots. Raw recordings, controller journals and
analyses live only on the robot host under `artifacts/hardware/`. The root
cause of the input loss was never established.

## Stationary reconnect failures

The first stationary reconnect trial (`20261012-stationary-reconnect`) failed to
confirm controller reconnection: the selected Bluetooth device still reported
disconnected at the end. The second trial aborted before the reconnect prompt
because raw odometry age reached roughly 0.276 seconds, above the unchanged
0.25-second freshness requirement. The physical-stop report age was about 1.22
seconds, and software-lock and limited-output reports were only about 0.01
seconds old. The second abort therefore concerns wheel observation freshness,
not a failed controller reconnection. The preceding raw odometry gaps in that
recording were around 0.05 seconds; its final gap ended at observer abort, so
neither its total duration nor cause is established. The freshness threshold
was deliberately not increased to pass a trial; recorder errors were changed to
identify the stale topic, observed age and threshold.

## Connected but no input

A read-only controller inspection confirmed Bluetooth connected, and the
controller driver logged a reconnect at 2026-10-12 10:38:27 UTC, followed by
repeated `0 reports/s` warnings.

| Recording | Joystick states | Combiner commands | Notes |
|---|---|---|---|
| `20261012-controller-input-check` | 378 samples, one all-zero state | none | All 19 firmware stop and 399 software-lock reports asserted; whether the operator exercised the controller awaited confirmation |
| `20261012-controller-input-check-2` | 378 identical zero states | none | Operator confirmed exercising driving-enable and forward; driver still logging `0 reports/s` |
| `20261012-controller-input-after-restart` | 376 identical zero states | none | After restarting only `ds4drv` at 10:47:05 UTC; physical stop reports became released during the recording, contrary to the stationary procedure |
| `20261012-controller-input-after-powercycle` | 376 identical zero states | none | All physical stop and software-lock reports asserted |

In every recording limited commands, wheel commands and raw odometry stayed
zero. The input inventory for the second recording distinguished the
driver-managed virtual controller on `js0` (the `/dev/input/ps4` target) from
the kernel's physical wireless controller on `js1`; that made the
physical-to-virtual driver path the next diagnostic boundary, without
establishing which component failed. Restarting `ds4drv` recreated the virtual
joystick and reopened the Bluetooth controller but did not restore input.
Read-only descriptor inspection showed ROS using the current `js0` inode, so a
removed old joystick file descriptor was not evidenced. After the
release-during-recording deviation, the observer was changed to abort
stationary recordings on physical stop release or unexpected downstream
commands or wheel motion. Further identical operator recordings were judged not
to be diagnostic progress.

The new controller-input diagnostic then compared cached kernel and virtual
device state while the operator confirmed physical input: 399 samples each of
physical and virtual cached state remained unchanged. The physical device
identity matched the paired controller MAC. This placed the missing input at
or before physical report reception rather than in ROS forwarding. Kernel
history retained the same physical connection from 10:38:26 despite the
attempted manual power cycle; no intervening disconnect was recorded.

## Manual recovery

A device-specific recovery disconnect at 2026-10-12 11:12:29 UTC returned
success, Bluetooth reported `Connected: no` and the driver logged
`Disconnected`. No platform, sensor or desktop restart was performed. A manual
PS-button reconnect followed: the driver logged a fresh connection at
11:13:36 UTC and a battery report one second later. The subsequent
`20261012-controller-input-clean-reconnect` recording captured 2,842 joystick
samples with 73 distinct states, including 848 held driving inputs and 992
combiner source messages reaching 0.4 m/s. All 19 physical stop reports and 400
software-lock reports stayed asserted; all limited commands, wheel commands and
raw odometry were zero. This verified restored controller input delivery after
manual recovery, not reliability of automatic reconnection or the cause of the
earlier report loss. The recording ended with driving-enable and forward still
held; neutral return was not captured.

## Autoconnect helper installation

On 2026-10-12 the stationary setup installer replaced the platform-restarting
connection handler with the root-pinned `controller_autoconnect.py` and an
ExecStart-only service drop-in. Independent post-install verification matched
all six setup files, root owners and modes; protected platform, guard, camera,
sensor and merger process IDs were unchanged. The passive probe recorded seven
asserted stop reports and 140 stationary raw-odometry samples with correct
guarded routing. The helper reported `connected-link-only`, which is not proof
of physical reports, live joystick input or return to neutral, so the reconnect
defect remained open.

The `20261012-controller-after-setup` cached-state recording captured 399
samples each from the physical and managed virtual devices, 23 distinct states
on each, driving-enable and forward input and a released-button neutral tail.
Both devices responded and returned to neutral. The before/after passive probes
confirmed asserted stop, stationary raw odometry and correct guarded routing.
No simultaneous ROS joystick/source-command capture or Bluetooth transition was
recorded in this run. The held-input interval was longer than requested.

The `20261012-controller-ros-input` stationary ROS recording captured 2,888
joystick messages and 1,556 combiner requests, including positive forward
input, followed by released driving-enable buttons and more than two seconds
of neutral axes. All 15 firmware stop and 315 relay reports asserted the stop.
The mux published no commands, and all 300 samples each of limited commands,
wheel commands and raw odometry were zero. This verified end-to-end ROS input,
neutral return and stationary stop suppression; it is not a Bluetooth reconnect
or moving stop test. Input was held about eight seconds rather than the
requested one; no stop release was recorded. The observer's generic
joystick-input flag was not populated for stationary mode, so the raw
joystick and combiner samples carry the conclusion.

A trial preceding `20261012-reconnect-after-setup-2` aborted on button 4: the
operator clarified they pressed driving-enable after PS, contrary to the
neutral-only procedure. It is not evidence of a spontaneous reconnect button
fault. The `20261012-reconnect-after-setup-2` trial then recorded Bluetooth
changing from disconnected to connected, 841 neutral joystick samples, 43
asserted firmware stop reports and 888 asserted relay samples. All 846 limited
commands, 845 wheel commands and 845 raw odometry samples were zero; the mux
published no commands. Platform, combiner and guard process IDs remained
unchanged.

The companion `20261012-input-after-reconnect` recording verified restored live
ROS input: 543 joystick messages, 342 combiner requests containing forward
input, and a neutral tail of about 7.55 seconds. All 15 firmware stop reports
and 315 relay reports remained asserted. The mux published no commands, and all
300 limited commands, 300 wheel commands and 300 raw odometry samples were
zero. Together with the preceding neutral reconnect, this established one
successful post-repair reconnect/input/neutral-return cycle without a platform
restart. Driving-enable was held about 6.45 seconds rather than the requested
one second; this was an engaged-stop observation, not a motion exposure.

## Repeated stationary cycles

On 2026-10-16, `controller-cycles-20261016T085114Z` (the stationary cycle
recorder) completed all three deliberate disconnect/neutral-reconnect cycles in
approximately 187 seconds. Each reconnect was followed by new held joystick and
combiner input, then a sustained neutral return. No combiner source messages
arrived during the confirmed-disconnected or neutral-reconnect phases. All 177
physical stop reports and 3,699 software-lock reports stayed asserted. All
3,522 motor command batches, limited-command samples and raw-odometry samples
were exactly zero; wheel-command and odometry receive gaps stayed below 54 ms.
Initial and final checks verified an active controller and guarded routing. The
raw trace, tested source snapshots, Bluetooth state queries and review are
preserved in that evidence directory. Repeated **stationary** reconnect and
input recovery is observed for this session; silent radio loss, moving
reconnect and longer-term reliability remained open.
