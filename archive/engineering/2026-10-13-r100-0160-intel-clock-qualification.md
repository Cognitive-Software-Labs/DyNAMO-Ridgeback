# r100_0160 Intel clock gate, sources and boot-step qualification

Recorded dates: 2026-10-13, 2026-10-14, 2026-10-16
Tested revisions: unknown
Provenance: partial

The Intel platform clock gate's first reboot check, the clock-source expansion
and poll bound, the October 14 startup delay and the boot-clock step that fixed
it, as recorded on `r100_0160`. Extracted from the stationary auxiliary
reference at the [documentation snapshot](https://github.com/Cognitive-Software-Labs/DyNAMO-Ridgeback/blob/077692ab033fd3a86d4849bbbbdb443a9f0c2125/docs/physical/stationary_auxiliary_checks.md).
Provenance gaps: the text names the branch `feat/boot-clock-policy` but no
commit for the installed tools; receipts and raw reports live only on the robot
host under `artifacts/hardware/`.

## First supervised reboot

The installed platform clock gate's successful prestart was verified during the
auxiliary-repair service start. Its first actual reboot check passed on 2026-10-13. A pre-reboot boot ID is saved in the installer receipt, and the
maintained `verify_auxiliary_boot.py` checked that the boot changed, inspected
the named gate's successful non-ignored prestart record, verified service and
listener ownership and read actual stationary IMU and battery data; it sent no
commands. An operator confirmed the robot stationary with its physical e-stop
pressed and the controller centred and untouched for this reboot check.


The supervised reboot changed the boot ID and started at 09:33:51 UTC. The
platform clock gate ran from 09:34:04 to 09:34:42, exited successfully and then
allowed platform startup. The post-boot verifier passed all checks: all nine
selected services active with zero reported restarts, expected desktop listener
ownership, repaired file contents, asserted e-stop, stationary raw odometry,
intended IMU parameters and unknown battery-cell semantics. Actual IMU and
battery data were received. Evidence is under
`artifacts/hardware/20261013-auxiliary-after-reboot/`.

The later local clock report had about 1.94 ms remaining correction, within the
5 ms gate limit, but estimated UTC error bound about 71 ms because of upstream
network delay/dispersion. This demonstrates local convergence/startup ordering,
not sub-millisecond absolute UTC or Intel–Thor timing. A single supervised reboot
does not qualify repeated cold power cycles or time-source loss/holdover. Full
reference-based IMU calibration and additional physical battery sensing remain
hardware/measurement dependencies.


A later October 13 fast batch passed every non-clock check but failed the
provisional frequency-skew limit. Remaining system correction was only about
0.34 ms at the batch and 0.74 ms at follow-up; frequency uncertainty increased
from 22.55 to 29.05 ppm against the 10 ppm limit. The selected single internet
source remained reachable (`377`) but source statistics reported about 5.8 ms
sample scatter and roughly 28 ppm frequency uncertainty. This is source-quality
uncertainty, not evidence of a new large system-clock jump or failed sensors.
The gate limit is not relaxed to turn the result green. Stable source selection,
repeated convergence and time-source-loss behavior remain owned by the time
qualification workflow; coordinate with its deployment owner before changing
the shared Intel–Thor time profile. Evidence is under
`artifacts/hardware/20261013-clock-quality/`.


## Intel clock source expansion

`tools/hardware/expand_clock_sources.py` changes the installed Intel profile's
single `server ntp.ubuntu.com iburst` declaration to
`pool ntp.ubuntu.com iburst maxsources 4`. It preserves every other byte,
including the 500 ppm slew rate, robot-subnet service and startup step policy.
Apply requires root and a stationary operator. It validates the candidate under
`/etc/chrony/conf.d/` because Ubuntu confines chronyd's configuration reads,
resolves Ubuntu's current IPv4 addresses and adds missing servers through
chronyc while retaining the existing source and its measurement history.
It neither restarts chrony nor issues clock-step commands. At the next ordinary
chrony startup, the persistent pool declaration owns address replacement.
Thor's files and hardware services are untouched.

The dated run directory records exact original/configured text, owner/mode,
added addresses and before/after clock reports. Restore refuses a subsequently
edited profile, removes only this run's added sources and restores original
bytes/ownership; an interrupted apply can be restored from the saved mutation
intent. The apply and restore commands were then documented in the root README.

The source expansion was installed on October 13, with receipt under
`artifacts/hardware/clock-sources-20261013T095808Z/`. All four servers responded
at follow-up. The immediate gate had only 8.3 microseconds remaining correction
but skew 11.877 ppm; the next short check had 1.38 ms correction and 24.917 ppm
skew after source selection changed. Neither check passed the 10 ppm limit.
New sources had only six samples over about 135 seconds at the first follow-up.
A later bounded watch under `artifacts/hardware/20261013-clock-convergence-watch/`
recorded 13 of 13 passing gate samples over 120 seconds, including three
readings spaced one minute apart; skew fell from 9.348 to 5.562 ppm, with the
last correction about 0.17 ms. This qualifies short-run local convergence
against the installed 10 ppm gate. It does not qualify Intel–Thor offset,
source loss, repeated reboots or longer Wi-Fi stability. Multiple reachable
sources can improve selection but do not guarantee the existing 10 ppm skew
limit. Check only `check_clock_ready.py` afterward; preserve the other successful
stationary checks. Longer convergence, source-loss behavior and post-change reboot verification
remain separate qualification gates. The maintained time
installer must preserve this profile's pool source when re-rendering upstreams.


### Poll interval and clock startup gate

A later feedback-plugin install preflight failed **before service changes**:
the selected Ubuntu source was last updated 219 seconds earlier despite
normal leap status, 0.025 ms correction and 1.812 ppm skew. Chrony had raised
its selected-source poll interval to 256 seconds. The installed readiness gate
requires a reference age at most 180 seconds, so the source schedule and gate
contradicted each other. All original platform/guard services stayed active.

The staged [poll-bound tool](https://github.com/Cognitive-Software-Labs/DyNAMO-Ridgeback/blob/077692ab033fd3a86d4849bbbbdb443a9f0c2125/tools/hardware/bound_clock_poll.py) changes
only the Intel `pool` line to add `maxpoll 7` (128 seconds), and sets the same
limit on current sources through chronyc without restarting chrony or stepping
the clock. It keeps the 180-second freshness gate and 500 ppm slew policy.
Its receipt records source addresses, original profile and rollback. A missed
poll can still fail the gate, as it should. The poll bound was installed on October 13; receipt is under
`artifacts/hardware/20261013-clock-poll-stage/`. After `chronyc burst 2/4`,
the local gate passed all six checks: remaining correction 0.120 ms, skew
1.315 ppm, and reference age 0.656 s. This verifies local convergence at that
instant. A supervised reboot after this change then passed the maintained
`verify_feedback_boot.py` check: the boot ID changed, the platform clock gate
completed successfully before platform startup, local clock readiness passed,
and the pinned feedback plugin and other stationary services returned. Evidence
is under `artifacts/hardware/20261013-feedback-clock-postboot-2/`. Repeated
cold starts and loss of all external sources remain open. Thor is unchanged. Chrony's
[official configuration reference](https://chrony-project.org/doc/4.5/chrony.conf.html)
defines `maxpoll` as a power-of-two interval, and its
[control reference](https://chrony-project.org/doc/4.5/chronyc.html) notes a
runtime poll change takes effect after the next measurement.

A later 16-second read-only stationary batch after the motor-link recovery
passed all 22 checks. Front, rear and merged scans were each near 40 Hz, IMU
near 50 Hz, with asserted physical/software stops, stationary raw odometry,
guarded routing, active services and a passing local clock gate. The report is
under `artifacts/hardware/stationary-batch-20261013T130511Z-ca5147/`.
This is stationary health, not driving or Intel–Thor time qualification.

## October 14 startup blocked by remaining clock correction

The camera traffic screen at `camera-port-screen-20261014T084637335854Z`
stopped before any readers or packet capture. Its probe received no firmware
stop messages or raw odometry and found no controller. A separate service
inspection showed `clearpath-platform` in `activating/start-pre`, with the
guard running. The installed clock pre-start gate now waits up to 900 seconds
with a 5 ms remaining-correction limit (`TimeoutStartSec=930`). This differs
from the earlier 180-second configuration described above; the current
maintained auxiliary installer also renders 900 seconds.

Chrony had a selected external source, normal leap, recent reference and skew
around 2–3 ppm, but its **System time** remained about 344–357 ms fast. Small
latest sample offsets did not mean that pending correction had completed.
The installed `makestep 1 3` excludes sub-second offsets from stepping and
`maxslewrate 500` can take roughly eleven more minutes to remove 344 ms.
Thus a reachable NTP source alone does not guarantee startup within the gate's
15-minute window. This is a reproduced startup-availability limitation, not an
e-stop-release observation or evidence that a camera subscriber broke the base.
No time step, clock configuration change, service restart or gate relaxation
was performed during diagnosis. A boot-specific correction policy needs
qualification before closing repeated-boot readiness; do not automatically step
an active sensor/PTP clock to conceal this failure.

## Boot clock step and S6 boots

Root cause of the October 14 startup delay: the RTC keeps whole seconds, so
Intel can boot up to a second off, and `makestep 1 3` slews anything below a
second at 500 ppm. On that boot `phc2sys` started at 7.7 s and the sensors and
camera at 13.8 s, before chrony selected its first source at 16.3 s, so even a
correct step would have jumped live sensor stamps. `dynamo-boot-clock.service`
(branch `feat/boot-clock-policy`, installed with `time_sync boot-step-install
--host intel` on October 14) runs after chrony and before `clearpath-robot`,
the sensors, platform, camera and both PTP units. It waits up to 60 s for an
external source, steps once above 5 ms, and otherwise leaves holdover; it never
steps under a running consumer. `tools/hardware/verify_boot_clock.py` checks a
boot read-only.

| Boot | Start | Offset found | Action | Consumers started | Platform gate |
|---|---|---|---|---|---|
| `7bbe3817` | Oct 14 14:26 UTC, reboot after install | +55.5 ms | boot step stepped at 15.9 s | PTP 16.95 s, sensors/camera 21.6 s, platform 28.7 s | passed (journal ends at 31 s; host then off, cause not recorded) |
| `1ea01ea5` | Oct 16 08:39 UTC, cold start after two days off | −1.43 s | chrony's own `makestep` at 16.3 s; boot step found +0.000 ms and did nothing | PTP 16.9 s, sensors/camera 21.6 s | 0.1 s; verifier passed all checks |
| `89f18506` | Oct 16 08:44 UTC, supervised reboot | +15.3 ms | boot step stepped at 16.2 s; −2.6 ms one second later | PTP 17.2 s, sensors/camera 21.9 s | 0.1 s; verifier passed all checks |

Both observed paths stepped before any consumer started and the platform clock
gate no longer waits. Three of three boots passed. **S6 is recorded as passed with internet time
available** (user decision, 2026-10-16). A boot without the internet NTP source
was not tested: the boot step's 60-second give-up path is covered only by
offline tests, and the platform clock gate is then expected to hold the base
until time returns. Retest that path before relying on the robot where internet
time can be missing at boot. Evidence: `artifacts/hardware/20261014-s6-boot-clock/`.
