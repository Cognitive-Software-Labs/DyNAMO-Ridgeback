# Stationary auxiliary-sensor checks

This reference records at-rest inspection limits for the Ridgeback IMU, battery
telemetry and supporting scans. The sensor measurement plan owns remaining
acceptance work; the [driving reference](driving_readiness.md) owns actuation.

## At-rest capture

On 2026-10-13 a 25-second subscriptions-only recording, with five seconds
excluded for discovery, observed the following on `r100_0160`, domain 0:

| Signal | Received rate | Largest receive gap |
|---|---:|---:|
| Front scan | 39.94 Hz | 36.6 ms |
| Rear scan | 39.95 Hz | 50.1 ms |
| Raw/processed IMU | 50.00 Hz | 25.6 ms |
| BatteryState | 10.03 Hz | 102.0 ms |
| Raw wheel odometry | 20.00 Hz | 55.3 ms |

All 25 received firmware e-stop reports were asserted, and raw odometry stayed
stationary. Captured stamped streams had no duplicate or regressing stamps in
the measured window. No camera subscribers, commands, control service requests,
Bluetooth transitions or service restarts were used.

An explicit merged-scan subscription in a separate 20-second capture received
39.91 Hz after discovery, with a maximum gap of 35.3 ms. The topic was absent
from the first immediate graph inventory; absence in that inventory did not
establish absent delivery. The middleware exception and lazy conversion pipeline
require explicit discovery/subscription evidence. Neither short capture qualifies
long-run scan continuity or Hokuyo timeout recovery.

Raw IMU frame was `imu_0_link`. Mean acceleration magnitude was 9.813 m/s²;
mean angular rates were approximately (0.00055, −0.00087, −0.00048) rad/s.
These are recorded at-rest means, not calibrated biases or verified axis signs.
Raw orientation marks orientation unavailable; processed orientation carries
all-zero covariance. ROS defines those zeros as unknown covariance, so no measured orientation
certainty can be inferred. Sensor calibration and extended bias/drift
observation remain open; consumer selection and source diagnosis are below.

Battery telemetry reported 26.48–26.59 V and 87.7–88.8% during the capture,
with current 1.44–2.77 A. Temperature and cell-temperature fields were NaN;
cell-voltage entries duplicated the pack voltage. Per-cell health, temperature
and charge accuracy are therefore not established. Battery guard, sysfs bridge
and power logger were active. The guard configuration remains `ACTION=warn`;
this observation did not test warning thresholds or change shutdown policy.

Evidence, raw messages, recorder source and analyses are under
`artifacts/hardware/20261013-stationary-health/`. The recorder used monotonic
receive times; reported rates do not depend on inter-host clock synchronization.

## Time and boot limitations

Chrony reported the host about 170 ms fast early in this inspection, decreasing
to about 100 ms fast roughly two minutes later, despite a selected normal NTP
source. A small latest NTP measurement offset did not mean the system clock had
finished correction. No clock step or configuration change was made. Verify
convergence and startup/time-readiness policy before cross-host acquisition-age
claims. Thor offset, holdover and synchronized sensor timestamps are unqualified.

The installed desktop/guard repair survived the observed October 13 boot with
correct listeners and zero reported restarts. Detailed setup ownership and the
limits of that single boot live in the [recovery runbook](ridgeback_recovery.md).

## Source diagnosis and installed repairs

The installed ROS message definition explicitly interprets zero IMU covariance
as **unknown**, not zero uncertainty. No nonzero orientation covariance is
invented from the short at-rest window. The EKF's live selection uses IMU yaw
rate and x acceleration, with gravity removal, but does not fuse IMU orientation
as an absolute heading measurement. The raw gyro covariance is a supplied model;
measured short-window scatter and mean are characterization, not a qualified
bias/noise calibration. Multi-orientation reference measurements, temperature
stability and longer stationary characterization remain needed before changing
calibration constants. The earlier concern about zero covariance alone is
resolved as message semantics, not a new sensor defect.

A real configuration mismatch was reproduced: generated YAML targets
`imu_filter_node`, while the launch runs `imu_filter_madgwick`. Live parameters
showed default `use_mag=true`, despite the intended `false`. An isolated real
filter test on domain 216 reproduced ignored settings and verified a corrected
selector restores `use_mag=false` and `publish_tf=false` without changing the
node name. The installed platform wrapper passes corrected parameters through the
existing launch argument, preserving the commissioning wrapper. The separate
upstream launch-name patch is an alternative and is not applied to the host.
This repairs parameter selection, not absolute orientation/gyro calibration.

Installed Clearpath BatteryState code initializes temperature to NaN, derives
charge percentage from a pack-voltage lookup, and synthesizes cell-voltage entries
from pack voltage divided by the battery configuration's series count. For this
configuration those entries repeat pack voltage. They are not independent cell
measurements. The maintained narrow battery patch and installed root-pinned Python
overlay instead emit NaN for each unknown individual voltage, per BatteryState
semantics; tests execute the actual patched method and confirm pack fields are
unchanged. No available independent cell-voltage or battery-temperature sensor
was identified in this data source. Obtain hardware telemetry before declaring
those values measurable; CPU/MCU temperature is not battery temperature.

Clock correction is caused by the installed time profile's `maxslewrate 500` and
`makestep 1 3`: a sub-second boot offset can converge slowly instead of stepping.
That profile belongs to the ongoing Intel–Thor time workflow; this repair does
not replace it or step a running robot clock. Subsequent monitoring recorded
about 39 µs remaining system correction, passing the local convergence gate;
root delay/dispersion still gave an estimated UTC error bound near 17 ms. Do not
interpret the small remaining correction as absolute UTC/cross-host accuracy.
The gate checks the **System time** field, not just Last offset, and rejects
unsynchronized/local-only sources, stale references and nonfinite reports.

The maintained `install_auxiliary_repairs.py` repair was **installed and verified
stationary on 2026-10-13**. The first application left its receipt at `applying`;
verification-only completion checked the installed state without another restart.
It changes only new scoped platform integration/helper files, keeps all existing
caps/Puma sources untouched, and adds bounded clock readiness before platform
startup. A failure restores its own prior integration; installation requires an
asserted-stop stationary probe. Final verification recorded seven asserted stop
reports, 139 stationary raw-odometry samples, correct guarded routing, 100 IMU
messages with intended filter settings and 20 BatteryState samples with unknown
individual voltages and valid pack voltage. Local clock readiness passed with
about 2 µs correction remaining; the estimated UTC error bound was still about
20 ms, not microsecond absolute accuracy. Protected camera, sensor, merger and
desktop service states matched their prior process IDs. Camera, sensors, merger and desktop are outside
its restart scope. Public stage/apply/restore commands are in the README.
Physical calibration, new battery sensing and repeated boot qualification are
separate from these software corrections.

## IMU characterization and actual battery hardware boundary

A further 60-second post-repair fixed-pose raw IMU capture recorded 2,998 samples
with the stop asserted and stationary odometry. Gyro mean was approximately
(0.000084, −0.001157, −0.000981) rad/s; standard deviations were
(0.000511, 0.000547, 0.000547) rad/s. The first/second-half means changed by
roughly (−0.000060, −0.000087, −0.000082) rad/s. These are a reproducible
zero-rate/noise characterization, not constants installed as calibration.
Accelerometer mean was (0.00516, −0.00439, 9.79655) m/s², which still combines
sensor error, gravity and mount alignment. The maintained recorder preserves
raw samples and covariance; it never changes the live filter. Evidence is under
`artifacts/hardware/20261013-imu-stationary-characterization/`.

Full calibration needs independent reference measurements across orientations
and known rotation rates, plus repeat observations through relevant temperature
conditions. Do not tip or lift the Ridgeback to collect arbitrary calibration
poses. If the embedded sensor is not safely accessible, arrange a technician's
fixture/approved reference procedure or calibrate its installed output against
an external calibrated reference. Apply constants only after independent
validation and defining their source/temperature validity. This cannot be
completed from a single stationary pose by software alone.

The installed R100 Proton protocol confirms exactly one pack-voltage channel
plus accessory rails, one aggregate current, and two temperature channels labelled
MCU and PCB. During the capture those temperatures were around 42.2°C and
24.8°C. Neither is battery temperature. No independent cell-voltage or battery
thermistor/BMS channel is configured. The Linux `test_power` battery device is a
synthetic desktop bridge: its `TEMP=26` and chemistry/model defaults are not
physical battery evidence, and the bridge does not update temperature from a
battery sensor. No connected smart BMS was established by this inventory.

The operator could not identify the installed battery model/chemistry. Confirm
it from an accessible label or integrator record before selecting monitoring
hardware or changing the estimator model. Actual battery temperature and
individual voltages require suitable sensors or a supported BMS, installation
by qualified personnel, and measured channel correspondence. Software must not
substitute MCU/PCB temperature or inferred voltages. The current output correctly
marks those fields unknown; new physical sensing remains a hardware dependency.

## Reboot qualification prepared

The installed platform clock gate's successful prestart was verified during the
auxiliary-repair service start. Its first actual reboot check passed on 2026-10-13. A pre-reboot boot ID is saved in the installer receipt, and the
maintained `verify_auxiliary_boot.py` will check that the boot changed, inspect
the named gate's successful non-ignored prestart record, verify service/listener
ownership and read actual stationary IMU/battery data. It sends no commands.
An operator confirmed the robot stationary with its physical e-stop pressed for
this reboot check; leave the controller centred and untouched throughout.


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
added addresses and before/after clock reports. To undo this experiment, use
`sudo python3 tools/hardware/expand_clock_sources.py --restore --operator-confirmed
--run-dir <the-original-run-directory>`; it refuses a subsequently edited profile,
removes only this run's added sources and restores original bytes/ownership.
An interrupted apply can be restored from the saved mutation intent.
Re-running apply requires a fresh directory and the original profile.

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

The staged [poll-bound tool](../../tools/hardware/bound_clock_poll.py) changes
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

## Boot clock step (S6)

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
