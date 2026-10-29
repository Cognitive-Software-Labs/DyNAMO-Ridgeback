# Puma feedback-consumer repair, installation and stationary interruptions

Recorded dates: 2026-10-12, 2026-10-13
Tested revisions: unknown
Provenance: partial

This record covers the inspection of the stock Puma hardware consumer, the
qualification and installation of the feedback-consumer repair, and the first
stationary interruptions of real wheel feedback on the physical Ridgeback. The
source text recorded no repository commit for the builder, stager, installer or
checkers, so tested revisions are unknown. Receipts on the robot host pin
source, library and input hashes, but were not copied into the repository. All
directories named below are on the robot host under `artifacts/hardware/`,
except where a journal is named. The text was moved here from the
[wheel-feedback reference at the documentation snapshot](https://github.com/Cognitive-Software-Labs/DyNAMO-Ridgeback/blob/077692ab033fd3a86d4849bbbbdb443a9f0c2125/docs/physical/drive_feedback_diagnostics.md).

## Stock consumer inspection

An offline harness executed the actual `read`, `write`, joint-update and
command-building method bodies of the installed 2.9.4 consumer against inert
feedback and command sinks. After one complete nonzero feedback batch, one
hundred simulated 50 ms cycles without feedback retained the old velocity,
returned success and emitted four hundred nonzero wheel-command values. It also
reproduced partial batches mixing new and old states, an unknown joint name
inserting a map entry that overwrites joint index zero, and nonfinite speed
accepted without a read error. Installed library read/write disassembly was
consistent with the source. These cases were not shown to occur on the live
robot; no live feedback loss or malformed feedback was injected. Provenance,
disassembly and results are under `artifacts/hardware/20261012-feedback-consumer-offline/`.

## Candidate qualification

Earlier offline candidates failed the controller-manager gate. Accepting
already aged input without spending that age from the monotonic budget
extended inhibition toward 500 ms, and collecting only one middleware sample
per read accumulated a backlog behind the 20 Hz loop. Candidates
`reproducible-2` through `reproducible-4` are superseded for those reasons;
`artifacts/hardware/20261012-feedback-consumer-reproducible-5` passed. The passing manager test on
loopback-only ROS domain 215 used a fake URDF, a forward-command controller,
25 Hz fake feedback and a 20 Hz loop, stopped and restored feedback while
commands continued, and verified zero commands, deactivation, no automatic
restart, explicit stationary recovery without command replay, new-command
acceptance and orderly deactivation. The manager logged an
interface-mode-switch error when removing interfaces before controller
deactivation, although the controller was inactive and fake commands stayed
zero.

## Plugin prefix and installation attempts

On 2026-10-13 the plugin-only stager built an overlay from `reproducible-5`.
The first isolated manager test of the manually assembled prefix failed before
activation because `package.xml` was missing; adding it fixed plugin discovery.
The second fake-wheel test loaded the staged library by exact path and passed
all 19 checks, including zero commands about 250 ms after feedback stopped,
controller deactivation and explicit neutral recovery. Evidence is under
`artifacts/hardware/20261013-puma-plugin-prefix-manager-check-2/`; the stager reproduced the
verified library and metadata under
`artifacts/hardware/20261013-puma-feedback-plugin-stage-reproduced/`.

The first live install attempt used a minimal prefix without the package's
executable directory. Its fake-wheel manager test had passed because it
exercised only the hardware plugin, but the real Clearpath launch failed when
resolving `clearpath_hardware_interfaces` executables. The installer removed its
drop-in and restarted the original platform. Its receipt stayed at `applying`
after the sudo process ended; the separate
`artifacts/hardware/20261013-puma-feedback-install-stage-3/recovery-audit.json` independently
verified the original apt plugin loaded, platform and guard active, a
stationary physical stop, unchanged camera, sensor and desktop services, and an
absent drop-in. The unused first pinned prefix remains on disk as evidence and
is not selected.

The revised stager added symlinks to the six original apt executables beside
the new Puma library and companion plugin descriptors. Stage 5
(`artifacts/hardware/20261013-puma-feedback-install-stage-5/`) used its full manifest hash for a
distinct root prefix and passed the fake-wheel manager checks, launch argument
parsing, package lookup and all six executable-resolution checks. Its install
first stopped before any service change because the local clock reference was
older than the platform's 180-second gate. After the chrony source poll was
bounded to 128 seconds and a fresh NTP sample obtained, the same stage
installed. Its receipt reports `installed-verified-stationary`: the running
controller manager mapped the exact pinned `libpuma_hardware.so`, the physical
stop was engaged, raw wheel odometry was stationary, the platform controller
and speed-guard routing were active, and camera, sensor and desktop services
were unchanged.

## Live feedback pause

With the physical e-stop engaged, a guarded test paused the actual
`multi_puma_node` process for 0.8 seconds. An independent systemd timer and the
observer both resumed it; the driver and controller-manager PIDs were
unchanged. ROS wheel feedback disappeared for the measured outage. Two zero
wheel-command messages arrived 0.290 seconds after the pause began, and the
controller manager logged a hardware read error and deactivated the platform
controller. The raw result under `artifacts/hardware/20261013-live-feedback-outage/` is labelled
**inconclusive** by its checker, whose zero-command window then started at
0.300 seconds, after those messages. The separate timestamped interpretation in
`artifacts/hardware/20261013-live-feedback-outage-analysis/` keeps that raw status and explains the
corrected 0.250–0.500 s window. This showed the installed plugin's response to
real ROS feedback interruption while stopped, not braking while moving,
electrical CAN loss, or how quickly the motors receive zero after the driver
resumes.

With the stop still engaged, only `clearpath-platform.service` was restarted. A
three-second recovery capture then verified real feedback, zero wheel commands,
stationary odometry, the active platform controller and the same pinned plugin
(`artifacts/hardware/20261013-feedback-outage-recovery/`). This was a supervised stationary
recovery, not a moving or repeated-fault qualification.

## Stationary incoming-link drop

The first run of `check_motor_can_ingress.py`, with the physical stop pressed,
added an INPUT-only UDP rule for MCU `192.168.131.2` to PC
`192.168.131.1:11412` after arming an independent systemd removal timer. The
baseline held 65 aggregate feedback samples over about 3.2 seconds. After the
rule was inserted one already in-flight aggregate arrived, and no later
aggregate appears in the captured blocked interval. The controller-manager
journal reports its read-cycle fault and controller deactivation about 290 ms
after insertion. Eight motor-command messages captured before the observer
stopped were all zero. The observer's **raw status is failed**: it treated the
resulting stale odometry as an observation failure at about 280 ms. Do not
relabel that run as a pass. Its finally block removed the exact rule
(`manual_rollback_rc=0`, final rule check absent). Evidence is in
`artifacts/hardware/20261013-stationary-can-ingress/` and the platform journal around 11:16:30 UTC.

The checker was changed to tolerate odometry going stale after a fault and to
require restored feedback before returning an observed status. A same-day
repeat returned `stationary-can-ingress-observed`: 64 baseline samples, two
samples already in flight during the 0.8-second block and 39 after removal. The
driver PID stayed the same and the rule was absent afterwards. All observed
motor commands and raw wheel velocities were zero. Command and odometry streams
did not resume after incoming feedback returned, so restored transport alone
did not restore active control (`artifacts/hardware/20261013-stationary-can-ingress-repeat/`).

Under the still-pressed stop, a platform restart and the read-only recovery
checker passed after each run: pinned driver and plugin loaded, active
controller, fresh real feedback, zero commands and stationary odometry
(`artifacts/hardware/20261013-can-ingress-recovery/` and `artifacts/hardware/20261013-can-ingress-repeat-recovery/`).
The firewall drop models full incoming motor-link loss at the PC boundary, not
one missing wheel field, a broken CAN conductor, failed outgoing zero delivery
or physical stopping under motion.

## Raised-wheel attempts

Two raised-wheel attempts on 2026-10-13 did not reach fault injection. In the
second the physical stop was released, but every recorded Joy message stayed
neutral; the configured Bluetooth controller was subsequently confirmed
disconnected. The recorder was changed to check the physical controller link
before arming a bounded moving trial. Neither attempt is evidence of a moving
fault response.

The moving check planned at that time had to keep the pinned driver and its
command delivery running while withholding or invalidating only feedback,
because the 0.8-second pause had stopped the whole driver, which both publishes
feedback and consumes commands. Under the commissioning test interlock it was
to start from a fresh pressed stop, neutral controller, active platform and
verified fresh feedback; use one bounded forward permit at the 5 cm/s cap;
inject only after measured forward motion began; and record the limited
command, motor command, raw wheel odometry, fault time, controller lifecycle
state and physical stopping distance. A persistent injection needed an
independent cleanup watchdog. A pass required zero motor commands, controller
fault latching, measured stopping and a supervised pressed-stop recovery. The
prepared invalid-batch check published one malformed ROS feedback batch under
a one-shot permit that expires even if the terminal closes; it tests the
consumer's invalid-batch path rather than missing CAN frames. D6 and D7 were
later prepared with `check_raised_wheel_link_fault.py` instead.
