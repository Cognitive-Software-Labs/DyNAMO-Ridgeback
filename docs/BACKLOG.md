# Documentation-backed engineering backlog

This backlog owns robot, exploration, deployment, and cross-cutting gaps.
The [target-distance benchmarking backlog](target_distance_benchmarking/BACKLOG.md)
owns benchmark tooling and controlled estimator-evaluation work. Each item
has one owner; cross-cutting prerequisites are linked rather than copied.

Only active, evidenced gaps belong here. Implemented behaviour belongs in the
technical references; dated experiments and migrations belong in the archive;
unselected alternatives belong in `docs/do_not_try_again/`. Closing an item means satisfying
its completion criteria and moving durable results to the relevant reference or
archival record, rather than retaining a crossed-out entry here.

## Isaac exploration recertification / P5

**Gap.** No statistically valid autonomous Isaac exploration baseline exists
for the current robot. The September 17–18, 2026 closed-loop qualification
passed its bounded gates; it did not qualify frontier exploration or supply a
matching Gazebo coverage comparison. Its noisy-seed tradeoffs motivate the
SLAM-source follow-up below. Earlier invalidated exploration numbers remain void.

**Completion criteria.** With corrected seating, regenerated maps, and the
qualified LiDAR pipeline, run autonomous exploration over 3–5 seeds per
comparison condition on a suitably quiet host. Establish a current matching
Gazebo baseline. P5 passes
only when 3/3 runs complete, coverage is at least the Gazebo mean minus 10
points, genuine aborts do not exceed the Gazebo maximum, and throttled headless
RTF is at least 0.8.

**SLAM-source follow-up.** Compare `front_only` and `merged` with matched
seeds at zero and configured noise during exploration. Investigate the noisy
seed-0 regression and larger EKF/SLAM yaw components using signed yaw traces
and total yaw error; component RMS values are not additive. Report per-seed
pose/map quality, coverage, aborts, and timing alongside aggregate results.
Keep `front_only` as the public default until a separate, evidence-backed
default decision addresses these tradeoffs and the exploration/P5 results.

**Context.** [Exploration evaluation](exploration/evaluation.md),
[Isaac lidar pipeline](isaac/lidar-pipeline.md), and the resolved orphan-lidar
investigation in the archived evidence below.

## Physical stationary integration

**Gap.** The package split and workstation checks are complete, and Intel–Thor
transport tooling and preliminary network experiments are recorded. Matching
robot installations, live GPU localization, the physical sensor suite, and joint
stationary target tests have not been qualified. The three plans own their
current progress and remaining acceptance gates:

| Plan | Owner | Completion boundary |
|---|---|---|
| [PHYSICAL — Package split for distributed deployment](plans/PHYSICAL_package_split.md) | Workstation | Independently installable localization, preserved Gazebo/Isaac and benchmark workflows, tested interfaces and handoff. |
| [PHYSICAL — Intel–Thor deployment and transport qualification](plans/PHYSICAL_intel_thor_deployment.md) | Robot deployment team | Matching installed revision, GPU offload, measured transport/performance, stationary failure and recovery evidence. |
| [PHYSICAL — Robot measurements and stationary sensor validation](plans/PHYSICAL_robot_measurements_and_validation.md) | People at the robot with agent assistance | Quick sensor-mount readings/photos, agent-run sensor/alignment checks, and distributed G1/person tests. |

**Coordination.** The workstation handoff is available in the package-split plan;
both robot hosts must acknowledge the same installed revision before final
distributed tests. Mount measurements and sensor inspection can proceed in
parallel. Shared-code defects return to the workstation; robot teams own machine
configuration and physical evidence. Start with the D455 on Intel and explicit
per-host commands; camera relocation is an evidence-led alternative. One-command
remote startup and boot services are optional later stages.

**Auxiliary findings.** The [October 13 at-rest check](physical/stationary_auxiliary_checks.md)
observed healthy short-window sensor delivery, but IMU covariance/calibration,
battery field accuracy and startup timing remain unqualified. Source diagnosis
found an IMU parameter-selector mismatch and synthesized cell-voltage fields;
the rollback-capable platform-only repair was installed and independently
verified stationary on October 13. Zero IMU covariance
and NaN temperature are valid unknown/unmeasured semantics, not calibration
values to fabricate. The clock converged and the bounded platform startup gate is installed;
one supervised post-install reboot now verifies about 38 seconds of clock-gate
waiting before successful platform startup and preserved sensor/desktop/data
contracts. Repeated cold boots and external-source loss remain unqualified.

**Completion criteria.** All three plans pass their stationary boundaries on
the recorded configuration. Apply the existing camera, geometry, calibration,
and command-chain criteria independently: stationary success does not close
moving-robot calibration, physical stop/clearance, simulator contact, or
autonomous-mission work. Pointcloud qualification remains optional.

## Physical command-chain validation

**Deployment gate.** The hardware adapter is attach-only and autonomous motion
defaults off. Stationary live inspection confirmed a compatible stamped
mux/controller interface and an active four-wheel mecanum controller. Legacy
generated `use_stamped_vel: false` is not the installed controller's interface
selector. A commissioning guard is installed with 0.05 m/s planar and 0.1 rad/s
yaw command caps; stationary routing verification passed. Physical stopping
remains partially qualified: bounded command loss and gently ramped motion in
all six directions recorded clean stops with operator confirmation. A low-speed
deadman release also stopped with operator confirmation, with trial deviations
preserved in the driving reference. Physical e-stop stopping was observed with
forward still held, and neutral e-stop release produced no movement; the e-stop
trial exceeded its requested one-second exposure. A deliberate controller disconnect during motion now recorded automatic
stopping before the bounded-permit backstop, with operator confirmation.
Silent radio loss/reconnection, takeover and cold boot remain open. Reconnection
is now a reproduced failed gate: Bluetooth reconnects but the driver reports
zero physical reports and ROS input stays neutral despite operator input.
A verified device-specific disconnect and manual reconnect restored live input
while the stop remained pressed. The root cause, neutral-return evidence and
automatic reconnect reliability remain open; manual recovery is not a gate pass.
On 2026-10-12 the platform-restarting autoconnect handler was replaced by a
root-pinned paired/trusted-device helper that makes no ROS/base service calls.
Its installation preserved platform/guard/sensor/camera process IDs and verified
asserted-stop stationary state. A subsequent cached-state recording observed changing physical and virtual
input and return to neutral, with asserted-stop/stationary probes before and
after. A subsequent stationary ROS capture also verified changing joystick/combiner
input and neutral return, with asserted-stop suppression and zero limited/wheel
commands and raw odometry. One post-repair neutral reconnect plus companion live ROS input/neutral-return
recording now passes with stop suppression and no platform restart. Repeated
reconnect reliability and silent radio loss remain open; link-connected status
alone is insufficient.
Abrupt direction tests encountered repeatable startup-transient
aborts; resolve physical versus feedback-timing causes before qualifying those
profiles. The diagnostic fault-mask and cache-memory repair is installed and verified
stationary from a root-pinned package, preserving the test lock. The old
unpatched build was withdrawn;
the CAN-rate/freshness investigation remains
open. A stationary paired capture separates steady status-speed reception on
all wheels from rear setpoint-response count asymmetry; loss location and
per-field freshness are unqualified. An offline completeness/diagnostic-read
patch passes extracted-routine checks and full compilation, but timing replay
predicts about 16.5 Hz and 200 ms publication gaps. Consumer inspection/offline reproduction shows stale states retained and
commands continuing without an age check. An offline consumer candidate adds age/batch validation, latched command
inhibition and stationary lifecycle activation; extracted-body checks pass and
the package compiles and a direct compiled-plugin fixture passes loopback
ROS messaging checks. The corrected candidate also passes controller-manager initialization,
feedback-loss inhibition and explicit neutral recovery with fake wheels. The
plugin-only deployment now passes stationary install and reboot checks. A
supervised interruption of the real ROS feedback stream with the physical stop
engaged produced zero wheel commands at about 290 ms and deactivated the
controller; a stationary platform restart restored active, zero-command
operation. Qualify braking under movement, electrical CAN loss and repeated
fault recovery before deploying the driver-completeness candidate. Details are in the [wheel-feedback reference](physical/drive_feedback_diagnostics.md).
An incoming-only motor-CAN/UDP interruption with the physical stop pressed
also removed aggregate feedback and faulted the controller about 290 ms later.
The corrected stationary repeat observed feedback return after the exact rule
was removed, but control remained inactive until a supervised platform restart;
stationary recovery passed. This is not a passed moving gate.
The separate bounded invalid-feedback trial is prepared for securely raised
wheels; full bidirectional electrical-link loss and per-wheel field freshness
remain open.
The native e-stop publisher/mux subscription had incompatible QoS. A reliable
guard stop relay and mux-only remap passed isolated mock checks and were
installed and verified stationary on 2026-10-09. A deliberate wireless
controller power-off and a device-specific automatic disconnect both reset held
input to neutral while the physical stop was held; the automatic trial retained
twenty seconds of neutral input and zero commands/wheel motion.
Silent radio-loss/reconnection qualification remains open. The [stop-delivery reference](physical/driving_readiness.md#e-stop-delivery-to-the-mux)
owns its deployment state.
Automatic moving-disconnect testing is disabled after a preflight-unarmed
trial contained prolonged commands without triggering disconnection. A bounded test-permit interlock is now installed and verified stationary;
Neutral release and held-command suppression with the physical stop released
are now verified, including operator confirmation of no movement. The bounded-permit workflow now cut commands despite ongoing source input and
permits, and the operator confirmed stopping while forward remained held.
A repeat interlocked deliberate moving-disconnect trial now passed its recorded
command-stop gate and operator physical stopping observation. Sustained/silent
radio loss and reconnection remain unqualified.
Manual mux inputs bypass the Nav2 collision monitor; optional software stop
locks had no publishers during inspection. The maintained
[driving reference](physical/driving_readiness.md) owns findings and the
remaining test sequence.

**Completion criteria.** On the exact approved robot configuration, trace the
message type, topic, QoS, frame/timeout semantics, mux ownership, collision
monitor, controller input, e-stop, and deadman behavior from Nav2 to the base.
Reconcile drive feedback frequency targets and direct/aggregated fault
reporting. Resolve any mismatch at the narrowest owning layer; then perform stationary,
lifted/blocked-wheel where approved, low-speed clear-space, stop, timeout, and
e-stop tests before enabling `autonomous_motion_enabled:=true`.

**Context.** [Exploration architecture](exploration/architecture.md#hardware-safety-boundary)
and the public hardware workflow in the [README](../README.md). The
[stationary measurement plan](plans/PHYSICAL_robot_measurements_and_validation.md#both-lidars-and-stationary-auxiliary-sensors)
owns inspection only; its completion does not close this actuation gate.

## Hokuyo driver recovery after read timeouts

**Gap.** `urg_node` (`ros-jazzy-urg-node` 1.1.2) cannot recover from a burst
of read timeouts. It reconnects without closing its old TCP session, and a
Hokuyo UST serves one client at a time, so the new connection is refused
indefinitely. The robot then has no scans until someone restarts
`clearpath-sensors`, whatever the middleware. September 18 handoffs report three
subscriber-associated lockouts under the CycloneDDS services configuration and
measured camera traffic leaving the bridge onto the sensor/MCU ports. The
initiating mechanism remains unproven; the maintained
[transport blocker](physical/intel_thor_transport.md#camera-subscriber-blocker)
distinguishes observations from the multicast hypothesis.

**Progress.** The lockout mechanism was found and fixed on 2026-10-13: when
`urg_open` fails after the TCP connect, `urg_node` 1.1.2 throws without closing
the socket, and `urg_c` writes without `MSG_NOSIGNAL`, so a reset session killed
the driver with SIGPIPE. One recorded patch,
`patches/urg_node_reconnect_recovery.patch` on `src/urg_node` 1.1.2 (pinned in
`dependencies/core.repos` on `origin/dev`), closes the failed socket and ignores
SIGPIPE. It runs on `r100_0160` as the overlay
`/home/robot/ros_overlays/urg_node_fix`, listed under `system.ros2.workspaces`
in `/etc/clearpath/robot.yaml` (rollback copy `robot.yaml.bak-urg-node-fix`).
A rear-LiDAR session reset on 2026-10-13 reconnected the same driver process
after a 2.25 s scan gap. On 2026-10-14 (S3) `tools/hardware/observe_lidar_interruption.py`
reset the **front** session with the base e-stopped: the same driver process
reconnected after 2.25 s with no service or merger restart, the rear scan never
gapped more than 42 ms, and over the 30 s after recovery front, rear and merged
scans each held 39.93 Hz with gaps up to 43 ms (`artifacts/hardware/20261014T114600Z-s3-front-lidar-interruption/`). An earlier attempt
the same day is kept as `…-INVALID-settle-window`: its recovery was identical
but the tool's settle window overlapped the outage. The half-answering-sensor
case is shown only offline.

**Context.** Recovery steps are in
[troubleshooting](troubleshooting.md#hokuyo-drivers-stuck-reconnecting). The
incident timeline is in the archived evidence below. The CycloneDDS deployment is described in the
[transport reference](physical/intel_thor_transport.md).

**Completion criteria.**

- Resolve the services-configuration camera fault and extend `check_link` to
  cover an extra local camera reader under the installed services configuration,
  observing physical-port traffic, image delivery and LiDAR errors. A synthetic
  Ethernet-only probe pass does not satisfy this gate.
- A driver that loses its Hokuyo connection resumes publishing scans without
  manual intervention. Achieve this with a maintained `urg_node` patch under
  `patches/` that closes the old session before reconnecting, or with a
  documented supervisor that restarts only the sensor drivers. Demonstrate it
  by interrupting a LiDAR's TCP session on the robot. **Met** for a reset
  session on both LiDARs (see Progress); no supervisor is needed for that case.
- Either explain the 2026-09-18 simultaneous timeout, or bound its recurrence
  with a recorded multi-hour soak under CycloneDDS with the Intel services,
  SLAM, and Nav2 running.

## Remote desktop startup recovery

**Gap.** The October 5 independent VNC installation survived vendor webserver
reloads, but the October 6 cold boot recorded 37 VNC restarts, an NVIDIA EGL/GLX
crash and repeated display conflicts. The independent browser proxy did not
recover after its dependency failed; port 9005 reverted to a vendor-owned
listener. Desktop user-session/runtime failures were also recorded. The retained
DDS profile survived the boot, and a scan-only check passed; this is a desktop
startup gap, separate from camera transport.

**Context.** Read-only checks and ownership interpretation are in
[desktop startup troubleshooting](troubleshooting.md#remote-desktop-startup-and-proxy-ownership).
The camera-independent stationary setup repair was installed and independently
verified on 2026-10-12. The foreground VNC service owns display :1/5901;
independent units own 9005 and 9100, and the remote-screen override removes its
competing display launcher. All six root-owned file hashes/modes match maintained
sources; platform, guard, camera, sensors and merger kept their process IDs.
The vendor webserver remained inactive. Public installation and exact-file
rollback commands are in the [README](../README.md#stationary-desktop-and-controller-setup-repair).
The 2026-10-13 boot inspection now shows correct desktop listener ownership,
matching repaired files and zero reported desktop/guard restarts. One observed
boot does not establish repeated cold boots or browser continuity; backend-failure
recovery and vendor-reload continuity remain unqualified.

**Completion criteria.** Demonstrate clean cold boots with one owner for display
:1 and port 9005, a valid robot user session/runtime directory, automatic proxy
recovery after VNC failure, and browser/desktop continuity across vendor webserver
reloads. Preserve DDS, camera startup, robot YAML and merger settings throughout.
Resolve the graphics startup fault from captured evidence rather than accepting
repeated retries as successful boot qualification.

## Physical camera validation

**Gap.** The D455 input contract is implemented, but device identity, effective
profiles, colour/aligned-depth delivery, exact timestamps, TF ownership, and
application behavior still need proof on the deployed robot.

**Completion criteria.** Execute the D455 section of
[PHYSICAL — Robot measurements and stationary sensor validation](plans/PHYSICAL_robot_measurements_and_validation.md#d455-camera-validation):
verify the intended device and driver, inspect effective configuration, validate
image/depth/calibration contracts and timing, confirm the actual camera-to-base
TF chain, and run a stationary projective/Euclidean application smoke. Test
640x480@30 and 1280x720@30 separately; record unsupported/fallback profiles as
unresolved rather than silently accepting them. Freeze acceptance criteria
before measurement. Preserve evidence and compare live calibration with the
nominal simulation model without treating nominal values as hardware guarantees.

Close only after both required profiles and the core checks pass, or after an
explicit deployment-scope decision revises the required profile set. Pointcloud,
camera–LiDAR calibration, calibrated accuracy, and motion validation are separate.

**Context.** [Camera stack](target_localization/camera_stack.md) and
[aligned depth](target_localization/aligned_depth.md).

## Optional physical organized-pointcloud qualification

**Status.** Separate optional follow-up; not a blocker for core D455 colour and
aligned-depth integration. The hardware input contract leaves pointcloud unwired
until its organization and correspondence are demonstrated.

**Completion criteria if needed.** Before wiring a RealSense pointcloud input,
prove `PointCloud2.height > 1`, colour-grid dimensions and row-major pixel
correspondence, frame/TF correctness, compatible timestamps, and documented
invalid-point representation/density. Preserve device/profile provenance and
application evidence. An unverified or unsuitable cloud remains disabled.

**Context.** [Camera pointcloud contract](target_localization/camera_stack.md)
and [PHYSICAL — Robot measurements and stationary sensor validation](plans/PHYSICAL_robot_measurements_and_validation.md#completion-and-separate-work).

## Camera-LiDAR calibration

**Deployment gate.** Resolve this before treating polar-profiling measurements
from the physical robot as valid. Hardware bring-up and data collection may
precede it, but deployment must not rely on camera-to-LiDAR projection results
until the completion criteria below have been satisfied.

**Gap.** Simulation has exact model transforms; a documented physical
camera-to-LiDAR calibration and clock validation is absent.

**Why it matters.** Polar mask membership projects a free-running scan through
that transform. Spatial or temporal error can select the wrong beams.

**Completion criteria.** Define and execute a repeatable target/measurement
procedure; record transform convention, uncertainty, storage/ownership, and
timestamp evidence; validate projected beams on independent poses/ranges and
under robot motion; document recalibration triggers. Do not silently tune polar
segmentation to absorb an extrinsic error.

**Context.** [Polar profiling](target_localization/polar_profiling.md) and the
stationary alignment check in
[PHYSICAL — Robot measurements and stationary sensor validation](plans/PHYSICAL_robot_measurements_and_validation.md#stationary-cameralidar-alignment).
That field check can expose a calibration problem without requiring a workshop
survey. It does not establish formal extrinsic uncertainty or moving-robot
validity; those parts of this gate remain open after an integration pass.

## Shared camera mast and bracket geometry

**Implemented.** One shared Xacro now supplies both simulators' mast and bracket.
The camera pose and retained backend decks are preserved; Isaac adapts the shared
boxes onto its existing chassis body. See [shared camera support](robot/geometry.md#shared-camera-support).

**Remaining gap.** Detailed hardware attachment dimensions and mass/inertia are
not fully measured. Focused stationary/turn checks passed in both simulators;
broader sensor/contact qualification and affected benchmark reruns remain required.
The implemented shared definition alone does not close this item.

**Completion criteria.** Establish the measured attachment geometry in one
shared description or asset source consumed by Gazebo and Isaac. Remove the
independent Isaac authoring after equivalence checks. Preserve the configured
camera pose; compare rendered and collision geometry, TF, and sensor
self-occlusion in both simulators, including motion. Record physical dimensions
and measurement uncertainty. Rerun affected benchmarks before quoting results.

**Context.** [Shared robot geometry](robot/geometry.md#known-representation-differences)
and [Isaac additions](isaac/robot-model.md#hand-authored-parts). The
[field measurement visit](plans/PHYSICAL_robot_measurements_and_validation.md#quick-measurements)
supplies recorded offsets; mount photos were waived. Detailed
attachment dimensions and qualifying the shared assets remain owned by
this item, not prerequisites for the field visit.

## Collision-envelope and navigation-footprint validation

**Drive scope.** Retain the [Isaac planar drive](isaac/robot-model.md#why-this-abstraction-fits-the-current-work)
for flat-floor navigation/perception work. Physical roller simulation is deferred
until a traction, terrain, traversal, or wheel-control requirement justifies it.
Geometry and footprint validation can proceed independently; shared outlines do
not establish wheel-contact parity.

**Qualified simulator state.** The nominal local/global 16-point outlines remain
equal, enclose the aligned shared simulator envelope with 2 mm model margin, and
both costmaps pin the audited 10 mm runtime padding explicitly. The effective
footprint encloses the raw Gazebo and Isaac collision projections with at least
7.591 mm clearance. Front, side and 45° contact, held contact,
and stop/reverse recovery passed at 0.05/0.10/0.20 m/s across three cold boots
per backend. The [collision reference](robot/collision_model.md) owns the policy,
limits and reproduction path.

**Remaining gap.** Agreement with the deployed hardware, including attachments,
is unproven. The field visit supplied selected tape readings rather than a
surveyed planar outline, uncertainty model, or physical stopping/clearance test.

**Completion criteria.** Add the measured physical envelope to the existing
same-frame overlay, including height-dependent attachment limits and explicit
uncertainty. Fix any violation of the documented containment policy. Qualify
hardware clearance under the physical command-chain validation gate; simulator
contact results do not close that gate. Rerun affected benchmarks if a geometry
or footprint correction changes their inputs or measured behavior.

**Context.** [Collision model and footprint](robot/collision_model.md),
[Isaac colliders](isaac/robot-model.md#colliders), and
[physical command-chain validation](#physical-command-chain-validation). The
[field measurement visit](plans/PHYSICAL_robot_measurements_and_validation.md#quick-measurements)
flags visible protrusions; it does not supply a surveyed outline or contact and
stopping proof. Those are separate tasks under this gate.

## MyBotShop follow-ups

Open questions for the next call with MyBotShop, the robot's integrator.
Resolve each one here or move it to the item that owns it.

- **Rear LiDAR offset.** Where did x = −0.4278 come from in
  `/etc/clearpath/robot.yaml` (first seen 2026-08-06) and the scan-merger
  `params_ridgeback.yaml` (2026-08-10)? Was it measured? The merger comments say
  the offsets still needed live calibration. We changed it to the symmetric −0.3922
  ([deployed hardware mounts](robot/geometry.md#deployed-hardware-mounts)). Confirm that their
  tooling will not restore the old value.
- **Camera web readiness.** Earlier boot waits failed despite subsequently
  healthy camera topics. The direct discovery replacement and its deployment
  verification belong to [desktop startup recovery](#remote-desktop-startup-recovery).
- **Unit files changed on disk.** On 2026-09-18 systemd reported that the
  `realsense-camera`, `camera-web-ready`, `clearpath-platform` and
  `clearpath-sensors` units had changed since they were loaded. Which edits are
  intended, and is a `daemon-reload` safe?

## Archived evidence

- [LiDAR/closed-loop SLAM qualification](../archive/engineering/2026-09-17-isaac-lidar-qualification.md)
- [paired results and interpretation](../archive/engineering/2026-09-17-isaac-lidar-qualification.md#interpretation)
- [Isaac port history](../archive/engineering/port-history.md#lidars-detached-from-the-articulation-2026-09-11)
- [removed D435 transform](../archive/engineering/operational_incidents.md#d435-static-camera-transform--removed-2026-08-31)
- [Hokuyo reconnect lockout](../archive/engineering/operational_incidents.md#hokuyo-reconnect-lockout--2026-09-18)
- [r100_0160 field readings and LiDAR box checks](../archive/engineering/2026-09-18-r100-0160-field-measurements.md)
- [r100_0160 rear LiDAR side-target recheck](../archive/engineering/2026-09-24-r100-0160-rear-lidar-recheck.md)
