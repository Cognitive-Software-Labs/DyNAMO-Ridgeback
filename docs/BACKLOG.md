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

## Physical command-chain validation

**Deployment gate.** The hardware adapter is attach-only and autonomous motion
defaults off (`autonomous_motion_enabled:=false`). Keep it off until every open
gate below passes on the approved robot configuration.

**Qualified so far.** Every hop from Nav2 to the base carries `TwistStamped`
and delivers on the live graph (stationary, no goal sent). The
[commissioning guard](physical/commissioning_guard.md) bounds every source.
D1 (six bounded floor directions), D4 (live handover) and D5 (stationary
field-age acceptance) passed. The [driving](physical/driving_readiness.md) and
[feedback](physical/drive_feedback_diagnostics.md) references own installed
state; the [qualification plan](plans/PHYSICAL_remaining_qualification.md#driving-and-feedback)
schedules the trials and their acceptance counts.

**Open gates.**

- **D2.** Re-confirm deadman release, command loss and physical e-stop stops on
  the current build. Earlier observations remain historical evidence only.
- **D3.** Repeated abrupt and silent wireless input loss, neutral reconnect and
  restored input. One post-repair reconnect passed; repeated reliability and
  genuine silent report loss are untested. Manual recovery or link-connected
  status alone is not a pass.
- **D6.** Feedback fault while the wheels turn (raised), fault latch and
  supervised recovery.
- **D7.** Actual motor-link loss, including outgoing zeros that cannot be
  delivered, and recovery. It drops bridge packets; electrical CAN loss is a
  separate, untested case.
- **D8.** Measured stopping distance and clearance at the approved limits.
- **Abrupt starts.** Abrupt direction changes hit repeatable startup-transient
  aborts. Resolve physical versus feedback-timing causes before qualifying
  those profiles.
- **Collision-monitor bypass.** Manual mux inputs bypass the Nav2 collision
  monitor; optional software stop locks had no publishers during inspection.
- **P4, then P5.** Not yet run: the navigation TF chain, collision-monitor
  suppression and stale-source handling, Nav2 command-chain integration with
  the base, and then a bounded autonomous clear-space smoke. Both need explicit
  session approval.

**Completion criteria.** On the exact approved robot configuration, trace the
message type, topic, QoS, frame/timeout semantics, mux ownership, collision
monitor, controller input, e-stop, and deadman behavior from Nav2 to the base.
Reconcile drive feedback frequency targets and direct/aggregated fault
reporting, including complete controller-manager and EKF parameter reads.
Resolve any mismatch at the narrowest owning layer; then perform stationary,
lifted/blocked-wheel where approved, low-speed clear-space, stop, timeout, and
e-stop tests before enabling `autonomous_motion_enabled:=true`.

**Benchmark rerun.** The 2026-10-13 live trace found `controller_server`
subscribed to `odom`, which has no publisher on the robot (record under
[Archived evidence](#archived-evidence)). `nav2_params.yaml` now sets its
`odom_topic` to `platform/odom/filtered`, the topic `bt_navigator` and the
velocity smoother already use in every backend. Earlier simulator benchmarks
ran the controller without velocity feedback, so rerun any benchmark before
comparing it with results from after this change.

**Context.** [Exploration architecture](exploration/architecture.md#hardware-safety-boundary)
and the public hardware workflow in the [README](../README.md).

## Deferred hardware gates

The stationary sensor, host and perception work (S1–S10, P1–P3 in the
[qualification plan](plans/PHYSICAL_remaining_qualification.md)) closed on
2026-10-22 for the agreed scope. The gates below were deliberately not
qualified. Each needs a new scope decision; reopen it before the named use.

| Gate | Not qualified | Reopen before |
|---|---|---|
| LiDAR timeout recurrence | The 2026-09-18 simultaneous front/rear timeout is unexplained, and no loaded multi-hour soak ran (`tools/hardware/run_loaded_soak.sh` is ready). In-driver reconnect after a session reset is fixed and verified on both LiDARs ([troubleshooting](troubleshooting.md#hokuyo-drivers-stuck-reconnecting)). | Long unattended operation |
| Boot without internet time | The boot clock step passed three boots with internet time; the no-source path is untested. | Field use without internet time |
| Remote desktop continuity | Repeated-boot ownership and automatic VNC/proxy process recovery passed ([recovery reference](physical/ridgeback_recovery.md)). Network loss, display corruption, browser-side session persistence and vendor webserver reloads are untested. | Relying on the desktop across those events |
| D455 at 1280x720 | Raw HD to Thor exceeds the 1 Gb/s link, so Thor stays at 640x480 (user decision). Local HD failed one of nine captures with a 166.8 ms source dropout. | Using HD |
| Hardware localization accuracy | Distance accuracy at taped marks (including off-centre), silhouette re-measurement and G1 target cases. Person detection scores sit near 0.3, leaving little threshold margin. | Quoting localization accuracy |
| IMU calibration | Factory calibration is accepted without a reference calibration; covariance stays reported as unknown. | Heading drift or SLAM smearing that points to the IMU |
| Battery health | Only pack voltage, total current and MCU/PCB temperatures exist; the battery model is unidentified. | Long or unattended missions that depend on battery health |
| Driving-stack cold boot | The guard unit's `After=network.target` does not wait for `wlp3s0` ([boot ordering](physical/commissioning_guard.md#boot-ordering-limitation)). The consumer plugin has one supervised post-install reboot check; the 2026-10-22 field driver and the repair's DDS-readiness helper have no reboot or cold-boot evidence. | Unattended power-up |
| Feedback timing under load | Field age and live queue timing are measured stationary and in short moving captures only; no sustained motion or CPU load. Ages exclude MCU, wire and kernel delay. | Long missions, or quoting feedback latency |
| Consumer CAN-ID check | The consumer learns CAN IDs from feedback without checking them against the robot declaration. | Changing wheel IDs or the controller configuration |
| Electrical motor-link loss | D7 drops CAN-over-UDP bridge packets; a physical CAN or power loss on the motor link is untested ([fault paths](physical/motor_link_fault_injection.md#electrical-link-loss)). | Relying on stopping after a motor-link hardware failure |

**Completion criteria.** A row leaves this table when a scope decision reopens
it as an active item with its own criteria, or when its gate is qualified and
the result is recorded in its reference.

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

**Progress.** Stationary overlay captures on `r100_0160` show camera depth at
projected LiDAR pixels reading long, from about +1 cm below 1.5 m to +23 cm at
2.5–4 m; the cause is not separated. No flat-target test, calibration with
uncertainty or motion check has run, so polar profiling remains unqualified on
hardware. The [alignment session plan](plans/PHYSICAL_camera_alignment_session.md)
holds the prepared next session.

## Shared camera mast and bracket geometry

**Implemented.** One shared Xacro now supplies both simulators' mast and bracket.
The camera pose and retained backend decks are preserved; Isaac adapts the shared
boxes onto its existing chassis body. See [shared camera support](robot/geometry.md#shared-camera-support).

**Remaining gap.** Detailed hardware attachment dimensions and mass/inertia are
not fully measured. Focused stationary/turn checks passed in both simulators;
broader sensor/contact qualification and affected benchmark reruns remain required.
The implemented shared definition alone does not close this item.

**Completion criteria.** Record the remaining physical attachment dimensions
and measurement uncertainty. Qualify rendered/collision geometry, TF, sensor
self-occlusion and relevant contact behavior in both simulators, including
motion; correct any discrepancy in the shared source. Rerun the benchmarks
affected by the geometry change before quoting new results.

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

**Hardware footprint.** The `r100_0160` outline survey corrected the Nav2
footprint to 99.4 × 82.7 cm, enclosing the measured outline by at least 12 mm.
The equal 2 cm front/rear split of the 4 cm length excess is an assumption.

**Remaining gap.** The corrected footprint has no static simulator comparison
or Gazebo/Isaac contact matrices yet, and the affected benchmarks have not been
rerun with it. Height-dependent attachment limits, an uncertainty model and a
physical stopping/clearance test are still missing.

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

**Completion criteria.** Each question is answered, and the answer is recorded
in the reference that owns the fact or moved to the item that owns the work.

- **Rear LiDAR offset.** Where did x = −0.4278 come from in
  `/etc/clearpath/robot.yaml` (first seen 2026-08-06) and the scan-merger
  `params_ridgeback.yaml` (2026-08-10)? Was it measured? The merger comments say
  the offsets still needed live calibration. We changed it to the symmetric −0.3922
  ([deployed hardware mounts](robot/geometry.md#deployed-hardware-mounts)). Confirm that their
  tooling will not restore the old value.
- **Slow platform stop.** Five of six `clearpath-robot` restarts on 2026-10-06
  took 60 s to stop `clearpath-platform`; one took 7 s and the stop during a
  reboot took 10 s. A `proton_ros2_node` segmentation fault in `librcl.so` was
  logged once during a stop. Which process ignores the stop request, and is the
  fault known? Every Clearpath restart costs that minute of motor power.
- **Battery model.** Which battery is installed? Only pack voltage, total
  current and MCU/PCB temperatures are exposed; battery temperature and
  per-cell voltages need its BMS or added sensors.
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
- [r100_0160 D455 camera service qualification](../archive/engineering/2026-10-06-r100-0160-camera-service.md)
- [r100_0160 first Thor localization smoke](../archive/engineering/2026-10-12-r100-0160-thor-localization-smoke.md)
- [r100_0160 Nav2 command chain, stationary](../archive/engineering/2026-10-13-r100-0160-nav2-command-chain.md)
- [r100_0160 D455 calibration vs the nominal model](../archive/engineering/2026-10-13-r100-0160-d455-calibration-vs-nominal.md)
- [urg_node reconnect leak and recovery](../archive/engineering/2026-10-13-urg-node-reconnect-leak.md)
- [r100_0160 rear LiDAR side-target recheck](../archive/engineering/2026-09-24-r100-0160-rear-lidar-recheck.md)
