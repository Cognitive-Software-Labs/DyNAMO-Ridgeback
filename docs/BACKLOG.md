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

**Completion criteria.** All three plans pass their stationary boundaries on
the recorded configuration. Apply the existing camera, geometry, calibration,
and command-chain criteria independently: stationary success does not close
moving-robot calibration, physical stop/clearance, simulator contact, or
autonomous-mission work. Pointcloud qualification remains optional.

## Physical command-chain validation

**Deployment gate.** The hardware adapter is attach-only and autonomous motion
defaults off, but the repository has not proven the deployed base controller's
velocity message contract. The generated setup currently mixes stamped Nav2,
velocity-smoother, collision-monitor, and mux commands with a generated R100
mecanum controller setting that reports `use_stamped_vel: false`.

**Completion criteria.** On the exact approved robot configuration, trace the
message type, topic, QoS, frame/timeout semantics, mux ownership, collision
monitor, controller input, e-stop, and deadman behavior from Nav2 to the base.
Resolve any mismatch at the narrowest owning layer; then perform stationary,
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
  by interrupting a LiDAR's TCP session on the robot.
- Either explain the 2026-09-18 simultaneous timeout, or bound its recurrence
  with a recorded multi-hour soak under CycloneDDS with the Intel services,
  SLAM, and Nav2 running.

## Optional: restore Gazebo exploration worlds and backend-specific maps

**Opportunity.** Gazebo remains a supported backend, but the adapter currently
ships only `mock_hospital` for exploration and `target_distance_calibration`
for target benchmarking. The only Gazebo `mock_hospital`, `warehouse`, and
`office` coverage maps are archived under `historical/`; the latter two source
worlds are no longer present. Meanwhile the coverage overlay looks up
`<world>.pgm` without considering the backend. Restoring same-named Gazebo and
Isaac worlds under that contract could silently score one simulator against
the other simulator's geometry.

**Completion criteria.** Regenerate a complete analytical map set for the
current Gazebo `mock_hospital` from its SDF. Restore or re-import the exact
Gazebo `warehouse` and `office` world sources with recorded provenance before
promoting their maps; do not promote the archived captures on name alone.
Make map identity explicitly backend-qualified—for example,
`ground_truth_maps/gz/<world>` and `ground_truth_maps/isaac/<world>`—and pass
the selected backend into coverage-map resolution. Preserve convenient
operator world names, but treat `(backend, world)` as the unique identity in
storage, diagnostics, artifacts, and documentation. Add tests proving that a
Gazebo run cannot fall back to an Isaac map, or vice versa, even when both
worlds are named `warehouse` or `office`. Generate `.pgm`, `.yaml`, `.png`, and
`.npz` artifacts for every restored map and visually verify them against their
own simulator world.

**Context.** [Ground-truth map runbook](../src/ridgeback_autonomy/sim/ground_truth_maps/README.md),
[exploration evaluation runbook](exploration/evaluation.md), and the Gazebo
adapter worlds under `src/ridgeback_autonomy_gz/sim/worlds/`.

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

## Robot geometry and drawing audit

**Gap.** The shared reference retains dimensions and a hand-plotted drawing
from the Isaac USD. It does not establish agreement with the generated Gazebo
model or physical hardware, and the drawing cannot detect configuration drift.

**Completion criteria.** Compare frame origins, sensor emission/optical origins,
body bounds, attachment bounds, and floor-contact references in a common frame.
Record source revisions and physical measurement uncertainty; distinguish
configured, model-derived, and measured values. Correct discrepancies at their
source. Regenerate the drawing from authoritative geometry where practical,
and add a repeatable check for its declared dimensions and mount annotations.
Rerun any benchmarks affected by geometry corrections.

**Context.** [Geometry and dimensional provenance](robot/geometry.md) and the
[initial sensor-mount checks](plans/PHYSICAL_robot_measurements_and_validation.md#quick-measurements).
The full dimensional audit here is separate from that limited field visit.

## Collision-envelope and navigation-footprint validation

**Drive scope.** Retain the [Isaac planar drive](isaac/robot-model.md#why-this-abstraction-fits-the-current-work)
for flat-floor navigation/perception work. Physical roller simulation is deferred
until a traction, terrain, traversal, or wheel-control requirement justifies it.
Geometry and footprint validation can proceed independently; shared outlines do
not establish wheel-contact parity.

**Qualified simulator state.** The nominal local/global octagons remain equal,
and both costmaps now pin the audited 10 mm runtime padding explicitly. The
effective footprint encloses the current Gazebo and Isaac collision projections
with at least 5.337 mm static margin. Front, side and 45° contact, held contact,
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

## Rear LiDAR mounting offset

**Gap.** On `r100_0160` the deployed rear LiDAR was configured at x = −0.4278 m
(parent `chassis_link`). The 2026-09-18 field visit found it about 3.6 cm too far back.
A shared side target seen by both LiDARs needed a +3.5 cm rear shift to align,
and the tape readings show symmetric mounts. The symmetric −0.3922 m used by
Clearpath's dual-Hokuyo sample and the repository's `clearpath/robot.yaml`
fits. The offset is duplicated in two robot files:
`/etc/clearpath/robot.yaml` (TF, and therefore SLAM, costmaps and the collision
monitor) and MyBotShop's scan-merger `params_ridgeback.yaml` (the merged
`sensors/scan`). The user chose the symmetric value on 2026-09-18. It was applied
at 18:58 UTC the same day: both files were edited (backups `*.bak-rear-lidar`) and
`clearpath-robot` was restarted, which also restarted the merger. The live TF and
the merger's `laser2XOff` read −0.3922, and all three scans resumed at 40 Hz.

**Context.** Measurements, method and the yaw-error caveat are in the archived
evidence below.

**Completion criteria.**
- The shared side-target check, repeated at a new placement, aligns the two
  scanners without an extra shift.

## Simulator and robot namespace mismatch

**Gap.** The repository's `clearpath/robot.yaml`, the simulators and the launch
defaults use the namespace `r100_0001`. The physical robot runs `r100_0160`,
taken from its robot-local `/etc/clearpath/robot.yaml`. Every hardware command must
therefore override the namespace, and a missed override silently
targets topics that do not exist.

**Next step.** Look into converging on one namespace, or making the hardware
default follow the robot-local configuration. First find every place that
hard-codes `r100_0001` (launches, tools, tests, Isaac assets, docs), and assess the
effect on simulator and benchmark workflows. No scope or decision has been made yet.

## MyBotShop follow-ups

Open questions for the next call with MyBotShop, the robot's integrator.
Resolve each one here or move it to the item that owns it.

- **Rear LiDAR offset.** Where did x = −0.4278 come from in
  `/etc/clearpath/robot.yaml` (first seen 2026-08-06) and the scan-merger
  `params_ridgeback.yaml` (2026-08-10)? Was it measured? The merger comments say
  the offsets still needed live calibration. We changed it to the symmetric −0.3922
  ([rear LiDAR mounting offset](#rear-lidar-mounting-offset)). Confirm that their
  tooling will not restore the old value.
- **`camera-web-ready.service`** fails at every boot because the RealSense topics
  do not appear within its wait, observed after the 14:23 and 14:54 boots on
  2026-09-18. Is it needed, and what should its wait be?
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
