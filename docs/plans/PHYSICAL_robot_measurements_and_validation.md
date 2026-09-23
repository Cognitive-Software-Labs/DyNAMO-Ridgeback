# PHYSICAL — Robot measurements and stationary sensor validation

Status: **mount readings and LiDAR box checks recorded (2026-09-18); camera,
alignment and target acceptance still open.** Owner:
people at the robot, assisted by an agent. The
[project backlog](../BACKLOG.md#physical-stationary-integration) owns the overall
stationary milestone; its individual geometry, camera, calibration, and motion
gates retain their own completion criteria.

This is a field bring-up visit, not a survey of every robot component. The
September 18 mount readings are complete and the user waived mount photos.
People now help place targets; the agent collects and checks the ROS data.
Repeat mount readings only after a mount change or a specific discrepancy.
Keep the detailed D455 checks below as the agent's procedure, not a manual
worksheet for people.

The visit covers quick mount measurements, both LiDARs, stationary camera–LiDAR
alignment, and G1/person integration.
Measurements and sensor inspection can proceed alongside deployment. The
[workstation package split](PHYSICAL_package_split.md) supplies an available
handoff candidate; final distributed tests use its accepted installed revision and the
[Intel–Thor deployment](PHYSICAL_intel_thor_deployment.md). Physical driving,
braking, dynamic calibration, and autonomous missions remain outside this plan.

## Progress and evidence still needed

The September 18 field record names checkout
`a01a4532a170fcb85df1a5aaffb8c28bf1601f96`, with partial provenance: it tested
robot-local services with ad-hoc probes, and raw captures remain on the robot.
The later [camera handoff](handoffs/PHYSICAL_d455_camera_runs_agent1.md) reports
VGA and HD aligned-depth tests on Fast DDS, followed by failures on CycloneDDS.
Those passes apply to the recorded configurations; final deployment acceptance
requires reruns after the [transport blocker](../physical/intel_thor_transport.md#camera-subscriber-blocker)
is resolved. No new hardware tests were performed for this documentation update.

| Part of this visit | Recorded progress | Evidence still needed |
|---|---|---|
| Mount readings and photos | **Complete for this visit.** [A–E readings and precision](../robot/geometry.md#physical-field-readings) recorded; camera reported centred and level; mount photos waived. | No repeat survey or retrospective photo requirement. Recheck only changed or disputed mounts. |
| D455 profiles and timing | Fast DDS VGA alignment and HD cold-start with infra2 off reportedly passed checks 1–3. HD with both infrared streams failed. CycloneDDS camera delivery failed. | Preserve device/profile manifests from the captures; implement the chosen camera launch and repeat both profiles on the final middleware. |
| Camera TF and application smoke | Camera frames were reported disconnected from the robot tree. | Establish one camera-to-base TF owner using observed frame IDs, then validate exact-depth localization smoke. |
| Front and rear LiDARs | **Box checks done 2026-09-18:** both pass near/far/side range and side checks; 40 Hz, ±135°, 0.25° scan timing and geometry recorded. The deployed rear x was ≈3.6 cm too far back and was corrected to the symmetric −0.3922 the same day; see the [backlog item](../BACKLOG.md#rear-lidar-mounting-offset). A timeout/reconnect incident is reported. | Merged-scan comparison, a saved scan overlay, a side-target recheck of the corrected rear offset, and stable recovery evidence. |
| Camera–LiDAR alignment | No completed projection-overlay check is committed. | Near/farther/off-centre overlays and an independent placement after any correction. |
| IMU, odometry and command-chain inspection | No completed stationary validation report is committed. | At-rest observations and configuration inspection, without base commands. |
| G1/person integration | No completed distributed target runs are committed. | Both labels, required mask/depth modes, empty-scene behavior, and joint run evidence. |

The [geometry reference](../robot/geometry.md#deployed-hardware-mounts) separates
physical readings, robot-local configuration, and nominal simulator geometry.
The rear offset correction changed the robot YAML and vendor merger, not the
repository's already symmetric declaration. Live parameter/TF verification does
not replace the outstanding independent side-target recheck.

Continue in this order: resolve and validate the services transport fault;
confirm stable individual and merged scans and recheck the rear correction;
implement the selected camera launch and missing camera-to-base TF; repeat
camera checks; then run camera–LiDAR overlays and G1/person integration.

## Who does what

| People beside the robot | Agent |
|---|---|
| Reuse the completed A–E note; no additional mount photos are required for this visit. | Inspect the deployed setup and capture versions, parameters, topics, timing, and TF. |
| Remeasure a mount only if it moved or a specific mismatch needs resolving. | Interpret the recorded endpoints without treating housing points as calibrated sensing origins. |
| Place a box/board, then the G1 and a person at a few positions. | Record streams, produce camera/scan overlays, run the model modes, and summarize passed/blocked checks. |

No disassembly, workshop, calipers, custom calibration rig, or new drawing of
every component is required. An ordinary upright box or board is sufficient
for the first range/projection checks. If a check reveals a real calibration
problem, record a targeted follow-up instead of expanding the whole visit.

## Preparation and evidence

Coordinate physical access and permission for any robot-side configuration
changes. Inspect the deployed setup before proposing a diff; the checked-in
CycloneDDS setting does not establish what is running on the robot or imply
that it still needs changing. Preserve the robot's configuration and services.
Do not overwrite its setup from the repository or run simulator cleanup.

The agent records code/installed-workspace/dependency revisions, effective setup
path and configuration, ROS domain/RMW, host load, USB topology, device firmware, driver
version, commands, and any applied configuration diff. Use a unique artifact
root under `artifacts/hardware/` and preserve logs, topic captures, parameter
dumps, timestamps, and TF evidence with checksums.

Reuse the completed field note, including its units and approximate precision.
For any targeted follow-up, record the endpoints and add photo references
if available.
Write practical precision, such as "about 74 cm, readable to roughly 1 cm";
repeat a reading only if it is unclear or disagrees with the model. Leave an
inaccessible dimension unknown rather than requiring special equipment.

The agent fixes profiles, warm-up/capture windows, software acceptance criteria,
and target positions before the corresponding checks. Keep the robot stationary,
autonomous motion disabled, and platform/sensor bringup owned by its existing
approved services. Disabling frontier goals alone is not a physical motion lock;
use the site's stationary setup and issue no base commands. A failed gate should
identify a specific follow-up; do not tune recipes, alter timestamps, or change
drivers/TF during the evidence run to turn it into a pass.

## Quick measurements

**Completed for the September 18 visit.** Values and their uncertainty live in
the [physical field readings](../robot/geometry.md#physical-field-readings).
The guide below identifies the recorded endpoints and supports future targeted
remeasurement; it is not a new checklist to repeat. Mount photos were waived.

Start with the existing
[dimensioned robot drawing](../robot/geometry.md#dimensioned-reference-drawing)
to identify the camera, recessed LiDARs, deck, and mast. Its numbers are
Isaac-model coordinates relative to `base_link`, not physical tape readings
from the floor. Do not try to remeasure every number on it.

![Existing side and plan views of the Ridgeback, identifying the sensor mounts and showing model-derived dimensions.](../robot/assets/isaac-model-geometry.svg)

*Existing reference, reused unchanged. Use it for orientation; hardware
agreement and the full dimensional audit remain separate work.*

The following schematic shows the small set of accessible tape endpoints for
this visit. For a future reading, name the face or window point in the note;
a photo can help identify an ambiguous endpoint.

![Practical measurement guide: deck height, camera height above the deck, LiDAR window heights, and camera/LiDAR setbacks from visible deck or body edges.](assets/ridgeback-field-measurements.svg)

| Mark | Take this reading | Why it is useful |
|---|---|---|
| A | Floor to top of the main deck plate. | Relates the visible deck to floor-based sensor heights. |
| B | Deck top to the bottom face of the camera housing. | Checks the camera's mounting height; A + B gives its housing-bottom height above the floor. |
| C | Floor to an identifiable point on each LiDAR window/housing; name the point. | Checks both scan-height estimates without guessing a hidden laser origin. |
| D | Horizontal setback from the deck's front edge to the camera housing's front face. | Checks the camera's forward placement. Use a side/top photo to confirm it is roughly centred and level; measure a lateral offset only if visibly off-centre. |
| E | Each LiDAR housing's outward face to the nearest front/rear body edge. | Checks whether the recessed front/rear mounting matches the description. Record a visible sideways offset if present. |

Use normal tape precision. If an endpoint cannot be reached, photograph it and
mark the reading unavailable. The agent uses the description/device transforms
for housing-to-sensing-origin offsets and labels unmeasured values as nominal.
Do not identify the floor with `base_link` or assume the robot's frame origin
is exactly the midpoint of a body dimension.

Photograph any obviously unexpected protrusion or loose/tilted mount. Measure
overall width/length or an attachment only if the photos reveal a mismatch that
matters to the current setup. Wheel/contact geometry, bracket thicknesses,
fastenings, cable routing, and a full footprint outline are not required readings
for this visit. The [geometry reference and its trigger boundary](../robot/geometry.md#dimensioned-reference-drawing),
[shared-attachment](../BACKLOG.md#shared-camera-mast-and-bracket-geometry), and
[collision-envelope](../BACKLOG.md#collision-envelope-and-navigation-footprint-validation)
work remain separate. Do not require people to redraw the robot or survey an
outline before testing the sensors.

## D455 camera validation

**Current gate:** do not start routine camera subscribers under the reported
faulty services configuration. Controlled transport diagnosis belongs to the
deployment owner; resume the procedure below after its validation.

The recorded implementation choice is a repository-owned camera launch and
parameter file, with the vendor service stopped during our sessions: aligned
depth on, infra2 off, pinned serial, and an explicit camera-to-base transform.
This launch is not implemented yet. Infra1 can create additional aligned streams;
record its final setting and load instead of assuming those streams are free.
HD must be tested by cold start: the vendor driver's runtime profile switch
stopped colour delivery. The reason both infrared streams prevent HD colour
is unresolved; the successful alternative is configuration-specific evidence.

The [camera reference](../target_localization/camera_stack.md) owns the current
input contract, nominal profiles, and TF ownership. This section retains the
core camera gate: a pass establishes integration readiness for the tested
device/profiles, independently of the later camera–LiDAR and joint tests.

### 1. Device identity and effective configuration

- Enumerate the connected cameras. Record the intended D455 serial, firmware,
  USB mode, and advertised profiles; `serial_no: "0"` alone proves no identity.
- Identify the camera process/node and confirm one driver owns the intended
  device. Verify the participating stack's domain, RMW, and wall-time settings.
- Record effective parameters for colour/depth enablement, alignment, and
  synchronization. Inspect active driver profiles rather than inferring them
  from repository YAML.
- Request `640x480@30` and `1280x720@30` as separate runs. Record the actual
  colour and native-depth profiles and resulting aligned-depth grid. Unsupported
  requests or fallback profiles are unresolved findings, not successful tests
  of the requested profile.

Pass requires unambiguous device selection, exclusive ownership, and the frozen
configuration actually running. Propose only the configuration changes the
inspection demonstrates are necessary.

### 2. Image, depth, and calibration contract

Discover resolved colour, colour `CameraInfo`, and colour-aligned depth topics
from the running graph. Compare them with the configured application inputs.
For each topic record type, publisher/subscriber ownership, QoS, encoding,
units, dimensions, `frame_id`, rate, and bandwidth. Preserve live `K`, `D`, `R`,
and `P` calibration fields for each profile.

Pass requires supported encodings/units, compatible colour/aligned-depth/
calibration grids, and the expected publishing ownership. Compare the observed
calibration with the nominal simulation profiles and record a keep/update
recommendation; one device's calibration does not automatically redefine the
shared simulation model.

### 3. Timing and exact-frame availability

After the frozen warm-up, capture at least 60 seconds per profile using
QoS-compatible observers. Preserve colour and aligned-depth header stamp sets
and the resolved TF topics. Report received counts, unique stamps, duplicates,
regressions, out-of-order delivery, receive gaps, middleware loss reports, and
recording/subscriber overhead.

For unique colour stamps `C` and aligned-depth stamps `D`, report both
`|C ∩ D| / |C|` and `|C ∩ D| / |D|`. An empty stream fails the stream gate;
it does not receive a perfect matching score. Use the same declared steady-state
capture interval and document boundary handling.

The camera handoff records the accepted criteria as at least 99% exact matching
in both directions, no duplicate/regressing stamps, sender-side rate at least
29 Hz, and a longest sender-side gap of at most 150 ms (revised from 100 ms).
Freeze these for the final-configuration rerun. The earlier passes used
per-frame CameraInfo stamps to distinguish sender timing from Python image-probe
loss; preserve image counts separately and verify that proxy against actual
image delivery. These criteria do not guarantee depth for every detection:
the application smoke still requires an exact depth frame for each processed
detection stamp. A buffer or a nominal detector rate cannot establish a bound
on lost measurements without the application evidence.

Do not rewrite timestamps, widen matching tolerance, or substitute nearby frames.

### 4. Transform ownership and availability

Inspect the live TF graph, including namespaced topics, and record its publishers.
Query the observed colour-optical frame to the base frame the application
actually requests. The public exploration launch requires an explicitly supplied
`base_frame` on hardware; it rejects an empty value
when localization is enabled. Standalone compute and observer launches also
require the observed frame. The namespace-based default is for simulation;
a ROS topic namespace alone does not establish a physical frame. Record a
mismatch as an integration failure.

Pass requires a finite, stable camera-to-base transform and one owner of the
camera's internal calibrated transforms: the hardware RealSense driver. Verify
image/calibration frame IDs against the live graph and reject competing nominal
simulation or compatibility publishers. The configured mount is a sanity check;
it does not prove camera–LiDAR calibration.

### 5. Stationary application smoke

For the existing single-host smoke, attach to hardware services using
`backend:=hardware autonomous_motion_enabled:=false start_hardware_platform:=false`.
Use the public launch argument
`estimators:=projective_ranging,euclidean_reconstruction`, with
`depth_source:=stereoscopic mask_gate:=box depth_match_debug:=true`.
Use the device's domain, wall time, the observed `base_frame`, and the verified
camera input topics.
Keep pointcloud and polar estimators out of this initial camera smoke. After
installing the package-split handoff on both hosts, use its per-host commands
with the same stationary defaults and estimator/input contracts; do not start
a second Intel detector alongside Thor.

Use predeclared visible, in-range targets at near, middle, farther, and one
off-centre position. Record target placement, duration, detections, measurements,
status histograms, exact-depth diagnostics, and host load. Images can illustrate
the run but cannot substitute for topic evidence.

Pass requires live detection/measurement topics, exact depth for every processed
detection, zero `NO_DEPTH_FRAME`, `NO_CAMERA_INFO`, `GRID_MISMATCH`, or
`TF_MISS_EXTRINSIC` results, and finite plausible outputs for the declared valid
target cases. Record driver restarts, callback failures, receive losses, and
stale-data warnings. These distance checks are integration sanity checks, not
an accuracy benchmark. The joint tests below extend this initial smoke to the
required model modes after their GPU environment and inputs are qualified.

## Both LiDARs and stationary auxiliary sensors

The September 18 near/far/side box checks and 60 s individual-scan timing
capture are complete for their recorded setup. Continue with a new shared
side-target placement after the rear-offset correction, a comparison of merged
and individual scans, saved overlays, and recovery evidence. The procedure
below is the reference for affected rechecks; do not repeat all field work
solely because this plan remains open.

The agent records each front/rear LiDAR's device identity, driver/configuration,
publisher ownership, topic/type/QoS, frame and TF, angular bounds/increment,
range bounds, scan timing fields, rate, gaps, and timestamp behavior. Use the
deployed device/configuration as evidence rather than assuming nominal settings.

Have a person place an upright box/board at a near and a farther position in
front of each scanner, then one position to the side. Record approximate tape
distance to the facing surface and the tape's starting point. The agent checks
that the return appears at the expected side and distance, inspects invalid
returns/self-hits, and compares any merged scan with the individual scans.
Save a simple scan overlay. This catches gross mounting/range errors; an angular
accuracy sweep or surveyed fixture set is not part of this visit.

The agent inventories IMU and odometry publishers actually present and captures
their frames, time bases, rates, covariance fields, stationary bias/drift, and
consistency with the stationary scene. Absence of an expected signal is an
explicit finding; an at-rest check cannot validate wheel scale, lateral motion,
dynamic odometry, or drift during travel.

The agent inspects the deployed command-chain types, topics, QoS, mux ownership,
controller/timeout settings, and available e-stop/deadman configuration without
issuing base commands. Hand findings to
[physical command-chain validation](../BACKLOG.md#physical-command-chain-validation);
its actuation, stop, timeout, and e-stop tests remain open.

## Stationary camera–LiDAR alignment

Start with a practical alignment check using live camera calibration and the
configured transforms, informed by the mount readings above. A full extrinsic
calibration session is a follow-up if the check exposes a problem, not a
prerequisite for collecting useful camera and LiDAR data.

1. Put an upright box/board in view of both the camera and the front scan plane.
   The agent overlays the projected scan points on the image. Use a near,
   farther, and off-centre placement; the base stays still.
2. Check whether points on the board/box land on that object in the image.
   Save the overlays and describe any consistent vertical/sideways mismatch.
   Distinguish a target outside the camera/scan overlap from bad projection.
3. If alignment is wrong, the agent checks frame conventions, live intrinsics,
   transform ownership, and the few measured offsets first. If a real extrinsic
   fit is needed, record the specific missing observation/setup as follow-up.
   Do not ask for a whole-robot survey or tune segmentation to hide the error.
4. After a transform correction, repeat at a new placement not used to make the
   correction. Keep one transform publisher and the previous configuration for
   rollback. A mount movement requires rechecking this alignment.

Call a successful result a stationary alignment/integration check for those
placements, not a calibrated-accuracy claim. If alignment cannot be established,
mark polar profiling blocked while reporting the independent camera/depth
checks normally. Rear-LiDAR orientation/range checks do not establish a
camera–rear calibration. Formal extrinsic uncertainty and moving-robot timing
remain in the [camera–LiDAR deployment gate](../BACKLOG.md#camera-lidar-calibration);
this visit does not close that broader gate.

## Joint stationary target tests

Use the package-split handoff and the deployment plan's qualified environment,
topology, model-mode matrix, and recording protocol. The deployment plan
owns transport/resource measurements; this plan owns scene placement and sensor
validity. Record one shared run identifier so both reports refer to the same
revision, configuration, captures, and acceptance observations.

Test a real G1 and a person as separately labelled cases. Use the default
`humanoid robot` label for G1 and the new configurable `person` label for the
person; record exact prompts and model/checkpoint versions. Calibration fixtures
or supplied boxes cannot substitute for the live detector in this acceptance.
Test near/middle/farther visible placements, one off-centre position, and an
empty scene. Reuse floor marks and record approximate distances from a named
robot edge to the target's facing surface. The agent accounts for that reference
when comparing results; people do not need to survey the G1 or a person's shape.

Run projective ranging and Euclidean reconstruction with all four combinations
of box/silhouette masks and stereoscopic/monocular depth. Add polar profiling
after its stationary alignment check; do not expect every visible target to
intersect the scan plane. Declare valid-target and expected-no-return cases
before the run. Keep organized point clouds optional and disabled unless their
separate contract has been demonstrated.

Require current detections and finite plausible results in the declared valid
cases, correct status for invalid/no-return cases, consistent labels/frames,
and no concealed missing calibration, TF, or required input. Stereo cases
retain exact-depth matching; monocular cases verify actual GPU inference and
source-frame identity rather than asserting a stereo requirement they do not
use. Report detection misses and estimator failures separately. Compare with
measured references while preserving their uncertainty and point/surface
conventions; do not present these integration checks as a general accuracy
benchmark or tune parameters during acceptance.

## Completion and separate work

The A–E note is complete and mount photos were waived. The remaining people's
contribution is target placement and any specifically justified remeasurement;
retain target photos if collected. No complete CAD reconstruction, component
inventory, precision worksheet, or new dimensioned drawing is required.

The agent produces a short passed/blocked/not-tested summary for the D455,
each LiDAR, stationary alignment, and G1/person tests, backed by saved software
captures and overlays. Include tested revisions/configuration, approximate
measurement precision, observed limits, and the next action for each failure.
Unmeasured component dimensions do not block the core camera/depth smoke.
Required sensor or alignment failures remain open; do not call the whole
stationary integration passed until the required target tests pass.

Hand the deployment team verified topics/frames, device/profile identities,
the mount note, and shared run references. The agent updates canonical geometry
and sensor references only with observations this visit actually establishes,
and preserves meaningful evidence with its tested provenance.

[Organized-pointcloud qualification](../BACKLOG.md#optional-physical-organized-pointcloud-qualification)
remains optional. Full dimensional/footprint audits, formal calibration work
exposed by a failed alignment check, physical motion, wheel response, braking,
contact tests, and autonomous missions remain separate. A correction requires
rerunning the affected check; a field visit does not certify simulator geometry
or physical motion safety.

## Archived evidence

- [r100_0160 field readings and LiDAR box checks](../../archive/engineering/2026-09-18-r100-0160-field-measurements.md)
