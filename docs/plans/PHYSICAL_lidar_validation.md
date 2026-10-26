# PHYSICAL — Real-hardware LiDAR validation

Status: planned, 2026-10-23. No test in this plan has been executed as part of
its preparation. The objective is to assess both physical scanners, their ROS
delivery and the scan products actually used by the robot.

The [measurement plan](PHYSICAL_robot_measurements_and_validation.md#both-lidars-and-stationary-auxiliary-sensors)
owns earlier field-check scope; the [qualification checkpoint](PHYSICAL_remaining_qualification.md)
records earlier short stationary and reset-session results and the unperformed
loaded multi-hour soak. Those results guide test selection, but do not establish
the health of a new deployment or close the remaining reliability question.

## Setup and execution boundary

Start with tests L0–L7 while the robot is stationary, with the physical e-stop
pressed, autonomy disabled and an operator present for target placement and
controlled interruptions. Move the board rather than the robot. L8 is a
separate motion stage after the existing driving gates are satisfied; a
stationary pass does not qualify motion, collision stopping or autonomous use.

Have a flat reference target, a tape or surveyed distance reference, marked
floor positions and a camera for setup photos. A rigid board covered in matte
white paper provides a convenient reference; an existing flat light-coloured
wall can be substituted if its material and geometry are recorded. Position
the target vertically, extending above and below the actual laser scan height
so several neighbouring beams hit its interior. It need not cover the scanner's
whole field of view. Record target dimensions, material and incidence angle.
Measure from a documented laser reference, accounting for any offset from an
accessible housing/body edge. A housing measurement without that offset cannot
support a centimetre-scale accuracy claim.

Session condition agreed with the operator: indoors, room temperature believed
to be within the sensor's operating range, with no room thermometer available.
Record that statement as operator-reported context, not a numerical measurement.
No room-temperature or lux measurement is required for this session. Record
the lighting description and any direct sunlight or lighting changes. Measure
the Ridgeback's available hardware temperatures throughout the tests as below.

### Blue rectangular basket as the session target

A blue rectangular basket can be the common target for the initial hardware
session; a separate white panel is not a prerequisite. Inspect its actual
cross-section at laser height before deciding which accuracy tests it supports.
The sensor uses 905 nm infrared according to the manufacturer specification
linked below; do not infer its return strength from the visible blue colour.

- Photograph and measure the basket at the scan height, including taper,
  protruding rim, ribs and openings. Keep a broad side facing the scanner for
  the first range check, and record that side's measured position and angle.
- For L2, a broad solid, sufficiently flat side can provide a measured reference
  surface. Use interior beams with known geometry and label the resulting
  accuracy/repeatability as measured on this basket. Manufacturer white-sheet
  conditions have not been reproduced. If the side is perforated, curved or
  stepped, separate near-face, opening and background returns; a single tape
  distance cannot be treated as ground truth for all of them. Mark precision
  accuracy inconclusive where a suitable reference patch cannot be established.
- For L3/L4, use the measured basket faces/cross-section rather than assuming
  every return lies on one plane. Compare front/rear returns only on surfaces
  visible to both. Test the broad side and corner views separately. If geometry
  is ambiguous, use an existing flat wall as a diagnostic cross-check.
- L1 and L5–L7 assess stream timing, load, reconnect and soak independently of
  target colour. Leave the basket visible as an identifiable target, and check
  that its returns are fresh when moved/removed during the relevant trials.
- A basket-only L2b is one object/material case. Include the whole basket's
  detection behaviour, including openings; do not report only selected good
  ribs as overall detection success. Other material/obstacle cases remain
  untested, so the session can establish hardware health and basket detection
  under its recorded conditions, not general environment suitability.

In the procedure below, “board” can be read as the chosen basket target where
these geometry conditions are met. The single-basket option narrows the L2b
coverage and the final verdict; it does not remove missing test cases from the
report. Continuous robot-temperature logging applies throughout either setup.

Configuration read during preparation on Intel `cpr-r100-0160`:

| Input | Deployed declaration / expected topic |
|---|---|
| Robot identity | `/etc/clearpath/robot.yaml`, namespace `r100_0160` |
| Front | `192.168.131.21:10940`, `/r100_0160/sensors/lidar2d_0/scan`, frame `lidar2d_0_laser` |
| Rear | `192.168.131.22:10940`, `/r100_0160/sensors/lidar2d_1/scan`, frame `lidar2d_1_laser` |
| Driver | Hokuyo UST declaration, reconnect overlay in `/home/robot/ros_overlays/urg_node_fix` |
| Vendor merged scan | Expected `/r100_0160/sensors/scan`; verify publisher and actual consumers |
| Application SLAM merge | `sensors/scan_slam_merged` if launched; a separate implementation and output |

These are configuration observations, not live stream measurements. Confirm
the physical model/serials from labels or existing driver information before
using model-specific limits. Do not open a second SCIP/TCP client on either
scanner while its driver owns the connection. Use the existing ROS publishers.
Inspect the actual TF chain: the declaration parent, laser frame and base frame
are different references; do not infer them from the topic namespace.

Use the installed deployment domain and DDS environment. The inspected Intel
setup declares domain 0. Cross-host tests must first pass the maintained
[Intel–Thor pre-deploy procedure](../../README.md#intel-thor-link-setup-and-pre-deploy-check).
Do not rebuild or replace the deployed stack as a prerequisite for observing it.
Resolve any checkout merge before a later build, and pin the tested revision,
dirty changes and installed executable/configuration hashes independently.

## Robot temperature recording

Start continuous temperature collection before L1 and keep it running through
the final test, target changes and workload transitions. Retain native ROS
temperature messages and sample host sensors approximately once per second.
Record timestamps, units, channel labels and source freshness; an unchanged
last value from a missing publisher is not a new measurement. Confirm the live
channel mapping in L0 using the
[stationary telemetry reference](../physical/stationary_auxiliary_checks.md).

- Ridgeback MCU/PCB channels from
  `/r100_0160/platform/mcu/status/temperature`.
- Per-motor-controller reported temperatures from
  `/r100_0160/platform/motors/status`, where available and valid. Preserve each
  controller identity rather than only the maximum.
- Intel host CPU/package temperatures from labelled Linux thermal/hwmon
  sensors. Include Thor CPU/GPU temperatures when Thor participates in a test.
- LiDAR internal temperature only if the existing driver exposes a supported
  channel. Mark unavailable channels explicitly; do not open an extra sensor
  connection to obtain them. Battery temperature is currently unavailable.

Plot the channels alongside scan gaps/age, CPU load and test-stage markers;
report start, maximum and end temperatures per channel. Keep MCU, PCB, motor
controller, host and any LiDAR channels separately labelled. The LiDAR's
ambient operating range does not set limits for these other components; use
their reported alarms or verified component limits. The collection is an
execution prerequisite still to prepare, not a claim that the short existing
stationary checker already records every channel continuously.

## Ordered tests

Times below are planning allowances, including board placement. Stop progression
when a prerequisite fails; retain the failed capture before making a correction.

| Test | Procedure and duration | Information / acceptance |
|---|---|---|
| L0 — Identity and installation | 5–10 min. Inspect windows, mounting rigidity, plugs and cable strain without disturbing connections. Save model/serials, deployed YAML/parameters, process executable paths, service PIDs/restart counts, topic/type/QoS, consumers and TF. | Establish which physical device produces each topic, whether the patched driver is actually running, and which merge reaches each consumer. Record any obscured/dirty window or loose mount before correction; repeat affected tests afterward. |
| L1 — Baseline delivery and scan contract | 5 min after discovery/warm-up. Record both full raw scans and every merged output in use, with a monotonic receipt watchdog. Keep the existing production workload and identify it; call this a production baseline rather than assuming it is sensor-only. | Individual scanners meet the continuity gates below; stamps strictly increase, metadata stays consistent, and streams do not silently stop at capture boundaries. Check bin count, angular/range bounds, timing fields, invalid values, diagnostics, TCP state and restarts. Save rate, gap and age distributions, not just averages. |
| L2 — Range bias and repeatability | 20–30 min. For **each** scanner place the board near normal incidence at 0.5, 1, 2 and 4 m; add 6–8 m if space/reference quality permit. Capture 30 s per placement, then independently reposition and repeat a near and far placement. | Compare a preselected central beam/interior ROI against the predicted ray-to-plane distance. Report bias, per-beam temporal standard deviation, p95 error and return availability with reference uncertainty. Use the proposed range gates below. This separates scanner error from pose/merger error. |
| L2b — Representative indoor obstacles | 20–30 min. Test actual walls, cardboard/wood, a dark bag or dark fabric-covered target, chair/table legs, and metal or glass surfaces present in the intended area. For each scanner, use measured near/far positions within the intended operating distances and both square-on and oblique views. Where practical, compare white paper and dark material on the same stationary panel to hold geometry fixed. | Record detection fraction, longest missing-target interval, range error and false/behind-surface returns by material, size, distance and bearing. Declare which obstacles/ranges must be detected before capture. Missing an operationally required obstacle is a failed application case even when the reference target passes. This stage is required for a real-world suitability verdict. |
| L3 — Angular coverage, orientation and self-hits | 10–15 min. Move the board through marked bearings around each scanner: 0°, ±45°, ±90° and near the visible ±135° limits where physically observable. Include both robot sides and independently surveyed overlap placements. Remove the board between placements. | Front/rear and left/right must appear in the correct physical locations. Map chassis shadows and persistent self-returns; distinguish obstruction and mixed-edge returns from a dead sector. Compare surveyed bearing against beam spacing and placement uncertainty. Record uncovered sectors; do not require finite returns in empty space or through the chassis. |
| L4 — TF and merged-scan fidelity | 10–15 min. Use at least two independent overlap placements, on opposite sides if possible, plus a straight wall. Transform each raw scan with the deployed laser-to-base TF and compare both to each merged output in use. Move/remove the board to check freshness. | Check both sensor contributions, rear orientation/translation, correct output frame and stable wall shape. Apply the proposed static correspondence gates below. A healthy merged rate alone cannot pass: a merger may continue publishing without rear coverage. No target ghosts may remain over 500 ms after removal is established in both raw inputs. |
| L5 — Camera and compute load | 10–20 min. Record current workload, then add one local colour/depth reader pair, then two independent pairs, 2 min each after warm-up. Include the intended Thor perception/RViz/recording workload for 10 min if used operationally, keeping settings fixed. Do not silently stop existing consumers to improve a result. | Continuity gates hold throughout discovery, warm-up and measurement. Compare receipt gaps, stamp age, CPU/thermal load, NIC rates/errors and driver journals with L1. Identify whether degradation starts with a local reader, cross-host traffic, compute, visualization or recording. A combined load pass does not identify the cause of a historical failure. |
| L6 — Controlled reconnect | 10–15 min. After L1/L5 pass, test one scanner TCP reset at a time using the existing observer, with the operator issuing its single targeted command. Repeat three times per scanner, restoring a stable baseline between trials. | Each interrupted scanner resumes within 10 s from the same driver PID; the other scanner remains within its normal continuity limits. Every stream settles for 30 s with increasing stamps and no service restart. Inspect merged coverage during the interruption. This qualifies a TCP reset only; a silent timeout or power interruption remains a distinct, separately prepared case. |
| L7 — Sustained loaded operation | 2 h under the intended stationary production workload, with a 30 min checkpoint. Keep both scans and merged outputs guarded continuously; retain metrics/logs throughout and full scans in bounded segments. | Zero unplanned driver errors, reconnects, restarts or continuity violations. Compare first/last 10 min for drift, scan age, CPU, temperature and network counters. Failure ends the passing run even if streams later recover. A two-hour pass establishes that workload/window, not unlimited operation. |
| L8 — Motion consistency, conditional | 15–30 min after the maintained driving prerequisites are met. An operator drives at qualified low speeds past a fixed wall/board, straight in both directions and with gentle turns. Record independent reference geometry, scans, odometry, TF and merge diagnostics. | Check wall/target displacement in a common frame, motion-dependent doubling/smearing, scan pairing and TF availability. Freeze limits from the intended speed, timestamp semantics and reference uncertainty before execution. Do not score dynamic calibration from stationary results or use SLAM's own output as independent ground truth. |

L0–L6, including L2b, take approximately 90–150 minutes with a prepared target
area; L7 adds two hours. L8 has a separate readiness boundary. If time is limited, L0/L1 and
near/far placements in L2 give a useful first screen; mark the omitted tests
untested rather than issuing a complete pass.

## Limits fixed before measurement

Existing project continuity limits, also enforced by the
[interruption observer](../../tools/hardware/observe_lidar_interruption.py):

- Each individual scan: at least **38 Hz**, maximum receive gap **250 ms**.
- Vendor merged scan: at least **90% of its healthy baseline rate**, maximum
  receive gap **500 ms**. Establish that baseline with both inputs present;
  measure rear coverage separately. Apply the same relative gate to any
  application merge under test and report its baseline explicitly.
- Normal operation: no new reconnect errors or unexpected PID/service restarts.
  The expected reset/reconnect in L6 is confined to its declared fault window.
- Stamps strictly increase within each uninterrupted run. Save receipt times
  using a monotonic clock, acquisition/header stamps, and wall-clock age
  separately. Include first-message deadlines and the final silent tail in gap
  evaluation. A clock step invalidates age comparisons; it does not erase a
  monotonic receive-gap failure. Header age alone does not prove acquisition
  latency, and `ros2 topic hz` alone cannot qualify continuity.

The [camera contract checker](../../tools/camera_contract_check) applies the
stricter **38 Hz to all three hardware scans**, including the vendor merge.
Record its verdict as implemented; do not relabel a failed checker as passed
using the relative baseline criterion. Investigate a lower-rate merge and
agree its operational requirement before accepting it.

For a confirmed UST-10LX, the
[manufacturer specification](https://www.hokuyo-aut.jp/dl/UST-10LX_Specification.pdf)
lists 270° coverage, 0.25° steps, 25 ms scan period, ±40 mm accuracy and
repeatability σ <30 mm under factory conditions using white Kent sheet.
Its guaranteed range depends on reflectivity (white sheet up to 10 m; 10%
reflectance up to 4 m). Check the observed ROS contract rather than inferring
physical coverage from the requested ±π driver limits. The shared
[LiDAR contract](../../src/ridgeback_common/ridgeback_common/lidar_contract.py)
expects 1,081 raw bins, ±135°, 0.06–10 m and 40 Hz; a deviation needs explanation
against the actual hardware, driver and configuration.

The following are **proposed field acceptance gates**, to freeze before capture;
they do not constitute factory certification:

- L2, suitable white-board interior returns: absolute bias + estimated 95%
  reference uncertainty ≤40 mm, temporal per-beam σ <30 mm, and ≥99% valid
  returns in the predeclared observable ROI. If uncertainty overlaps the 40 mm
  boundary, label accuracy inconclusive; report the distribution and uncertainty
  rather than rounding to a pass. Exclude board edges by a rule fixed before
  analysis. Compute oblique ray/plane distances explicitly.
- L3: surveyed target bearing agrees within one 0.25° bin plus stated bearing
  uncertainty. This is a gross orientation/sector check, not angular calibration.
- L4: static board/wall nearest-surface residuals have median ≤30 mm and p95
  ≤50 mm for front/rear agreement and raw-to-merged agreement, allowing for the
  merged angular grid. Expected target returns appear in ≥99% of merged scans
  while the corresponding raw target is valid. Analyze each sensor/placement
  separately so a dense front cloud cannot hide missing rear data. Preserve
  actual raw/merged geometry and binning uncertainty; a close boundary is
  inconclusive. Do not fit the extrinsics on these acceptance placements.

L2's reference target makes faults easier to isolate: a large flat surface with
a known position removes object shape and a deliberately difficult surface as
competing explanations. It does not establish operational suitability. The
manufacturer's white-sheet accuracy conditions also do not automatically apply
to an arbitrary painted wall or paper target.

L2b is a required application check, rather than an optional follow-up. Define
required target support and acceptable missed-target duration for each actual
obstacle/range before measuring, using its size and angular footprint. Report
dark, glossy, glass and thin-object cases separately. A no-return case can be
a material/geometry limitation rather than a defective unit, but a required
obstacle remaining invisible still blocks acceptance for that environment.
Do not average material-specific failures away in an overall detection rate.

## Evidence and available tools

Use a fresh `artifacts/hardware/<UTC>-lidar-validation/` directory, with a
subdirectory per test/trial. Preserve failed runs. Save:

- Tested source revision, dirty patch/untracked test inputs, installed driver
  paths/hashes, deployed YAML and parameters, DDS/domain and observer versions.
- Full `LaserScan` messages for geometry tests; raw and merged topics, namespaced
  TF/static TF, odometry, diagnostics and emergency-stop state. Use matching
  best-effort sensor QoS and transient-local static-TF capture. Verify bag counts
  and dropped-message evidence; do not assume the recording captured everything.
- Independent callback receipt times, header stamps, per-stream rate and
  p50/p95/p99/max gaps/age, validity statistics by ROI, process identities,
  journals, NIC counters, host load and continuously recorded hardware
  temperatures with labelled sources. Record recorder/observer overhead.
- Target placement sheet, distance/bearing uncertainty, photos, raw/base-frame
  overlays, range-error plots and time traces showing both gaps and workload
  transitions. Capture an independent placement after any geometry correction.

Existing commands below are execution examples, not instructions to run every
stage automatically. Source the deployed Intel setup first; the preparation
inspection found its domain is 0. Use fresh output names and run only one robot
experiment at a time.

```bash
source /etc/clearpath/setup.bash
source src/ridgeback_autonomy_hardware/config/intel_thor/dds_env.sh intel
LIDAR_RUN="$PWD/artifacts/hardware/$(date -u +%Y%m%dT%H%M%SZ)-lidar-validation"
mkdir -p "$LIDAR_RUN"

# Short passive screen; this tool writes its own timestamped evidence directory.
python3 tools/hardware/fast_stationary_check.py --duration 30

# One guarded local camera-reader pair for L5; scans are discovered first.
python3 tools/camera_contract_check --backend hardware --guard \
  --warmup 10 --duration 120 --output "$LIDAR_RUN/load-one-pair.json"

# L6: separate attended trials, after baseline/load gates pass.
# Each observer prints the targeted reset command for the operator to run once.
python3 tools/hardware/observe_lidar_interruption.py --lidar front \
  --run-dir "$LIDAR_RUN/reconnect-front-1"
python3 tools/hardware/observe_lidar_interruption.py --lidar rear \
  --run-dir "$LIDAR_RUN/reconnect-rear-1"
```

The passive batch is a short screen, not L1's five-minute range/timing capture.
The interruption observer records header/receipt timing and process state, not
full scan geometry. Its summary needs inspection alongside the raw timeline;
do not infer that a missing boundary interval was guarded just from its verdict.
The camera checker adds readers; it cannot serve as a sensor-only baseline.
The [loaded-soak helper](../../tools/hardware/run_loaded_soak.sh) is a starting
point that launches a specific stack/build prefix. Review its installed paths,
process ownership and cleanup before reuse; do not invoke it against an already
running stack or mistake it for a LiDAR-only observer. It does not provide all
the full-run metrics required by L7.

Before execution, prepare one bounded full-scan recorder/analyzer for L1–L4,
continuous robot/host temperature collection, and the continuous watchdog/metrics
collection for L7. The existing tools do not
automate board ground truth, uncertainty, raw-to-merged geometry or rear-coverage
acceptance. This preparation is remaining work, not an already available
one-command qualification suite. The
[offline reconnect checker](../../tools/urg_reconnect_check) tests a fake TCP
server on an isolated domain; passing it cannot substitute for L6 on hardware.

For L5/L7, a continuity violation or unexpected driver error ends the added test
load; keep the robot stopped and retain diagnostic collection. During L6, only
the declared scanner's outage is expected; an outage of the other scanner or
failure to recover within the bound fails the trial. Capture logs/socket/process
state before any manual service recovery. A restart restoring scans does not
turn the failed trial into a pass; repeat with a new evidence directory after
the cause or configuration change is documented.

Report separate verdicts for front scanner, rear scanner, representative
obstacle/material detection, TF/merge geometry, loaded continuity, TCP-reset
recovery and motion. Use pass, fail, inconclusive or untested, with the evidence
path and conditions. Do not reduce these to a
single “LiDAR works” statement that conceals an untested layer.
