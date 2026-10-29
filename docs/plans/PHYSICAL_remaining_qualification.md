# PHYSICAL — Qualification status and remaining work

Status update (2026-10-22): **S1–S10 and P1–P3 are finished for the agreed
scope.** The other agent completed its work on the sensor, host and perception
items. They are not part of the remaining-work list or estimates.
The specific test results and limits below remain the record of what was
measured; reopening an item requires a new scope decision. P4 and P5 have not
been run and stay open; autonomous motion remains disabled until they are
tested and approved. Active driving qualification work is D2, D3, D4
regression, and D6–D8, then P4 and P5. D5 is the installed freshness baseline
that those trials must preserve.

Planning checkpoint: **2026-10-14**, after the passing bounded-forward trial
`forward-permit-20261014T090919Z`. Estimates below are hands-on test/setup time,
not commitments. Failed gates add diagnosis, implementation and repeats. Existing
topic references retain acceptance criteria; this page schedules their remaining
evidence rather than defining weaker replacements.

Checkpoint update: S1 (quiet local-reader camera/traffic screen) passed on
October 14 in `camera-port-screen-20261014T093624119430Z`. Thor perception was
temporarily stopped and its launch restored after capture. The
[camera deployment reference](../physical/robot_local_deployment.md#local-reader-checks)
owns the conclusions; the run measurements are in its archived records. Do not schedule another S1 merely to repeat a passing unchanged
configuration.

## Integrated D-lane execution — 2026-10-22

D1–D8 are one qualification program. Code preparation, deployment, motion,
fault injection, recovery and final-mode restoration all write distinct run
directories, but their acceptance is evaluated together. Only one live robot
action may run at a time. Camera, DDS and completed sensor work do not own or
block this lane unless they change the driving graph or consume the same robot
service window.

Every new D-lane run uses `driving_qualification.py finalize` to join its raw
recorder result with the physical observation, independent video where required,
recovery and final installed mode. The generated matrix preserves the four
verdicts `passed`, `failed`, `inconclusive` and `aborted`; missing evidence can
only keep or lower a verdict. D6, D7 and D8 cannot pass without nonempty hashed
video evidence. D6 and D7 also require an explicit supervised recovery result.

Already evidenced: stationary health, controller input/neutral return under the
stop, capped one-second forward movement and automatic stopping, short VGA
camera/scan delivery, no added wired load in two short reader comparisons,
stationary incoming motor-feedback interruption and recovery, one earlier
post-install reboot. These do not establish full driving readiness.

## Driving and feedback

| ID | Status and scope | Estimated test/setup time | Prerequisite |
|---|---|---|---|
| D1 | Complete: forward, backward, both lateral and both yaw signs have one bounded floor trial with operator confirmation | — | Recorded in [passed lanes](../physical/driving_qualification_procedure.md#passed-lanes); physical stopping distance remains D8 |
| D2 | Confirm deadman release, command loss and physical e-stop stops on the current build. Acceptance: [acceptance matrix](../physical/driving_qualification_procedure.md#evidence-and-verdicts), plus a physical check that held input is not replayed after a stop release. | 15–25 min | D1, agreed clearance; earlier observations remain valid historical evidence |
| D3 | Abrupt/silent wireless input loss, neutral reconnect and restored input over repeated cycles Acceptance: [acceptance matrix](../physical/driving_qualification_procedure.md#evidence-and-verdicts). Also untested: reconnect with the stop released, a multi-minute outage, and the root cause of the [connected-but-no-input failure](../physical/controller_input.md#connected-but-no-input-failure). | 15–25 min | Stationary first; distinguish power-off/disconnect from genuine silent report loss |
| D4 | **Live handover passed 2026-10-21** (`d4-handover-can-20261021-6`, with operator observation): stationary until commanded, low 0.025 m/s, higher-priority 0.045 m/s takeover, return 0.525 s after the last high command (0.5 s mux timeout), self-stop when silent, e-stop pressed 3.5 s after standstill; controller untouched. Earlier attempts aborted safely before or at the start of motion (operator L1 press, stop not released/pressed). Isolated D4 harness passed 17 checks earlier the same day. Not covered: a physical RC-controller takeover and recurrence under load. | — | One bounded floor trial; not stopping distance (D8) |
| D5 | **Passed (stationary) 2026-10-22:** per-wheel field-age driver (motion 150 ms, auxiliary 500 ms) installed via `install_puma_field_driver.py` (`d5-field-driver-install-20261022-2`) and accepted in `d5-field-live-acceptance-20261022-2` (20 Hz, motion age ≤ 51 ms, auxiliary ≤ 161 ms, queue ≤ 51 ms, stop pressed). The first acceptance run failed only the original 50 ms queue limit, which was below the driver's 50 ms drain cycle; the limit was revised and the run repeated. Offline: five-capture replay, missing/asymmetric injections, pinned build with 119+240 assertions. Rollback: `--restore` | — | Moving behaviour is covered by D6–D8 |
| D6 | Feedback-consumer fault while wheels are turning, fault latch and supervised recovery. **Prepared:** `check_raised_wheel_link_fault.py --fault feedback` drops MCU→PC bridge packets after measured motion while the driver keeps sending commands; independent cleanup unit; `--recover`. Acceptance: [acceptance matrix](../physical/driving_qualification_procedure.md#evidence-and-verdicts). | 20–40 min | D5 accepted, rated stands/all wheels clear, qualified helper, exclusive window |
| D7 | Actual motor-link loss/watchdog, including inability to deliver outgoing zeros, and recovery. **Prepared:** same tool with `--fault command-link` drops PC→MCU bridge packets; wheel stop is measured from still-arriving feedback, plus operator video Acceptance: [acceptance matrix](../physical/driving_qualification_procedure.md#evidence-and-verdicts). Puma driver reconnect after a motor-link loss is not qualified; stationary recovery used a supervised platform restart. This trial drops bridge packets; [electrical link loss](../physical/motor_link_fault_injection.md#electrical-link-loss) is a separate, untested case. | 30–60 min | D5 accepted, rated stands, qualified help, exclusive window |
| D8 | Measured stopping distance and clearance at the approved operating limits. Acceptance counts, speed ladder and pass rules are in the [stopping-distance procedure](../physical/driving_qualification_procedure.md#stopping-distance-measurement). | 20–40 min | Measured geometry, defined floor/load/limits; timestamped commands alone are insufficient |

The [driving reference](../physical/driving_readiness.md) and
[feedback reference](../physical/drive_feedback_diagnostics.md) own the exact
conditions. Stands-only fault trials are not authorized floor trials. A one-shot
permit stop must not mask the fault-specific stop mechanism being measured.

## Stationary hardware and host setup

| ID | Status and scope | Estimated test/setup time | Prerequisite |
|---|---|---|---|
| S2 | VGA passed 2026-10-14 (the deployment profile). HD 2026-10-16: raw HD to Thor cannot fit the 1 Gb/s link (diagnosis `20261016T095428Z-s2-hd-diagnosis`); Thor stays at VGA (user decision). HD local matrix 8/9: one 5-frame source dropout (166.8 ms > 150 ms) in `20261016T101433Z-s2-hd-reader-matrix`, so HD local is not qualified | — | 1280x720 is unsupported on the robot (user decision, 2026-10-29); no further HD testing |
| S3 | Passed 2026-10-14 (`20261014T114600Z-s3-front-lidar-interruption`): front session reset, same driver reconnected in 2.25 s, three scans settled at 39.93 Hz; rear passed 2026-10-13 | — | In-driver reconnect patch deployed as an overlay; no watchdog needed for a reset session |
| S4 | Passed 2026-10-14 (`20261014T114755Z-s4-reader-pair-soak`): two reader pairs for 30 min, full contract, 39.9 Hz scans, +1.9 Mbit/s TX, no LiDAR errors | — | VGA only; loaded soak is S5 |
| S5 | **Finished.** No loaded multi-hour soak was run. Evidence kept: 30-min two-reader-pair soak (S4) and both LiDAR interruption recoveries (S3). The 2026-09-18 simultaneous LiDAR timeout remains unexplained and unbounded; the runner `tools/hardware/run_loaded_soak.sh` (stack `~/DyNAMO-Ridgeback-intel`) is ready | — | Reopen before long unattended operation |
| S6 | Passed with internet time 2026-10-16: boot clock step installed, 3 of 3 boots stepped before any consumer and the platform gate took 0.1 s. Boot without the internet source not tested (user decision) | — | Retest the no-source path before field use without internet time |
| S7 | Passed 2026-10-16: repeated-boot ownership plus deliberate main-process failure and automatic listener recovery for VNC and both proxies; operator reconnected (`20261016-s7-desktop-recovery`) | — | Does not qualify network loss, display corruption or browser session persistence |
| S8 | **Finished.** Survey done and footprint corrected on `dev` (99.4 × 82.7 cm, measured outline enclosed by ≥12 mm). Not done: static simulator comparison, Gazebo/Isaac contact matrices and affected benchmark reruns with the new footprint; the 2 cm/2 cm front/rear split of the 4 cm length excess is an assumption | — | Rerun before quoting simulator or benchmark results |
| S9 | **Finished:** factory IMU calibration accepted. No reference calibration was done. Fixed-pose characterisation (2026-10-13: gyro bias ~0.001 rad/s, noise ~0.0005 rad/s) and the corrected filter parameters stand; covariance stays reported as unknown, not invented. Reopen if heading drift or SLAM smearing points to the IMU | — | Reference equipment would be needed to reopen |
| S10 | **Finished:** current battery telemetry accepted. Only pack voltage, total current and MCU/PCB temperatures exist; battery temperature and per-cell voltages remain explicitly unknown (not substituted), and the desktop battery bridge is synthetic. Battery model still unidentified (question for the integrator). Reopen before long or unattended missions that depend on battery health | — | Battery identity, then BMS or sensors, to reopen |

Current source diagnosis, recorded limits and reconstruction inputs live in
[stationary auxiliary checks](../physical/stationary_auxiliary_checks.md),
[camera deployment](../physical/robot_local_deployment.md),
[transport](../physical/intel_thor_transport.md), and
[host recovery](../physical/ridgeback_recovery.md).
The original multi-hour recurrence gate has no frozen duration; 2–4 h is a
planning allowance, not a newly established statistical reliability criterion.
IMU thermal/drift validation can require additional hours beyond the initial
calibration session. Battery values remain explicitly unknown until measured.

## Distributed perception and navigation dependencies

| ID | Status and scope | Estimated test/setup time | Prerequisite |
|---|---|---|---|
| P1 | **Passed 2026-10-16.** Both hosts run `dev` `a31e770` (`check_link --no-traffic`: 30 PASS, revision match, Thor −0.02 ms); Intel camera service reinstalled from `dev`. F1 detector kill: health `failure`, `processing` 8 s after relaunch; since 2026-10-21 workers respawn automatically (killed detector replaced in 6 s, `processing` in 13 s, service not restarted). F2 cable pull: PHC step fix verified, recovery ~5 s. 2026-10-21: Thor localization runs as `dynamo-localization.service` from `dev` `25ebd2a` (after PTP; verified across a Thor reboot: `processing` with no manual launch, 382 Mbit/s, −0.02 ms). Perception now loads models offline by default (Thor Wi-Fi recovered by 2026-10-21; its new NetworkManager profile had lost the Thor LAN route, restored with `thor_lan_route apply`; `check_link --no-traffic` 0 failures at `070b284`). Known divergence: Intel chrony profile differs from `time_sync`'s record (pool/maxpoll edits) | — | Fix Thor Wi-Fi (antennas/router) |
| P2 | **Finished.** Evidence kept: GPU environment, all four modes start and reach `processing`, delays (box-stereo 367/383 ms, box-mono 575/592 ms), Thor thermal ≤68 °C, person tracked at two marks, 0 false detections in a 28 s empty segment, silhouette buffer fix deployed. Not done: distance accuracy at taped marks, silhouette re-measurement, G1 target cases | — | Reopen before quoting localization accuracy |
| P3 | **Finished.** Evidence kept: overlay captures show camera depth at projected LiDAR pixels reading long by +1 cm (<1.5 m) to +23 cm (2.5–4 m), cause not separated (`ANALYSIS.md`). Not done: flat-target test, calibration with uncertainty, motion check. **Polar profiling stays unqualified and must not be relied on** | — | Reopen before using camera–LiDAR projection |
| P4 | **Open, not run.** Remaining: navigation TF chain, collision-monitor suppression and stale-source handling, command-chain integration with Nav2 | — | Required before autonomous motion |
| P5 | **Open, not run.** Remaining: bounded autonomous clear-space smoke. **Autonomous motion remains unqualified; keep `autonomous_motion_enabled:=false` (the hardware default)** until P4/P5 are actually performed | — | Explicit session approval and P4 first |

Optional organized-pointcloud qualification adds approximately 30–60 min if
that input is selected. It remains disabled/unqualified otherwise. Simulator
recertification and benchmark reruns are separate repository work, not part of
this robot-session estimate. A spare-SSD recovery rehearsal was explicitly
declined and is not scheduled.

## Active remaining work and estimates

- D2–D3: stop-action isolation and silent wireless-loss evidence; approximately
  **30–50 minutes** of test/setup time if the gates pass.
- D5: passed stationary acceptance on 2026-10-22; nothing remains before D6.
- D6–D7: tooling prepared; raised-wheel feedback and actual motor-link fault
  tests with qualified help need approximately **50–100 minutes** after D5.
- D8: measured stopping distance at approved limits; approximately
  **20–40 minutes** after geometry and test conditions are fixed.

These are planning allowances, not completion promises. Finished S/P items
above are excluded. Do not overlap faults, boot tests or conflicting deployments.

Batch evidence analysis and passive monitoring, but keep each physical action
and fault individually bounded and interpretable. Reuse valid evidence and rerun
only when configuration changes or a failure invalidates it.
