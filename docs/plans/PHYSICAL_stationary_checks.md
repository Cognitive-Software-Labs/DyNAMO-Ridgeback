# PHYSICAL — Checks without movement or a controller

Scope: the Ridgeback stays stationary with its physical e-stop pressed. No
controller input, motion publisher, test permit or autonomy startup is part of
this procedure. Passive checks and controlled stationary disruptions have
different operational effects; do not bundle a service restart into a passive
health command. Current hardware gates remain owned by the
[backlog](../BACKLOG.md), with details in the linked topic references.

## Evidence audit and order

| Check | Existing evidence | Next useful check and completion boundary |
|---|---|---|
| Stationary sensor health and guarded routing | October 13 short batch passed all 22 checks: three scans, IMU, battery, stop, odometry, service activity and local clock | Repeat one bounded passive batch after an actual setup change; a pass establishes only that observation window. |
| Camera contract under local readers | Checker startup topic-override bug repaired; no new live report | Discover scans before creating image readers, guard warm-up and measurement, then measure color/depth pairing, profiles, frame/TF and continuity. Start with a 12-second screen; use existing full profile criteria for qualification. |
| Hokuyo reconnect recovery | Historical reconnect failure; current idle scans healthy | First validate the bounded read-only checker on healthy hardware. Then separately prepare a timed, independently rolled-back interruption of one LiDAR TCP flow. Record failure, at most one stationary sensor restart, and sustained scan recovery. No automatic boot watchdog is installed. |
| Clock and desktop startup | One supervised reboot passed the local clock gate, exact plugin/config checks and stationary startup; desktop listener ownership inspected | Repeat boot only to qualify repeatability, or run a separately prepared desktop-backend recovery trial. Preserve access/recovery evidence. Local convergence does not establish Intel–Thor offset or source-loss holdover. |
| Per-wheel feedback freshness | 30-second passive CAN capture; offline motion-field model about 20 Hz | Replay asymmetric/missing-field cases and consumer failure semantics offline. Deployment still requires its own provenance and live stationary checks; timer projections do not qualify physical braking. |
| IMU at rest | 60-second raw fixed-pose characterization, intended filter parameters verified | Analyze existing raw samples and repeat only for a defined temperature/drift condition. Full calibration needs an independent reference; a stationary pose cannot distinguish accelerometer bias from gravity/mount error. |
| Battery telemetry | Pack fields measured; unavailable cell/temperature fields correctly marked unknown | Read-only source/protocol inventory can verify what is exposed. Actual battery temperature/per-cell values require identified hardware telemetry. Do not substitute MCU temperatures. |
| Physical footprint/mast | Nominal/shared geometry and partial physical measurements available | Review frame/footprint consistency in software. A person can measure the stationary outer envelope and attachments; dimensions and clearance cannot be invented from configuration. |

## Passive commands

The October 13 audit repeated the live stationary batch successfully (all 22
checks) and ran the new passive Hokuyo checker for 35 seconds with zero service
restarts. Front/rear delivery averaged 39.91/39.92 Hz, with maximum receive gaps
43.8/37.7 ms. Evidence is under
`artifacts/hardware/stationary-batch-20261013T132501Z-78c5ac/` and
`artifacts/hardware/20261013-hokuyo-passive-audit.json`. This confirms short-window
healthy scans, not timeout recovery. The camera trial was not executed because
host-access automatic approval review could not complete after reaching its
usage limit. The operator subsequently ran it in the host terminal; all image
and scan contract criteria passed. The [camera deployment reference](../physical/robot_local_deployment.md)
owns that result and the unresolved ~98 Mbit/s physical-port TX observation.
Measure quiet versus reader traffic and inspect packet headers before further
deliberate sensor faults; do not infer reader-induced traffic from total TX.

Run from the repository root with ROS Jazzy sourced. Each command is separate
so a failed camera screen cannot obscure a health report. Use fresh output names
for repeats and retain failed runs.

```bash
cd /home/robot/DyNAMO-Ridgeback
source /opt/ros/jazzy/setup.bash
python3 tools/hardware/fast_stationary_check.py
python3 tools/camera_contract_check --backend hardware --guard \
  --warmup 3 --duration 12 \
  --output "artifacts/hardware/camera-screen-$(date -u +%Y%m%dT%H%M%SZ).json"
python3 tools/hardware/hokuyo_recovery_watch.py --dry-run --duration 35 \
  --report "artifacts/hardware/hokuyo-passive-$(date -u +%Y%m%dT%H%M%SZ).json"
```

The camera command creates local image subscribers; the scan guard ends the
reader trial on individual gaps above 250 ms, merged gaps above 500 ms, driver
errors or service changes. It does not restart services. The Hokuyo command
does not create a fault or restart anything. If journal access is unavailable,
that is a failed observation rather than evidence of healthy reconnect handling.

## Disturbance boundary

LiDAR interruption, desktop backend restart, reboot and external-time-source
loss need dedicated rollback and evidence plans. Do not interrupt the MCU/CAN
link as part of the LiDAR check. Do not disable networking broadly to test time
sources: it also removes the operator's desktop path. A success count from unit
tests cannot substitute for a recorded hardware trial.

Remaining motion/controller qualification stays in
[driving readiness](../physical/driving_readiness.md); raw IMU/battery and time
limits stay in [stationary auxiliary checks](../physical/stationary_auxiliary_checks.md),
and CAN interpretation stays in
[feedback diagnostics](../physical/drive_feedback_diagnostics.md).
