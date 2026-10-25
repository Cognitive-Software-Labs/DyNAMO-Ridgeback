# Troubleshooting

This page owns current failure signatures and recovery steps. Dated root-cause
investigations and measurements live in the Archived evidence section below.

## Quick checks

| Symptom | Check or action |
|---|---|
| Robot does not move | Inspect `/<namespace>/cmd_vel`; if it is empty, Nav2 may not be active |
| No map in RViz | Check `/<namespace>/map`, `slam_toolbox`, and the scan topic |
| Target overlay missing | Confirm `target_localization_enabled:=true`, enable RViz's `Perception overlay`, and inspect the color and `debug/target/overlay` topics |
| Overlay is a sliver in the left dock | Drag the pane to the bottom, stretch it, and save the RViz config; pane placement is stored in `QMainWindow State` |
| Estimator ring or HUD column missing | Verify the estimator was selected and the corresponding measurement node/topic exists; `--` means the estimator ran but returned no value |
| All mask rows blank, pointcloud works | Inspect startup logs for an unavailable configured `base_frame` |
| Explorer finds no frontiers | Confirm the global costmap has `track_unknown_space: true` |
| Startup stage stalls | Read the matching `gate_*` process's `unmet:` list; do not add a fixed timer |
| Benchmark opens Gazebo/RViz but its first trial stalls | If the config gate has the camera image but lists `detections/target/raw` as unmet, inspect the environment launch's `target_detector_node` log and check the perception venv; see [Missing benchmark detections](#missing-benchmark-detections) |
| Stale simulator processes | Run `bash cleanup.sh` before a simulation launch or once before a sweep; never use it as hardware bringup cleanup |
| Need a log summary | Run `bash tools/diag.sh <console.log> <world> [namespace]`; the namespace defaults to `RIDGEBACK_NAMESPACE`, then `r100_0001` |
| Images on Thor stall or lag although ping is fast | See [Intel–Thor stream stalls](#intelthor-stream-stalls) |

## Hardware launch is deliberately motionless

`backend:=hardware` defaults to `start_hardware_platform:=false` and
`autonomous_motion_enabled:=false`. The first setting attaches to existing
Clearpath services; the second prevents `frontier_explorer_node` from sending
goals. `start_exploration.sh` also skips `cleanup.sh` and leaves
`ROS_DOMAIN_ID` untouched for hardware. Unless explicitly overridden, the
hardware namespace comes from `<setup_path>/robot.yaml` and launch aborts if
that configuration cannot be read; it never silently falls back to the
simulator's `r100_0001` identity.

If the readiness gate lists scan or filtered odometry as missing, inspect the
approved robot-side bringup, namespace, domain, RMW, and effective topics. Do
not bypass the gate by switching to a simulator profile or enable platform
bringup/motion just to make a topic appear. The physical command-chain and D455
validation gates remain in the [backlog](BACKLOG.md) and
[PHYSICAL — Robot measurements and stationary sensor validation](plans/PHYSICAL_robot_measurements_and_validation.md#d455-camera-validation).

## Missing benchmark detections

The persistent benchmark environment starts the detector before the first
configuration. If its Python environment lacks Torch or Transformers, the
detector logs a fatal message and exits without advertising
`/<namespace>/detections/target/raw`. The configuration gate may then wait its
full 300 seconds and proceed even though the topic remains absent; a clean gate
exit is not proof that the detector became ready. Check the detector log under
the sweep's `environment/logs/ros/` and the gate log under
`<config>/logs/ros/`.

Set up the [perception venv](../README.md#3-optional-set-up-target-localization-venv)
in the sweep's checkout, or set `RIDGEBACK_PERCEPTION_VENV` to an existing venv
before starting the sweep or configurator. Restart the configurator if it was
already running. Stop the failed sweep from its terminal before retrying.

## Benchmark sweep cleanup

`cleanup.sh` intentionally kills Gazebo servers, RViz, ROS launches, and package
nodes, while preserving an existing `gz sim gui` process. Run it once before
`target_benchmark_sweep`, never between configurations: the sweep deliberately
keeps its Gazebo server, RViz, robot, camera TF, detector, and clock alive while
it restarts only the configuration layer.

Two `gz sim server` processes indicate that a stale single-run environment was
not cleaned. Gazebo transport may then spawn a target in one server and query
its pose from the other, producing a misleading `Target pose ... not present`
failure. Clean once and restart the sweep. A valid `run.json` lets the supervisor
resume completed configurations.

### What `cleanup.sh` leaves alone

Every kill pass and the final leftover check share one ownership filter. It
skips cleanup's own ancestors (so a configurator-started sweep cannot kill the
configurator), the Gazebo GUI, `target_benchmark_configurator` and the offline
replay CLIs, and any process whose `AMENT_PREFIX_PATH` names another colcon
workspace, such as another worktree or the robot's vendor overlays. Node names
match whole path components, so `less notes/explore.md` is not a target.
`CLEANUP_DRY_RUN=1 bash cleanup.sh` lists what a real run would kill. A real run
refuses while `clearpath-platform.service` is active, because the robot's
Clearpath services run as the same user.

## Editing a CLI module changes nothing until you rebuild

`ridgeback_autonomy` is a `--symlink-install` package, so editing a module under
`src/` takes effect immediately — for anything imported as
`ridgeback_autonomy.*`. The executables under
`install/ridgeback_autonomy/lib/ridgeback_autonomy/` are not symlinks: CMake
copies them with `install(PROGRAMS)`, and several are copies of a module file
rather than thin wrappers around it. `target_benchmark_configurator` is a byte
copy of `benchmarking/configurator.py`.

The failure is confusing rather than loud, because the two halves disagree: the
running executable is the stale copy, while everything it imports is live. A
configurator started this way served a current capability contract and current
artifact summaries from the symlinked modules, but ran request handlers from
whenever the package was last built — so a validation rule added that morning
silently did not fire, with no error anywhere.

```bash
# The tell: the executable's mtime is older than the module it was copied from.
stat -c '%y %n' install/ridgeback_autonomy/lib/ridgeback_autonomy/target_benchmark_configurator \
                src/ridgeback_autonomy/ridgeback_autonomy/benchmarking/configurator.py
colcon build --packages-select ridgeback_autonomy --symlink-install   # ~1 s
```

Running the same call in-process (`python3 -c 'import ...; validate_job(...)'`)
and over HTTP is what separates the two: identical inputs, different answers
means the executable is stale, not the logic wrong.

## Long sweep loses Gazebo

A long GUI-backed sweep can fail in Gazebo's render thread while creating a
new visual. The sequence is usually:

- configuration wall times rise steadily although short-window RTF remains healthy;
- pose lookup times out;
- later spawns time out;
- `pgrep -f "gz sim"` finds no simulator while the supervisor keeps running.

Stop the supervisor, run `bash cleanup.sh`, then rerun the identical sweep
command so valid configuration outputs resume. Split runs expected to exceed
roughly five hours. RTF alone does not detect this GUI-resource degradation.

## Sweep YAML changed during a run

Installed configuration files are symlinked to the source tree. Editing a
`benchmark_sweep_*.yaml` file therefore changes the source a running sweep will
hash. Resume requires the same source SHA-256 and selected configuration set;
even a comment change or a different `--only` selection creates a new sweep.
Compare the source hash with `resume_signature.sweep_source_sha256` in
`sweep.json` and restore the original bytes before resuming.

## Event-driven startup

Bringup uses `ros2 run ridgeback_autonomy launch_wait` and launch events, not
fixed delays. Exploration waits in this order:

| Gate | Requires | Starts |
|---|---|---|
| `gate_slam` | scan and filtered odometry publishers | SLAM |
| `gate_slam_configure` | `slam_toolbox/change_state` service | SLAM configure/activate |
| `gate_nav2` | map publisher | Nav2 |
| `gate_explorer` | global costmap publisher | frontier explorer |

Timeouts are bounded safety fallbacks: they report missing prerequisites and
permit launch to continue. If a stage stalls, fix the named input or raise the
timeout for a genuinely slow host. `manual_mapping.launch.py` uses the same
pattern for SLAM and controller activation.

## Platform command timestamp warnings

Occasional `platform_velocity_controller` warnings about a command older than
the generated `reference_timeout: 0.1` can be simulation jitter. Do not edit the
generated Clearpath controller configuration first. Check Nav2 retarget churn,
collision-monitor approach flicker, the command chain (`cmd_vel`,
`cmd_vel_smoothed`, `platform/cmd_vel`), simulation rate, and controller timing.
The generated file is under the active Clearpath `setup_path`, normally
`~/clearpath/platform/config/control.yaml`.

## CycloneDDS participant ceiling

If Gazebo, RViz, and detection start but SLAM/Nav2 disappear with
`Failed to find a free participant index`, verify `CYCLONEDDS_URI`. The packaged
`config/cyclonedds.xml` raises `MaxAutoParticipantIndex`, and public entrypoints
set it only when the caller has not supplied a value. An unrelated environment
override can silently restore the too-small ceiling:

```bash
echo "${CYCLONEDDS_URI:-<unset>}"
```

## Large sensor samples stop reaching localization health

If high-resolution color/detections advance but localization health reports
`stale_input` for depth or point clouds, inspect `CYCLONEDDS_URI` and
`sysctl net.core.rmem_max`. The workspace CycloneDDS configuration requests a
16 MiB socket receive buffer for large sensor bursts. The OS may cap the
request; it must permit at least 16777216 bytes to provide that capacity.
An explicit DDS configuration override must carry the receive-buffer setting
as well as the participant ceiling. Restart affected processes after changing
DDS configuration. Confirm advancing depth/point-cloud source stamps and
`processing` health; extending health timeouts does not repair missing samples.

## Camera rate collapses under software rendering

If the simulated camera falls from about 28 Hz to 4–6 Hz, inspect the renderer:

```bash
glxinfo | grep "OpenGL renderer"
```

`llvmpipe` means Gazebo is rasterizing on the CPU. Launch through
`tools/gpu-run` for the measured NVIDIA GLX path. For a server-only session,
select the NVIDIA EGL vendor and pass `headless_rendering:=true`. Disabling the
Gazebo GUI alone does not turn software sensor rendering into GPU rendering,
and healthy RTF does not prove the camera rate is healthy.

The camera stream is best-effort, so a default reliable `ros2 topic hz`
subscription may not match. Use a sensor-data QoS subscriber or the mask node's
depth-match receive counters.

## SLAM TF lag aborts valid Nav2 plans

If MPPI reports `Unable to transform goal pose into costmap frame` with a small
future-extrapolation gap, keep these settings:

- `restamp_tf: true`, so `slam_toolbox` owns a current `map -> odom` stamp;
- `scan_queue_size: 1`, so asynchronous mapping keeps only the latest scan.

Increasing MPPI's transform wait does not fix stale producer timestamps and can
cause missed controller cycles. The measured investigation is preserved in the Archived evidence section below.

## SLAM drift while stationary

Featureless worlds can drift if `slam_toolbox` processes every stationary scan.
Keep `minimum_travel_distance: 0.05` and `minimum_travel_heading: 0.05` in
`slam_toolbox_params.yaml`; these gates require about 5 cm or 3 degrees of
motion before another scan update.

## Intel–Thor stream stalls

**Symptom.** Camera topics received on Thor drop to a few hertz or pause for
seconds, or a Thor topic inspected on Intel is slow. Ping between the hosts
stays below 1 ms, and `nstat UdpRcvbufErrors` on the receiving host climbs.

**Cause.** The receiving process is not on its Ethernet-only CycloneDDS
configuration. It may be running Fast DDS defaults, CycloneDDS's own choice of
interface, or, on Intel, the robot services' Ethernet + Wi-Fi configuration
from `/etc/clearpath/setup.bash`. The sender then also uses the lab Wi-Fi.

**Fix.** Source `dds_env.sh` in the receiving shell, confirm with
`tools/intel_thor/intel_services_rmw status` that Intel's services run
CycloneDDS, and run `tools/intel_thor/check_link`. The
[transport reference](physical/intel_thor_transport.md) owns the contract.

## Thor receives sensor streams but no static TF

**Symptom.** On Thor, camera images and scans arrive, but `tf_static`,
`robot_description` and other transient-local topics from Intel never do.
`tf2_echo base_link camera_0_color_optical_frame` waits indefinitely, the
localization mask node reports `TF_MISS_EXTRINSIC` (status 4) with NaN
distances, and `ros2 topic info -v` on Thor shows Intel's nodes as
`_NODE_NAME_UNKNOWN_`. Topic matching from Thor can also be slow.

**Cause.** Intel's robot services use the CycloneDDS profile on br0 and Wi-Fi,
so they advertise Intel's Wi-Fi address (192.168.123.14) as well as
192.168.131.1. Thor is on the same Wi-Fi subnet, so its kernel sends its DDS
replies to that address over Thor's own Wi-Fi, and the lab access point does
not deliver traffic between its clients (Thor and Intel cannot ping each other
over Wi-Fi). Intel never sees Thor's readers or acknowledgements, so it never
sends stored samples; data it streams continuously still arrives over the
cable. Reproduced on 2026-10-12: a once-published transient-local message from
the services profile never reached Thor in domain 0, and arrived within 3 s
with Wi-Fi removed from the publisher's profile, with the Ethernet-only
profile, or with the route below.

**Fix.** On Thor, `sudo tools/intel_thor/thor_lan_route apply` routes
192.168.123.14 over the robot Ethernet, persistently; `check_link` fails
without it. Laptops on the same Wi-Fi meet the same isolation, so RViz over
Wi-Fi needs client isolation disabled on the access point.

## Intel receives nothing from a restarted Thor for minutes

**Symptom.** After Thor's localization restarts, Intel's `localization/status`
stays `missing` and no health or measurements arrive for about 5–6 minutes,
then everything resumes without any action. Thor-local echoes look healthy.
Sometimes one early message gets through, and some publishers (for example
the detector) appear on Intel while others never do.

**Cause.** DDS discovery announcements are multicast. With no IGMP querier on
the robot Ethernet, IGMP snooping stops forwarding the restarted processes'
multicast to Intel until the membership entry ages out. Measured on
2026-10-13: Intel heard Thor's own announcements stop within seconds of a
restart and resume 313–381 s later, and delivery resumed at the same moment.
Firewalls, ports and socket buffers were ruled out. Unicast DDS peers did not
help Intel's already-running consumers.

**Fix.** Make `br0` the querier. Live:
`echo 1 | sudo tee /sys/class/net/br0/bridge/multicast_querier`. Persistently,
install `src/ridgeback_autonomy_hardware/config/intel_thor/90-dynamo-multicast-querier.conf`
as `/etc/systemd/network/10-netplan-br0.netdev.d/90-dynamo-multicast-querier.conf`.
With it on, a restarted Thor's results reached Intel in 0.2 s.
`tools/intel_thor/check_link` checks both.

## Camera frame gaps while RViz runs on Intel

**Symptom.** `tools/camera_contract_check --guard` fails on depth stamps
(gaps of several hundred ms) and on colour/depth pairing whenever RViz is open
on Intel's VNC desktop, and passes with RViz closed.

**Cause.** The VNC display has no GPU, so RViz renders in software. At 30 fps
Mesa's renderer took a full core in per-frame bursts across every core. On
2026-10-13 that gave 567 and 467 ms depth gaps and 98.6 % pairing. With RViz
closed: 133 ms and 99.45 %.

**Fix.** `backend:=hardware` already applies it: a 10 fps hardware layout,
`nice -n 19` on the last two cores, and `LP_NUM_THREADS=2`. On 2026-10-14 that
measured a 100 ms maximum depth gap and 99.26 % pairing with RViz open, with
RViz at ~25 % of one core. If the check still fails, close RViz on Intel and
view it from another computer.

## CLI probes show no data although consumers are healthy

**Symptom.** `timeout 10 ros2 topic echo|hz ... | grep ...` prints nothing for
Thor's results, while `localization/status` reports `processing` and
`ros2 topic info -v` lists the remote publisher. Or `ros2 topic hz` reports
nothing for a best-effort 640×480 image stream that RViz displays.

**Cause.** Piped ros2 CLI output is block-buffered, and `timeout`'s default
SIGTERM ends the process before the buffer is flushed. Separately, the Python
`hz` subscriber falls behind on large best-effort images.

**Fix.** Use `PYTHONUNBUFFERED=1` and `timeout -s INT`, and write to a file
before filtering. For image rates, count headers with
`ros2 topic echo --qos-reliability best_effort <topic> --field header` over a
fixed interval, or read the rate in RViz.

## Hokuyo drivers stuck reconnecting

**Symptom.** No `sensors/lidar2d_*/scan` data, and the `clearpath-sensors`
journal repeats `Could not grab single echo scan`, then `Error count exceeded
limit, reconnecting`, then `Error connecting to Hokuyo: Could not open network
Hokuyo` every 2.5 s. `ss -tn | grep 10940` shows an established connection to
each LiDAR next to a pending new one.

**Cause.** Stock `urg_node` 1.1.2 leaves the socket open when an open fails
after the TCP connect, and a reset session can kill it with SIGPIPE. A Hokuyo
UST serves one client at a time, so the stock driver never recovers on its own.
`r100_0160` runs the patched driver from `/home/robot/ros_overlays/urg_node_fix`
(`patches/urg_node_reconnect_recovery.patch`): after a reset session each
LiDAR's driver reconnects by itself in about 2.25 s (rear 2026-10-13, front
2026-10-14). If this symptom returns, first check that `robot.yaml` still lists
the overlay and that `pgrep -af urg_node_driver` shows its path; a reinstall or
Clearpath regeneration that drops the overlay brings the lockout back. Repeated local-camera-subscriber-associated failures
and Ethernet traffic spikes are recorded in the
[transport blocker](physical/intel_thor_transport.md#camera-subscriber-blocker);
the initiating mechanism remains unproven.
The [backlog](BACKLOG.md#hokuyo-driver-recovery-after-read-timeouts) tracks the
recovery fix, and the incident timeline is in the archived evidence below.

**Recovery.** Stop the offending test subscribers and record the time and active
configuration before restarting `clearpath-sensors`. This restarts only sensor
drivers and clears the reconnect lockout; it does not fix the initiating fault.
Resolve and validate the transport configuration before routine camera tests
resume. The reported rear-offset service restart later restored scans, but no
camera-load retest is recorded. Do not treat restored idle scans as acceptance.

`tools/hardware/observe_lidar_interruption.py` reproduces the reset-session
check read-only, with an operator running one `ss -K`. A candidate
[recovery checker](../tools/hardware/hokuyo_recovery_watch.py) is kept only as a
manual fallback for a driver that is not running the patch; it is **not
installed or live-qualified**, and the patched driver does not need it. Its default is a bounded,
read-only observation. Explicit stationary recovery requires root, an operator,
fresh asserted e-stop and zero raw odometry. Both scans must previously have
been seen; a 12-second missing stream plus both specific reconnect errors in the
recent journal permits one `clearpath-sensors` restart. Success requires both
streams to sustain at least 38 Hz for three seconds, with increasing stamps and
no receive gap over 250 ms. There is no boot service or automatic restart loop.
Offline tests do not prove interrupted-TCP recovery on the physical LiDAR.

## Remote desktop startup and proxy ownership

For repeated disconnects during an otherwise running session, inspect
`/var/log/unattended-upgrades/unattended-upgrades.log`, its `-dpkg.log`, and the
desktop/platform journals before attributing the problem to VNC graphics or DDS.
On 2026-10-14 unattended package updates ran from about 09:25 to 09:32 UTC.
The installer logged repeated service-restart passes; the system journal
recorded requested stops/restarts of VNC, proxies, networking and robot
services. The boot ID remained unchanged, while VNC was replaced repeatedly,
most recently at 09:31:34. Updates completed at 09:32:31; a subsequent passive
health batch passed all 22 checks. This explains that incident's lost desktop
sessions; it does not qualify future update-time continuity. `NRestarts=0`
alone is insufficient: explicitly requested service restarts reset lifecycle
state and need not appear as automatic failure retries.

Evidence is under `artifacts/hardware/20261014-desktop-disconnect-diagnosis/`
and `stationary-batch-20261014T093333Z-18bc1d/`. No maintenance-policy change or
reboot was performed during diagnosis. The update installed a newer kernel and
left a reboot-required marker; the running robot had not rebooted into it.
Do not test motion during package installation or its service-restart phase.

The independent VNC service protects the desktop from vendor webserver reloads,
but its October 6 cold boot showed 37 restarts. Logs included an Xtigervnc
segmentation fault in NVIDIA EGL/GLX and repeated already-running-display errors.
The independent port-9005 proxy failed its VNC dependency and remained inactive;
the vendor webserver subsequently started the listener itself. A browser
connection through that listener can therefore close on a webserver reload,
even while the desktop remains in the independent VNC control group.

Check ownership without restarting the desktop:

```bash
systemctl show ridgeback-vnc.service ridgeback-vnc-websockify.service \
  -p ActiveState -p MainPID -p NRestarts
ss -ltnp '( sport = :5901 or sport = :9005 )'
journalctl -b -u ridgeback-vnc.service -u ridgeback-vnc-websockify.service
```

For each listener PID, read `/proc/PID/cgroup`: port 5901 should belong to
`ridgeback-vnc.service`, and port 9005 to `ridgeback-vnc-websockify.service`.
An active VNC unit alone does not establish proxy independence. Use an independent
SSH session to investigate startup; restarting VNC closes the desktop. Preserve
the journal before recovery. The graphics initialization failure, competing
display ownership and proxy recovery need separate boot verification; they
are not evidence that the retained DDS configuration failed.

The six-file stationary setup repair was installed and independently verified
on 2026-10-12. Its remote-screen drop-in bypasses the old launcher and runs
only the port-9100 proxy. Foreground VNC supervision and independent proxy units
replace the failed dependency arrangement. Mesa selection applies to the
virtual X server; applications retain their own GPU selection. The desktop uses
the real user runtime and an isolated session bus.

Current listener ownership and stationary hardware preservation passed. Cold
boot, backend failure/recovery and vendor-reload continuity remain open. Use the
[root stationary repair workflow](../README.md#stationary-desktop-and-controller-setup-repair)
for exact receipt-based rollback; restarting it closes remote access temporarily.
The legacy combined vendor-camera repair is not the current installation path.
The vendor camera-readiness unit is disabled with the replaced vendor camera
stack and is not part of this desktop repair.

## Camera optical-frame ownership

Mask estimators require `camera_0_color_optical_frame` to the configured base
frame. Exactly one component publishes the camera chain:

| Deployment | Owner |
|---|---|
| Hardware | `realsense2_camera`, using calibrated device extrinsics |
| Simulation | `robot_state_publisher`, using the selected camera model's fixed joints |

The description must not publish a competing nominal chain on hardware. In
simulation the render sensor hangs from the model's color frame, so image pose
and advertised optical frame agree.

## Namespace and base-frame checks

New namespaced nodes should set their namespace, remap absolute TF topics to
relative `tf`/`tf_static`, use a wildcard node key in parameter YAML, and enable
simulation time. A topic namespace still does not determine TF frame IDs.

Every mask-node launch caller passes `base_frame` explicitly. When that frame
does not exist, the mask node can skip the entire batch with
`TF_MISS_EXTRINSIC`, making all mask rows look like ordinary misses. Inspect the
actual graph instead of deriving the frame from the namespace:

```bash
ros2 run tf2_tools view_frames
ros2 topic echo /r100_0001/tf --once
```

The shared TF helper tries the configured frame and its last path segment. It
checks both without waiting before applying the bounded lookup timeout, so an
already-buffered fallback does not cost half a second per batch. If mask
coverage is low, distinguish `UNSET` (no mask result produced) from
`NO_DEPTH_FRAME` (the batch ran but exact-stamp depth was absent) in `run.json`.

## Required local patches

The stack currently requires the patches listed in the root README. In
particular, `patches/slam_toolbox_tf_namespace.patch` makes the TF listener use
the namespaced node interface. Apply each patch only after `git apply --check`;
the upstream dependency pins and patches are one compatibility unit.

## Archived evidence

- [operational incidents](../archive/engineering/operational_incidents.md)
- [operational incidents](../archive/engineering/operational_incidents.md#slam-tf-lag--2026-09-10)
- [operational incidents](../archive/engineering/operational_incidents.md#hokuyo-reconnect-lockout--2026-09-18)
