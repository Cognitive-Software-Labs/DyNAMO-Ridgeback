# Package split simulator runtime qualification

Recorded dates: 2026-09-18
Tested revisions: `abdbc12566c7cb7f4ab86d89b53eabf5fc119040`
Provenance: partial

## Conditions and correction

The workstation had Gazebo Harmonic 8.11.0 and an NVIDIA RTX PRO 6000 Blackwell
Workstation Edition, driver 595.84. The earlier claim that this host lacked
Gazebo and NVIDIA access was corrected on 2026-09-18: an unsourced shell hid
ROS vendor executables, and the execution sandbox hid workstation GPU access.
Runtime checks sourced ROS Jazzy plus the workspace install and ran with host
device access. Isaac used the workstation's existing 6.1 environment.

The tested implementation was the revision above. Final cases additionally
applied the preserved [DDS configuration patch](assets/package-split-runtime/runtime.patch).
The [probe](assets/package-split-runtime/probe.py),
[robot input](assets/package-split-runtime/robot.yaml),
[core manifest](assets/package-split-runtime/core.repos), and
[Gazebo manifest](assets/package-split-runtime/gz.repos) are preserved.
The installed workspace used its existing simulator and inference environments;
model cache contents and all system/virtual-environment binaries were not
snapshotted. This is bounded integration evidence, not a fully reproducible
performance benchmark. Dependency revision/patch checks passed.

## Method

Each case launched the public exploration entrypoint in `mock_hospital`, with
headless rendering, RViz/coverage overlay off and autonomous motion disabled.
No navigation goals were submitted. Each launch used its own process group and
Gazebo partition, copied robot configuration, and ROS domain 178 with localhost
discovery. Cleanup targeted only that launch's process group.

The probe required both `controller_server` and `bt_navigator` active, at least
three clock/color/depth/scan/odometry/map messages and the requested color image
size. Enabled localization additionally required both measurement streams and
raw detections, an acquisition stamp shared by all three, and three consecutive
`processing` health reports. Disabled localization required no publishers on
raw detections, mask measurements or localization health. Each case had a
240-second wall-clock deadline. The [JSON results](assets/package-split-runtime/results.json) preserve launch arguments,
counts, frames, sizes and recent health reports.

Run the probe from the workspace root after sourcing ROS and the built overlay:

```bash
export ROS_DOMAIN_ID=178 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$PWD/src/ridgeback_autonomy/config/cyclonedds.xml"
python3 archive/engineering/assets/package-split-runtime/probe.py isaac 1280x720 true
```

## Receive-buffer comparison

At the tested revision's original CycloneDDS settings, seven of eight cases
passed. Isaac at 1280×720 with localization enabled failed its 240-second limit:
health reported stale depth/point-cloud input despite advancing detection and
measurement batches. A live health sample showed depth stuck at simulation
5.6 seconds and point-cloud input at 0.233 seconds while color reached 30.267
seconds. Empty measurement batches alone would have hidden the missing inputs.

The installed CycloneDDS schema specifies a default 1 MiB receive-buffer
request. The host's `net.core.rmem_max` was 2147483647 bytes. Repeating the failed
case with a [16 MiB minimum buffer](assets/package-split-runtime/comparison.xml) passed. The final patch requests 16 MiB while
allowing the operating system's cap, preserving startup on hosts with smaller
limits. Such hosts must separately qualify sensor delivery; this fallback does
not guarantee high-resolution operation.

## Final matrix

| Backend | Camera | Localization | Result |
|---|---|---|---|
| gz | 1280x720 | off | PASS |
| gz | 1280x720 | on | PASS |
| gz | 640x480 | off | PASS |
| gz | 640x480 | on | PASS |
| isaac | 1280x720 | off | PASS |
| isaac | 1280x720 | on | PASS |
| isaac | 640x480 | off | PASS |
| isaac | 640x480 | on | PASS |

## Limits

All enabled cases produced empty target detections and no finite target-distance
estimates. This establishes startup, sensor transport and empty-frame pipeline
progress, not nonempty inference or estimator accuracy. It does not establish
sustained throughput, GUI behavior, multi-host transport or physical robot
acceptance. Runtime timing and non-default labels require their own benchmark
qualification. Some SIGINT shutdown logs included the pre-existing localization
overlay's repeated `rcl_shutdown` exception; the probe cleaned up its process
group, but these runs do not qualify graceful teardown.
