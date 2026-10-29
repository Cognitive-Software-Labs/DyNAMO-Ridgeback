# Robot-local deployment

This reference owns the installed D455 service and its reconstruction inputs.
Public commands live in the [README](../../README.md#hardware-camera-service).
Host reconstruction and credentials belong to the
[recovery runbook](ridgeback_recovery.md).

<a id="camera-service"></a>

## Camera owner and source

`dynamo-camera.service` owns the D455; the vendor camera units stay disabled
and inactive. The deployment snapshot of the installed service, captured
2026-10-12, matched the restored `tools/intel_thor/camera_service`: the
description-only camera declaration, driver parameters, unit and
discovery-only multicast configuration. The tool's source and templates were
recovered from the installed metadata's Git revision
`4e5a9b5476ad9c502d90718709629e2f2052cccb`; that recovery changed no live
service. The later [1280×720 trials](#high-definition-profile) re-ran the
maintained installer, so the robot's own `install.json`, not that snapshot,
records the current installation.

The maintained sources are `tools/intel_thor/camera_service`, the D455 and DDS
configs under `src/ridgeback_autonomy_hardware/config/`, and the systemd template
there. The selected robot input is `deployment/r100_0160/camera.json`: D455
serial 338522302134, 640×480 at 30 FPS, services DDS profile. The driver reads
`/etc/clearpath/dynamo-camera/d455.yaml` and runs directly from the apt ROS
installation rather than this checkout's build/install outputs. Metadata at
`/etc/clearpath/dynamo-camera/install.json` records its installed profile and
source revision. A different checkout revision alone does not imply file drift.

The camera service owns colour and colour-aligned depth, remapped to
`/r100_0160/sensors/camera_0/color/image` and `…/depth/image`, with colour
CameraInfo at `…/color/camera_info`. Infrared, IMU and pointcloud outputs are off.
The robot declaration owns the camera mount; the driver owns its factory
extrinsics and remaps TF to the namespaced tree. Installation conflicts with the
vendor camera unit. Do not run the legacy vendor DDS/repair workflow against
this service or re-enable its competing camera units.

`tools/camera_contract_check` targets these installed names explicitly.
`ridgeback_common.camera_inputs` gives the RealSense backend the same three
names as the simulators, so localization launches consume the installed
service's topics. Restoring the
tooling and matching files does not qualify image timing, TF, target estimation
or hardware performance.

## Local reader checks

`tools/camera_contract_check --backend hardware --guard` checks the installed
service from Intel. It decodes only image headers, so its own load cannot pose
as a camera fault. With `--guard` it discovers the scans before starting
readers and watches their gaps through warm-up and measurement. Its pass
criteria are frozen in the tool. `tools/intel_thor/check_link --camera-reader`
runs one such reader as part of the
[pre-deploy check](intel_thor_transport.md#camera-subscriber-blocker).
`tools/hardware/screen_camera_port_traffic.py` adds packet evidence: it compares
five quiet seconds with a guarded reader trial and captures 96-byte packet
prefixes on both wired ports and Wi-Fi. It sends no commands, restarts nothing
and preserves evidence for packet destination and protocol analysis. Total
port traffic alone does not prove multicast flooding or a LiDAR fault
mechanism.

At 640×480, extra local readers leave the camera, the LiDARs and the wired
ports undisturbed. One and two independent reader pairs, including two pairs
held for 30 minutes, pass the full contract: about 30 Hz unique colour and
depth with no duplicate or regressing stamps, at least 99.7% exact
colour/depth pairing, the base-to-optical TF lookup, and front, rear and merged
scans near 39.9 Hz. Wired TX stays within 5 Mbit/s of its baseline, and
`clearpath-sensors` logs no LiDAR grab or reconnect errors. These are S1, S2
and S4 in the [qualification plan](../plans/PHYSICAL_remaining_qualification.md).
All ran stationary with SLAM and Nav2 absent; the loaded multi-hour soak was
not run and remains a [deferred gate](../BACKLOG.md#deferred-hardware-gates).

Local delivery adds no wired traffic. The standing load on `eno1` is the
camera's UDP unicast from `192.168.131.1` to Thor perception subscribers at
`192.168.131.51`, sent by the `realsense2_camera` socket. With Thor perception
stopped, `eno1` TX falls below 1 Mbit/s. The
[transport reference](intel_thor_transport.md#topology) owns how many image
copies Thor pulls.

## High-definition profile

**1280×720 is not supported on the robot (user decision, 2026-10-29).**
**Decision (user, 2026-10-16): Thor stays at 640×480**, so the deployment
profile is 640×480 at 30 Hz raw. The archived S2 trials are the only hardware
evidence for 1280×720. No further HD testing or development is planned, and anyone who
needs HD must develop and qualify it first. Compressed or reduced-rate HD for
Thor is not pursued. The trials established two limits:

- Raw 1280×720 colour (`rgb8`, 2,764,800 B) plus aligned depth (`16UC1`,
  1,843,200 B) at 30 Hz is about 1,106 Mbit/s for one remote copy, before DDS
  and Ethernet overhead, so it does not fit the 1 Gb/s robot Ethernet. While
  Thor pulls raw HD the link saturates and the driver itself publishes only
  about 22 Hz, so local Intel readers drop to that rate too. The LiDAR scans are
  unaffected.
- With Thor's consumers stopped, local HD delivery reaches about 30 Hz but has
  source-side frame dropouts, up to 166.8 ms against the 150 ms stamp-gap
  limit. HD local delivery is therefore not qualified.

## Clock synchronization and camera transport candidates

Intel–Thor clock synchronization and latency qualification are owned by the
[sensor timing reference](intel_thor_sensor_latency.md). The
[clock installation commands](intel_thor_sensor_latency.md#clock-installation)
own the public `tools/intel_thor/time_sync` and `tools/intel_thor/ptp_sync`
installation and rollback. Use those maintained tools rather than
temporary session scripts. The
[transport reference](intel_thor_transport.md#sensor-timing-and-time-synchronization)
describes PTP ownership; camera reader and traffic conclusions are above.

## Reconstruction and verification

The recovery preparer uses camera.json to regenerate the description-only
camera entry, service and parameters from maintained sources. Installed unit,
parameters, private DDS profile and metadata are labelled snapshots in the
deployment manifest. They are evidence, not a substitute for rendering
or fresh installation metadata. Select the camera service instead of the
historical vendor wrapper; retain the merger's explicit Fast DDS exception.

On a replacement host, establish ROS packages, interfaces, namespace and the
services DDS profile first. Then use the maintained camera installer to render
and record the actual installation, following the root workflow. Its apply and
rollback operations can regenerate Clearpath state and restart services; they
are not read-only checks and are not part of the current stationary desktop
repair. Driver serial selection uses the RealSense camera serial, not its
separate USB descriptor serial.

The public reconstruction recipe and camera tool tests verify offline rendering
and failure/rollback behavior. Clean installation, cold boot and perception
qualification remain separate evidence gates. Changes to profiles or middleware
need a fresh guarded sensor test; Thor remains unchanged until needed.

## Archived evidence

- [r100_0160 D455 camera service qualification](../../archive/engineering/2026-10-06-r100-0160-camera-service.md)
- [r100_0160 local camera-reader and port-traffic screens](../../archive/engineering/2026-10-13-r100-0160-local-camera-reader-screens.md)
- [r100_0160 Thor image copies and 1280×720 camera trials](../../archive/engineering/2026-10-16-r100-0160-thor-image-copies-and-hd-trials.md)
