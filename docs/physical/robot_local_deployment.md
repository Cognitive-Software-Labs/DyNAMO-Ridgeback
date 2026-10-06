# Robot-local deployment

This page owns the changes DyNAMO makes to a physical robot's startup
configuration on the Intel PC, the order in which to re-apply them after a
reinstall, and the camera service contract that robot-side diagnostics rely on.
Operator commands live in the [README](../../README.md#hardware-camera-service);
the DDS transport design lives in the [transport reference](intel_thor_transport.md).

Every change below is made by a tracked repository tool that snapshots what it
replaces, reports drift with `status`, and restores its snapshot with
`rollback`. Re-running the tools in order rebuilds the deployment; nothing on
this page needs to be recreated by hand.

## Re-apply order

| Order | Tool | Robot-local state it owns | Snapshot |
|---|---|---|---|
| 1 | `sudo tools/intel_thor/intel_services_rmw apply` | `/etc/clearpath/cyclonedds.xml`, `/etc/sysctl.d/60-dynamo-dds-buffers.conf`, the middleware and `CYCLONEDDS_URI` in `/etc/clearpath/robot.yaml`, and `60-dynamo-cyclonedds.conf` drop-ins for the MyBotShop units | `/etc/clearpath/dynamo-rmw-backup/` |
| 2 | `sudo tools/intel_thor/camera_service apply` | the `camera_0` entry in `/etc/clearpath/robot.yaml`, `/etc/clearpath/dynamo-camera/`, `/etc/systemd/system/dynamo-camera.service`, and the enablement of the four vendor camera units | `/etc/clearpath/dynamo-camera-backup/` |
| 3 | `sudo tools/intel_thor/time_sync apply --host intel` (Thor: `--host thor`, from its worktree) | chrony and `/etc/chrony/` on each host; Intel keeps internet upstreams, slews at most 500 ppm and serves the subnet | `/var/lib/dynamo-time-sync-backup/` |
| 4 | `sudo tools/intel_thor/ptp_sync apply --host intel`, then `--host thor` on Thor | `linuxptp`, `/etc/linuxptp/dynamo-ptp4l.conf`, `dynamo-ptp4l.service` and `dynamo-phc2sys.service`; on Thor, chrony stopped and disabled while PTP owns the wall clock | `/var/lib/dynamo-ptp-sync-backup/` |

The camera service depends on step 1: its unit sources `/etc/clearpath/setup.bash`
for the middleware, and `apply` refuses unless the installed DDS profile restricts
multicast to discovery (`<AllowMulticast>spdp</AllowMulticast>`). Without that
restriction a single local camera reader floods both bridge ports and interrupts
the LiDARs; see the [transport reference](intel_thor_transport.md#camera-subscriber-blocker).

The desktop is a separate workstream. As of 2026-10-06 the VNC server runs from
`ridgeback-vnc.service`, while the port-9005 browser proxy is still owned by
`mbs-webserver.service`, so stopping the webserver can close a browser desktop.
Its installer is not yet in the repository; record it here when it lands. The
camera service never stops or restarts `mbs-webserver`.

A Clearpath or MyBotShop reinstall can rewrite `/etc/clearpath/robot.yaml`,
re-enable the vendor camera units, or replace their scripts. After one, run
each tool's `status`; any `DIFF` row means that step must be re-applied.

## Camera service

`dynamo-camera.service` owns the D455 from boot and publishes the camera contract
the simulators publish, so the same detector and estimator nodes run unchanged on
every backend. The [camera stack](../target_localization/camera_stack.md) owns
that contract; this section owns how the robot provides it.

`camera_service apply`:

1. Refuses before changing anything unless the robot-local YAML has a namespace,
   `/etc/clearpath/setup.bash` selects CycloneDDS with the installed profile, the
   profile restricts multicast to discovery, exactly one D455 is attached, and no
   other camera is declared.
2. Snapshots `robot.yaml` and the enablement and activity of the vendor units,
   once. A re-apply keeps the first snapshot.
3. Disables and stops `realsense-camera`, `depth-to-mono8`,
   `ridgeback-camera-mjpeg` and `camera-web-ready`, which exist only for the
   vendor web camera view.
4. Declares `camera_0` in `robot.yaml` with the repository mount
   (`default_mount` + `[0.2692, 0, 0.725]`), `urdf_enabled: true` and
   `launch_enabled: false`. The Clearpath description then owns
   `base_link → … → camera_0_bottom_screw_frame → camera_0_link`, and the
   Clearpath generator launches no camera of its own.
5. Renders the driver parameters from
   `src/ridgeback_autonomy_hardware/config/d455.yaml` and the unit from
   `config/systemd/dynamo-camera.service.in`, restarts `clearpath-robot` to
   regenerate the description (platform and sensors restart with it) only if the
   `camera_0` declaration changed, and enables and restarts the service. A
   re-apply that changes only the profile, rate or DDS tuning restarts the camera
   alone.

`apply` and `rollback` take `/run/lock/dynamo-local-dds-test.lock`, the lock the
robot-side DDS experiment tools hold, and refuse while it is held. Any failure
after the snapshot restores it. `rollback`, and an `apply` that changes the
declaration, restart `clearpath-robot`, which drops motor power and teleop
until the platform is back:
stopping `clearpath-platform` usually takes about a minute on `r100_0160`, and
the tool prints nothing while it waits. Run them with the robot stationary and
the e-stop in reach, and let them finish; an interrupted run can leave the
vendor units disabled without the new service started, which `rollback` repairs.

### Installed files

| Path | Content |
|---|---|
| `/etc/clearpath/robot.yaml` | `sensors.camera[0]`: the `camera_0` declaration, including the camera serial |
| `/etc/clearpath/dynamo-camera/d455.yaml` | rendered driver parameters (do not edit) |
| `/etc/clearpath/dynamo-camera/install.json` | installed profile, serial, namespace and source revision |
| `/etc/systemd/system/dynamo-camera.service` | rendered unit (do not edit) |
| `/etc/clearpath/dynamo-camera-backup/` | `robot.yaml` and `units.json` from before the first apply |

The unit runs only the apt `realsense2_camera` driver and the rendered files; it
runs no workspace code. Rebuilding or switching the checkout therefore cannot
change the camera at the next boot. Editing the repository configuration does
nothing until `apply` is re-run; `status` shows the revision installed and
flags rendered files that no longer match the checkout.

### Profile and serial

The profile is chosen at install (`apply --profile 640x480|1280x720`, default
`640x480`), not per launch: a driver started at boot cannot change it for one
run. Both infrared streams are off, which 1280x720 requires, and the pointcloud
is off until an organized cloud is qualified.

`serial_no` is the camera serial that librealsense reports (`Device Serial No`
in the driver log, or `rs-enumerate-devices -s` while no driver holds the
device). The USB descriptor serial in `/sys/bus/usb/devices/*/serial` is a
different number and does not select the camera. `apply` enumerates the camera
after the vendor driver releases it; `--serial` overrides that, and a re-apply
reuses the serial already declared.

### Contract for diagnostics

Tools that observe or restart the camera, such as robot-side DDS tests, should
target this layout rather than the vendor one.

| Item | Value |
|---|---|
| Unit | `dynamo-camera.service`, `User=robot`, `Restart=always`, `Conflicts=realsense-camera.service` |
| Process | one: the unit's main PID is `realsense2_camera_node` itself |
| Environment | `/etc/clearpath/setup.bash`: domain, `RMW_IMPLEMENTATION`, `CYCLONEDDS_URI` identical to the Clearpath services |
| Node | `/<ns>/sensors/camera_0` |
| Colour | `/<ns>/sensors/camera_0/color/image`, `sensor_msgs/Image`, colour optical frame |
| Colour calibration | `/<ns>/sensors/camera_0/color/camera_info` |
| Aligned depth | `/<ns>/sensors/camera_0/depth/image`, `16UC1` millimetres on the colour grid, colour optical frame, stamps identical to colour |
| Compressed variants | `color/image/{compressed,compressedDepth,ffmpeg,theora,zstd}` and the same under `depth/image/`, encoded only while subscribed. Encoder parameters keep the driver's original names, e.g. `camera_0.color.image_raw.ffmpeg.encoder` and `.camera_0.color.image_raw.compressed.jpeg_quality` |
| Other driver outputs | keep driver names, for example native depth `depth/image_rect_raw` |
| Optical frame | `camera_0_color_optical_frame` |
| TF | driver: `camera_0_link →` its frames on `/<ns>/tf_static`; Clearpath description: `base_link → camera_0_link` |
| Installed profile | `/etc/clearpath/dynamo-camera/install.json` |

### Verify

```bash
tools/intel_thor/camera_service status
source /etc/clearpath/setup.bash
tools/camera_contract_check --backend hardware --output artifacts/hardware/<run>/camera-contract.json
```

Source `/etc/clearpath/setup.bash` first so the check joins the robot graph with
the services' domain, middleware and DDS profile. It subscribes to the three
camera topics and the three scans for ten seconds of warm-up and sixty seconds
of measurement, and passes only if every criterion it prints passes. It does not
prove application accuracy, motion behaviour, or camera–LiDAR calibration.

## Clock synchronization and camera transport candidates

`tools/intel_thor/time_sync` supplies `status`, operator-run `apply --host
intel|thor`, and `rollback`. It installs chrony, preserving Intel's existing
configured chrony upstreams or the active timesyncd server. `--upstream` can
explicitly retain several Intel internet servers. Intel allows the robot
subnet and serves a local stratum-10 clock when upstreams are unavailable;
Thor uses only `192.168.131.1`, with no DHCP or internet source includes.
Its LAN source uses `minpoll -4 maxpoll -4 xleave filter 4`: 16 requests per
second with four-poll median filtering and interleaved server timestamps.
This is a candidate repair for the several-millisecond startup drift observed
with the default 64-second polling while Intel slewed to its internet source;
verify the independent bound before accepting it. Sub-second polling is
supported for reachable LAN servers with round trips below 10 ms by
[chrony 4.5](https://chrony-project.org/doc/4.5/chrony.conf.html#server).

The managed main file `/etc/chrony/chrony.conf` includes only
`/etc/chrony/conf.d/dynamo-time.conf`. Before package installation,
`/var/lib/dynamo-time-sync-backup/` saves the original chrony directory,
package selection, time-service activity/enablement and rendered upstreams.
Rollback reinstalls systemd-timesyncd when it was originally installed and
restores original files and active/enabled services. File drift causes reapply
and rollback to refuse before changing services. An installation failure keeps
the snapshot for operator rollback. Package versions/cache are not snapshotted;
rollback may require apt connectivity. Time changes can step wall clocks, so
apply while the robot is stationary and no timing run holds the experiment lock.

chrony serves an estimate of true time, while sensor stamps come from Intel's
wall clock, so a chrony client follows Intel only to within Intel's pending
internet correction. `tools/intel_thor/ptp_sync` (`status`, operator-run
`apply --host intel|thor`, `rollback`) carries the wall clock itself. On Intel,
`dynamo-phc2sys.service` copies the wall clock into `eno1`'s hardware clock and
`dynamo-ptp4l.service` serves it over raw Ethernet (L2, hardware timestamps),
since Intel's address is on `br0` rather than the port. On Thor, `ptp4l` follows
on `enP2p1s0` and `phc2sys` sets the wall clock; chrony is stopped and disabled
while PTP owns it, and its unit `Conflicts=` with chrony. Both hardware clocks
hold UTC. Intel's chrony must first slew at most 500 ppm (`time_sync`), which
`apply` checks, so internet corrections reach Thor smoothly. `apply` installs
`linuxptp`, renders `config/intel_thor/ptp/`, and fails if either daemon does
not stay running; `/var/lib/dynamo-ptp-sync-backup/` keeps the package choice
and chrony's state for rollback. Thor's wall clock may step once at
installation by the current offset.

The camera installer adds `--fps 15|30`, `--dds-profile services|camera`, and
`--max-message-size default|65500`. New installs keep `services` at 30 Hz;
reapply preserves installed options. `camera` renders
`config/cyclonedds_camera.xml` to
`/etc/clearpath/dynamo-camera/cyclonedds.xml`: `br0` only, discovery-only
multicast, a Thor peer and 16 MB send/receive buffers. The unit overrides
`CYCLONEDDS_URI` **after** sourcing the Clearpath environment. The services'
`/etc/clearpath/cyclonedds.xml` stays intact.

Camera mode also installs `/etc/sysctl.d/61-dynamo-camera-buffers.conf` and
sets `net.core.wmem_max=16777216`. It extends older camera snapshots with the
original sysctl file and runtime value before modifying them. Switching back
to `services`, or rolling back, restores both. Metadata records fps, DDS mode
and maximum-message-size selection; `status` checks the rendered profile and
required send-buffer ceiling. These are tuning candidates, not adopted robot
settings; use the [sensor timing reference](intel_thor_sensor_latency.md) and
active plan to qualify them before adoption.

## Archived evidence

- [r100_0160 D455 camera service qualification](../../archive/engineering/2026-10-06-r100-0160-camera-service.md)
