# Intel–Thor DDS transport

This page owns the DDS transport contract between the robot's Intel PC and the
Jetson AGX Thor: the middleware, which interfaces carry ROS traffic, the
receive-buffer requirement, the Intel service switch, and the pre-deploy check.
Operator commands live in the
[README](../../README.md#intel-thor-link-setup-and-pre-deploy-check); unfinished
deployment work lives in the
[Intel–Thor deployment plan](../plans/PHYSICAL_intel_thor_deployment.md).

## Topology

| Host | Robot Ethernet | Also attached to |
|---|---|---|
| Intel (`cpr-r100-0160`) | `br0` 192.168.131.1; Thor is learned on bridge port `eno1` (1 Gb/s), the same port as both Hokuyo LiDARs | Lab Wi-Fi `wlp3s0` 192.168.123.0/24 (DHCP), Tailscale |
| Thor (`nvidia-thor-r100-0160`) | `enP2p1s0` 192.168.131.51 (1 Gb/s, static) | Lab Wi-Fi 192.168.123.0/24 (default route) |

## Middleware

The deployment selects CycloneDDS (`rmw_cyclonedds_cpp`) on both ROS 2 Jazzy
hosts, matching the repository's simulator default. Confirm effective settings
on each running process before acceptance; repository profiles do not prove
the installed service state. Intel's Clearpath services ran Fast DDS
only because the live `/etc/clearpath/robot.yaml` did not name a middleware.
In a cross-host benchmark the two middlewares tied within the link's capacity.
The team selected CycloneDDS so that the robot and the simulators use the same
middleware. Mixing them is avoided: when the link was overloaded, a Fast DDS
publisher feeding a CycloneDDS subscriber queued frames for about 11 s.

## Why each host needs an interface restriction

Both hosts are on the lab Wi-Fi as well as the robot Ethernet. Unconfigured,
Fast DDS sends every cross-host sample over both networks, and CycloneDDS picks
an interface itself, which may be the Wi-Fi default route. Either way, camera
frames (about 900 KiB each) cross the lab Wi-Fi, and unplugging the Ethernet
silently moves traffic onto Wi-Fi instead of failing visibly. With the
distribution's 212,992-byte socket buffers, duplicated frames overflow the
receive socket and the stream stalls, even though ping stays below 1 ms.

The restriction must be on the **receiving** side. A writer sends to every
address its reader advertises, whatever the writer's own configuration.
In one test Thor was pinned to Ethernet and the Intel reader also advertised
Wi-Fi: Thor then sent the stream through its own Wi-Fi (466 Mbit/s across both
hosts).

## Configuration contract

The deployment files are in
[`src/ridgeback_autonomy_hardware/config/intel_thor/`](../../src/ridgeback_autonomy_hardware/config/intel_thor/):

| File | Used by | Interfaces |
|---|---|---|
| `cyclonedds_thor.xml` | Every ROS process on Thor, through `dds_env.sh` | 192.168.131.51 only |
| `cyclonedds_intel.xml` | Intel repository nodes and `ros2` sessions that receive from Thor, through `dds_env.sh` | 192.168.131.1 only |
| `cyclonedds_intel_services.xml` | Intel robot services, installed as `/etc/clearpath/cyclonedds.xml` | `br0` and `wlp3s0` by name |
| `dds_env.sh` | Sourced by operators and launches on either host. It selects `rmw_cyclonedds_cpp` and the host's Ethernet-only configuration (the role comes from the Ethernet address present, or `intel`/`thor` explicitly), and leaves `ROS_DOMAIN_ID` to the deployment. | — |
| `60-dynamo-dds-buffers.conf` | `/etc/sysctl.d/` on both hosts | Raises only `net.core.rmem_max` to 16 MiB |
| `fastdds_thor.xml`, `fastdds_intel.xml` | `rmw_benchmark` comparisons only | Ethernet address only |

Rules:

- **Thor is Ethernet-only.** Every Thor reader therefore advertises only
  192.168.131.51, so Intel sends Thor's inputs over `br0` even from the
  robot services, which are also on Wi-Fi.
- **Intel readers of Thor output are Ethernet-only.** Repository nodes and any
  `ros2` inspection of Thor topics source `dds_env.sh`. An ordinary Intel login
  shell sources `/etc/clearpath/setup.bash`, which carries the services'
  Ethernet + Wi-Fi configuration. If such a shell subscribes to a large Thor
  topic, Thor sends that topic over Wi-Fi.
- **The robot services stay on Wi-Fi as well**, so lab laptops keep access to
  the robot graph. These services publish to Thor and never subscribe to its
  output.
- **Buffers are a soft maximum.** Each configuration asks for up to 16 MiB,
  capped by `rmem_max`. A host without the sysctl still starts its services;
  `check_link` reports the missing limit. `rmem_default` stays at the
  distribution default.
- **Addresses and interface names are fixed in the files.** If a robot
  Ethernet address or interface name changes, update the configurations,
  `dds_env.sh`, and `check_link` together.
- The participant-index ceiling from
  [`config/cyclonedds.xml`](../../src/ridgeback_autonomy/config/cyclonedds.xml)
  is repeated in every configuration. `dds_env.sh` sets `CYCLONEDDS_URI`, so
  the launch files do not apply their own default.
- If `dds_env.sh` cannot select a host, or receives an invalid role, it returns
  nonzero and preserves the previous DDS environment. A chained
  `source .../dds_env.sh intel && ros2 ...` therefore stops on setup failure.

## Intel service switch

[`tools/intel_thor/intel_services_rmw`](../../tools/intel_thor/intel_services_rmw)
moves Intel's robot services between the middlewares. `status` needs no root.
`apply` and `rollback` run with sudo and restart services, which briefly drops
motor power and teleop. Run them with the robot stationary and the e-stop in
reach.

`apply` performs these steps:

1. Saves `/etc/clearpath/robot.yaml`, its middleware selection, the original
   `/etc/clearpath/cyclonedds.xml`, and the managed service drop-ins to
   `/etc/clearpath/dynamo-rmw-backup/`. File absence, permissions, and symlinks
   are preserved. Repeated applies keep the first complete backup.
2. Installs `cyclonedds_intel_services.xml` as `/etc/clearpath/cyclonedds.xml`,
   and installs the sysctl file.
3. Sets `system.ros2.middleware.implementation: rmw_cyclonedds_cpp` and
   `system.bash.env.CYCLONEDDS_URI` in `robot.yaml`. `clearpath-robot.service`
   regenerates `/etc/clearpath/setup.bash` from these keys. The platform and
   sensor start scripts export Fast DDS first and then source that file, so the
   regenerated values win. Clearpath's `profile` key only sets the Fast DDS
   variable, so the Cyclone path is passed through `bash.env` instead. The
   Clearpath installer is not rerun, which keeps the MyBotShop edits to the
   generated units.
4. Adds `60-dynamo-cyclonedds.conf` drop-ins to the MyBotShop units that do not
   source the Clearpath setup: `clearpath-scan-merger`,
   `clearpath-joy-x-combiner`, `realsense-camera`, `depth-to-mono8`,
   `ridgeback-camera-mjpeg` and `mbs-webserver`. The one-shot or normally
   disabled `camera-web-ready`, `ridgeback-startup-recover`, `robot-slam` and
   `robot-map-navi` get the drop-in as well, so they use the same middleware
   whenever they run.
5. Restarts the units that were running and are not already on CycloneDDS.
   Stopped units stay stopped, so running `apply` again restarts nothing that
   has already switched.

`rollback` restores the saved `robot.yaml`, DDS configuration, and managed
drop-ins before regenerating the Clearpath setup and restarting the currently
active units on the saved middleware. Files originally absent are removed;
existing files and symlinks are restored. Settings are reloaded even when the
previous middleware was already CycloneDDS. Stopped units remain stopped.
The backup is removed only after successful restarts; a failed rollback can
be retried. The receive-buffer sysctl file and live limit remain in place.

An incomplete backup, including an older backup containing only `robot.yaml`,
blocks both `apply` and `rollback` before host configuration is changed. Those
older backups cannot establish the original DDS file or drop-in contents.
Recover those originals manually and retain the old evidence separately before
starting a new managed switch; do not treat current installed files as originals.

`battery-sysfs-bridge` sources the Clearpath setup, so the switch reaches it.
The Fast DDS profile it exports has no effect under CycloneDDS.

## Pre-deploy check

[`tools/intel_thor/check_link`](../../tools/intel_thor/check_link) runs on Intel
and reaches Thor over key-based SSH. It exits nonzero on any failure.

| Area | Fails when | Warns when |
|---|---|---|
| Routing and link | Intel lacks 192.168.131.1, the route to Thor uses Wi-Fi, either Thor-facing port is below 1000 Mb/s full duplex, or ping loses packets | — |
| Buffers, each host | `rmem_max` is below 16 MiB | The limit is not persisted in `/etc/sysctl.d` |
| Intel services | — | `/etc/clearpath/setup.bash` does not select CycloneDDS, or a running robot service is still on Fast DDS |
| Thor host | SSH fails, or `rmw_cyclonedds_cpp` is missing | Thor's checkout is missing or at a different revision |
| Configurations | A file does not parse, or its interfaces differ from the table above | — |
| Clock | Offset exceeds 10 ms | Offset exceeds 2 ms, or a host reports NTP unsynchronized |
| Traffic, each direction | The probe receives nothing or under 90% of 30 Hz, or Wi-Fi carries more than 20 Mbit/s during the stream | The largest frame gap exceeds 100 ms, or the probe's own sockets drop packets |

The clock check measures the offset over one SSH channel and bounds its error
by half the smallest round trip. The traffic test uses
[`dds_link_probe.py`](../../tools/intel_thor/dds_link_probe.py) with the
deployment configurations to stream synthetic 900 KiB frames at 30 Hz, for
10 s each way (`--duration`). It uses ROS domain 87 (`--domain`), so it never
joins the robot graph. It shares the Ethernet segment with the LiDARs, so run
it only with the robot stationary; `--no-traffic` skips it. Drops are counted
on the probe's own sockets, because the host-wide `UdpRcvbufErrors` counter
includes unrelated processes. The Wi-Fi threshold allows for the background
traffic of the robot's web and Foxglove services; a duplicate camera path
shows up at roughly 200 Mbit/s.

[`tools/intel_thor/rmw_benchmark`](../../tools/intel_thor/rmw_benchmark)
repeats the middleware comparison. It covers 640×480 and 1280×720, with and
without depth, in both directions, and measures loss, exact-stamp
colour/depth pairing, round-trip time and CPU. It writes its results under
`artifacts/hardware/rmw_benchmark/`.

## Known limits

- The raw 1280×720 colour + depth stream is about 1.1 Gbit/s, which does not
  fit the 1 Gb/s link on either middleware. Senders fall behind, and frames
  arrive more than a second old.
- Intel's vendor image consumers `depth_to_mono8.py` and the MJPEG camera
  server each drop UDP packets continuously (about 100 per second combined,
  recorded under Fast DDS). They are outside this repository and do not affect
  Thor, but they make Intel's host-wide drop counter useless as a link metric.
- Both hosts take time from internet NTP over the lab Wi-Fi. The check measured
  Thor between 0.75 ms and 4.2 ms ahead of Intel during one day. Hardening
  time sync (Intel serving the robot subnet) is open work in the deployment
  plan.
- About 4 minutes after Intel's services first moved to CycloneDDS, both
  Hokuyo drivers timed out together. Because of a reconnect bug in `urg_node`,
  they could not recover until `clearpath-sensors` was restarted. The trigger
  was not reproduced; see
  [troubleshooting](../troubleshooting.md#hokuyo-drivers-stuck-reconnecting).
- The check and the benchmark use synthetic frames from a Python probe. They
  do not prove camera-driver health, the real driver's timing, or the rate of
  the real perception pipeline.

## Archived evidence

- [Intel–Thor DDS transport diagnosis](../../archive/engineering/2026-09-18-intel-thor-dds-transport.md)
- [Intel–Thor middleware benchmark](../../archive/engineering/2026-09-18-intel-thor-rmw-benchmark.md)
