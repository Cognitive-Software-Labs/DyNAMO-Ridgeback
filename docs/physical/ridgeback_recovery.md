# Ridgeback host setup and SSD recovery

This is the reconstruction runbook for Intel host `cpr-r100-0160`, robot
`r100-0160`. It describes the deployment inspected on **2026-10-06**, with
namespace `r100_0160`, ROS domain `0` and retained CycloneDDS discovery-only
multicast. Small deployed inputs are preserved in
[deployment/r100_0160](../../deployment/r100_0160/README.md), with hashes,
ownership, permissions, package versions and service activity. That capture
was taken at 10:00 UTC, before the camera moved from the vendor services to
`dynamo-camera.service` the same morning; section 6 describes the change.

The root [README](../../README.md#installation) owns normal workspace
installation and public commands. This page owns the additional robot-host
setup needed after replacing the SSD. A blank-SSD reconstruction has **not**
been tested. The user declined a separate spare-SSD rehearsal on 2026-10-06;
these instructions are retained for future recovery. Current sensor verification does not establish desktop cold-boot
reliability, base motion readiness or long-term reconnect recovery.

The selected recovery scope is **ROS hardware and an independent remote desktop**;
the vendor dashboard is optional. Follow the executable preparation/build recipe
in [recovery without the vendor dashboard](ridgeback_public_recovery.md).
Captured files below still describe the observed vendor deployment; preparation
adapts the selected files without changing that baseline or the running robot.

## Reconstruction sources and limits

| Component | Reconstruction source | What must be supplied or checked |
|---|---|---|
| Ubuntu, ROS Jazzy, Clearpath services | Official installers; recorded package inventory | Ubuntu 24.04 amd64; compare versions and generated files |
| This workspace | This Git repository, dependency manifests and patches | Save the intended revision; rebuild rather than copying `install/` |
| Robot identity and sensor declarations | Captured `/etc/clearpath/robot.yaml` | Physical namespace differs from repository simulator `r100_0001` |
| DDS | Maintained Intel profiles plus installed copy and unit drop-ins | SPDP multicast; inspect worker environments, not only launch parents |
| Scan merger | Pinned public source and captured local launch/config | Rebuilt independently of vendor workspace; fresh runtime commissioning required |
| Optional vendor dashboard | `MYBOTSHOP/mbs_ridgeback` and vendor handover source | Private access unavailable to current account; not required for selected recovery |
| Local host scripts, units and overrides | Captured `files/`, with `manifest.json` | Select files deliberately; some are disabled or have known faults |
| Ethernet and Wi-Fi | Observed topology below; fresh netplan template | Local console, real interface names, lab addressing, new Wi-Fi credential |
| Tailscale, SSH, VNC, Bluetooth | Fresh enrolment, public keys and pairing | New credentials/identity; do not expect the old Tailscale IP |
| Saved maps and historical recordings | Separate mission data, if available | New mapping can recreate maps; logs are not required to boot |

The observed `/opt/mybotshop` is a manually deployed Colcon workspace, not a
clone. An on-host August handover identifies vendor repository
`https://github.com/MYBOTSHOP/mbs_ridgeback` and bundle
`/opt/mybotshop/handover/mbs_ridgeback-2026-08-12.bundle`, whose `master` is
`6f0701c3d41f737950c60590944caeee3bb5413d`. The bundle needs base commit
`ffa4d2457057e0ca0cb44cc2ddb1271258b34879` and is **not self-contained**. It and the later
`robot_webserver_handover_20260813_v2.tar.gz` are leads for obtaining vendor
source, **not proof that a fresh clone reproduces the current installation**.
They are not included here, and their presence on the failed SSD cannot be
assumed. Request the appropriate source from the vendor when needed. Their
README/config files may contain credentials; do not import them wholesale.

The captured vendor package identities include `robot_webserver` 8.0.8 and
`ros2_laser_scan_merger` 1.0.0. Local launch/config changes are preserved, but
complete vendor C++/Python sources and their exact deployed revision remain an
external prerequisite only for optional vendor features. The public scan merger
is independently reconstructible at its pinned revision. The repository's `dependencies/core.repos` does not
fetch those MyBotShop packages.

## 1. Install the host and recreate the account

Use Clearpath's [Jazzy robot installation guide](https://docs.clearpathrobotics.com/docs/ros/installation/robot/)
for its Ubuntu 24.04 x86 image or standard Ubuntu installation plus computer
installer. Select this robot's identity and inspect what the installer enables
before bringing up the robot. Keep navigation stopped during reconstruction.
SSD replacement does not by itself require flashing the MCU or camera firmware.
An installed firmware-package version does not identify the firmware on the MCU.

Observed baseline: Ubuntu **24.04.4 LTS**, kernel **6.8.0-139-generic**, ROS
**Jazzy**. Create `robot` with home `/home/robot`; observed UID/GID are 1000.
Add the applicable `sudo`, `adm`, `dialout`, `video`, `plugdev`, `input` and
Bluetooth/device access groups. Create or retain other vendor groups only when
their installed packages require them; do not blindly reuse numeric group IDs.
Set the account password locally. Retain hostname `cpr-r100-0160`.

Configure ROS/Clearpath package sources with their official signed installer.
The observed Clearpath apt channel is `stable/ubuntu noble main`, despite the
local filename `clearpath-latest.list`. Install `ros-jazzy-clearpath-robot`,
`ros-jazzy-rmw-cyclonedds-cpp`, `ros-jazzy-rmw-fastrtps-cpp`,
`ros-jazzy-realsense2-camera`, `ros-jazzy-pointcloud-to-laserscan`, and vendor
package dependencies resolved by rosdep. Restore `python3-ds4drv`, `bluez`,
`socat`, `can-utils`, `python3-opencv`, `python3-numpy`, `python3-yaml`,
`python3-psutil`, TigerVNC, websockify, `dbus-x11` and XFCE/Openbox as needed.
The [package inventory](../../deployment/r100_0160/packages.tsv) records exact
observed versions, including Clearpath robot 2.9.4, Cyclone RMW 2.2.3,
RealSense ROS 4.58.1, ds4drv 0.8.0-noble, Tailscale 1.102.2 and TigerVNC 1.13.1.

NVIDIA open driver **610.57.04-1ubuntu1** was installed. Select a supported
matching driver through the OS/vendor instructions; graphics package versions
alone do not guarantee VNC will boot. The local librealsense source was
`/home/robot/librealsense`, revision
`7c3ee3fb7c640e9f315e663907208cb56c4febfd` (`v2.58.3-6-g7c3ee3fb7`). Confirm
which library the camera process loads before substituting an apt/source SDK.

## 2. Recreate networking and access

| Link/device | Observed role/address |
|---|---|
| `eno1`, `enp2s0` | Both members of Linux bridge `br0`; no individual IPv4 address |
| `br0` | Robot subnet `192.168.131.1/24`; extra lab address `192.168.123.8/32` |
| MCU | `192.168.131.2`; CAN-over-UDP bridge port 11412 |
| Front / rear Hokuyo UST-20LX | `192.168.131.21` / `.22`; TCP port 10940 |
| `wlp3s0` | Lab Wi-Fi `192.168.123.14/24`, gateway `192.168.123.2`, metric 50 |
| Wired default route | Via `192.168.123.2` on `br0`, on-link, metric 100 |
| `tailscale0` | Tailnet access; observed address is not a recovery guarantee |

Inspect `ip -br address`, `ip route`, `bridge link` and `nmcli connection show`
at the physical console. Network interface names can change after reinstalling.
Review Clearpath-generated netplan and remove conflicting declarations before
using the [Ethernet template](../../deployment/r100_0160/templates/50-ridgeback-ethernet.yaml).
It reconstructs the observed addressing; it is not an exact export of the
installed root-private netplan. Keep Wi-Fi managed by NetworkManager, separately
from the Ethernet bridge. The extra lab address and wired gateway require that
lab subnet actually be available on Ethernet; omit them if the topology differs.

After installing the reviewed template as root mode 0600, use `sudo netplan
generate`, then `sudo netplan try` from the console. Verify MCU/LiDAR routes
before accepting. Configure Wi-Fi interactively with `nmcli --ask device wifi
connect '<SSID>' ifname wlp3s0 name ridgeback-wifi`; then modify that connection's
IPv4 method/address/gateway and route metric to the values above if still valid.
Use local prompts for the password. Restore the captured dual-homing sysctl
file to limit ARP flux when wired and wireless links share a lab subnet.

Install/enrol [Tailscale on Linux](https://tailscale.com/docs/install/linux),
run `sudo tailscale up` interactively and check `tailscale status` and the new
address. Recreate SSH authorised public keys and verify the active SSH socket.
Tailscale provides a route to the robot; TigerVNC supplies the desktop and
websockify exposes that desktop to the browser. A vendor webserver restart can
close the browser desktop connection even while Tailscale and Linux keep running.

## 3. Restore robot declarations, DDS and generated Clearpath state

From the repository root, an individual captured file can be installed with
its recorded mode and owner. For example:

```bash
sudo install -D -o robot -g robot -m 0644 \
  deployment/r100_0160/files/etc/clearpath/robot.yaml /etc/clearpath/robot.yaml
sudo install -D -o root -g root -m 0644 \
  deployment/r100_0160/files/etc/clearpath/cyclonedds.xml /etc/clearpath/cyclonedds.xml
```

Do not substitute repository `clearpath/robot.yaml`: it owns the simulator
identity and is not this physical deployment. Keep both LiDAR declarations and
merger offsets consistent with the [geometry owner](../robot/geometry.md#deployed-hardware-mounts).
Observed front/rear positions are `x=+0.3922/-0.3922`, `z=0.1856` metres; rear
yaw is pi. The merger's old comment mentions `-0.4278`; its effective parameter
is `-0.3922`. Preserve the parameter, not that stale comment.

Install `60-dynamo-dds-buffers.conf`; it raises `net.core.rmem_max` to 16777216.
Install the corresponding middleware service drop-ins from the manifest and
run `sudo systemctl daemon-reload`. Cyclone services use
`CYCLONEDDS_URI=file:///etc/clearpath/cyclonedds.xml`, `ROS_DOMAIN_ID=0`,
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. The installed XML enables only SPDP
multicast and selects `br0` and `wlp3s0`. Its outdated comment says Wi-Fi uses
DHCP; the observed connection uses a static address. Follow the
[transport reference](intel_thor_transport.md) for maintained profiles.

Clearpath's `/usr/sbin/clearpath-robot-generate` reads `/etc/clearpath/robot.yaml`
and regenerates setup, descriptions, parameters and launches. Ensure the
installer's `/etc/clearpath` directory is writable by the generator's `robot`
user; inspection found root-owned mode 0777, generated files owned by `robot`.
On a fresh installation, preserve working generator access rather than creating
a root-only directory. Once packages,
permissions and declarations are ready, run it as the `robot` user with
`HOME=/home/robot`, then inspect generated outputs before starting services.
The packaged `clearpath-robot.service` also runs it as `ExecStartPre`.
Do not use a stale backup of generated `/etc/clearpath/platform/` or
`/etc/clearpath/sensors/` as the declaration source. Do not use the old
`dynamo-rmw-backup` or legacy DDS apply/rollback to recover newer geometry.

The scan-merger launch deliberately sets its merger and converter child
processes to **Fast DDS**, even though the service launcher uses Cyclone.
Preserve that exception and both local respawn settings. Domain and topics
still match the robot. Verify descendant environments after starts; inspecting
only the unit's RMW setting misses the launch override.

## 4. Rebuild the hardware and project workspaces

Use [the public reconstruction recipe](ridgeback_public_recovery.md) to verify
captured inputs, assemble the pinned scan merger and host-file candidates, then
build the hardware overlay at `/opt/dynamo-hardware` on the replacement host.
It sources that overlay in a merger unit drop-in and removes the camera wrapper's
optional vendor dependency. The dashboard and its camera-readiness service are
excluded; packaged noVNC supplies the independent desktop's browser assets.
Preparation never installs files or starts services on the current host.

Optional vendor source identities, unresolved imports and private inputs are
recorded in the same recipe. Rebuilding the vendor dashboard is not required
for the selected hardware/desktop recovery scope.

Clone this project into `/home/robot/DyNAMO-Ridgeback`, follow the root README's
pinned core import/patch, rosdep and build instructions, and verify dependencies
before building. Simulation, perception model downloads and Thor installation
are separate workflows; they are not prerequisites for restoring the robot's
stationary sensors. Keep Thor unchanged until a deployment needs it.

## 5. Restore local integration selectively

Use the manifest to restore the selected `/usr/local/` scripts, custom units
and drop-ins, `udev`, Bluetooth and power configs to their original paths and
permissions. Do not bulk-copy all of `files/` to `/`: some inputs are reference
code that is disabled, pending replacement or known to fail at boot.

| Integration | Required state and reconstruction detail |
|---|---|
| USB / camera | Restore RealSense udev rule as a real file; captured origin was a symlink into `~/librealsense`. Preserve existing GRUB options and add `usbcore.usbfs_memory_mb=2048`, run `update-grub`, verify after reboot |
| PS4 controller | Restore `ds4drv.conf`, Bluetooth `disable_ertm=1`, PS4 udev rule, pair/trust actual controller and recreate `controllers.txt`; verify `/dev/input/ps4` |
| Joystick combiner | Restore `joy_x_combiner.py`, its unit, and platform overrides routing twist_mux to `joy_teleop/cmd_vel_combined` |
| Controller autoconnect | It can restart platform and combiner on connection; keep stopped until deliberate controller/base commissioning |
| Persistent fixes | Root boot script edits vendor WebRTC, webserver, SLAM and generated joystick config; review its paths and effects before enabling, especially after camera migration |
| CAN bridge | Current packaged bridge backgrounds `socat`/`slcand`; custom supervised scripts exist but no active override or watchdog timer was found. Do not claim them installed or enable them incidentally |
| Base startup recovery | `ridgeback-startup-recover.service` is disabled; script calls robot reset/enable and can restart base services. Keep disabled during sensor reconstruction |
| Battery | `test_power` module, sysfs bridge, power logger and guard; retain `ACTION=warn`, not automatic shutdown |
| Sleep | Restore captured logind no-suspend override; observed `sleep`, `suspend`, `hibernate`, `hybrid-sleep` targets masked. Recreate masks on the robot host to avoid idle suspend losing network/CAN |
| Navigation | `robot-slam` and `robot-map-navi` disabled/inactive; `robot-gps-navi` is a no-op placeholder. Do not enable them as part of SSD recovery |

The battery bridge exports a Fast DDS profile path but sources the Cyclone
Clearpath environment; that export does not prove it runs Fast DDS. The power
logger creates files in `/home/robot/power_blackbox/`; old recordings are not
required for it to start. Preserve the guard's warn-only action and document
any later threshold/action changes separately.

## 6. Camera and desktop service ownership

Since 2026-10-06 the camera owner is `dynamo-camera.service`, installed by
`sudo tools/intel_thor/camera_service apply --profile 640x480`. The unit runs
the apt `realsense2_camera_node` (4.58.1) directly with parameters rendered into
`/etc/clearpath/dynamo-camera/`. It runs no workspace code, so rebuilding or
switching this checkout does not change the camera at the next boot;
`/etc/clearpath/dynamo-camera/install.json` records the profile, serial and
the repository revision it was rendered from. It streams colour and
colour-aligned depth only at the installed profile (640×480×30), with both
infrared streams and the point cloud off and the D455 pinned by its camera
serial 338522302134. The USB descriptor serial in sysfs (346543063318) is a
different number and does not select the camera. Topics are
`/r100_0160/sensors/camera_0/color/image`, `…/color/camera_info` and
`…/depth/image` (aligned, `16UC1`), with the driver's default image QoS. The
robot-local `robot.yaml` declares `camera_0` as description only, so the
Clearpath description owns `base_link → camera_0_link` and the driver owns the
factory extrinsics below it.

`apply` disabled `realsense-camera`, `depth-to-mono8`, `ridgeback-camera-mjpeg`
and `camera-web-ready`, and with them the vendor web camera view; the new unit
also `Conflicts=` with `realsense-camera`. Do not re-enable them while the
service is installed. `tools/intel_thor/camera_service status` reports drift,
and `rollback` restores the vendor state it snapshotted. After a
reconstruction, re-apply the DDS services configuration first and the camera
service second. `docs/physical/robot_local_deployment.md` (added with the camera
service) owns its files, re-apply order and the contract diagnostics should
target. Re-capture `deployment/r100_0160` to include `/etc/clearpath/dynamo-camera/`,
`/etc/systemd/system/dynamo-camera.service` and the `camera_0` entry.

`run_local_dds_test.py` still requires the old vendor units and topics: adapt it
to the contract above before using it. Its `d455_serial()` reads the USB
descriptor serial, which would not select the camera in its aligned stage. The
combined hardware repair runner is staged for the vendor stack and must not be
run while the camera service is installed.

Desktop baseline has three competing pieces: `ridgeback-vnc` creates display
`:1` on 5901; a websockify proxy uses 9005; `ridgeback-remote-screen` serves 9100
and can also attempt to create the same display. The VNC display currently has
its own unit, but the active 9005 proxy can belong to `mbs-webserver`. Restarting
that vendor unit can close browser access. The old VNC startup also showed
NVIDIA GLX/user-runtime failures; the independent proxy unit was enabled but
inactive at inspection. These are documented faults, not a known-good target.

For a fresh reconstruction use the desktop candidates and packaged noVNC in
[the public recipe](ridgeback_public_recovery.md#independent-desktop-and-fresh-credentials),
then recreate credentials with `vncpasswd` as `robot`. For diagnosis of the
observed host and its staged foreground/Mesa/private D-Bus repair, follow [desktop troubleshooting](../troubleshooting.md#remote-desktop-startup-and-proxy-ownership)
and the owning [backlog](../BACKLOG.md#remote-desktop-startup-recovery).
It has not passed a repaired cold boot on this host. Verify display and proxy
ownership separately; do not assume that an open port proves the intended unit
owns it. Tailscale remains the transport whichever desktop owner is chosen.

## 7. Start and verify, without driving

With the robot stationary and an operator present, start reviewed services in
stages: network/access, Clearpath generation and sensors, merger, selected
camera owner, display consumers, then battery/desktop integration. Base and
controller commissioning is a deliberate later step; do not issue navigation,
velocity, reset or enable commands during sensor recovery. Inspect unit Wants
and boot enablement first: enabling a unit is different from starting it, and
a dependency can pull in a disabled unit.

The [captured service state](../../deployment/r100_0160/service-state.txt)
records the baseline. `clearpath-robot/platform/sensors/vcan`, merger, camera,
combiner and battery services were active/enabled; `mbs-webserver` was
active/disabled; camera readiness and independent 9005 proxy were
inactive/enabled. Discovery/manipulator/extras/Zenoh units were enabled but
inactive. Preserve these distinctions when diagnosing a reconstructed boot.

Use read-only checks first:

```bash
source /opt/ros/jazzy/setup.bash
source /etc/clearpath/setup.bash
ip -br address
ip route
systemctl --failed --no-pager
systemctl status clearpath-sensors clearpath-scan-merger --no-pager
ros2 topic list
ros2 topic info /r100_0160/sensors/lidar2d_0/scan --verbose
ros2 topic info /r100_0160/sensors/lidar2d_1/scan --verbose
ros2 topic info /r100_0160/sensors/scan --verbose
```

Use the [stationary DDS measurement workflow](intel_dds_testing.md) for guarded
camera subscribers, exact image timestamps, scan continuity and port traffic.
Its current-profile mode avoids DDS reloads, but still expects the captured
vendor camera contract, so it needs adapting to `dynamo-camera.service` first
(section 6). For the installed camera itself, `source /etc/clearpath/setup.bash`
and run `tools/camera_contract_check --backend hardware`, which checks images,
CameraInfo, aligned-depth pairing, the optical TF path and scan continuity
together. A healthy UST-20LX publishes about 40 Hz; the merger's `scan_time`
metadata is not a substitute for measuring received rate. Check calibrated
front/rear geometry in both declaration and merger, service descendants' DDS
settings, USB enumeration and serial, packet evidence and absence of restarts.

Finally, qualify a cold boot with an operator: reachability, fresh desktop
session, intended proxy cgroups, no competing display launch, sensors at expected
rates, and no unexpected readiness/webserver restarts. Record changed file
hashes and results under `artifacts/hardware/`. A recreated desktop or a passing
sensor test alone does not qualify motion, TF/localization, aligned depth,
LiDAR timeout recovery or Thor deployment.

## Keeping the reconstruction record current

Every host setup change must update this runbook and its simple deployment
inputs in the same repository change. Capture installed bytes, origin, mode,
owner and SHA after deployment; label templates and pending work explicitly.
Document what generated the files, how services are enabled, how to verify and
how to undo the change. Keep secrets local and name the fresh-entry step.
The [documentation policy](../project/documentation.md#host-setup-and-reconstruction)
owns this requirement. This task added no backup job and changed no live services.
