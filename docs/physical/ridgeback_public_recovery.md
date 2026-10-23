# Ridgeback recovery without the vendor dashboard

This procedure reconstructs ROS hardware support and an independent Linux
remote desktop. The MyBotShop dashboard is optional, as selected on 2026-10-06.
It does not change the currently running robot. The
[host recovery runbook](ridgeback_recovery.md) owns OS, account, networking,
Clearpath generation, DDS, device permissions and commissioning boundaries.

“Hardware” here means the host services for the Clearpath base interface,
Hokuyo LiDARs, D455 camera, DDS, controller and battery monitoring. “Desktop”
means the Linux graphical session served by TigerVNC and websockify, reachable
through Tailscale or the robot network. Neither requires the vendor dashboard's
camera pages, browser robot controls or webserver packages.

## What can be recreated

| Part | Inputs | Verification scope |
|---|---|---|
| Clearpath base and sensor services | Official Jazzy packages and captured robot declaration | Existing deployment observed; fresh host generation and commissioning still required |
| Scan merger | Public Git pin, byte-matched deployed source, captured launch/config | Isolated Colcon build passed on this host; no recovered service was launched |
| D455 native streams | Apt ROS driver and captured wrapper adapted to source Clearpath setup | Candidate preserves native stream flags; serial/alignment migration and runtime checks remain separate |
| Controller and power helpers | Curated local scripts/configs and packaged dependencies | Input integrity/syntax verified; controller re-pairing and commissioning still required |
| Desktop | Maintained foreground/Mesa/private D-Bus renderer; packaged TigerVNC, websockify, noVNC | Prepared candidates only; fresh boot/browser qualification still required |
| Vendor dashboard | Private vendor source and credential-bearing config | Optional; not reconstructed by this procedure |

The scan merger's build-relevant deployed files match upstream revision
`4110a4831c0976c018c7cdb6625f8a10ee4974fa` exactly. The
[source lock](../../deployment/r100_0160/reconstruction/scan_merger.lock.json)
records hashes; the
[vcstool manifest](../../deployment/r100_0160/reconstruction/hardware.repos)
pins that revision. A current upstream branch is different and is not an
interchangeable reconstruction input. The copied local launch keeps the Fast
DDS worker override and both child respawn settings. Its geometry config keeps
the corrected rear x = −0.3922 m.

## Verify and prepare, without installing

From this repository root, install/use PyYAML and run:

```bash
python3 tools/hardware/verify_recovery.py
```

The verifier checks manifest coverage, hashes, path boundaries, owner/mode
metadata, Python/shell/XML/YAML syntax and redacted placeholders. It never
executes captured programs or contacts ROS. The snapshot's `nav2_map.yaml`
repeats `set_initial_pose: true`; identical-value duplicates warn, conflicting
values fail, and credential templates reject all duplicates. That navigation
file is not selected for this recovery. The verifier checks captured inputs,
not live deployment or availability of every apt version.

Prepare a clean public checkout and fresh candidate directory:

```bash
mkdir -p artifacts/hardware/reconstruction
git clone https://github.com/mich1342/ros2_laser_scan_merger.git \
  artifacts/hardware/reconstruction/scan-merger-upstream
git -C artifacts/hardware/reconstruction/scan-merger-upstream checkout --detach \
  4110a4831c0976c018c7cdb6625f8a10ee4974fa
python3 tools/hardware/prepare_recovery_workspace.py \
  --upstream artifacts/hardware/reconstruction/scan-merger-upstream \
  --output artifacts/hardware/reconstruction/prepared
```

Choose a new directory on subsequent attempts: the tool refuses existing output, dirty
source, a wrong pin, unsafe archive entries and failed input verification.
Outputs must be below repository `artifacts/` or `/tmp/`; it cannot prepare into
an installed host path.

The output contains source under `src/`, inert candidates under
`host-files/`, and `RECOVERY_PREPARATION.json` with destinations, required owner,
mode, hashes and provenance. Candidate files remain mode 0644 until deliberately
installed. Nothing under `/etc`, `/usr/local`, `/opt` or `/home/robot` is changed.
The source is exported from the pinned Git commit, then receives the captured
Ridgeback launch and parameters. There is no vendor clone or dashboard source.

## Build at the intended hardware prefix on a replacement host

Follow the parent runbook first. In addition to the Clearpath/RealSense packages,
install `ros-jazzy-pointcloud-to-laserscan`, `ros-jazzy-pcl-conversions`,
`libpcl-dev` and `libboost-system-dev`; use rosdep for the public merger source.
The restored host also needs Cyclone and Fast DDS RMW packages.

For an isolated check on the existing robot, keep every output under
`artifacts/hardware/` and use a sequential, low-priority build. Do not source
`/opt/mybotshop/install/setup.bash` into that build. On a replacement host only,
copy the prepared source into the final prefix and rebuild there:

```bash
sudo install -d -o robot -g robot -m 0755 /opt/dynamo-hardware
# As robot, after inspecting that the target is fresh:
cp -a artifacts/hardware/reconstruction/prepared/src /opt/dynamo-hardware/
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths /opt/dynamo-hardware/src --ignore-src -r -y \
  --rosdistro jazzy
cd /opt/dynamo-hardware
colcon build --executor sequential \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
```

Do not relocate built `install/` output: Colcon scripts encode prefixes. The
merger service candidate includes a drop-in sourcing
`/opt/dynamo-hardware/install/local_setup.bash` after Clearpath setup. Its main
unit still carries the original command as capture evidence, but the drop-in
replaces that command. Confirm the resulting effective `ExecStart` before use.
The historical `/opt/mybotshop/scripts/joy_x_combiner.py` path is retained for
compatibility; this is a standalone preserved script, not a vendor workspace.

Four additional candidates implement the maintained
[commissioning caps](driving_readiness.md#commissioning-caps): a pinned Python
guard, a launch wrapper, its unit and a platform drop-in. Their source is
`tools/hardware/install_commissioning_caps.py`, not the captured baseline.
Install them before qualifying the recovered base. The wrapper includes the
generated platform job and applies the controller remap without editing
generated Clearpath files. Keep the physical stop engaged until command routing
and the guarded motion checks pass; these candidates do not certify recovery.
The maintained candidate includes a reliable e-stop relay and a mux-only remap
for the native firmware/mux QoS mismatch. The
[stop-delivery reference](driving_readiness.md#e-stop-delivery-to-the-mux) owns
deployment status and verification. Keep the original firmware stop topic
for the guard and other consumers; do not apply a global e-stop-topic remap.

The official Puma 2.9.4 package has a confirmed diagnostic fault-decoding
defect. The [wheel-feedback reference](drive_feedback_diagnostics.md) owns its
maintained source patch and verification. That patch is prepared, not included
in the installed/candidate base package here; fresh-host qualification must
check raw fault masks and must not trust the affected human-readable flags.

## Install selected host candidates on the replacement host

Review `RECOVERY_PREPARATION.json`. Install only its listed files to their
listed destinations with recorded owner/mode, as root. Keep `/etc/clearpath`
writable by the Clearpath generator's `robot` user. The candidates deliberately
exclude vendor webserver/readiness units, navigation units, the reset/enable
startup helper, and the boot script that edits vendor configs. They include
platform joystick overrides independently of that boot script.

A single-file installation example, from the prepared directory:

```bash
sudo install -D -o root -g root -m 0644 \
  host-files/etc/systemd/system/clearpath-scan-merger.service.d/30-recovery-workspace.conf \
  /etc/systemd/system/clearpath-scan-merger.service.d/30-recovery-workspace.conf
```

No blanket restore or automatic service enablement is provided: the inspected
baseline has known boot faults and differing motion/service boundaries. Retain
sensor/desktop commissioning separately from base/controller commissioning.
In particular, `controller-autoconnect` can restart platform services when a
controller connects. Keep it stopped until deliberately checking that path.
`battery-guard` remains `ACTION=warn`; no shutdown action is added.

The camera wrapper sources `/etc/clearpath/setup.bash` instead of the optional
vendor overlay. It preserves the captured 640×480×30 native streams, infrared
flags and point cloud. That is a reconstruction of the captured baseline, not
approval of those flags for the planned camera migration. If the new camera
owner has landed, refresh the recovery inputs first and install that owner;
do not start both implementations.

## Independent desktop and fresh credentials

Install TigerVNC, websockify, `novnc`, `dbus-x11`, XFCE/Openbox and Mesa EGL/GL
packages. Check `/usr/share/glvnd/egl_vendor.d/50_mesa.json` exists. The desktop
candidates reuse the maintained renderer from
`tools/intel_thor/repair_hardware_workflow.py` and select only desktop files;
they do not invoke the vendor-dependent combined installer.

The unit expects `robot` UID 1000 and `/run/user/1000`; adjust **both** its
`user@1000.service` dependency and environment paths if the fresh account's UID
differs. It starts one foreground TigerVNC display, applies software graphics
to that virtual server, and starts XFCE under a private D-Bus session. Both
proxies are independent and retry if the backend fails. The remote-screen unit's
new drop-in removes its competing VNC launcher; no vendor camera gate is used.

Create fresh VNC credentials as `robot` with `vncpasswd`. Restore the selected
remote-screen HTML and link its noVNC dependency to the Ubuntu package:

```bash
# As robot, with the target directory already restored:
ln -s /usr/share/novnc /home/robot/ridgeback-remote-screen/noVNC1.3.0
```

Do not overwrite an existing vendor symlink on the running host. The packaged
noVNC core must be at `/usr/share/novnc/core/rfb.js`. Re-enrol Tailscale and
supply SSH public keys separately; credentials and pairing keys are never
included in the candidates. Existing listener addresses/ports are retained;
this procedure does not introduce a new access policy.

After installation, reload systemd and commission the reviewed units with an
operator. Check display 5901, proxy 9005 and remote-screen 9100 belong to their
intended unit cgroups, reconnect through the browser, then qualify a cold boot.
Do not infer reliability just because a candidate parses or a port listens.
The existing VNC faults remain open until that test passes.

## What was verified, and what remains

On 2026-10-06 the source preparation and 27 offline tests passed, and the pinned
public merger built successfully with ROS Jazzy in an isolated prefix. Installed
launch/config bytes matched the prepared copies; launch-description construction
was inspected without executing nodes. No ROS subscribers, base commands,
service restarts, desktop transitions or host-file installations were performed
for this reconstruction check.

The 2026-10-12 stationary audit rechecked the original captured-input manifest.
The missing camera tools/templates were recovered from the installed camera's
recorded Git revision and verified against installed state. The manifest now
contains labelled camera and stationary-setup snapshots;
`camera.json` selects serial/profile and the preparer regenerates the direct
camera service instead of the vendor camera wrapper. Camera setup ownership and
remaining application-topic reconciliation live in
[robot-local deployment](robot_local_deployment.md).

The preparer also uses the maintained stationary desktop/Bluetooth renderer.
Its autoconnect helper no longer restarts platform services. The six-file
desktop/Bluetooth repair was installed and independently verified stationary
on 2026-10-12; all protected hardware process IDs were unchanged. Its exact
receipt is under `artifacts/hardware/20261012-stationary-setup-install/`.
The deployment manifest now includes 89 files with per-entry capture dates;
maintained renderers remain authoritative. Cold boot and live reconnect
qualification remain open. Public stage,
apply and receipt-based rollback commands are in the root README.

A full recovery has not been demonstrated. On 2026-10-06 the user chose to
preserve the instructions and commit the inputs without performing a spare-SSD
restoration. No separate recovery rehearsal is scheduled. During an actual
future recovery, verify the fresh-host OS/package/network/generator setup,
selected service activation and cold boot. Current apt repositories
serve newer Clearpath and some ROS package builds than the inventory; exact
older deb availability is not promised. Treat a changed package set as a new
configuration to qualify. Camera serial/alignment, TF/localization, LiDAR
reconnect qualification and motion remain their own gates.

Optional vendor restoration requires an accessible source release with permission
to use it. The [vendor source inventory](../../deployment/r100_0160/reconstruction/vendor_sources.json)
records package identities, selected build-file hashes, handover artifact hashes
and external inputs without copying full proprietary sources or credentials.
The local bundle needs base commit
`ffa4d2457057e0ca0cb44cc2ddb1271258b34879`; it is not self-contained. Current
GitHub access returns 404, which does not distinguish missing repository from
missing permissions. If the optional dashboard is rebuilt, add the preserved
whitelist helper to `robot_webserver/launch/` and reconcile the captured installed
webserver overrides into source before rebuilding and verifying its imports.

These repository changes still require normal review/commit/publication to
survive loss of this checkout. The project remote is public: do not publish
private vendor archives, sources, credentials or keys as part of that step.

The commissioning guard additionally has an optional temporary test lock,
selected by the caps installer’s `--test-interlock`. Do not enable it for normal
manual operation: it blocks all motion without a fresh bounded test permit.
Its source, installation and exact receipt-based rollback are maintained in the
[root operator workflow](../../README.md) and
[test-lock contract](driving_readiness.md#temporary-commissioning-test-interlock).
Recovery defaults to the normal capped guard. The temporary mode was installed
and verified stationary on 2026-10-09; its receipt is under
`artifacts/hardware/20261009-test-interlock-install/`. Recreate that temporary
state explicitly with `--test-interlock` only when resuming commissioning tests.

The optional pinned Puma repair is reconstructed by the maintained
`tools/hardware/build_puma_repair.py` and `install_puma_repair.py`, using the
official 2.9.4 source archive and tracked fault/cache patches. Follow the
[root build/install/rollback workflow](../../README.md). The installer recreates
a root-controlled package prefix plus DDS interface-readiness and scoped platform
overlay drop-ins, preserving the exact normal/test-interlocked capped guard.
The corrected repair was installed and verified stationary on 2026-10-12; its
receipt is under `artifacts/hardware/20261012-puma-repair-install/`. The older
October 9 compiled candidate was unpatched and is withdrawn. Do not reuse artifacts as the recovery
configuration authority: rebuild, verify actual applied patches and dependencies,
and create a fresh deployment receipt. Cold boot and per-wheel freshness remain
unqualified even after a successful stationary installation.
