# r100_0160 host reconstruction inputs

The [Ridgeback recovery runbook](../../docs/physical/ridgeback_recovery.md)
owns the reconstruction order and the meaning of these files. Captured from
`cpr-r100-0160` on 2026-10-06; this is the observed vendor-camera deployment,
including known desktop startup faults. It has not been restored onto a blank SSD.

- `files/` mirrors the absolute paths of 99 curated small configuration files,
  service units, drop-ins and local scripts. These are readable deployment inputs,
  not a filesystem backup or an instruction to install every file.
- `manifest.json` records each origin, content SHA-256, original owner/group,
  original mode and any symlink target. Repository copies have mode 0644;
  restore executable bits from the manifest only for files selected by the runbook.
- `packages.tsv` records installed versions of relevant ROS, camera, graphics,
  networking, Python and desktop packages. It is an inventory, not an apt lockfile:
  archives may no longer serve every recorded version.
- `service-state.txt` records enabled/disabled and active/inactive separately.
  An active disabled service can have been pulled in by another unit.
- `templates/50-ridgeback-ethernet.yaml` is a credential-free reconstruction
  template based on observed links/routes. The installed netplan was not copied;
  the template still needs local-console validation and reconciliation with
  Clearpath-generated networking.

The selected recovery scope excludes the vendor dashboard. The
[public reconstruction recipe](../../docs/physical/ridgeback_public_recovery.md)
owns source preparation, selected host candidates and fresh-host build commands.

The captured joystick/base files describe the original uncapped baseline.
The current [commissioning caps](../../docs/physical/commissioning_guard.md#speed-modes)
are maintained in `tools/hardware/commissioning_speed_guard.py` and rendered by
`tools/hardware/install_commissioning_caps.py`. Recovery preparation includes
these newer candidates with explicit provenance rather than rewriting the
baseline snapshots.
`reconstruction/` contains the pinned public manifest, matched scan-merger hashes
and a metadata-only vendor source inventory. The captured baseline remains intact. A narrow `.gitignore` exception includes
the curated installed webserver config; it does not track normal build output.
`.gitattributes` preserves original line endings and whitespace in `files/`;
manifest checksums validate those bytes instead of reformatting deployed inputs.

## Read the important inputs

| Setup | Captured files |
|---|---|
| Identity / DDS | [robot.yaml](files/etc/clearpath/robot.yaml), [Cyclone XML](files/etc/clearpath/cyclonedds.xml) |
| Permissions / provenance | [Manifest](manifest.json), [service state](service-state.txt), [package inventory](packages.tsv) |
| Ethernet / Wi-Fi coexistence | [Ethernet template](templates/50-ridgeback-ethernet.yaml), [ARP/routing sysctls](files/etc/sysctl.d/99-ridgeback-dual-homing.conf) |
| Camera startup | [Camera unit](files/etc/systemd/system/realsense-camera.service), [startup script](files/usr/local/sbin/realsense-camera-start) |
| Merged LiDAR | [Launch](files/opt/mybotshop/src/third_party/tools/ros2_laser_scan_merger/launch/merge_ridgeback.launch.py), [parameters](files/opt/mybotshop/src/third_party/tools/ros2_laser_scan_merger/config/params_ridgeback.yaml) |
| Boot-time edits | [Persistent fixes](files/usr/local/sbin/ridgeback-persistent-fixes), [platform overrides](files/etc/systemd/system/clearpath-platform.service.d/override.conf) |
| Desktop | [VNC unit](files/etc/systemd/system/ridgeback-vnc.service), [session startup](files/home/robot/.vnc/xstartup), [browser proxy](files/etc/systemd/system/ridgeback-vnc-websockify.service), [remote-screen script](files/home/robot/ridgeback-remote-screen/start-remote-screen.sh) |
| Controller | [Driver config](files/etc/ds4drv.conf), [device rules](files/etc/udev/rules.d/99-ridgeback-ps4.rules), [autoconnect](files/usr/local/bin/controller-autoconnect.sh), [combiner](files/opt/mybotshop/scripts/joy_x_combiner.py) |
| Battery / sleep | [Warn-only guard config](files/etc/battery-guard.conf), [guard](files/usr/local/bin/battery_guard.py), [sysfs bridge](files/usr/local/bin/battery_sysfs_bridge.py), [logind](files/etc/systemd/logind.conf.d/10-ridgeback-no-suspend.conf) |
| Vendor web config | [Source](files/opt/mybotshop/src/mybotshop/robot_webserver/config/robot_webserver.yaml), [installed overrides](files/opt/mybotshop/install/robot_webserver/share/robot_webserver/config/robot_webserver.yaml) |

## Credentials and source ownership

Both source and installed webserver configs' `robot_password` is deliberately replaced by
`SET_LOCALLY_DURING_RECOVERY`. Enter a fresh value locally before running that
service. Wi-Fi credentials, private keys, VNC passwords, Tailscale identity,
Bluetooth pairing keys and vendor `robot_info.yaml` credentials are not included.
The controller MAC list is an identifier list, not a pairing-key backup.

DDS XML also has a maintained source in
`src/ridgeback_autonomy_hardware/config/intel_thor/`; geometry is owned by
`docs/robot/geometry.md`. These copies identify what was actually installed.
When deploying a change, update its maintained source first and refresh the
corresponding deployment copy and manifest after checking the host. Do not
silently edit a copy and claim it was deployed.

`mbs-vcan-bridge` and `mbs-vcan-watchdog` exist as files but are **not wired into
the captured live services**. `ridgeback-startup-recover` is disabled and can
reset/enable the base; preserve that disabled state. Navigation units remain
disabled. Historical camera helpers remain captured but are not selected when camera.json
is present. The foreground/Mesa desktop and non-restarting autoconnect repair
was installed on 2026-10-12; refreshed entries carry their own capture dates
and reference the maintained stationary setup renderer.

The installed `robot_webserver.yaml` differs from its source copy; both are
preserved with password placeholders. Reconcile the installed overrides into
source before a rebuild, rather than overwriting them with the old source.

The whitelist helper was available only under `handover_ws/src/`, whereas the
main vendor launch imports it from its installed package share. Reconstruct a
complete vendor source tree and include/install that helper before building;
blindly restoring the main launch alone leaves an unresolved import.

## Verify the copies

Verify the copied bytes without touching the robot:

```bash
python3 tools/hardware/verify_recovery.py
```

This checks coverage, safe paths, hashes, syntax and credential placeholders.
It reports the captured navigation file's identical duplicate key as a warning;
that navigation file is not selected for the public recovery.


The 2026-10-12 camera additions are labelled snapshots of the installed
`dynamo-camera.service` and its parameter/DDS/metadata files. They add four
manifest entries to the original 81. `camera.json` is the explicit serial/profile
input for regenerating current camera setup from the restored maintained tool,
not a historical install receipt. Recovery preparation selects the direct camera
service and the maintained desktop/Bluetooth repair; those desktop/controller
repair was installed and independently verified stationary on 2026-10-12.
The manifest now has 89 entries: original entries are retained or explicitly
refreshed, and four new stationary-setup paths are captured. The exact prior
files/activity remain in the local installer receipt for rollback; cold boot
and physical input/reconnect qualification are still open.


Seven auxiliary-repair files were installed and independently verified on
2026-10-13, bringing the manifest at that time to 96 entries. Their per-entry capture dates
and hashes identify installed state; `install_auxiliary_repairs.py` regenerates
them from the verified target package/parameters and owns narrow rollback.
These snapshots do not establish IMU calibration, new battery sensing or the
clock gate's next-boot behavior. See the recovery runbook for installation order.


The October 13 chrony snapshots preserve Intel's main configuration and the
installed Ubuntu pool profile (four sources), including subnet serving and the
500 ppm slew policy. Source expansion requires no chrony restart; its maintained
installer and exact rollback are `tools/hardware/expand_clock_sources.py`.
Reconstruction needs the `chrony` package, these root-owned 0644 files and an
enabled `chrony.service`, followed by `tools/hardware/check_clock_ready.py`.
The expanded profile and poll bound passed a local convergence check on
2026-10-13; reboot and source-loss behavior remain unqualified;
see [auxiliary checks](../../docs/physical/stationary_auxiliary_checks.md#intel-clock-and-boot-policy).


On 2026-10-13 the Intel pool profile was updated to `maxpoll 7` so its
128-second source interval fits the platform's 180-second reference-age gate.
Reconstruct the installed bytes from the labelled snapshot above, or apply the
maintained `tools/hardware/bound_clock_poll.py` to a matching pre-change pool
profile. The change does not restart chrony or alter its slew policy. Initial
runtime poll values can remain high until a fresh measurement; confirm source
poll levels and the read-only clock gate after reconstruction.

The 2026-10-13 installed Puma feedback plugin adds the captured platform
`95-dynamo-puma-feedback.conf` drop-in, bringing this manifest to 99 entries.
That file selects a generated, package-hash-named prefix under
`/opt/dynamo-puma-feedback/`. It cannot start the platform by itself on a new
SSD: first rebuild the plugin from the pinned Clearpath source and maintained
patch with `tools/hardware/build_puma_feedback_candidate.py`, stage it with
`tools/hardware/stage_puma_plugin_prefix.py`, and use the guarded
`tools/hardware/install_puma_feedback_plugin.py` workflow from the root README.
The local stage-5 receipt records the installed package manifest and mapped
library path; it is experiment evidence, not a substitute for the maintained
builder. The live check verified only stationary startup with the physical
e-stop engaged.
