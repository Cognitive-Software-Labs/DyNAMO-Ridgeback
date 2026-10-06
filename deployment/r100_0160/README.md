# r100_0160 host reconstruction inputs

The [Ridgeback recovery runbook](../../docs/physical/ridgeback_recovery.md)
owns the reconstruction order and the meaning of these files. Captured from
`cpr-r100-0160` on 2026-10-06; this is the observed vendor-camera deployment,
including known desktop startup faults. It has not been restored onto a blank SSD.

- `files/` mirrors the absolute paths of 81 curated small configuration files,
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
disabled. Old camera helpers must be reviewed when the replacement camera
service lands. The foreground/Mesa desktop repair in `tools/intel_thor/` is
staged separately and is not represented as installed here.

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
