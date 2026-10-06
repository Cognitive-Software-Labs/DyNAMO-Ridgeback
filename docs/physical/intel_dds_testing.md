# Intel local DDS comparison

This procedure tests local camera delivery, physical-port traffic, and LiDAR
continuity on `r100_0160`. It compares the installed services configuration
with discovery-only multicast. The [transport reference](intel_thor_transport.md)
owns the deployment contract and the historical failure reports; those reports
are hypotheses and prior observations, not acceptance of the current robot.

## Run the guarded comparison

Keep the robot stationary, with an operator present and the e-stop available.
Stop remote camera streaming for the local comparison. Leave the existing vendor
consumers running and do not start navigation or base commands. The runner uses
the live robot graph in domain 0, so it is an explicitly requested diagnostic,
not an unattended startup check.

The comparison runner requires an unrestricted installed A profile. After B is
retained, these comparison commands intentionally refuse the existing
`AllowMulticast` element before changing configuration. Do not roll back the
passing profile just to repeat a check: a dedicated current-profile test mode
is available with `--verify-current`:

```bash
sudo python3 tools/intel_thor/run_local_dds_test.py \
  --operator-confirmed --detach --verify-current \
  --run-dir "$PWD/artifacts/hardware/$(date -u +%Y%m%dT%H%M%SZ)-intel-dds-current"
```

It measures a quiet sixty-second baseline, two independent native color/depth
reader pairs with ten seconds of warm-up and sixty seconds measured, then checks
all three scans again. Allow about three minutes. It uses a byte-identical copy
of the installed profile and performs no service reload or configuration write.
Changed deployment files, service activity, worker identities or failed camera,
scan and traffic gates fail verification. It stops only its own observers and
captures; it reports external changes rather than repairing them. It neither
compares A/B nor qualifies alignment or a soak. Discovery alone cannot qualify
image delivery without received images.

For a short first diagnostic, choose a new evidence directory from the Intel
checkout:

```bash
sudo python3 tools/intel_thor/run_local_dds_test.py \
  --operator-confirmed --detach --screening \
  --run-dir "$PWD/artifacts/hardware/$(date -u +%Y%m%dT%H%M%SZ)-intel-dds-ab"
```

Screening uses a sixty-second scan-only baseline and five-second trials for
each reader combination under both configurations. It skips repeated captures,
alignment, and the soak, and always restores the original DDS settings. Allow
roughly five minutes including snapshots and service reloads; a failed gate
ends that configuration's matrix early. Rates and gaps are saved, but a short
trial passes only receipt and immediate safety checks. It cannot qualify
sustained camera delivery or justify retaining the candidate.

Omit `--screening` for the complete repeated comparison, aligned-depth tests,
and thirty-minute soak. A full passing run takes about sixty-five minutes.

After screening, use `--focused-native` instead of `--screening` for one warmed
B capture: sixty-second scan baseline, then two independent native color/depth
reader pairs with ten seconds of warm-up and sixty seconds measured. All camera,
scan and traffic acceptance gates apply; scan safety also covers warm-up. It
restores the original DDS configuration by default and skips the full matrix,
alignment and soak. Allow roughly four minutes including reloads. The two short
modes are mutually exclusive.

To repeat the focused test and keep its passing discovery-only multicast
setting, explicitly add `--retain-passing-native`:

```bash
sudo python3 tools/intel_thor/run_local_dds_test.py \
  --operator-confirmed --detach --focused-native --retain-passing-native \
  --run-dir "$PWD/artifacts/hardware/$(date -u +%Y%m%dT%H%M%SZ)-intel-dds-retain"
```

This opt-in retains B only if the scan baseline, warmed camera capture, and
final configuration/service/scan verification pass. Failure or interruption
restores the original DDS settings. Successful completion records
`candidate-retained` in `CURRENT_CONFIG.json`; the saved `--recover` command
still restores the exact original profile. Retention after this focused test
does not qualify alignment, a soak, distributed traffic, or motion. Camera
startup and the merger's Fast DDS override remain unchanged. New Intel
application processes using `dds_env.sh` select an Ethernet-only profile with
the same `AllowMulticast=spdp` restriction; restart already-running applications
to load changed configuration. Thor's profile is unchanged.

For image-delay diagnosis, the same focused run now saves middleware timestamps
for each received image and `image-timing.json` per reader. No additional tracing
flag or camera driver modification is required. For example:

```bash
sudo python3 tools/intel_thor/run_local_dds_test.py \
  --operator-confirmed --detach --focused-native \
  --run-dir "$PWD/artifacts/hardware/$(date -u +%Y%m%dT%H%M%SZ)-intel-image-timing"
```

The analysis splits consecutive receive gaps into frame-header spacing, changes
before publication, changes between publication and middleware take, and changes
between take and Python callback. In
[Cyclone RMW 2.2.3](https://github.com/ros2/rmw_cyclonedds/blob/2.2.3/rmw_cyclonedds_cpp/src/rmw_node.cpp),
the source timestamp is set before `dds_write_ts`; the received timestamp is set
when taking a sample. It is not packet arrival time. Publication-to-take combines
middleware write, transport, queueing and executor scheduling, so a delay there
does not establish network loss. Header stamps alone cannot establish when USB
acquisition happened. Missing, unordered or discontinuous timing evidence is
marked invalid. Analysis covers received frames and excludes warm-up.

`--detach` starts an independent systemd service and prints its name, journal
command, and stop-and-restore command. Closing the journal viewer does not stop
the experiment. Use the printed `systemctl kill --kill-whom=main --signal=SIGINT`
command to request cleanup.

The vendor launch can own VNC and the graphical desktop. When they share its
systemd control group, reloading it closes that desktop and its terminals.
Reconnect after the service starts; the detached controller continues saving
evidence. To separate desktop ownership before more trials, run:

```bash
sudo python3 tools/intel_thor/install_independent_vnc.py \
  --operator-confirmed --detach \
  --run-dir "$PWD/artifacts/hardware/$(date -u +%Y%m%dT%H%M%SZ)-independent-vnc"
```

This causes one desktop reconnect during the transition. It installs independent
VNC and port-9005 proxy services, retaining the existing password, display,
resolution and ports. The vendor launch already skips these listeners when they
exist. The installer verifies process identities across a webserver restart and
restores vendor ownership if installation fails. DDS, robot YAML and camera
startup bytes are preserved. Use the same command with `--recover` and its saved
run directory to remove this installation. Recovery also needs `--detach` from
VNC because restoring vendor ownership closes that desktop.

The October 6 boot audit found VNC startup retries and an inactive independent
proxy; the vendor webserver had taken over port 9005. Runtime restart verification
does not qualify cold boot. Check both unit state and actual listener ownership
before assuming the browser connection survives webserver reloads. See
[desktop startup troubleshooting](../troubleshooting.md#remote-desktop-startup-and-proxy-ownership).

For the already-installed Intel desktop, use the maintained repair rather than
rerunning the initial installer:

```bash
sudo python3 tools/intel_thor/repair_hardware_workflow.py \
  --operator-confirmed --detach \
  --run-dir "$PWD/artifacts/hardware/$(date -u +%Y%m%dT%H%M%SZ)-workflow-repair" \
  --then-verify-current "$PWD/artifacts/hardware/$(date -u +%Y%m%dT%H%M%SZ)-dds-current"
```

The repair stages exact candidates and checks prerequisites before stopping the
desktop. It gives display :1 one foreground VNC owner, selects Mesa for the
virtual X server, starts the real robot user runtime, and uses a private desktop
session bus. Application GPU selection is preserved. The existing port-9100
service becomes a proxy only; port 9005 remains independently supervised. Both
proxies retry without stopping when VNC fails. Vendor startup waits for listeners
in the expected services before its port guards run.

Camera-web readiness uses direct Image publisher discovery in the deployed
namespace/domain and exact entries in the local web `/topics` response. It uses
a bounded deadline, subscribes to no images, and restarts no services. Readiness
does not prove received frames or browser streaming.

Installation causes one desktop reconnect. The detached worker verifies both
proxy owners and a valid user runtime, then checks listener identities across a
vendor webserver restart and checks camera readiness again. It preserves DDS,
robot YAML, camera startup, original desktop scripts and merger settings. Failure
restores the exact managed files and original service activity. Use `--recover`
with the saved repair directory to undo the repair; this also requires `--detach`
from VNC. Recovery refuses external edits and paths outside its managed list.
Cold boot and actual browser continuity still require live verification.

Optional `--then-focused-run "$PWD/artifacts/hardware/NEW-focused"` launches the
focused test only after desktop separation verifies successfully. The installer
and DDS runner share a lock so ownership changes cannot overlap an experiment.

A direct
foreground run inside any service that the experiment reloads is refused before
configuration changes. An independent SSH terminal is also supported. `nohup`
and `tmux` inside the vendor service do not escape its systemd control group.

The runner needs root to capture packet headers and reload services. Sensor
observers run as the robot account. It saves deployed configuration and service
state before changes, then runs the current configuration and a candidate with
`AllowMulticast=spdp`. Interfaces, domain, buffers, and the scan-merger middleware
override stay fixed. It does not invoke the legacy middleware switch or restore
a whole robot YAML.

Each configuration starts with a scan-only baseline. The camera matrix adds a
color reader, a native-depth reader, both, then two independent reader pairs.
Five-second safety trials precede three measured runs with ten seconds of
warm-up and sixty seconds of capture. A passing native configuration advances
to a cold-start VGA aligned-depth test and a thirty-minute soak. VGA means
640 by 480 pixels; the selected rate is 30 frames per second. Alignment is never
toggled on a running camera driver.

The original camera startup settings and service activity are restored at the
end. The candidate multicast configuration is retained only if its required
stages pass. SIGINT/SIGTERM/SIGHUP trigger cleanup. Follow the saved recovery state
after an uncatchable interruption or host power loss; automatic cleanup cannot
run after SIGKILL or power loss.

Run the saved recovery command with `--detach` as well when using VNC. The
recovery reloads the original services and can close the desktop again.

## Acceptance and interpretation

- Required camera images: at least 29 unique frames per second, no duplicate or
  regressing stamps, and maximum stamp/receive gaps of 150 ms. Aligned-depth
  matching must reach 99% in both directions on received image stamps.
- Individual LiDARs: at least 38 Hz, receive gaps no longer than 250 ms. Merged
  scans: at least 90% of the scan-only baseline rate and gaps no longer than
  500 ms. New reconnect errors and unexpected restarts fail a run.
- Local camera readers: average physical-port TX increase no greater than
  5 Mbit/s above baseline. Packet headers distinguish multicast and unicast;
  counters alone do not establish a cause.
- Test readers stop automatically for scan-gap violations, driver errors or
  restarts, or physical-port TX increases above 20 Mbit/s in two consecutive
  one-second samples. Evidence collection continues before recovery.

CameraInfo supports interpretation but cannot substitute for received images.
Observer CPU and timing are recorded so probe overhead is visible. Host-wide
UDP drops include unrelated consumers and are not a pass/fail metric.

The report distinguishes observed failures, faults not reproduced, and stages
not tested. A current-config pass means the old fault was not reproduced. A
candidate pass establishes its behavior for the recorded workload; it does not
prove the historical failure mechanism. LiDAR reconnect behavior remains
untested unless a timeout and recovery are actually captured.

Thor installation, clock correction, HD camera profiles, TF/localization,
intentional LiDAR disconnection, motion, and multi-hour recovery qualification
are separate work. Existing operator commands and distributed prerequisites
remain in the [README](../../README.md#intel-thor-link-setup-and-pre-deploy-check).
