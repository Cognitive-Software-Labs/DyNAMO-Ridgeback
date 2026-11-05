# r100_0160 remote desktop: boot faults, repair, update restarts and recovery check

Recorded dates: 2026-10-06, 2026-10-12, 2026-10-13, 2026-10-14, 2026-10-16
Tested revisions: unknown
Provenance: partial

This record collects the dated desktop observations on Intel host
`cpr-r100-0160`. It was extracted from the
[troubleshooting page](https://github.com/Cognitive-Software-Labs/DyNAMO-Ridgeback/blob/10dc4940c8663619de1a88c2aa21e1ab5715b2e7/docs/troubleshooting.md#remote-desktop-startup-and-proxy-ownership),
the [host recovery runbook](https://github.com/Cognitive-Software-Labs/DyNAMO-Ridgeback/blob/10dc4940c8663619de1a88c2aa21e1ab5715b2e7/docs/physical/ridgeback_recovery.md#6-camera-and-desktop-service-ownership),
the [public recovery recipe](https://github.com/Cognitive-Software-Labs/DyNAMO-Ridgeback/blob/10dc4940c8663619de1a88c2aa21e1ab5715b2e7/docs/physical/ridgeback_public_recovery.md#what-was-verified-and-what-remains)
and the S7 row of the
[qualification plan](https://github.com/Cognitive-Software-Labs/DyNAMO-Ridgeback/blob/10dc4940c8663619de1a88c2aa21e1ab5715b2e7/docs/plans/PHYSICAL_remaining_qualification.md#stationary-hardware-and-host-setup),
all at that documentation snapshot.

Gaps: the repository does not record the revision of the maintained installers
or checkers used for each step, and the raw evidence was not committed. It
stays on the robot host under the `artifacts/hardware/` paths named below.
Installer receipts there hold the staged file hashes.

The earlier separation of VNC and its port-9005 proxy from the vendor webserver
on 2026-10-05 is in the
[Intel DDS screening record](2026-10-05-intel-dds-screening.md#warmed-native-follow-up-and-desktop-separation).

## Cold boot before the repair (2026-10-06)

The independent VNC service survived vendor webserver reloads, but on the
October 6 cold boot it restarted 37 times. Its logs included an Xtigervnc
segmentation fault in NVIDIA EGL/GLX and repeated already-running-display
errors. The independent port-9005 proxy failed its VNC dependency and stayed
inactive. The vendor webserver then started the 9005 listener itself, so a
browser connection through that listener could close on a webserver reload
even while the desktop stayed in the independent VNC control group.

## Stationary setup repair (2026-10-12)

The six-file desktop and Bluetooth repair was installed and independently
verified stationary. Its remote-screen drop-in bypassed the old launcher and
ran only the port-9100 proxy. Foreground VNC supervision and independent proxy
units replaced the failed dependency arrangement. Mesa selection applied to the
virtual X server only; applications kept their own GPU selection. The desktop
used the real user runtime and an isolated session bus. The controller
autoconnect helper stopped restarting platform services. Platform, guard,
camera, sensor and merger process IDs were unchanged. The receipt is
`artifacts/hardware/20261012-stationary-setup-install/`.

## First observed boot after the repair (2026-10-13)

A read-only inspection of the boot at 08:50:45 UTC found all three desktop
services active with zero reported restarts, correct listener cgroups and all
six repaired files matching their maintained sources. The selected boot
journals showed no repeated VNC or guard startup failure. The guard's DDS
readiness helper completed and the platform started without reported
restarts. This was one observed boot. Power-cycle conditions and operator
browser continuity were not recorded. Later supervised reboots checked
listener ownership through the boot verifiers; see the
[clock qualification record](2026-10-13-r100-0160-intel-clock-qualification.md).

## Unattended upgrades during testing (2026-10-14)

`unattended-upgrades` ran from about 09:25 to 09:32 UTC. The installer logged
repeated service-restart passes, and the system journal recorded requested
stops and restarts of VNC, the proxies, networking and robot services. The
boot ID stayed the same while VNC was replaced repeatedly, last at 09:31:34.
The updates completed at 09:32:31, and a passive health batch afterwards
passed all 22 checks. This explains that day's lost desktop sessions; it does
not qualify continuity during later updates.

`NRestarts=0` did not reveal these restarts: explicitly requested service
restarts reset lifecycle state and are not counted as automatic failure
retries. No maintenance-policy change or reboot was made during the diagnosis.
The update installed a newer kernel and left a reboot-required marker; the
robot had not rebooted into that kernel at the time. Evidence:
`artifacts/hardware/20261014-desktop-disconnect-diagnosis/` and
`stationary-batch-20261014T093333Z-18bc1d/`. The
[RViz observer record](2026-10-13-r100-0160-intel-rviz-observer.md#follow-up-on-2026-10-14)
describes the same window from the test stack's side, including the automatic
`needrestart` setting and the invalidated guard runs.

## Process-failure recovery check (2026-10-16)

A detached recovery check killed the main process of each desktop unit in
turn. `ridgeback-vnc.service`, `ridgeback-vnc-websockify.service` and
`ridgeback-remote-screen.service` each restarted with a new PID and restored
its RFB or WebSocket listener on 5901, 9005 and 9100 respectively. The
operator reconnected to the desktop, and SSH socket activation stayed
available on port 22. The independent fallback restart timer was cancelled
after all three listeners recovered. The report is
`artifacts/hardware/20261016-s7-desktop-recovery/result.json`. The plan
recorded this as S7, together with listener ownership across repeated boots.

This qualified supervised recovery from backend and proxy process failure for
the installed services. It did not test network loss, display corruption,
browser-side session persistence or continuity across a vendor webserver
reload.
