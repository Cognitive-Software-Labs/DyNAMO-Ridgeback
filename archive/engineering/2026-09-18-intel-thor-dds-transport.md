# Intel–Thor DDS transport diagnosis

Recorded dates: 2026-09-18
Tested revisions: `a01a4532a170fcb85df1a5aaffb8c28bf1601f96`
Provenance: partial

## Scope and decision

Operators reported lag on the Intel–Thor Ethernet link. This record keeps the
measurements that attributed the lag to Fast DDS rather than to the cable or
network interfaces, and that selected the Ethernet-only profiles and receive
buffer limit. The resulting contract is maintained in the current reference
`docs/physical/intel_thor_transport.md`; this record only explains how it was
chosen.

Provenance is partial. The listed revision was checked out on Intel during the
tests. The camera measurements used the vendor `realsense-camera.service` and the
stock `ros2 topic hz`, not repository code. The synthetic-probe measurements used
`tools/intel_thor/dds_link_probe.py` and the `intel_thor` profiles while they
were still uncommitted. The committed probe adds socket-level drop accounting
to the version that was tested. Robot service configuration was not captured.

Correction, 2026-09-18, the same day: the deployment later moved to CycloneDDS
on both hosts. The Fast DDS profiles from this record are kept only for
benchmark comparisons; see the
[middleware benchmark](2026-09-18-intel-thor-rmw-benchmark.md).

## Conditions

- Intel `cpr-r100-0160`: `br0` 192.168.131.1 (Thor learned on `eno1`, 1 Gb/s
  full), lab Wi-Fi `wlp3s0`. Thor `nvidia-thor-r100-0160`: `enP2p1s0`
  192.168.131.51 (1 Gb/s full), lab Wi-Fi `wlP1p1s0`. Jazzy, Fast DDS 2.14.6,
  domain 0 for the robot graph and 77/87 for the probe tests.
- Vendor D455 publisher: 640×480 rgb8 at ~30 Hz, measured at 29.7 Hz locally
  on Intel.
- Default `net.core.rmem_max`/`rmem_default` of 212,992 on both hosts until the
  buffer change.

## Measurements

The link layer was healthy throughout. Ping from Intel to Thor averaged 0.38 to
0.52 ms with no loss, both idle and during a camera stream. Neither interface
reported errors. Thor's NIC had 186,573 `rx_mac_missed` accumulated from
earlier image tests, and the count did not grow while the link was idle.

Camera colour stream received on Thor (`ros2 topic hz`, 15 to 25 s windows):

| Configuration | Thor rate | Wi-Fi carrying the same stream | Thor UDP buffer drops |
|---|---|---|---|
| Default DDS | 9.6 Hz with gaps up to 3.0 s in one run; about 30 Hz in two runs; no frames in one run | ~190 to 200 Mbit/s | 13,459 to 20,325 per run |
| Thor Ethernet-only profile | 29.87 Hz, max gap 67 ms | 0 | 7,057 per 25 s |

Synthetic probe after `rmem_max` and `rmem_default` were raised to 16 MiB,
30 Hz 900 KiB frames:

| Direction and profiles | Received | Max gap | Intel Wi-Fi TX |
|---|---|---|---|
| Intel → Thor, no profile | 30.0 Hz | 151 ms | 120 Mbit/s |
| Intel → Thor, Thor profiled | 30.07 Hz | 35 ms | 1 Mbit/s |
| Thor → Intel, only Thor profiled | 1 frame in 15 s | — | — |
| Thor → Intel, both profiled | 30.07 Hz | 38 ms | 0 (Thor Wi-Fi TX) |

A profiled Intel process still received local Clearpath topics, including
filtered odometry and the LiDAR scan.

On Intel at idle, 1,080 UDP receive-buffer drops were counted in 10 s. Per-socket
counters attributed them to the vendor `depth_to_mono8.py` and to
`ridgeback-camera-mjpeg-server.py`.

The final `check_link` run passed every link and traffic check. Both directions
ran at 30.0 to 30.1 Hz with max gaps of 35 to 37 ms, no drops on the probe
sockets, and 1.6 to 1.8 Mbit/s of background Wi-Fi traffic. Thor's clock
was 4.21 ± 0.13 ms ahead of Intel.

## Pitfalls encountered

- The whitelist first appeared to have no effect. Fast DDS 2.14 reads
  `FASTRTPS_DEFAULT_PROFILES_FILE` and silently ignores `FASTDDS_DEFAULT_PROFILES_FILE`.
  Thor also rebooted during testing, which wiped the profile from `/tmp`.
  Profiles now live in the repository, and effectiveness was confirmed from
  socket bindings: every probe socket was bound to 192.168.131.51.
- `ss -m` on these kernels does not report UDP socket memory, so the 16 MiB
  buffer request in the profile was not directly observed. It relies on Fast
  DDS's documented `receiveBufferSize` behavior, capped by `rmem_max`, and on
  the check's per-socket drop count.
- `ros2 topic hz --no-daemon` exits before discovery completes and measured
  nothing. The probe waits for the first frame before opening its window.

## Limits

The camera stream stopped publishing after a runtime reconfiguration by another
workflow at about 16:30 UTC. Later tests therefore used synthetic frames only.
The benefit of the buffer change for the real camera stream was not measured
separately from the profile change. Exact-stamp aligned-depth delivery across
hosts was not tested.
