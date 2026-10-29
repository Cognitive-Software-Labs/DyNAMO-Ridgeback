# r100_0160 local camera-reader and port-traffic screens

Recorded dates: 2026-10-13, 2026-10-14, 2026-10-21
Tested revisions: unknown
Provenance: partial

Short guarded screens on Intel host `cpr-r100-0160` (namespace `r100_0160`,
domain 0) that tested whether extra local readers of the installed
`dynamo-camera.service`, at 640×480 and 30 Hz, disturb image delivery, LiDAR
continuity or wired port traffic. They became the S1, S2 (640×480) and S4 stages
of the robot qualification plan. The text was extracted from the
[camera deployment reference](https://github.com/Cognitive-Software-Labs/DyNAMO-Ridgeback/blob/10dc4940c8663619de1a88c2aa21e1ab5715b2e7/docs/physical/robot_local_deployment.md)
and the
[Intel–Thor transport reference](https://github.com/Cognitive-Software-Labs/DyNAMO-Ridgeback/blob/10dc4940c8663619de1a88c2aa21e1ab5715b2e7/docs/physical/intel_thor_transport.md#camera-subscriber-blocker)
at a later documentation snapshot; the
[qualification plan at the same documentation snapshot](https://github.com/Cognitive-Software-Labs/DyNAMO-Ridgeback/blob/10dc4940c8663619de1a88c2aa21e1ab5715b2e7/docs/plans/PHYSICAL_remaining_qualification.md)
names the stages.

Provenance gaps: the source text names no commit for
`tools/camera_contract_check`, `tools/hardware/screen_camera_port_traffic.py`,
`tools/intel_thor/check_link` or the installed services, and no Intel checkout
for the evidence. Reports, packet captures and run scripts live only on the
Intel host under `artifacts/hardware/` and were not committed. The 2026-10-21
run's report path was not recorded.

## Method and conditions

The source text names `tools/camera_contract_check --guard` as the reader for
S2, S4 and the 2026-10-21 check. The 2026-10-13 and S1 screens are described
only as guarded reader screens, 12 s long where a duration is stated; at the
documentation snapshot, `tools/hardware/screen_camera_port_traffic.py` runs
`camera_contract_check --guard --warmup 3 --duration 12` as its reader, but the
revision used on those dates is unknown. The check reads the installed colour,
aligned-depth and colour CameraInfo topics plus the front, rear and merged
scans; the guard discovers the scans before starting readers and monitors their
gaps through warm-up and measurement. The platform was stationary throughout.

The runs fall into populations that must not be pooled. In the two 2026-10-13
port comparisons and in S2 and S4, Thor perception was running and its roughly
760 Mbit/s camera flow was present on `eno1`. The first 2026-10-13 screen
recorded only about 98 Mbit/s of total `eno1` TX, without packet attribution.
The S1 quiet repeat stopped Thor localization first. The 2026-10-21 text does
not state Thor's state.

## First contract screen (2026-10-13)

A guarded local-reader screen measured 12 seconds after three seconds of
warm-up. The 640×480 RGB/aligned-depth streams each received 359 unique frames
(29.92 Hz), with no duplicate or regressing stamps, a maximum stamp gap of
33.5 ms and 100% exact timestamp matching in both directions. Types, encodings,
grids, optical frame and the base-to-optical TF lookup passed. Front, rear and
merged scans stayed near 40 Hz with maximum receive gaps of 71.0, 57.8 and
70.5 ms. The report is `artifacts/hardware/camera-screen-20261013T141447Z.json`.
This was a short contract screen, not repeated-profile, application or soak
qualification.

The same report recorded total `eno1` TX near 98.2 Mbit/s. It contained no quiet
baseline or packet destinations, so it could not attribute that traffic to the
test readers or close the physical-port traffic gate. The source text does not
reconcile this figure with the 763.70 Mbit/s baseline measured minutes later.

## Port-traffic comparisons with Thor perception running (2026-10-13)

`tools/hardware/screen_camera_port_traffic.py` compared five quiet seconds with
a guarded reader trial and captured 96-byte packet prefixes on both wired ports
and Wi-Fi. The first trial, `camera-port-screen-20261013T141921629818Z`, passed
its camera contract and stationary pre/post probes. Wired sysfs-counter TX was:

| Port | Before readers | Reader trial | Difference |
|---|---:|---:|---:|
| eno1 | 763.70 Mbit/s | 761.75 Mbit/s | −1.95 Mbit/s |
| enp2s0 | 0.1294 Mbit/s | 0.1288 Mbit/s | −0.0006 Mbit/s |

No wired traffic rise was attributable to the local readers in this short
comparison. Colour and depth delivery remained about 30 Hz, with 99.72% and
100% exact pairing, and all scan outputs stayed around 39.92 Hz.

Packet headers explained the large existing wired load. Over 99% of captured
`eno1` IPv4 bytes were UDP unicast from `192.168.131.1` to Thor at
`192.168.131.51`: about 754 Mbit/s before and 751 Mbit/s during the readers.
The dominant first-fragment destination ports were 7411, 7413 and 7415, with
sender port 53708. This identified the network flow, not its application,
process or ROS topics. Fragmented packets were included in the IP-byte totals;
header captures and sysfs counters used slightly different intervals.
Captured `eno1` IPv4 multicast totalled about 0.42 and 0.52 Mbit/s. The
MCU-facing `enp2s0` capture held almost entirely normal robot–MCU UDP
exchanges, with no image-sized traffic. Tcpdump reported zero kernel drops.
The analysis, with capture hashes, is
`artifacts/hardware/20261013-camera-port-analysis/packet-summary.json`.

Because the Thor flow was active, this was not the requested
remote-streaming-off comparison. A second comparison,
`camera-port-screen-20261013T142757522717Z`, again passed the camera and scan
contract with stationary pre/post probes. `eno1` TX was 763.70 Mbit/s before
and 762.73 Mbit/s during the readers, with an increase of only 0.00083 Mbit/s
on `enp2s0`. Colour and depth ran at 30.00 and 29.75 Hz; the largest depth
stamp gap was 66.7 ms, within the screen limit. Packet headers still showed
the camera socket `192.168.131.1:53708` sending to Thor, and socket inspection
identified that sender as `realsense2_camera`, PID 3049. Stopping one named
localization launch had not removed all remote readers.

## Quiet local-reader screen, S1 (2026-10-14)

This repeat stopped the exact Thor localization launch and its three
perception children, then restored the launch from a privately saved command,
environment and working directory after capture.
`camera-port-screen-20261014T093624119430Z` passed its stationary pre/post
probes, the complete short camera contract, and scan continuity. `eno1` TX was
0.233 Mbit/s before the readers and 0.196 Mbit/s during them; MCU-port TX was
0.12866 and 0.12879 Mbit/s. Each image stream received 359 unique frames in
twelve seconds (29.92 Hz), with 100% timestamp pairing, no duplicate or
regressing stamps and a maximum stamp gap of 33.44 ms. Individual scans
measured 40 Hz and the merged scan 39.92 Hz, with maximum receive gaps of 46.1,
35.5 and 45.6 ms. Packet headers showed `eno1` multicast around 0.029 and
0.021 Mbit/s and no high-volume Thor flow; all six captures reported zero
kernel drops.

This passed the short quiet local-reader traffic screen. It did not qualify
additional independent reader pairs, repeated profiles, a long soak or timeout
recovery. The earlier high wired load depended on active Thor perception
subscribers; the comparison did not by itself resolve duplicate remote
subscriptions or reduce their bandwidth.

## Reader matrix at 640×480, S2 (2026-10-14, 11:21–11:31 UTC)

With Thor perception running (its ~763 Mbit/s `eno1` flow present throughout),
`artifacts/hardware/20261014T112047Z-s2-vga-reader-matrix/run.sh` took a
60-second wired-counter baseline, then ran `tools/camera_contract_check --guard`
(10 s warm-up, 60 s measured) three times with one reader pair and three times
with two independent pairs at once. All nine reader runs passed the full
contract: about 30 Hz unique colour and depth, no duplicate or regressing
stamps, 99.78–100% exact pairing in both directions, the base-to-optical TF at
the last image stamp, and front, rear and merged scans near 39.9 Hz with gaps up
to about 51 ms. Wired TX changed by −0.6 to 0.0 Mbit/s on `eno1` and by under
0.001 Mbit/s on `enp2s0` against the baseline, within the 5 Mbit/s limit
(`summary.txt`). This passed S2 at 640×480; the 1280×720 profile and its
cross-host delivery were separate gates.

## Two-pair soak, S4 (2026-10-14, 11:48–12:20 UTC)

The same procedure measured two independent reader pairs together for 1,800 s
after 10 s of warm-up
(`artifacts/hardware/20261014T114755Z-s4-reader-pair-soak/`; its summary line
reuses the S2 script's label). Both passed the full contract: 29.96 Hz unique
colour and depth, no duplicate or regressing stamps, maximum colour and depth
stamp gaps of 33 and 66 ms, 99.97% and 100% exact pairing, the TF lookup, and
front, rear and merged scans at 39.92–39.93 Hz with gaps up to 62 ms. Wired TX
rose 1.9 Mbit/s on `eno1` over its 761 Mbit/s baseline (Thor perception
running), within the 5 Mbit/s limit, and `clearpath-sensors` logged no LiDAR
grab or reconnect errors. This passed S4 at 640×480 with SLAM and Nav2 absent;
the loaded recurrence soak was the separate S5.

## Pre-deploy reader check (2026-10-21)

`tools/intel_thor/check_link --camera-reader` passed on `r100_0160`, reporting
29.95 Hz, a wired TX rise of +0.64 Mbit/s on `eno1`, and no LiDAR grab or
reconnect errors. At the documentation snapshot this option runs one guarded
reader for 20 s after a 10 s quiet baseline; it is one short reader, not the
multi-hour soak.

## Limits

- Every screen was stationary, at 640×480, without SLAM, Nav2 or motion.
- None reproduced the September lockout, captured a Hokuyo timeout, or
  exercised timeout recovery.
- The longest run was 30 minutes. The loaded multi-hour soak was not run.
- Sysfs counters and header captures cover slightly different intervals, and
  the S2 and S4 wired baselines included the active Thor flow.
