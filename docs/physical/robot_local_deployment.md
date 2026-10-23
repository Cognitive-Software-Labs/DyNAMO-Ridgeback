# Robot-local deployment

This reference owns the installed D455 service and its reconstruction inputs.
Public commands live in the [README](../../README.md#hardware-camera-service).
Host reconstruction and credentials belong to the
[recovery runbook](ridgeback_recovery.md).

## Camera owner and source

On 2026-10-12 `dynamo-camera.service` was active and enabled, with the vendor
camera units disabled and inactive. The restored `tools/intel_thor/camera_service`
matched the installed description-only camera declaration, driver parameters,
unit and discovery-only multicast configuration. Its source and templates were
recovered from the installed metadata's Git revision
`4e5a9b5476ad9c502d90718709629e2f2052cccb`; no live service was changed.

The maintained sources are `tools/intel_thor/camera_service`, the D455 and DDS
configs under `src/ridgeback_autonomy_hardware/config/`, and the systemd template
there. The selected robot input is `deployment/r100_0160/camera.json`: D455
serial 338522302134, 640×480 at 30 FPS, services DDS profile. The driver reads
`/etc/clearpath/dynamo-camera/d455.yaml` and runs directly from the apt ROS
installation rather than this checkout's build/install outputs. Metadata at
`/etc/clearpath/dynamo-camera/install.json` records its installed profile and
source revision. A different checkout revision alone does not imply file drift.

The camera service owns colour and colour-aligned depth, remapped to
`/r100_0160/sensors/camera_0/color/image` and `…/depth/image`, with colour
CameraInfo at `…/color/camera_info`. Infrared, IMU and pointcloud outputs are off.
The robot declaration owns the camera mount; the driver owns its factory
extrinsics and remaps TF to the namespaced tree. Installation conflicts with the
vendor camera unit. Do not run the legacy vendor DDS/repair workflow against
this service or re-enable its competing camera units.

`tools/camera_contract_check` targets these installed names explicitly. On
`origin/dev`, `ridgeback_common.camera_inputs` gives the RealSense backend the
same three names as the simulators, so the localization launches already
consume the installed service's topics; this checkout's older
`camera_inputs.py` still names `image_raw` and `aligned_depth_to_color/image_raw`
until it is merged with `origin/dev`. Do not launch localization from this
checkout's source before that merge. Restoring the
tooling and matching files does not qualify image timing, TF, target estimation
or hardware performance.

On 2026-10-13 a guarded local-reader screen measured 12 seconds after three
seconds of warm-up. The 640×480 RGB/aligned-depth streams each received 359
unique frames (29.92 Hz), with no duplicate/regressing stamps, a maximum stamp
gap of 33.5 ms and 100% exact timestamp matching in both directions. Types,
encodings, grids, optical frame and the base-to-optical TF lookup passed.
Front/rear/merged scans stayed near 40 Hz with maximum receive gaps
71.0/57.8/70.5 ms. The report is
`artifacts/hardware/camera-screen-20261013T141447Z.json`. This is a short VGA
contract screen, not repeated-profile, application or soak qualification.

That same report recorded total `eno1` TX near 98.2 Mbit/s. It contains no quiet
baseline or packet destinations, so it cannot attribute that traffic to the
test readers or close the physical-port traffic gate. The maintained
`tools/hardware/screen_camera_port_traffic.py` compares five quiet seconds with
a guarded reader trial and captures 96-byte packet prefixes on both wired ports
and Wi-Fi. It sends no commands, restarts nothing and preserves evidence for
packet destination/protocol analysis. Total port traffic alone does not prove
multicast flooding or a LiDAR fault mechanism.

The subsequent `camera-port-screen-20261013T141921629818Z` trial passed its
camera contract and stationary pre/post probes. Wired sysfs-counter TX was:

| Port | Before readers | Reader trial | Difference |
|---|---:|---:|---:|
| eno1 | 763.70 Mbit/s | 761.75 Mbit/s | −1.95 Mbit/s |
| enp2s0 | 0.1294 Mbit/s | 0.1288 Mbit/s | −0.0006 Mbit/s |

There was no wired traffic rise attributable to introducing these local
readers in this short comparison. Color/depth delivery remained about 30 Hz,
with 99.72%/100% exact pairing and all scan outputs around 39.92 Hz.
Packet headers explain the large existing wired load: over 99% of captured
`eno1` IPv4 bytes were UDP unicast from `192.168.131.1` to Thor
`192.168.131.51`, approximately 754 Mbit/s before and 751 Mbit/s during readers.
Dominant first-fragment destination ports were 7411, 7413 and 7415, with sender
port 53708. This identifies the network flow, not its application/process or
ROS topics. Fragmented packets were included in IP-byte totals; header captures
and sysfs counters use slightly different measurement intervals.

Captured `eno1` IPv4 multicast totaled about 0.42/0.52 Mbit/s. The MCU-facing
`enp2s0` capture consisted almost entirely of normal robot–MCU UDP exchanges,
with no high-volume image-sized traffic. Tcpdump reported zero kernel drops in
these captures. Analysis with capture hashes is under
`artifacts/hardware/20261013-camera-port-analysis/packet-summary.json`.
The active high-volume Thor flow means this was **not** the requested
remote-streaming-off comparison. Identify its sender/subscribers and arrange a
quiet repeat before claiming that full gate. These short windows neither
reproduce the historical lockout nor qualify a long soak.

A second comparison (`camera-port-screen-20261013T142757522717Z`) again passed
the camera and scan contract with safe stationary pre/post probes. Wired TX
was 763.70 Mbit/s before and 762.73 Mbit/s during readers on `eno1`, with only
0.00083 Mbit/s increase on `enp2s0`. Color/depth rates were 30.00/29.75 Hz;
largest depth stamp gap was 66.7 ms, within the screen limit. Packet headers
still show the camera socket `192.168.131.1:53708` sending to Thor. Socket
inspection identified that sender as `realsense2_camera`, PID 3049. Therefore
the remote-streaming-off condition was not achieved in this repeat; stopping
one named localization launch cannot be assumed to remove all remote readers.
Identify the remaining Thor subscriptions before another quiet comparison.

The October 14 quiet repeat finally removed those readers: the exact Thor
localization launch and three perception children were stopped, and the launch
was restored from private saved command/environment/cwd after capture.
`camera-port-screen-20261014T093624119430Z` passed stationary pre/post probes,
the complete short camera contract, and scan continuity. `eno1` TX was
0.233 Mbit/s before readers and 0.196 Mbit/s during them; MCU-port TX was
0.12866/0.12879 Mbit/s. Each image stream received 359 unique frames in twelve
seconds (29.92 Hz), with 100% timestamp pairing, no duplicate/regressing stamps
and maximum stamp gap 33.44 ms. Individual scans measured 40 Hz and merged
39.92 Hz, with maximum receive gaps 46.1/35.5/45.6 ms. Packet headers show
`eno1` multicast around 0.029/0.021 Mbit/s and no high-volume Thor flow; all six
captures reported zero kernel drops. This passes the short quiet local-reader
traffic screen. It does not qualify additional independent reader pairs,
repeated profiles, a long soak or timeout recovery. The prior high wired load
was dependent on active Thor perception subscribers; this comparison does not
by itself resolve duplicate remote subscriptions or reduce their bandwidth.

**S2 VGA reader matrix (2026-10-14, 11:21–11:31 UTC).** With Thor perception
running (its ~763 Mbit/s `eno1` flow present throughout) and the platform
stationary, `artifacts/hardware/20261014T112047Z-s2-vga-reader-matrix/run.sh` took a 60-second wired-counter baseline, then ran
`tools/camera_contract_check --guard` (10 s warm-up, 60 s measured) three times
with one reader pair and three times with two independent pairs at once. All
nine reader runs passed the full contract: about 30 Hz unique colour and depth,
no duplicate or regressing stamps, 99.78–100 % exact pairing in both directions,
the base-to-optical TF at the last image stamp, and front/rear/merged scans near
39.9 Hz with gaps up to about 51 ms. Wired TX changed by −0.6 to 0.0 Mbit/s on
`eno1` and under 0.001 Mbit/s on `enp2s0` against the baseline, within the
5 Mbit/s limit (`summary.txt`). This passes S2 at 640×480; the HD profile and
HD cross-host delivery remain separate gates.

**S4 reader-pair soak (2026-10-14, 11:48–12:20 UTC).** The same procedure with
two independent reader pairs measured together for 1,800 s after 10 s of
warm-up (`artifacts/hardware/20261014T114755Z-s4-reader-pair-soak/`; its summary line reuses the S2 script's label). Both passed the
full contract: 29.96 Hz unique colour and depth, no duplicate or regressing
stamps, maximum colour/depth stamp gaps 33/66 ms, 99.97 %/100 % exact pairing,
the TF lookup, and front/rear/merged scans at 39.92–39.93 Hz with gaps up to
62 ms. Wired TX rose 1.9 Mbit/s on `eno1` over its 761 Mbit/s baseline (Thor
perception running), within the 5 Mbit/s limit, and `clearpath-sensors` logged
no LiDAR grab or reconnect errors. This passes S4 at 640×480 with SLAM and Nav2
absent; the loaded recurrence soak is S5.

**S2 HD trial (2026-10-16).** The operator selected 1280×720 at 30 FPS with
the maintained camera installer. The robot declaration did not change, so only
the camera service restarted. A guarded five-second screen after three seconds
of warm-up delivered 22.40 Hz for both images. A second guarded run after ten
seconds of warm-up measured colour at 22.58 Hz and depth at 22.08 Hz over
60 seconds; the largest depth stamp gap was 400.3 ms and colour-to-depth exact
stamp pairing was 97.79%. Both failed the 29 Hz image gate; the front, rear
and merged LiDAR scans stayed near 39.9 Hz. The evidence is under
`artifacts/hardware/20261016-s2-hd-short.json` and
`20261016-s2-hd-full.json`. During diagnosis `eno1` transmitted about
865 Mbit/s with Thor localization active. That observation suggests a possible
cross-host bandwidth limit but does not establish the cause; a quiet local HD
comparison and HD cross-host delivery remain untested. At 1280×720,
RGB8 plus Z16 at 30 FPS would require about 1.106 Gbit/s of image payload
before DDS and Ethernet overhead if both raw streams crossed to Thor. The
camera journal confirms that both HD sensor profiles opened at 30 FPS over
USB 3.2, but it does not count frames produced at the sensor. Thor's
localization launch was running from an abandoned interactive session rather
than a restartable service, so its active consumer was left untouched.
The original 640×480
profile was restored and passed a guarded 12-second screen at 29.92/30.00 Hz
colour/depth with all three scans near 39.92 Hz
(`20261016-s2-vga-restored.json`).

**S2 HD diagnosis (2026-10-16, 09:55–10:00 UTC).** The cause is the 1 Gb/s
robot Ethernet, not the camera or Intel. Raw 1280×720 colour (`rgb8`, 2,764,800 B)
plus aligned depth (`16UC1`, 1,843,200 B) at 30 Hz is about 1,106 Mbit/s for a
single remote copy. With the profile at 1280×720 and the same guarded 60-second
check (`artifacts/hardware/20261016T095428Z-s2-hd-diagnosis/`):

| Run | Thor localization | Colour / depth | Largest depth gap | `eno1` TX |
|---|---|---|---|---|
| A | stopped | 29.92 / 29.92 Hz, 100 % pairing | 66.9 ms | 0.3 Mbit/s |
| B | running (one subscriber per image) | 22.45 / 22.37 Hz | 300.5 ms | 856.6 Mbit/s |

856.6 / 1,106 × 30 Hz ≈ 23 Hz, the observed rate. The driver itself publishes
fewer frames while the link is saturated, so **local Intel readers also drop to
about 22 Hz** whenever Thor pulls raw HD; the LiDAR scans were unaffected. Local
HD is therefore a camera-and-host question that run A answers, while raw HD to
Thor cannot meet 30 Hz on this link. **Decision (user, 2026-10-16): Thor stays at 640×480.** The deployment profile
remains 640×480 at 30 Hz raw; 1280×720 is a local-only profile, selected only
while Thor localization is stopped. Compressed or reduced-rate HD for Thor is not
pursued.

**S2 HD local reader matrix (2026-10-16, 10:22–10:33 UTC).** With 1280×720
selected and Thor localization stopped, the same nine-run procedure as at VGA
(`artifacts/hardware/20261016T101433Z-s2-hd-reader-matrix/`) passed eight runs: 29.4–30.0 Hz colour and depth, 99.77–100 % exact
pairing, the TF lookup, all three scans near 39.9 Hz, and no wired traffic
rise. Run `single_1` failed only its stamp-gap limit: one 166.8 ms gap (five
frames) in colour and depth together, against 150 ms; other HD runs reached
100–133 ms where VGA never exceeded 67 ms. The camera journal shows no error
and the profile stayed at 1280×720 throughout, so these are source-side frame
dropouts at HD. **HD local delivery is therefore not qualified** under the
frozen criteria. HD is local-only and not the deployment profile, so this does
not block VGA operation; qualifying HD would need the dropouts characterised
in a longer HD capture.

## Reconstruction and verification

The recovery preparer uses camera.json to regenerate the description-only
camera entry, service and parameters from maintained sources. Installed unit,
parameters, private DDS profile and metadata are labelled snapshots in the
deployment manifest. They are evidence, not a substitute for rendering
or fresh installation metadata. Select the camera service instead of the
historical vendor wrapper; retain the merger's explicit Fast DDS exception.

On a replacement host, establish ROS packages, interfaces, namespace and the
services DDS profile first. Then use the maintained camera installer to render
and record the actual installation, following the root workflow. Its apply and
rollback operations can regenerate Clearpath state and restart services; they
are not read-only checks and are not part of the current stationary desktop
repair. Driver serial selection uses the RealSense camera serial, not its
separate USB descriptor serial.

The public reconstruction recipe and camera tool tests verify offline rendering
and failure/rollback behavior. Clean installation, cold boot and perception
qualification remain separate evidence gates. Changes to profiles or middleware
need a fresh guarded sensor test; Thor remains unchanged until needed.
