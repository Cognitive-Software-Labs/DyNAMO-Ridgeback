# PHYSICAL — D455 camera runs handoff (agent 1)

**Author:** agent 1 (Claude Code session on the robot host `cpr-r100-0160`)
**Written:** 2026-09-18, 18:45 UTC · **Repository HEAD:** `950aacb` on
`feat/real-hardware-exploration`
**Governing plan:** [PHYSICAL — Robot measurements and stationary sensor validation](../PHYSICAL_robot_measurements_and_validation.md),
section "D455 camera validation", checks 1–3
**Related:** [Intel–Thor transport](../../physical/intel_thor_transport.md)
(owned by the Intel–Thor work, referred to below as agent 2)

The evidence folders listed here are under `artifacts/hardware/` on the robot
host. They are not tracked by git. Each has `notes/result.md`, `notes/timeline.txt`
and a `SHA256SUMS`.

## 1. Robot state at handoff (18:44 UTC)

| Item | State |
|---|---|
| Robot services middleware | **CycloneDDS**, switched at 17:39 by agent 2 (`tools/intel_thor/intel_services_rmw`). `/etc/clearpath/setup.bash` selects `rmw_cyclonedds_cpp` |
| `realsense-camera.service` | Active, vendor config (640×480×30 for all streams, `align_depth.enable: false`, infra1+infra2 on, pointcloud on). Driver PID 178509. It was restarted by systemd at 18:35:33 after a driver crash (§4) |
| `clearpath-sensors.service` | Active since 18:30:12, but **both LiDARs have been locked out since 18:30:40** (urg_node reconnect loop, no scans). Recovery: `sudo systemctl restart clearpath-sensors` |
| Agent 1 processes | None running. No robot files edited. Every camera change was a runtime parameter or a short-lived driver that has since stopped |
| Driving | **Do not drive** until the LiDAR/transport issue in §4 is resolved |

## 2. Decisions made by the user in this session

- **Gate 3 gap criterion changed** from "longest gap ≤ 100 ms" to "longest
  sender-side gap ≤ 150 ms (at most 3 consecutive frames missing)". Reason:
  `mask_measurement_node` matches depth to the detection's colour frame by
  exact stamp, from a 15-frame buffer. A gap loses at most one measurement
  and never pairs wrong depth. The detector runs at ≤ 10 Hz, so a 100 ms
  colour gap only delays one detection. The ≥ 99 % exact-match criterion is
  unchanged.
- **Permanent aligned depth: option B chosen**, a repo-owned camera launch
  and parameter file, with the vendor service stopped for our sessions. Not
  implemented yet. The rejected options were editing the root-owned vendor
  start script (A) and setting the parameter on the vendor driver at stack
  start (C).

## 3. Results on Fast DDS (valid)

"Sender side" means rates and gaps from the per-frame `camera_info` stamps. The
Python probe drops some large images itself, and this separates that loss.

### P1: 640×480 with `align_depth.enable: true` (runtime parameter on the vendor driver)

Evidence: `20260918T162241-p1-p2-camera-runs` (first run) and
`20260918T163553-p1-rerun` (baseline plus 3 reruns).

| Check | Result |
|---|---|
| Aligned image | 640×480, `16UC1`, `camera_color_optical_frame`; aligned `camera_info` identical to colour (K fx 388.14, fy 387.64, cx 322.20, cy 245.48) — **pass** |
| Rates (sender side) | colour 29.75–29.91 Hz, aligned 29.65–29.78 Hz — **pass** |
| Duplicates / regressions | 0 — **pass** |
| Longest gap (sender side) | 66.8–100.3 ms over 4 runs. 100 ms happened in 2 of 4 runs (aligned) and 1 of 4 (colour) — **pass** under the 150 ms rule |
| Exact match colour→aligned / aligned→colour | 99.05–99.66 % / 99.83–99.94 % — **pass** |
| Driver CPU | 18–30 % of one core (single `top` samples) |

Enabling alignment also creates `aligned_depth_to_infra1/*` while infra1 is on.
That costs extra CPU, and option B should account for it. Without alignment
the sender drops about 3 single frames per minute. Alignment adds about 6–17.

### P2: 1280×720

- **Runtime switch on the vendor driver fails.** Evidence: `20260918T162241-p1-p2-camera-runs`.
  Depth and infrared switch, but colour never delivers frames
  (`Frames didn't arrived within 5 seconds`). Revert worked.
- **Cold start with our own driver** (vendor arguments + 1280×720 + align),
  vendor service stopped by the user. Evidence: `20260918T163728-p2-coldstart`
  (A and B once each on Fast DDS) and `20260918T181451-p2-coldstart-repeat`
  (A1 B1 A2 B2 A3 B3 interleaved on CycloneDDS).

| Variant | Colour from the camera (driver log) |
|---|---|
| A: infra1 + infra2 on | **fails 4 of 4**: 34–130 frame timeouts per run, colour never streams |
| B: infra2 off | **works 4 of 4**: 0 timeouts |

B on Fast DDS (17:01) against the criteria: colour 29.05 Hz and aligned 29.10 Hz
(sender side). At 1280×720 the real frame period is 33.91 ms (29.49 fps), so the
≥ 29 Hz margin is only about 0.5 Hz. Longest gap 135.6 / 101.7 ms. Exact match
99.83 % / 99.66 %. Aligned `camera_info` equals colour. 0 duplicates on colour
and aligned. **B passes gates 1–3.**

Why A fails is unknown. It is not the cable or bandwidth:

- The link negotiates 5 Gbit/s.
- The camera is alone on its hub.
- The kernel logged no USB errors.
- The USB payload is about 1.3 Gbit/s with infra2 on.

A test with the camera plugged directly into the PC is not possible (no
physical access). Option B already plans to turn infra2 off.

## 4. Results on CycloneDDS: camera transport broken (open problem)

Evidence: `20260918T181451-p2-coldstart-repeat` (1280×720) and
`20260918T183015-p1-cyclone` (640×480).

| | Fast DDS | CycloneDDS |
|---|---|---|
| 640×480 colour, alignment off | 29.9 Hz | 7.7 Hz, 2–3 frames missing in almost every gap |
| 640×480 with alignment on | colour 29.9, aligned 29.7 Hz | colour **0**, aligned **0**, depth ~19 Hz |
| 1280×720 B (camera delivering) | about 29 Hz | 4–6 Hz, and only one large stream per run reaches the probe |
| `align_depth.enable` on→off | clean 3 of 3 | **driver SIGSEGV** (exit -11) at 18:35:28 |

**Wire measurement** (`20260918T183015-p1-cyclone/notes/wire_test.txt`):

- Idle (only the vendor consumers subscribed): TX on `br0` is 0 Mbit/s.
- During one **local**, camera-only subscription: `br0`, `enp2s0` and `eno1`
  each send about **92 Mbit/s**.

Camera data meant for a reader on the same host leaves through the bridge
onto both physical Ethernet ports. The Hokuyos (192.168.131.21/.22) and the
Thor sit on `br0`. `/etc/clearpath/cyclonedds.xml` binds `br0` and `wlp3s0`
with the default multicast settings.

**LiDAR lockouts:** 17:43:53 (agent 2's commands), 18:17:06 (1 s after an
agent 1 probe that also subscribed to the scans) and 18:30:40 (2 s after a
**camera-only** agent 1 probe). None happened in 2.5 h on Fast DDS.
Subscribing to the scans is therefore not required. Starting a camera
subscriber under the current CycloneDDS configuration is enough.

The LiDAR link is inferred from the timing (3 of 3) plus the wire measurement.
Nothing was measured at the Hokuyo side. The working hypothesis: large camera
samples go out as multicast in about 1500-byte fragments, the fragments are
lost so samples never complete, and the flood reaches the LiDAR segment. The
existing urg_node reconnect bug then makes the stall permanent.

## 5. Next steps

1. **Agent 2 / transport owner:** fix or roll back the Intel CycloneDDS
   configuration before any camera subscriber runs again. Candidates, all
   unverified:
   - Restrict multicast to discovery (`AllowMulticast` = `spdp`).
   - Keep same-host delivery off `br0` (loopback or shared memory).
   - Return the robot services to Fast DDS with
     `tools/intel_thor/intel_services_rmw` until a fix is tested.

   Verify with the wire test (expect about 0 Mbit/s on `enp2s0`/`eno1` for a
   local subscriber) and a 60 s camera capture.
2. Then restart the LiDARs (`sudo systemctl restart clearpath-sensors`) and
   **repeat P1 at 640×480 and P2 variant B** on the final middleware. Use the
   probe and scripts in the evidence folders: `run.sh` for P1 (runtime
   parameters, no sudo) and `coldstart.sh B <label>` for P2 (needs
   `sudo systemctl stop/start realsense-camera.service` from the user). The
   probe `captures/stream_capture.py` is camera-only from
   `20260918T181451-p2-coldstart-repeat` onward.
3. Implement option B (repo camera launch plus parameter file, infra2 off,
   align on, serial pinned, camera-to-base transform). It is blocked on step 1,
   and the camera-to-base transform (P3) still needs the mount measurement.
4. Fold the confirmed results into the governing plan's D455 section once
   they have been rerun on the final middleware.

## 6. What not to do

- Do not subscribe to camera topics on the robot under the current CycloneDDS
  configuration. That includes the estimators, `ros2 topic hz` and streaming
  to the Thor. Each new subscriber has so far locked out the LiDARs.
- Do not switch 1280×720 at runtime on the vendor driver: colour stops. Do
  not toggle `align_depth.enable` on CycloneDDS: the driver crashed.
- Do not drive the robot while the LiDARs are locked out.
