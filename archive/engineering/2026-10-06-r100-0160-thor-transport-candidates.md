# r100_0160 Intel–Thor camera transport candidates

Recorded dates: 2026-10-06
Tested revisions: `5f9467eb7f45dc81de872f5c1569d085de0e099e`, `fbf730ae98fc096e5dad288ef6fb1a25fe8a0942`, `c52384a2739994ba7268e2b42dfb8438c28b595f`, `1e72cc972e4e33f1c66cc6ad6c5c5baa8eede439`
Provenance: partial

Raw artifacts stay on the Intel host under the `DyNAMO-Ridgeback-camera`
worktree's `artifacts/hardware/` and were not committed. Thor's package
versions are recorded below; its full package state was not captured.

## Conditions

D455 at 640×480, 30 Hz, colour and colour-aligned depth, driven by
`dynamo-camera.service` on Intel; Thor subscribes over the 1 Gb/s `eno1`
Ethernet through `br0`, CycloneDDS 0.10.5 with `AllowMulticast spdp`, domain 0.
Each full collection is `tools/intel_thor/sensor_latency_run`: three
interleaved Intel/Thor repeats of 10 s warm-up and 60 s measurement, with a
simultaneous Intel guard observer. Cross-host components use the continuous
LAN clock calibration; O4 additionally ran under PTP (Intel wall clock
distributed to Thor). Robot stationary.

## Results (Thor observer, median / p95 ms per repeat)

| Run | Colour | Depth | Merged scan |
|---|---|---|---|
| Raw baseline (`5f9467e`) | total 29.5–29.8 / 32.9–34.6; driver ~20.8, network ~8.8 | total 21.4–21.8 / 24.9–25.7; network 5.7 | total 1.1 / 2.0–2.1; network p95 1.5–1.6 |
| O1 camera DDS profile, 16 MiB send buffer (`fbf730a`) | total 29.8–30.8 / 33.4–34.0; driver ~18.3, network ~11.7 | total 22.0–23.1 / 25.6–26.3 | total 1.0 / 9.0–9.4; network p95 8.5–9.0 |
| O4 H.264 (NVENC, software decode) + RVL depth (`1e72cc9`) | total 33.9–35.1 / 48.3–49.9; encode 26–27, network 0.4, decode 5.8–6.0 / 19.6–20.0 | total 27.9–29.0 / 31.1–32.9; encode 19–20, network 3.6, decode 3.8 | total 0.9–1.1 / 1.7–1.9 |

All nine Thor observations passed the camera/scan contract with resolved clocks
and no frame missing against the Intel observer. Baseline and O1 met every
working target; O4 failed only colour total p95 ≤ 40 ms in all three repeats.

**O1:** the larger send buffer moved queueing from the driver's publish into
the kernel's egress queue. Totals did not change, because each frame's wire
time did not, and camera bursts then delayed scans sharing the link.

**O4:** compression removed nearly all network time but encode and decode
cost more. The slow colour decodes were exactly every 15th frame (119 of 1797
in repeat 1), the H.264 keyframes at `gop_size` 15, at about 20 ms each in
software.

## O4 diagnostics (one 15 s repeat each, not qualification evidence)

| Variant | Revision | Finding |
|---|---|---|
| H.264 + zstd depth | `c52384a` | zstd decoded depth with stamp 0 and empty frame ID (450 frames, one stamp) while the compressed messages carried 449 distinct stamps; contract failed |
| H.264 (`h264_cuvid`) + PNG depth | `c52384a` | colour decode 137 / 139 ms, total 179 / 182; PNG depth encode 36, decode 9, total 49 / 52 |
| H.264 (`h264_cuvid`, `flags:low_delay`) + RVL | `1e72cc9` | `cannot set option flags ... Option not found`; colour decode unchanged at 137 ms; RVL depth total 29.7 / 32.7 |
| H.264 (software `h264`) + raw depth | `1e72cc9` | colour decode 5.9 / 20.4, total 32.1 / 47.0 |

Two earlier attempts at `c52384a`'s predecessors stopped on collector bugs
before measuring and hold no data. JPEG colour was not run.

Thor packages: `image_transport` 5.1.7, `image_transport_plugins` 4.0.7,
`ffmpeg_image_transport` 3.0.4, `ffmpeg_encoder_decoder` 3.0.1. Intel encoder:
`h264_nvenc` on an RTX 5060, `preset:p1,tune:ull,zerolatency:1,delay:0`,
`gop_size` 15, no B-frames, 8 Mb/s.

## Limits

Single robot, stationary, one camera profile and link; no detector or
exploration load. The diagnostic rows are single short runs. The O1 run used
continuous LAN calibration without PTP, so its cross-host figures carry that
method's bound (at most 0.2 ms).
