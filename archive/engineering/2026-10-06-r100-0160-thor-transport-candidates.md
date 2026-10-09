# r100_0160 Intel–Thor camera transport candidates

Recorded dates: 2026-10-06, 2026-10-09
Tested revisions: `5f9467eb7f45dc81de872f5c1569d085de0e099e`, `fbf730ae98fc096e5dad288ef6fb1a25fe8a0942`, `c52384a2739994ba7268e2b42dfb8438c28b595f`, `1e72cc972e4e33f1c66cc6ad6c5c5baa8eede439`, `de29c637b6fb4a047017d9c5b660b19aa15ac009`, `be224c69d94744ae00d4bb3b49a4be53419e0749`
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

## Clock synchronization

Before 2026-10-06 both hosts took internet NTP over Wi-Fi; Thor ran 2.01 ms
ahead of Intel (bound ±0.18 ms) at inspection. chrony with Intel serving the
subnet did not hold 0.2 ms during Intel's initial internet slews at Thor's
default 64-second polling. With 16 polls per second, the baseline still had
two of three Thor observations unresolved under a constant whole-run
correction; continuous calibration over short adjacent intervals then resolved
all three. Agreement, unlike measurement, followed Intel's internet
corrections: Thor was 0.20–0.24 ms ahead during the baseline and 1.5–1.9 ms
behind during O1, because chrony serves Intel's estimate of true time while
sensor stamps use Intel's wall clock.

PTP replaced Thor's chrony (`de29c637b6fb4a047017d9c5b660b19aa15ac009`,
`artifacts/hardware/20261006T145000Z-ptp-verification/`): Intel serves its
wall clock from `eno1`'s hardware clock over L2 and Thor follows on
`enP2p1s0`. Eighteen direct-LAN samples over three minutes measured offsets of
−38 to +1 µs, each bounded by 52–130 µs, through two Intel internet corrections
of +1.59 ms and −1.10 ms that Intel's 500 ppm slew limit spread over tens of
seconds. On 2026-10-09, after a Thor reboot, Thor's `phc2sys` reported 0.3–1 µs
rms.

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

## O6 driver pipeline and performance policy

Revision `be224c69d94744ae00d4bb3b49a4be53419e0749`, 2026-10-09, raw transport as in the baseline, PTP clock.
Both hosts at the performance governor (Intel energy preference performance,
turbo on), Thor in MAXN, installed by `host_performance`
(`artifacts/hardware/20261009T123123Z-thor-latency-o6-performance/`). The
2026-10-06 baseline ran with Intel's `powersave` governor
(`balance_performance`) and Thor at 120W under schedutil.

| median / p95 ms | Baseline (powersave) | Performance policy |
|---|---|---|
| Colour total on Thor | 29.5–29.8 / 32.9–34.6 | 27.8–28.3 / 30.5–31.6 |
| Depth total on Thor | 21.4–21.8 / 24.9–25.7 | 19.8–20.3 / 22.5–23.6 |
| Merged scan total on Thor | 1.1 / 2.0–2.1 | 0.5 / 1.6 |
| Colour driver, Intel-only run | 20.4–20.9 / 24.1–25.2 | 14.6–15.1 / 16.6–17.3 |

All nine observations passed the contract, resolved the clock, met every
working target and lost no frame against the Intel observer. The driver split
(host arrival in whole milliseconds) matched every frame:

| median / p95 ms | Camera+USB | Intel, Intel-only run | Intel, with Thor reading |
|---|---|---|---|
| Colour | 11.7–12.1 / 13.7–14.4 | 3.0 / 3.4 | 7.1–7.2 / 7.7 |
| Depth | 5.6–6.0 / 7.6–8.3 | 8.6 / 9.0 | 8.6 / 9.1 |

A single 10-second Intel check under `powersave` earlier the same day gave
colour Intel 5.9 / 7.9 ms, so the policy roughly halved Intel processing.
Depth's Intel share includes about 6 ms waiting for colour, whose metadata
stamps arrive later. Colour minus depth publish time (same stamp) was 0.48 ms
median without a Thor reader and 4.6 ms (p5 4.4, p95 5.0) with one: the driver
publishes aligned depth first, and that send to Thor occupies the 1 Gb/s link
for about depth's wire time before colour is published.

Two runs that day are not evidence for either condition:
`20261009T122214Z-thor-latency-o6-startup-miss` received no camera or scan
data during its first observer's warm-up while services and data were healthy
(a later check passed), and `20261009T122329Z-thor-latency-o6-policy-switched-mid-run`
spans the policy switch (Intel 12:24:38 UTC, Thor 12:28:59 UTC).

## Limits

Single robot, stationary, one camera profile and link; no detector or
exploration load. The diagnostic rows are single short runs. The O1 run used
continuous LAN calibration without PTP, so its cross-host figures carry that
method's bound (at most 0.2 ms).
