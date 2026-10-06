# Intel–Thor sensor latency qualification

This reference owns the sensor timing tools and their report semantics.
The [transport reference](intel_thor_transport.md) owns DDS settings and the
[robot-local deployment](robot_local_deployment.md) owns camera/time-service
installation. The [active plan](../plans/PHYSICAL_intel_thor_sensor_latency.md)
owns remaining live tuning and adoption.

## Measurement contract

`tools/camera_contract_check --timing` retains serialized image subscriptions
and decodes only image headers. Callback entry captures wall time before
parsing. Colour, aligned depth, CameraInfo, three hardware scans and dynamic
TF record RMW source/received timestamps. TF statistics cover individual
transforms, including multiple stamps within a single TFMessage.

| Component | Calculation |
|---|---|
| Driver | Intel source timestamp minus header stamp |
| Network + DDS | received timestamp minus source timestamp minus Thor–Intel offset |
| Executor | callback entry minus received timestamp |
| Total | callback entry minus header stamp minus Thor–Intel offset |

Components report sample count, median, p95, p99 and maximum in milliseconds,
using linear interpolation. Missing/zero RMW timestamps leave the affected
components empty. Negative samples are retained to expose timestamp faults.
Header stamps remain unchanged; they are driver time estimates rather than
independently verified acquisition timestamps.

`--host thor` requires an explicit namespace and does not read Intel's
`/etc/clearpath` defaults. `--clock-offset-ms` is **Thor minus Intel**;
`--clock-bound-ms` must be finite and at most 0.2 ms for cross-host components.
Without a qualifying bound, driver and executor components remain available;
network and total components are unresolved. Intel timing uses one host clock.
The caller must select the matching DDS environment and deployment domain.

`--samples-output` preserves raw timestamps for later clock validation.
Reports include exact pairing in both directions, observed rate, an expected
cadence deficit estimate, and RMW publication-sequence loss when supported.
The installed Jazzy RMW may omit sequence numbers; cadence deficit does not
prove network-only packet loss. The collector additionally compares Thor's
unique camera stamps with its simultaneous Intel observer within the common
stamp window. This removes subscriber start/end skew; frames missed by both
observers remain unobservable. Qualification uses this comparison when RMW
sequence numbers are unavailable, and leaves loss unresolved if there is no
common window. CPU busy percentage is host-wide; NIC byte
rates cover all traffic on each interface, without subtracting other workloads.

## Qualification collector

Run `tools/intel_thor/check_link` before deployment. The collector runs on Intel
with `/etc/clearpath/setup.bash` sourced and `ROS_DOMAIN_ID=0`. It requires
identical committed tooling on both hosts, with Thor using its separate latency
worktree. It runs Intel then Thor, three interleaved repeats by default, each
with 10 seconds of warm-up and 60 seconds of measurement. A simultaneous Intel
contract observer guards each Thor run; this adds a local camera subscriber,
which is recorded in provenance and must be kept constant for comparisons.

The collector owns the robot experiment lock. It stops the sequence on a
contract failure, a scan receipt gap over 250 ms on Intel, changed sensor/camera
PID or restart count, or a camera `[ERROR]` journal entry. The guard uses its
own subscriptions and system counters; it does not poll the ROS graph.

Before and after every observation, and continuously during it, `check_clock`
starts an ephemeral UDP echo peer through a persistent SSH connection, without
writing remote files. It takes at least 300 direct LAN round trips per burst,
then retries until the best half-RTT bound is at most 0.08 ms or the 0.75-second
retry budget expires. Selection depends only on clock uncertainty. The sampler
pauses 0.25 seconds between bursts; SSH stdin closure ends the peer. Standalone
`check_clock --transport ssh` retains the SSH echo method for diagnostics. Its midpoint estimates the actual ROS
wall-clock offset; half the round-trip duration bounds asymmetry. The collector
takes the smallest interval covering both adjacent offset uncertainty ranges,
uses its midpoint for each timestamp, and uses its half-width as uncertainty. It retains the complete clock series and every exchange.

Cross-host components require every interval's bound to be at most 0.2 ms,
calibration gaps no larger than two seconds, and no detected wall-clock step
relative to monotonic time (100 µs detection threshold). Each realtime read
is bracketed by monotonic reads; step detection uses the separation of the
resulting phase intervals, accounting for scheduling delays between reads.
The probe also checks every exchange for steps, including exchanges not selected
as a calibration point. Every received image
must lie inside the calibrated range. This assumes the offset stays within the
adjacent calibration envelopes; steps or excursions shorter than the sampling
interval can still be unobserved. Same-clock components remain available when
cross-host timing is unresolved. Clock calibration is part of the recorded
CPU/network load and remains identical for candidate comparisons.

Chrony's NTP time is a virtual clock and can differ from the system wall clock
while a correction is being slewed. A small Thor `chronyc tracking` offset does
not establish a small difference between Intel and Thor's ROS clocks; the
LAN monitor measures those clocks directly. See
[chrony's tracking documentation](https://chrony-project.org/doc/4.5/chronyc.html#tracking).

Time-service status, host CPU/NIC counters, guard reports, logs, source revisions
and raw samples stay under `artifacts/hardware/<UTC>-thor-latency-baseline/`.

The report's `working_targets_passed` checks VGA at 30 Hz with at least 59
seconds of observed data, network + DDS p95 at most 20 ms, total p95 at most
40 ms, loss at most 1%, exact pairing at least 99%, and the camera/scan contract.
The one-second duration tolerance admits scheduling at the window boundary.
Short or 15 Hz runs are diagnostic evidence and cannot pass the 30 Hz targets.
No configuration is adopted automatically. Commands live in the
[README](../../README.md#intel-thor-sensor-latency).

## Compressed transport candidates

The driver advertises every installed image_transport variant of colour and
aligned depth under the contract names, for example
`sensors/camera_0/color/image/compressed`, and encodes one only while it has a
subscriber. On Thor, `tools/intel_thor/thor_decoder` starts one Jazzy
`image_transport` republisher per compressed stream and publishes the decoded
`Image` under `sensors/camera_0/thor/<stream>/image`, so measured frames never
share a topic with Intel's raw publisher. Decoded frames keep the source header.

`camera_contract_check --host thor --color-transport T --depth-transport T`
applies the contract checks and pairing to the decoded topics and additionally
subscribes to each compressed variant. Colour offers `compressed` (JPEG) and
`ffmpeg` (H.264); aligned depth offers the lossless `zstd` and
`compressedDepth` (PNG, or RVL through its `format` parameter). Their timing
splits by matching header stamps:

| Component | Calculation |
|---|---|
| Encode | Intel publish of the compressed message minus header stamp |
| Network + DDS | Thor receipt of the compressed message minus that publish minus offset |
| Decode | decoder's publish minus the observer's receipt of the compressed message (Thor) |
| Delivery | callback entry minus the decoder's publish (Thor) |
| Total | callback entry minus header stamp minus offset |

Decode approximates the decoder's own receipt with the observer's, since both
Thor subscribers receive the same sample. The second subscriber also makes
CycloneDDS send each compressed frame to Thor twice, which is small next to the
raw frame it replaces. Coverage then requires a complete total and network
samples for at least 99% of decoded frames.

The collector's `--color-transport` and `--depth-transport` start the decoder
over SSH before each Thor observation and stop it afterwards; a decoder exit
during the run stops the sequence. `--camera-param NAME=VALUE` sets a camera
encoder parameter once before the repeats and restores the previous values at
the end. This is needed for the JPEG, PNG/RVL and zstd parameters, whose names
begin with a dot and so cannot be set from a parameters file. The H.264
settings live in `config/d455.yaml` (NVENC, no B-frames, 15-frame keyframe
interval); the ffmpeg encoder reads them when its first subscriber appears.
Runs are written to `<UTC>-thor-latency-o4-<colour>-<depth>/`.
`--decoder-param NAME=VALUE` passes a parameter to the Thor republishers.

Two decoder behaviours found on 2026-10-06 (Thor, `image_transport_plugins`
4.0.7, `ffmpeg_image_transport` 3.0.4) decide which variants are usable:

- **zstd loses the header.** Decoded depth arrives with stamp 0 and an empty
  frame ID, although the compressed messages carry correct stamps, so the
  contract check fails it on duplicate stamps and pairing. Use
  `compressedDepth` for lossless depth.
- **NVDEC holds four frames.** The default H.264 decoder on Thor is
  `h264_cuvid`, which FFmpeg runs with a four-frame display delay unless the
  codec context's `low_delay` flag is set. The plugin passes only the
  decoder's own options, so that flag cannot be set (`Option not found`) and
  decoding adds about 137 ms. The software decoder avoids it:
  `--decoder-param in.ffmpeg.decoders.h264=h264`.

## Limits and pending adoption

As inspected on 2026-10-06, the running camera remains on the previously
qualified VGA 30 Hz service profile. The new camera DDS profile is an opt-in
candidate. Chrony was installed by the operator on both hosts on
2026-10-06; live verification and measured transport selection are pending. Raw/compressed comparisons, detector accuracy under lossy colour,
CPU/NIC tuning and a camera move to Thor remain in the active plan.
