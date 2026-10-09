# Intel–Thor camera transport candidates

Rejected or unselected ways to cut camera latency from Intel to Thor. Raw
640×480 colour and aligned depth at 30 Hz, over Ethernet with the services DDS
profile, met every working target in the
[transport reference](../physical/intel_thor_transport.md#sensor-timing-and-time-synchronization)
and was faster than each candidate below. Measurements are in the archived record.

## Do not enlarge the camera's DDS send buffer (O1)

A camera-only profile with a 16 MiB socket send buffer and matching
`wmem_max` did not lower colour or depth latency: publishing returned sooner,
but each frame then waited as long in the kernel's egress queue, since its
wire time is unchanged. Camera bursts in that queue delayed LiDAR scans to
Thor from about 2 ms to about 9 ms at p95.

Reconsider only with a measured case where the driver's publish blocks for
reasons other than wire time, and check scan delivery alongside camera totals.

## Do not raise CycloneDDS MaxMessageSize to 65500 (O2)

Not measured; skipped because it builds on O1's profile. It can save CPU at
most, since wire time is unchanged. A lost packet then discards a whole 64 KB
datagram, and so a best-effort frame, and larger bursts queue ahead of LiDAR
traffic.

## Do not lower the camera rate (O3)

Withdrawn by decision: the deployment runs at 30 Hz.

## Do not compress camera frames for latency (O4)

At VGA 30 Hz on 1 Gb/s, raw spends about 9 ms of colour latency on the
network. Compression removes most of that but costs more to encode on Intel
and decode on Thor. The best pair, NVENC H.264 colour decoded in software and
RVL depth, raised colour to about 34 / 49 ms (median / p95) and depth to about
28 / 32 ms, failing colour's 40 ms p95 target. PNG depth was slower still.
Two variants are unusable with Thor's Jazzy plugins: zstd depth decodes
without its header, and NVDEC H.264 holds four frames (about 137 ms) because
the plugin cannot set FFmpeg's `low_delay` flag. The
[sensor timing reference](../physical/intel_thor_sensor_latency.md#compressed-transport-candidates)
documents both and how the tooling selects decoders.

Reconsider compression when bandwidth rather than latency is the limit:
higher resolution, more cameras, or a slower link. The tooling
(`thor_decoder`, the collector's transport options and the NVENC settings) stays
in place for that. JPEG colour was not measured, and any lossy colour option
needs the detector compared on raw and decoded frames before adoption.

## Do not turn off the driver's colour/depth pairing (O6)

Considered, not measured. Per-frame driver metadata shows colour reaches
Intel about 6 ms after depth, and depth waits for it, so colour's arrival
already decides when both are published. Without pairing (`enable_sync`)
colour would publish no sooner, and aligning depth to colour still needs the
colour frame.

Reconsider only for a consumer that uses unaligned depth on its own.

## Archived evidence

- [r100_0160 Intel–Thor camera transport candidates](../../archive/engineering/2026-10-06-r100-0160-thor-transport-candidates.md)
