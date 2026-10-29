# r100_0160 front LiDAR session reset with the patched driver (S3)

Recorded dates: 2026-10-14
Tested revisions: unknown
Provenance: partial

The S3 check of the in-driver reconnect fix on the front Hokuyo. The rear-LiDAR
reset and the fix itself are in the
[urg_node reconnect record](2026-10-13-urg-node-reconnect-leak.md). Provenance
gaps: the run names no repository commit for `observe_lidar_interruption.py`;
the driver ran as the `/home/robot/ros_overlays/urg_node_fix` overlay built
from `patches/urg_node_reconnect_recovery.patch`, and the raw report lives only
on the robot host. Extracted from the project backlog at the
[documentation snapshot](https://github.com/Cognitive-Software-Labs/DyNAMO-Ridgeback/blob/c0dd50f13f38de4433aa9ee7ede32736d1c73f72/docs/BACKLOG.md#hokuyo-driver-recovery-after-read-timeouts).

## Run

With the base e-stopped, `tools/hardware/observe_lidar_interruption.py` reset
the front LiDAR's TCP session. The same driver process reconnected after 2.25 s
with no service or scan-merger restart. The rear scan never gapped more than
42 ms. Over the 30 s after recovery, the front, rear and merged scans each held
39.93 Hz with gaps up to 43 ms. Report:
`artifacts/hardware/20261014T114600Z-s3-front-lidar-interruption/`.

An earlier attempt the same day is kept as `…-INVALID-settle-window`: its
recovery was identical, but the tool's settle window overlapped the outage, so
it is not counted.

## Limits

This qualifies recovery from a reset session only. A sensor that answers
partially was shown offline only, and the 2026-09-18 simultaneous front/rear
timeout remains unexplained; no loaded multi-hour soak ran.
