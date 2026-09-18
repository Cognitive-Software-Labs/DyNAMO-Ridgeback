# Operational incident history

Recorded dates: 2026-08-31, 2026-09-04, 2026-09-05, 2026-09-08, 2026-09-10, 2026-09-18

Tested revisions: unknown

Provenance: partial

This is dated evidence. Missing revisions or preserved worktree inputs limit
reproducibility; no new measurements were made during archive curation.

These are dated root-cause findings and measurements. They do not prescribe
recovery actions for the working tree.

## Detector throttle overshoot — fixed 2026-09-04

With `detector_fps: 5.0`, a 2026-09-03 concurrency sweep delivered a 242.0 ms
warm median interval (4.13 Hz). The worker advanced its cadence clock after
inference, making the cycle `configured period + inference`. Moving the advance
to immediately before the step made the period a floor on step starts:
`max(configured period, inference)`.

Stub measurements after the fix were 200.09 ms at 5 Hz, 100.05 ms at 10 Hz,
and 40.41 ms at a 40 Hz request with 40 ms inference. The diagnostic at that date kept `gate_wait` and `detector_step` separate and counted frames superseded by
the intentional latest-wins slot.

## slam_toolbox namespaced TF — patched

Stock `slam_toolbox` constructed a `TransformListener` with a hidden node. That
node subscribed to absolute `/tf` and `/tf_static`, bypassing the launch
remappings used by the namespaced Clearpath stack. Scans were filtered out and
no map appeared.

`patches/slam_toolbox_tf_namespace.patch` passes `shared_from_this()` to the
listener so it uses `slam_toolbox`'s node interfaces and honors the remappings.
TF relays, process-level remapping, and a larger scan queue did not fix the
ownership error.

## SLAM TF lag — 2026-09-10

A 40-minute warehouse run recorded 334 future-extrapolation failures between
MPPI's requested time and `map -> odom`: 25 ms median, 145 ms p90, 1.021 s p99,
and 1.714 s maximum. `slam_toolbox` also dropped 147 stale scans while configured
with an asynchronous queue of ten.

Setting `restamp_tf: true` and `scan_queue_size: 1` fixed timestamp ownership
and latest-scan back-pressure. A same-workload 15-minute validation reached
66.1% peak coverage with 12 successful goals and no extrapolations, controller
loop misses, aborts, blacklists, or node deaths. Eighty scans were deliberately
dropped when the depth-one queue was busy.

## D435 static camera transform — removed 2026-08-31

Simulation previously added a hand-written
`camera_0_link -> camera_0_color_optical_frame` transform using the D435 color
offset, while the configured device was a D455. The render sensor also sat on
`camera_0_link` while labeling output as the optical frame.

The August 31 correction assigned nominal simulation joints to the camera model, left hardware calibration with
the driver, and attached rendering to the color frame. This removed both the copied D435 constant and the
render-pose/frame mismatch.

## Base-frame fallback latency — fixed 2026-08-31

The TF helper formerly gave both a configured frame and its final path segment
the full 0.5 s timeout. In simulation the namespaced configured frame was absent
while bare `base_link` was already buffered, so every mask batch paid the first
timeout. Measurements were 502.8–512.0 ms for the failing namespaced lookup and
0.021–0.036 ms for the valid fallback.

The August 31 fix probed candidates with zero timeout first and performed bounded
waits only when neither is buffered. It did not cache the fallback, allowing a
configured frame that appears later to become authoritative.

## Camera software-rendering collapse — measured 2026-09-05

On the same workload, `llvmpipe` produced 3.80 Hz camera output while forced
NVIDIA GLX produced 28.07 Hz. Video capture, detector diagnostics, detector
rate, stale processes, QoS, and CUDA availability were ruled out. This is why
`tools/gpu-run` is the normal GUI-capable path and headless EGL remains a
separate server-only option.

## Long Gazebo GUI sweep failure — observed 2026-09-08

A 15-configuration sweep lost Gazebo in the GUI render thread around
configuration 9 after approximately 6.4 hours. Configuration durations climbed
from 0.43 to 0.93 hours while RTF stayed above 0.97 and memory remained stable.
The supervisor correctly retained completed `run.json` results, but could not
make progress against the dead simulator. This established rising wall time,
not short-window RTF, as the useful early warning and motivated resumable split
runs for long matrices.

## Hokuyo reconnect lockout — 2026-09-18

Intel's robot services moved from Fast DDS to CycloneDDS at 17:39 UTC with
`tools/intel_thor/intel_services_rmw apply`. The branch was
`feat/real-hardware-exploration` with the switch tooling uncommitted; it was
later committed as `0600400a`. Both `urg_node` drivers (`ros-jazzy-urg-node`
1.1.2) reconnected and logged `Streaming data` at 17:39:43. At 17:43:53, both
began logging `Could not grab single echo scan`, hit `Error count exceeded
limit, reconnecting`, and then logged `Error connecting to Hokuyo: Could not
open network Hokuyo` every 2.5 s. Over 17:39–17:48 there were 20 grab failures
and 203 connection errors, compared with none between 14:50 and 17:39 under
Fast DDS.

The Hokuyos answered ping. `ss` showed each driver holding an established
session with a nonzero receive queue while a second connection sat in
`SYN-SENT`. The driver had opened a new session without closing the old one,
and a UST serves one client. There were no kernel or link events, and the
Ethernet ports were idle, which ruled out a traffic flood. The vcan receive
timeouts and Foxglove schema errors in the same window already occurred at the
same rates before the switch.

The failure coincided with verification commands under CycloneDDS: `ros2 daemon
stop`, `ros2 node list --no-daemon`, and `ros2 topic hz` on the scan topics.
`sudo systemctl restart clearpath-sensors` at 17:48:17 restored both drivers.
The following did not reproduce the stall:

- three minutes of passive observation
- a best-effort `topic hz` subscriber
- `node list`
- a reliable `topic echo` subscriber killed with SIGKILL
- a full `check_link` run, including the 30 Hz cross-host stream

The trigger remains unknown. The lockout mechanism is a driver defect that is
independent of the middleware.
