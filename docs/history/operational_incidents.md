# Operational incident history

These dated investigations explain why current safeguards exist. They are not
the current troubleshooting entrypoint; use [troubleshooting](../troubleshooting.md)
for symptoms and recovery steps.

## Detector throttle overshoot — fixed 2026-09-04

With `detector_fps: 5.0`, a 2026-09-03 concurrency sweep delivered a 242.0 ms
warm median interval (4.13 Hz). The worker advanced its cadence clock after
inference, making the cycle `configured period + inference`. Moving the advance
to immediately before the step made the period a floor on step starts:
`max(configured period, inference)`.

Stub measurements after the fix were 200.09 ms at 5 Hz, 100.05 ms at 10 Hz,
and 40.41 ms at a 40 Hz request with 40 ms inference. The current diagnostic
keeps `gate_wait` and `detector_step` separate and counts frames superseded by
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

The camera model now owns its nominal fixed joints in simulation, the hardware
driver owns calibrated extrinsics on a robot, and the render sensor attaches to
the model's color frame. This removed both the copied D435 constant and the
render-pose/frame mismatch.

## Base-frame fallback latency — fixed 2026-08-31

The TF helper formerly gave both a configured frame and its final path segment
the full 0.5 s timeout. In simulation the namespaced configured frame was absent
while bare `base_link` was already buffered, so every mask batch paid the first
timeout. Measurements were 502.8–512.0 ms for the failing namespaced lookup and
0.021–0.036 ms for the valid fallback.

The helper now probes candidates with zero timeout first and performs bounded
waits only when neither is buffered. It does not cache the fallback, allowing a
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
