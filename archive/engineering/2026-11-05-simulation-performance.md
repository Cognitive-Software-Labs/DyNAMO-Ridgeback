# Simulation performance investigation

Recorded dates: 2026-11-05
Tested revisions: unknown
Provenance: partial

## Measurements and retained changes

Short two-second runs used eleven people in the restored modern office and
an RTX PRO 6000 Blackwell GPU. Each timing records the animation, render/sensor
and ROS monitoring portions of a simulation step, excluding truth export,
startup and explicit GUI pacing. Means discard the first ten frames. They
are loop-rate estimates, not display/remote-desktop FPS, and are too short
to establish a sustained frame-rate guarantee. Other GPU workloads were
present, so small differences are not a controlled hardware comparison.

| Variant | Mean timed work per step | Inverse mean |
|---|---:|---:|
| Original GUI viewport options, optimized sensor implementation | 70.27 ms | 14.23 Hz |
| Smaller viewport options | 66.54 ms | 15.03 Hz |
| Explicit 640x360 preview, indirect diffuse/reflections disabled | 98.00 ms | 10.20 Hz |
| Smaller viewport plus persistent ROS monitor executor | 66.65 ms | about 15 Hz |

The explicit fast preview performed worse and was removed. Lighting effects
are retained. The persistent executor avoids repeated node attachment but
showed no clear timing gain. The retained viewport requests 960x540, real-time
RTX, DLSS performance mode and a 1280x720 window. Do not infer a large FPS gain
from these results. Headless original and optimized runs remained near 24 Hz.

The D455 alignment previously sorted roughly 300,000 depth samples per frame.
A two-pass minimum buffer now selects nearest depth and earliest source index
for ties. In a seeded uniform-random 640x480-depth test with fixed noise,
20-call mean alignment time changed from 31.6 ms to 7.2 ms; depth and XYZ output
arrays matched exactly. That microbenchmark is not representative of every
smooth depth image and did not translate into a large observed whole-loop gain.
A regression checks stable ties, holes and empty inputs against sorted selection.

Realistic Gazebo actor writes are limited to 40 Hz rather than every 1 kHz
physics tick. Baseline/F3 cadence is retained. This reduces redundant actor
updates by a factor of 25; no before/after Gazebo GUI FPS was measured, so
this is not a measured 25-fold rendering speedup. Paths and sensor dimensions
are retained. Short scene/sensor validation covers the changed runtime.

## Evidence and limitations

The [source snapshot](assets/2026-11-05-simulation-performance/sources.tar.gz)
preserves the retained implementation. Frame timings for each experiment
are adjacent files in that asset directory; raw runs remain under
`artifacts/leader-following/fps-profile/`. Testing used a dirty tree; a single
exact revision and full environment are not established, hence partial
provenance. GUI capture/streaming was not profiled. Full 24-second playback
and perception-performance requalification were not part of this experiment.

## Regression rollback

On 2026-11-05 the user reported approximately 7-8 display FPS versus 13-14
previously. The earlier short loop timings had not established a sustained
GUI gain. Renderer overrides, actor throttling, the depth-alignment replacement
and persistent monitor executor were rolled back; timing instrumentation was
retained. Concurrent simulator/detector and GPU workloads from other checkouts
were observed but were not stopped. Their contribution was not quantified.
Restoring defaults does not establish that the user's display FPS recovered.
The human paths and office restoration were preserved.

## Preview experiments withdrawn

Further visual-only experiments did not meet the later 50-60 FPS target with
acceptable imagery. Minimal-render runs reached about 30 Hz but produced black
images and were rejected. The rasterized Storm path encountered GLX, sensor
multi-tick and scene-delegate incompatibilities; hardware OpenGL was available
through locally extracted VirtualGL, but a visible Storm run remained slow.
Static office batching retained geometry but did not establish the requested
speed. The preview CLI, renderer configuration and temporary code were removed
from the application after the user reported further regression. Original
full-mode rendering, office layout and approved choreography were preserved.
All test GUI processes had exited at the final process check. No unrelated
workloads were stopped. Locally downloaded packages remain only as ignored
tool/cache files and do not activate during normal scene launch. Raw trials
and abandoned code remain under `artifacts/leader-following/visual-only-performance/`.

The subsequent GPU-launch diagnostic verified NVIDIA OpenGL via a locally
extracted VirtualGL/EGL wrapper. The [renderer output](assets/2026-11-05-simulation-performance/opengl/nvidia-egl.txt)
and [package receipt](assets/2026-11-05-simulation-performance/opengl/virtualgl-receipt.json)
preserve that check. Unwrapped remote display `:1` reported Mesa llvmpipe with
`Accelerated: no`. This establishes different OpenGL backends, not application
FPS or a quantified explanation for Isaac's performance. The launcher was
checked without opening another scene; the 50-60 FPS target remains unverified.
