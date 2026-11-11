# Isaac update-cost breakdown for the modern-office crowd

Recorded dates: 2026-11-11
Tested revisions: `8eb3c81ecec238af7206a2250b4e8193fe969856`
Provenance: partial

The checkout was dirty. It carried the uncommitted performance work described in the
[runtime record](2026-11-11-runtime-performance.md) plus the changes tested
here. The [patch](assets/2026-11-11-isaac-update-cost/working-tree.patch)
holds only the changes this record tested. The host was the shared
RTX PRO 6000 workstation with other users' sessions running. Driver and
simulator binaries are not preserved.

## Method

Each run played the first 10 seconds of the realistic modern-office
`explainer` scenario in Isaac Sim 6.1 through `crowd_runner`. A
[wrapper](assets/2026-11-11-isaac-update-cost/timed_run.py) timed each
`SimulationApp.update` call, measuring wall time and main-thread CPU time. It
also applied one variant at a time: removing sensors, freezing animation,
disabling the viewport, changing physics or renderer settings, or breaking
down the per-loop Python cost. The
[summariser](assets/2026-11-11-isaac-update-cost/summ.py) reports the final
6 wall-seconds of each run. Runs were headless unless marked GUI. The GUI run
used the user's xrdp display at the default 1280x720 window size.

## Findings

[All summaries](assets/2026-11-11-isaac-update-cost/summaries.txt) list every
variant. The main steps:

| Variant | Update median | Loops/s |
|---|---:|---:|
| Reference, all sensors | 31.5 ms | 23.7 |
| No camera or LiDAR | 20.7 ms | 32.8 |
| No sensors, people frozen | 18.5 ms | 38.0 |
| No sensors, people frozen, physics 30 Hz | 8.7 ms | 59.3 |
| No sensors, people frozen, CPU PhysX | 5.1 ms | 75.0 |
| CPU PhysX, all sensors, 30 Hz steps | 17.3 ms | 36.5 |
| CPU PhysX, all sensors, 60 Hz steps | 10.5 ms | 50.4 |
| Plus persistent ROS executors, 60 Hz steps | 9.9 ms | 75.3 |
| Same, GUI, 60 Hz steps | 12.9 ms | 56.3 |

- Disabling GPU dynamics was the largest single gain. With GPU dynamics, each
  of the four 120 Hz substeps per update waited on the GPU.
- Lighting, DLSS, asynchronous rendering, RaytracedLighting mode, window
  size, simplified colliders, distant-human LODs and Kit rate limits each
  changed the sensorless update by under 1 ms.
- Mesh animation of the eleven people cost about 1–2 ms per update.
- The [loop breakdown](assets/2026-11-11-isaac-update-cost/loop-breakdown.txt)
  showed `rclpy.spin_once` called up to 44 times per loop cost about 7.5 ms.
  Each call adds and removes the node from the global executor. A persistent
  executor that stops at the first empty wait cut it to about 0.6 ms.
- At 60 Hz steps the camera stayed at 30 Hz, the LiDARs at 40 Hz, and every
  run completed scene validation. Clock, odometry and TF doubled to the step
  rate. The [sensor health files](assets/2026-11-11-isaac-update-cost/)
  record the counts.

## Limits

Loops per second counts Isaac updates, which is the viewport rate. It does not
measure frames delivered to a remote browser. A GUI run before the change was
not repeated here; the earlier
[runtime record](2026-11-11-runtime-performance.md) measured 14.4 viewport FPS
for the reference scene. The runs were single, short and on a shared host, so
the figures are indicative rather than qualified. No recognition or tracking
quality was evaluated, but the sensor contract is unchanged at the default
step rate.
