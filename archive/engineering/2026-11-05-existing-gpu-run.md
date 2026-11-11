# Full scene through the existing GPU launcher

Recorded dates: 2026-11-05
Tested revisions: `54654853fb3a49039b0f1698292f7bde03e0b81f`
Provenance: partial

## Method

The existing `/usr/local/bin/gpu-run` supplied NVIDIA GLX vendor selection and
PRIME render offload for the unchanged realistic modern-office `explainer`
scenario in Gazebo, with GUI and all sensors enabled. Eleven people followed
their 24-second routes. No sensors or scene objects were removed. Private ROS
domain 223 and the runner's private Gazebo partition isolated the run. Other
workstation workloads were not stopped, and no matched CPU-render baseline
was collected in this experiment.

[OpenGL output](assets/2026-11-05-existing-gpu-run/opengl.txt) identified the
NVIDIA RTX PRO 6000. The [measurement script](assets/2026-11-05-existing-gpu-run/measure.py)
subscribed to simulator statistics and observed crowd truth. The timing window
uses only scenario times 0.05 through 24.00 seconds, excluding shutdown messages
that continue beyond the route end. Sensor rates use message counts and stamp
spans; approximate wall rates multiply those by observed playback speed.
They are not instrumented viewport FPS or remote-stream FPS.

## Results and limits

The [summary](assets/2026-11-05-existing-gpu-run/summary.json) and
[raw timing trace](assets/2026-11-05-existing-gpu-run/performance.json) record:

- 23.95 seconds of observed scenario time in 41.53 wall seconds: 0.577x real time.
- Median server-reported real-time factor during that window: 0.459. This is a
  different statistic from total simulated time divided by total wall time.
- RGB and depth: 30.30 Hz in simulation time, approximately 17.48 Hz in wall time.
- Each lidar: 40.00 Hz in simulation time, approximately 23.07 Hz in wall time.
- Scene/sensor validation passed with 5,280 observed person samples and maximum
  pose error 2.3e-16 m/rad (rounded upper bound).

GPU selection was verified. The run did not establish 50-60 GUI FPS, nor a
speedup relative to a matched software-rendered run. Real-time playback still
was not reached under the observed machine load.

The base revision had uncommitted changes. The
[manifest](assets/2026-11-05-existing-gpu-run/manifest.json),
[patch](assets/2026-11-05-existing-gpu-run/package.diff) and
[runtime Python source](assets/2026-11-05-existing-gpu-run/runtime-source.tar.gz)
preserve the tested inputs; asset hashes are in the manifest and raw assets
remain under the local run. Full environment/driver reproduction is not
preserved, hence partial provenance. This is rendering/scene/sensor evidence,
not recognition, following-control or physical-robot validation.
