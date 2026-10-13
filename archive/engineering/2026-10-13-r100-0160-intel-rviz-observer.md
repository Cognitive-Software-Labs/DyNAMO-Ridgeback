# r100_0160 Intel RViz view of Thor localization

Recorded dates: 2026-10-13
Tested revisions: `5751d96ba264f7456ddb3b3cbf44c9659d749708`
Provenance: partial

Both hosts ran `5751d96` from fresh prefixes. On Intel this was the worktree
`/home/robot/DyNAMO-Ridgeback-rviz`, prefix `artifacts/colcon/intel-rviz-install`,
with `slam_toolbox` 2.8.5 (`02afdde`) built from source with
`slam_toolbox_tf_namespace.patch`. On Thor it was `~/DyNAMO-Ridgeback-rviz`,
prefix `artifacts/colcon/thor-install`. Raw evidence was not committed. On Intel it
is under `artifacts/hardware/20261013T092633Z-intel-rviz-observer/`
(`log.md`, the step-by-step operator log, and `rviz_1.png`). On Thor it is under
`artifacts/hardware/20261013T092633Z-thor-localization/` (`launch.txt`,
`launch.log`). The Intel node output went to the operator's terminal and was not
captured. No target was placed at a taped distance.

## Configuration

Robot stationary with the e-stop engaged, domain 0, CycloneDDS, both hosts
under `dds_env.sh`.

- Thor: `localization.launch.py namespace:=r100_0160 base_frame:=base_link`
  with the camera_0 colour, aligned-depth and CameraInfo topics,
  `projective_ranging,euclidean_reconstruction`, `depth_source:=stereoscopic`,
  `mask_gate:=box`, `target_labels:="full body person"`,
  `detection_threshold:=0.26` (the 2026-10-12 choice), `displays:=false`. OWLv2
  loaded on CUDA.
- Intel, operator-started on the VNC display `:1`:
  `ridgeback_exploration.launch.py backend:=hardware localization_mode:=external
  autonomous_motion_enabled:=false base_frame:=base_link
  estimators:=projective_ranging,euclidean_reconstruction`. It ran SLAM, Nav2,
  RViz, the target rings, the HUD and the overlay. `base_link` was read from the
  live TF (`odom → base_link`).

Intel was rebooted at 09:33:20 UTC by a `sudo reboot` from another terminal,
not as part of this test. The Intel launch was restarted at 09:36 with the same
arguments. Thor's launch was not restarted. All results below come from after
the restart.

## Results

- SLAM: the patched `slam_toolbox` built and published `map` at about 2 Hz
  (103×167 cells at 0.05 m before the reboot). The front scan ran at 40.3 Hz.
- Health: Intel's `localization/status` reported `processing`. Thor's health
  showed stereoscopic depth and the box gate.
- Rates on Intel (`ros2 topic hz`, 20-message window): detections 1.47 Hz,
  mask measurements 2.13 Hz, target markers 4.99 Hz. A Thor-local echo shortly
  before showed a detection about every 0.30 s. The Intel figures were taken
  shortly after Intel's boot (load average 15) and were not repeated.
- Image traffic: Thor's `debug/target/mask/aligned_depth` had no subscribers.
  The overlay read Intel's local depth. `debug/target/mask` had no publisher,
  because the mask node creates it only for the silhouette gate. With the box
  gate, no image crossed from Thor. Over 10 s, Thor sent 352 kbit/s on Ethernet
  and 6 kbit/s on Wi-Fi.
- RViz (screenshot at 09:40 UTC, 31 fps): it showed the map, both scans, the robot
  model, a velocity HUD at zero, and the target HUD with projective 1.998 m and
  Euclidean 1.982 m, both 0.6 s old. The overlay strip had the RGB box and
  estimates (score 29 %, lateral about −1.0 m, forward 1.7 m), Intel's stereo
  depth, and the box mask, for a seated person at the right image edge.

## Findings and limits

- The overlay redraws once per measurement, on the newest colour frame Intel
  holds, not the frame Thor measured. Boxes therefore trail a moving target by
  the inference latency. The depth panel also shows the newest frame.
- The ROS 2 CLI misreports on these hosts in two ways. Piped output under
  `timeout`'s default SIGTERM is lost before the buffer is written; use
  `timeout -s INT` and `PYTHONUNBUFFERED=1`. `ros2 topic hz` reports nothing for
  best-effort 640×480 images, while `echo --field header` counted colour at
  about 26 Hz.
- `tools/intel_thor/check_link` ran after the Thor launch, not before it. All
  checks passed except "Thor -> Intel: 29.6 Mbit/s on Wi-Fi". The measurement
  during the run (Thor Wi-Fi tx 6 kbit/s) does not reproduce duplication by the
  localization processes. The probe's failure was not explained. Its revision
  warning compared Thor's main checkout (`950aacb`), not the deployed worktree.
- Nav2 ran on hardware with its output connected to the twist mux. No goal was
  sent. The RViz *2D Goal Pose* tool would command motion.
- This was not checked: distance accuracy against tape, ring placement in
  `map`, sustained rates, and the silhouette gate's mask transfer.
