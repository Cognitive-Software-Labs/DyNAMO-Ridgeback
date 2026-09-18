# Gazebo measured-lidar integration check — 2026-09-18

Recorded dates: 2026-09-18

Tested revisions: unknown

Provenance: partial

This is dated evidence. Missing revisions or preserved worktree inputs limit
reproducibility; no new measurements were made during archive curation.

The measured lidar mounts passed a live Gazebo consumer integration check in
`mock_hospital`: both raw scans remained clean during stationary, forward,
turning, and stopping phases; collision monitoring passed the requested motion;
SLAM consumed the front scan and continued publishing maps.

## Run and evidence

Base revision: `701725cddd9fcc76bf64c8a7e6e8b21846498424`, with pre-existing uncommitted Isaac qualification and
shared launch changes. This validates the working tree and installed workspace
used for this run, not a clean checkout of that revision. No product code was
changed for the test.

Workspace sourced from `/opt/ros/jazzy/setup.bash` and `install/setup.bash`;
`ROS_DOMAIN_ID=87`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. The probe used
`src/ridgeback_autonomy/config/cyclonedds.xml` for participant discovery.

```bash
ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py \
  backend:=gz world:=mock_hospital setup_path:="$PWD/clearpath" \
  headless_rendering:=true exploration_rviz:=false \
  target_localization_enabled:=false autonomous_motion_enabled:=false \
  coverage_overlay_enabled:=false
```

Evidence is under `artifacts/exploration/gz_lidar_check_20260918/`:
`live_console.log`, `probe.py`, `probe.json`, and ROS logs. The first sandboxed
launch could not enumerate network interfaces; the live run used host access.
Two probe startup failures (DDS participant limit and integer command fields)
were corrected before the successful measurement; neither sent motion.

## Results

| Gate | Observation | Result |
|---|---|---|
| Stationary scans | 158 messages per lidar; 540 bins; ±135°; zero bins below 0.8 m | Pass |
| Forward scans | 156 messages per lidar; zero bins below 0.8 m | Pass |
| Turning scans | 156 messages per lidar; zero bins below 0.8 m; nearest return 1.289 m | Pass |
| Collision monitor | Active; all 194 forward and 194 turn output messages nonzero at requested speeds | Pass |
| Forward motion | 0.1 m/s request for 4 wall seconds; odometry displacement 0.381 m | Pass |
| Turning motion | 0.15 rad/s request for 4 wall seconds; odometry yaw change 0.579 rad (33.2°) | Pass |
| Stop | Zero command sent for 3 seconds; final phase translation 2.6 micrometres after initial settling | Pass |
| SLAM consumer | Active; `scan_topic=/r100_0001/sensors/lidar2d_0/scan`; 31 map messages | Pass |
| Camera topic | 444 CameraInfo messages at 640×480 during probe | Pass |

The probe published to `cmd_vel_smoothed`, immediately upstream of collision
monitoring, and observed its `cmd_vel` output plus filtered odometry. Frontier
goals were disabled. No collision-state messages arrived during this no-stop
case; the passing evidence is the active lifecycle state, unchanged output
commands, and measured motion. A single in-flight turn output was received at
the transition into the stop phase.

This closes the narrowly scoped collision-monitor/front-scan SLAM integration
gate for these mounts in this scene. It does not establish exploration coverage,
map accuracy, obstacle-stop performance, or all-world navigation parity.
The earlier aperture correction was a separate investigation.
