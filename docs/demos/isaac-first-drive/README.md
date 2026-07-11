# Isaac first-drive demo (P4, 2026-07-11)

`index.html` is a self-contained snapshot (images embedded) of the first
scripted drive on the Isaac Sim 6.0 port: 36 s through `mock_hospital`,
RTX lidar into slam_toolbox, noisy wheel odom + IMU fused by the EKF,
ground truth alongside. Open it in any browser — no server needed.

Headline numbers from that run: EKF within 2.0 cm of ground truth after
the drive, 14/14 contract topics live from the Isaac include alone,
1758 occupied map cells from one short loop.

## Regenerate

```bash
# terminal 1 — sim (add headless:=false for the Isaac GUI window)
ros2 launch install/ridgeback_autonomy/share/ridgeback_autonomy/launch/includes/simulation_isaac.launch.py world:=mock_hospital

# terminal 2 — slam
ros2 launch install/ridgeback_autonomy/share/ridgeback_autonomy/launch/includes/slam.launch.py use_sim_time:=true setup_path:=$PWD/clearpath/

# terminal 3 — scripted drive + capture (PNGs land next to the script)
python3 tools/isaac/demo_drive.py
```

All terminals need the workspace sourced and the repo's CycloneDDS env
(`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file://$PWD/cyclonedds.xml`).

## Watch it live instead

```bash
rviz2 -d src/ridgeback_autonomy/sim/rviz/exploration.rviz \
  --ros-args -r /tf:=/r100_0001/tf -r /tf_static:=/r100_0001/tf_static -p use_sim_time:=true
# drive it yourself (plain Twist is accepted on the TwistStamped topic):
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/r100_0001/cmd_vel
```
