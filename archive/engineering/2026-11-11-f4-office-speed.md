# Following speed and heading in the explainer office

Recorded dates: 2026-11-11
Tested revisions: `0b22d72604c51a5ed7c5303f493a24a3898114b3`
Provenance: partial

The checkout was dirty: it carried the follow Nav2 tuning, collision-monitor
stop zone and leader-facing turn control this record tested. Configuration
changed between runs as listed; source snapshots were not preserved.

## Method

Each run used `--scene-profile realistic --environment modern_office
--scenario explainer --drive` (eleven walkers, 24 s, real-time factor 0.5)
with `follow_leader.launch.py`, one run per row, via the
[run script](assets/2026-11-11-f4-office-speed/run_follow.sh). The
[recorder](assets/2026-11-11-f4-office-speed/recorder.py) logged leader states,
follow status and the `cmd_vel_nav` and final `cmd_vel` commands. The
[analysis](assets/2026-11-11-f4-office-speed/analyse.py) labels each `LOCKED`
target against simulator truth, nearest person within 0.75 m with 0.1 m
separation. It assumes odometry equals the world frame from the origin spawn.
Robot–leader distances use simulator truth.

## Results

| Run | Configuration | Median command | On leader | Wrong / unmatched | Median distance | Final distance |
|---|---|---:|---:|---:|---:|---:|
| Isaac | Committed follow config | 0.58 m/s | 19.8 s | 0 / 0 s | 7.0 m | 8.7 m |
| Gazebo | Committed follow config | 0.52 m/s | 10.8 s | 0.8 / 6.6 s | 6.9 m | 8.7 m |
| Isaac A | `vx_std` 0.3, PathFollow 10 (offset 8), PathAlign 5 | 0.81 m/s | 21.8 s | 0 / 0 s | 4.0 m | 3.8 m |
| Isaac B | A plus stop zone | 0.80 m/s | 21.3 s | 0 / 0 s | 4.1 m | 4.5 m |
| Gazebo B | A plus stop zone | 0.79 m/s | 8.0 s | 0.5 / 3.3 s | 5.0 m | 13.2 m |
| Gazebo C | B with PathAlign restored to 10 | 0.85 m/s | 7.3 s | 1.3 / 3.9 s | 3.9 m | 10.0 m |
| Gazebo D | C plus leader-facing turn control | 0.84 m/s | 8.4 s | 0.9 / 2.4 s | 4.4 m | 12.2 m |
| Isaac D | C plus leader-facing turn control | 0.78 m/s | 20.6 s | 0 / 0 s | 4.5 m | 4.6 m |

Per-run [results](assets/2026-11-11-f4-office-speed/results.txt), truth,
manifests and compressed event logs are kept. In the committed configuration,
MPPI's own `cmd_vel_nav` never exceeded 0.65 m/s. The smoother and collision
monitor reduced only 11 of 1,280 commands by more than 0.1 m/s. The
exploration critic weights, not the 0.9 m/s limit, held the robot near
0.6 m/s.

In Isaac A the look-alike walked head-on through the robot's lane and came
within 0.26 m of its centre while the robot still moved at 0.2–0.5 m/s. The
collision monitor's six-point threshold did not register legs at lidar height.
With the stop zone and a two-point threshold (B, D), the robot stopped about
0.44 m before the look-alike reached it and moved under 0.1 m while the
non-reacting scripted walker passed through.

In Gazebo B the robot's heading drifted up to 50° away from the leader and the
lock was lost. Leader-facing turn control (D) kept the true bearing within
about ±5° until the loss. The lock still dropped repeatedly through
`track_discontinuity` and `target_missing` at 3–4 m, and it did not recover
after 12.9 s, when other walkers passed near the leader. Isaac did not show
this.

## Limits

Single runs at half real time. The Gazebo wrong and unmatched labels
include sideways offsets that grow with distance. In earlier Gazebo runs the
locked track stayed the leader's from the start, consistent with odometry
heading drift rather than a wrong person, but that is unverified, so zero
wrong-follow time is not established for Gazebo. The cause of the Gazebo
continuity losses is not yet identified. Clearance values include scripted
walkers that pass through a stopped robot.
