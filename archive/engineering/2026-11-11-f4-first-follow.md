# First closed-loop leader following in Isaac

Recorded dates: 2026-11-11
Tested revisions: `8f870bf59439ffd8ad323fee7be254e9774e4a4c`
Provenance: partial

The checkout was dirty: it carried the uncommitted `follow_controller`,
follow Nav2 configuration, launch file and the crowd runner's `--drive`
option that this record tested. The values changed between runs are listed
below; source snapshots were not preserved.

## Method

Each run used the Isaac crowd runner with `--scene-profile f3 --scenario
leader_alone --drive --real-time-factor 0.5`, headless, in a private ROS
domain, together with `follow_leader.launch.py`. The
[run script](assets/2026-11-11-f4-first-follow/run_follow.sh) started both.
People held their start poses until the follow stack published, then the
20-second scenario played. The leader starts 1.5 m ahead, walks 6 m along +X
at 0.5 m/s, then turns left. The [metrics script](assets/2026-11-11-f4-first-follow/metrics.py)
compares `robot_truth.csv` with the simulator's leader truth after the first
3 seconds.

## Results

| Run | Change | Median distance | Min | Max |
|---|---|---:|---:|---:|
| r1 | First run, MPPI forward limit 0.5 m/s | 2.17 m | 1.53 m | 2.26 m |
| r2 | Forward limit 0.9 m/s | 2.18 m | 1.53 m | 2.23 m |
| r3 | Goal leads leader velocity by 1.0 s | 1.67 m | 1.29 m | 1.83 m |
| r4 | Lead 1.5 s, hold band 0.15 m (now defaults) | 1.47 m | 1.13 m | 1.79 m |

The set follow distance was 1.2 m. In every run the robot followed the leader
from the origin to about (6.5, 0.4) m and turned with it. The
[follow status](assets/2026-11-11-f4-first-follow/alone-r4/follow_status.csv)
was `stopped` only for the first two ticks, before the lock formed.

Raising the speed limit alone did not help: the peak command stayed at
0.51 m/s. The goal trailed the perceived leader, so the controller only
matched the leader's speed. Leading the goal by the leader's smoothed velocity
closed most of the gap. Per-run [truth, manifests and status](assets/2026-11-11-f4-first-follow/)
and [metrics output](assets/2026-11-11-f4-first-follow/metrics.txt) are kept.

## Limits

This covers one scenario with one person, one run per setting, Isaac only, at
half real time. Wrong-follow time is trivially zero with a single person.
Gazebo driving, the five multi-person scenarios, repetitions and the plan's
eight metrics remain open.
