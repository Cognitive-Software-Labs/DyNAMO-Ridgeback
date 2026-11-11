# Closed-loop leader following in Gazebo

Recorded dates: 2026-11-11
Tested revisions: `bebcb9873dd1d802b4392b7bf58d32445b0a2d1a`
Provenance: partial

The checkout was dirty: it carried the uncommitted Gazebo `--drive` option
that this record tested. The `follow_controller`, follow Nav2 configuration and
launch were as committed in the tested revision, with their default settings.

## Method

The [run script](assets/2026-11-11-f4-gazebo-follow/run_follow.sh) started
`crowd_scene --backend gz --scene-profile f3 --scenario leader_alone --drive`,
headless, together with `follow_leader.launch.py
odom_topic:=platform/odom/filtered`, in one private ROS domain. The generated
world's real-time factor was 0.5. The crowd started once the follow stack
published plus a 3-second settle. Robot truth came from the `r100_0001/robot`
model pose on Gazebo's world pose stream. Distances use the same metrics script
as the [Isaac record](2026-11-11-f4-first-follow.md), after the first 3 seconds.

## Result

| Simulator | Median distance | Min | Max | Robot end |
|---|---:|---:|---:|---|
| Gazebo | 1.58 m | 1.21 m | 1.92 m | (6.72, 0.57) m, yaw 0.99 rad |

The set follow distance was 1.2 m. The robot followed the leader 6 m along +X
and through the left turn, with a peak forward command of 0.59 m/s. The
[follow status](assets/2026-11-11-f4-gazebo-follow/gz-alone-r1/follow_status.csv)
was `stopped` only on the first tick, before the lock formed. Per-run
[truth, manifest](assets/2026-11-11-f4-gazebo-follow/gz-alone-r1/) and
[metrics output](assets/2026-11-11-f4-gazebo-follow/metrics.txt) are kept.

## Limits

One run of one single-person scenario at half real time. Wrong-follow time is
trivially zero with one person. The multi-person scenarios, repetitions and
the plan's eight metrics remain open.
