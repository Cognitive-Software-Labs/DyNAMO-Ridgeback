# Predicting bystanders during leader following

Recorded dates: 2026-11-12
Tested revisions: `0894eda09c2122c55a733ffaa02b08017a5f1bf1`
Provenance: partial

The checkout was dirty: it carried `people_predictor`, the lidar mover tracker,
the people costmap layers and the lock changes this record tested. Gazebo runs
used the [recorder](assets/2026-11-12-f4-people-prediction/recorder.py) and
the [run summary](assets/2026-11-12-f4-people-prediction/summary.py), which
calls [analyse.py](assets/2026-11-12-f4-people-prediction/analyse.py) and
[nav_check.py](assets/2026-11-12-f4-people-prediction/nav_check.py). The
host was shared with an unrelated job (load 28–45 on 32 cores) for most runs.

## Symptom

In the Gazebo modern-office crowd (`v7-gz-crowd-s0-96`) the robot passed
0.27 m from `lf_crowd_0`, a walker crossing at about 1.35 m/s from outside the
camera's ±43° view, at about 1.4 m/s. The tracker created people only from
camera detections, so lidar-only walkers never reached Nav2 or the speed
limit.

## Change

- The tracker follows unmatched lidar detections with a constant-velocity
  Kalman filter and publishes those that walk (at least 3 hits, 0.3 m moved,
  0.3 m/s) on `leader_follow/moving_people`. They serve collision avoidance
  only and are never leader candidates.
- `people_predictor` marks bystanders' current and predicted positions in a
  Nav2 people layer and publishes a speed limit. It rolls each candidate speed
  out with the 1 m/s² acceleration limit over 3 s and picks the one that keeps
  the robot clearest. A predicted contact (under 0.7 m) is scored by impact
  speed, so an unavoidable contact is met slowly. Lidar walkers within 1 m of
  the leader's last track are excluded.
- The follow costmaps use fully qualified observation topics. Jazzy resolved
  the relative lidar topics inside the costmap namespace, where nothing
  publishes.

## Offline crossing grid

The [crossing grid](assets/2026-11-12-f4-people-prediction/crossing_grid.py)
drives a point robot along a line at 0, 0.5, 1.1 or 1.3 m/s past one walker at
0.5, 0.9 or 1.35 m/s, crossing at offsets from −2 to +2 m: 204 crossings. The
cap starts after 0.3 s of perception delay.

| Speed limit | Contacts (< 0.7 m) | Mean robot speed at contact |
|---|---:|---:|
| None | 110 | 1.29 m/s |
| First rule (brake for the closest approach) | 80 | not recorded |
| Rollout with impact scoring (committed) | 30 | 0.37 m/s |

At least 24 of the 30 remaining contacts cannot be avoided by speed alone:
the walker reaches the robot's line before braking can matter. Two
urgency-weighted scores reached 30 and 33 contacts at 0.89 and 0.87 m/s
impact speed and were not adopted.

## Gazebo runs

All runs are in the modern office at real-time factor 0.5.

| Run | On leader | Wrong | Lock off | Closest bystander | Lowest speed limit |
|---|---:|---:|---:|---:|---:|
| `v7-gz-crowd-s0-96` (before) | 19.7 / 20 s | 0 | 0.2 s | 0.27 m | 0.15 |
| `v9-gz-office-crowd-1` | 8.2 / 20 s | 0 | 11.7 s | 1.79 m | 0.10 |
| `v9-gz-wander-s0-1` (seed 0) | 38.7 / 40 s | 0 | 1.2 s | 1.50 m | 0.05 |

The [results](assets/2026-11-12-f4-people-prediction/results.txt) list the
other columns. In `v9-gz-office-crowd-1` the mover tracker reported the
crosser (79 messages) and the robot yielded. The crosser then hid the leader,
the lock flickered and was lost for good about 10 s in with the leader about
6 m away, so the robot stopped. Leader re-acquisition after an occluding
crosser remains open. The earlier seed-0 wander run (`v6-gz-wander-s0-91`) had
29.9 s on the leader. In a stalled wander run, 94% of mover reports lay within
0.6 m of a real walker and the leader was never reported as a mover.

Seven crowd attempts were invalid: under load, Clearpath's controller spawner
timed out after its default 5 s switch timeout, so the velocity controller
never loaded and the robot stayed at its start. One further attempt failed to
start the crowd; the start request is now retried five times.

## Isaac crowd run

An operator run with `tools/run_leader_follow_sim --backend isaac --scenario
crowd --isaac-step-hz 60 --gui` (`20261112T103934Z-follow-isaac-crowd`)
finished and stopped by itself. The robot fell up to 6.2 m behind and caught
up to 1.6 m at the end, at up to 1.17 m/s, with no wrong-person time. Two
faults appeared:

- While the leader stood still for the first 3.2 s, the robot drove to
  0.08 m centre distance at about 0.8 m/s instead of holding 1.2 m back. Older
  runs show the same: an earlier Isaac crowd run reached 0.00 m and a Gazebo
  explainer diagnostic run 0.01 m. The cause is not yet known.
- At about 15 s it overtook `lf_crowd_2`, crossing at 0.8 m/s, at 0.45 m and
  about 1.1 m/s.
