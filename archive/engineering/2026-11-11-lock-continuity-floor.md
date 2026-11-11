# Lock continuity confidence floor

Recorded dates: 2026-11-11
Tested revisions: `0fbd7a37e1fba86249dce3630e83f0c8a676daf9`
Provenance: partial

The checkout was dirty: the tested change lowers the leader identity node's
continuity confidence floor and wires it through the F3 profile. The ROS
campaign's [source hashes](assets/2026-11-11-lock-continuity-floor/ros-source-hashes.json)
record the exact Python sources used. The recorded F3 corpus and models are the
ones used by the [F3 acceptance record](2026-11-03-f3-recognition.md).

## Cause of low lock coverage

The accepted F3 profile reached correct locks with zero wrong locks, but
visible-frame coverage ranged from 24% to 100%. Per-frame leader states showed
that every LOCKED-to-UNCERTAIN transition had the reason `track_discontinuity`.
The [drop analysis](assets/2026-11-11-lock-continuity-floor/baseline-drop-causes.txt)
classified 143 of 144 drops as the leader's own track arriving with confidence
below `.5`. Its position stayed within the gate and no other person was near
the prediction. Only one drop was a real nearby competitor.

Those observations were mostly camera detections that still carried an
appearance feature, scored about `.3` by OWLv2 when the leader was partly
occluded. The continuity rule rejected them before the appearance check ran.
Recovery then needed three consecutive confirmations, producing repeated
0.1–0.9 s gaps. In Gazebo look-alike crossing, one such drop happened near the
look-alike and never recovered.

## Offline sweep

The [replay](assets/2026-11-11-lock-continuity-floor/replay.py) feeds each
case's recorded `People` messages through the unchanged `LeaderIdentity` core.
It reproduced the original states and track IDs on 358/358, 408/409 and
309/310 frames of three checked cases. The
[evaluator](assets/2026-11-11-lock-continuity-floor/evaluate.py) scores every
case with `ros_identity_score`; its baseline matched the accepted coverage
within 0.2 percentage points.

| Floor | Minimum coverage | Mean coverage | Wrong locked frames |
|---|---:|---:|---:|
| `.5` (previous) | 24.0% | 79.8% | 0 |
| `.3` | 69.1% | 94.7% | 0 |
| `.25` | 87.1% | 97.3% | 0 |
| `.2` | 87.1% | 97.6% | 0 |
| `.1` | 92.4% | 98.1% | 0 |

`.25` was selected. It is below the leader's typical occluded scores of
`.28–.38` while still excluding the weakest boxes. The
[sweep results](assets/2026-11-11-lock-continuity-floor/offline-sweep.json)
hold every case. The value was chosen on the same eighteen recordings it was
evaluated on.

## ROS replay

All eighteen cases were replayed through the real tracker and identity nodes
with the [campaign script](assets/2026-11-11-lock-continuity-floor/ros-campaign.py),
matching the accepted campaign apart from the new floor. The
[summary](assets/2026-11-11-lock-continuity-floor/ros-continuity-floor-summary.json)
passed with zero wrong locked frames. Both camera-exit cases held every sampled
outside-camera observation.

| Case | Before | After |
|---|---:|---:|
| gz-crossing-4 | 89.3% | 95.7% |
| gz-crowd-2 | 92.7% | 99.3% |
| gz-crowd-4 | 89.5% | 99.2% |
| gz-crowd-8 | 71.3% | 100.0% |
| gz-leader_alone-4 | 99.8% | 99.6% |
| gz-leader_out_of_view-4 | 100.0% | 99.7% |
| gz-leader_stop-4 | 85.0% | 99.7% |
| gz-lookalike_crossing-4 | 24.0% | 95.2% |
| gz-lookalike_parallel-4 | 66.7% | 99.6% |
| isaac-crossing-4 | 88.5% | 97.2% |
| isaac-crowd-2 | 91.2% | 99.7% |
| isaac-crowd-4 | 93.3% | 100.0% |
| isaac-crowd-8 | 68.9% | 87.1% |
| isaac-leader_alone-4 | 100.0% | 100.0% |
| isaac-leader_out_of_view-4 | 96.4% | 100.0% |
| isaac-leader_stop-4 | 40.4% | 95.4% |
| isaac-lookalike_crossing-4 | 81.2% | 90.5% |
| isaac-lookalike_parallel-4 | 57.9% | 100.0% |

Mean coverage rose from 79.8% to 97.7%.

## Limits

The tracker published fewer messages per case than in the accepted campaign
(for example 311 rather than 404 scored frames in Gazebo look-alike crossing),
so tracker timing differed between runs. Two earlier single-case runs of Gazebo
look-alike crossing with the same floor scored 91.5% and 44.6%. In the low run
the leader walked directly behind its identical look-alike at about 4.5 m. The
leader's detection fell to `.12`, and recovery correctly refused to choose
between identical-looking candidates. Coverage in that case therefore depends
on tracker timing. One recording per case, a stationary observer and tuning on
the evaluation corpus limit generalisation; robot motion and hardware timing
remain untested.
