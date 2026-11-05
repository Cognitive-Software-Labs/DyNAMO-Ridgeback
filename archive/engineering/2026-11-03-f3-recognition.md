# F3 recognition comparison and ROS qualification investigation

Recorded dates: 2026-11-03
Tested revisions: `4d29410dfbcdd1f53bd498d55d2e2d8734b2db69`
Provenance: partial

The tested implementation includes uncommitted additions beyond the listed
base revision. The [R3 source snapshot](assets/2026-11-03-f3-recognition/source-ros-r3.tar.gz)
and [receipt](assets/2026-11-03-f3-recognition/source-ros-r3.json) preserve recognition
code, configuration, launch files, compatibility patches and tests. Earlier
cached inference/evaluation utilities changed during the campaign, so this
record does not claim exact reconstruction of every intermediate result from
the base commit. Scene assets, model caches, recorded bags and detailed logs
remain under `artifacts/leader-following/` in the tested checkout.

## Populations and interpretation

The original sixteen F1/F2 recordings used one stylized character with identical
appearance. Their F3 comparison exposed recognition ambiguity and is retained
under `f3-comparison/`; it must not be mixed with the later outfit results.
The F3 appearance corpus has sixteen configurations across Gazebo and Isaac,
with distinct outfits except in the deliberate look-alike cases. A separate
camera-exit scenario adds one recording per backend. All observers remain
stationary; no following controller or driving command is part of these tests.

Five candidate families were evaluated on shared detector crops: OSNet-AIN,
CLIP-ReID, TransReID, SOLIDER and DINOv2. Checkpoint variants, preprocessing,
revisions and hashes are preserved in the source/configuration receipts and
per-case reports. The selected candidate profile uses OSNet-AIN, similarity
`.65` and recovery margin `.15`. Selection and publication do not establish
identity correctness; integrated replay and real ROS evaluation are separate.

## Evaluation corrections

The Gazebo F3 crowd-eight recording contains approximately `-0.85 m` of
`odom -> base_link` X translation by the end of its twenty-second stationary
fixture. Comparing odometry-frame outputs directly with world-frame person
poses introduced an evaluation mismatch. The corrected annotations and lock
scoring align through recorded robot TF and the declared stationary-base
surveyed planar pose. No transform is fitted to pedestrian truth. This
alignment is not valid for an arbitrarily moving robot.

Leader-lock correctness labels only the selected target against nearby truth,
with a `.75 m` distance gate and `.1 m` separation. An unselected duplicate
track can no longer steal the selected target's truth assignment. Earlier
metrics are retained in per-case backup files. Duplicate penalties in
people-tracking evaluation are a separate question. Camera-exit evaluation
also exposed null projected boxes; recognition scoring now excludes these
from image-IoU assignment while retaining absent-leader frames in the counts.

## Fusion and continuity changes

Appearance-assisted fusion remains physically gated, with camera-ID preference
weakened in the F3 profile. Duplicate camera measurements within `.25 m` are
suppressed by confidence. Known bystander tracks supply negative identity
history: promotion requires new appearance evidence separated from that
track's stored appearance, not merely entry into the leader's predicted pose.
Appearance ambiguity during recovery is evaluated among motion-plausible
candidates. Brief original-track lidar gaps can recover without a new image;
a visible appearance contradiction disables that exception.

Partial horizontal crops are withheld from appearance matching to avoid
interpreting a camera exit as an identity change. Narrow raw scan clusters can
support already camera-confirmed tracks when the learned lidar detector misses
a leg pose. They cannot create people, do not double-update already observed
tracks in a tick, and cannot update the appearance gallery. These adaptations
are F3 profile behavior; the F2 default retains zero appearance weight.

## ROS boundary failures and fixes

The first full actual-ROS replay matrix completed eighteen cases: twelve passed
its identity check and six failed. Isaac crossing, Isaac crowd-four, Gazebo
leader-stop and Isaac camera-exit never locked. Logs showed repeated
`no_base_transform` and future-extrapolation errors. Both look-alike-crossing
runs initially locked the designated bystander. Cached offline replay had not
exposed these startup and callback-ordering failures.

The identity callback performed a blocking stamped lookup on the same executor
that needed to receive TF. It now queues observations, retries nonblocking
lookups on a 20 ms timer, and fails closed when stamped TF remains unavailable
for `.25` seconds of node time. Regression tests cover delayed TF and expiry;
no latest-transform substitution is used.

A subsequent diagnostic run still exposed initial-selection failures. Empty
startup observations consumed acquisition intervals while cold inference
created gaps in the first real observations. The selected tracker now warms
camera and appearance inference before advertising readiness. Validation waits
for both publishers, uses separate ROS domains per case, and the selected F3
profile attempts acquisition on each usable message.

Equal-range starts also need an explicit geometric tie rule. Ranges within
`.25 m` are treated as uncertain ties, central candidates are preferred within
`.1 m` lateral tolerance, then the robot's right side breaks the remaining tie.
The rule reads positions only, never simulator role or outfit labels. The base
helper's zero-tolerance defaults retain strict nearest-person behavior.

## Evidence locations and acceptance boundary

- `f3-appearance-evaluation/`: five-model comparison, calibration, corrected
  annotations, and selected offline replay reports.
- `f3-camera-exit-evaluation/`: corresponding supplementary evidence.
- `f3-live-validation/`: first complete actual-ROS matrix, including failures.
- `f3-live-validation-r2/`: diagnostic rerun retained before the final startup fixes.
- `f3-live-validation-r3/`: partial diagnostic rerun using queued TF, warmed
  inference and deterministic acquisition; stopped before the memory revision.
- `f3-live-validation-memory3/`: fresh eighteen-case campaign with three-second
  internal track retention; `source-hashes.json` identifies the source files.
- `f3-selected-memory3-verified.log`: cached offline replay using the same
  memory/acquisition profile in the F2 evaluation environment.
- `f3-ros-fix-build.log`, `f3-ros-fix-tests.log`: build and regression evidence.

The memory-revision campaign must finish and its results must be reviewed
before marking F3 complete. A publisher-only
`validation.json` pass does not imply that `identity_score.json` passes.
Offline success does not substitute for the real ROS matrix. Robot/Thor timing,
GPU-memory certification, robot motion and closed-loop following remain outside
this record's qualification scope.

## Short-occlusion memory revision

The diagnostic ROS rerun removed wrong initial acquisition in its completed
cases, but look-alike recovery coverage remained poor. F3 tracks previously
expired after 750 ms, shorter than the tested occlusions. The selected profile
now retains internal tracks for three seconds, matching the identity recovery
horizon. Unobserved tracks remain unpublished. A regression test verifies that
retention does not create observations. The F2 default is unchanged.

The package rebuilt successfully; the full package pytest run reported
101 passed and two optional-dependency skips. Final replay uses fresh output
folders and isolated ROS domains 140–157. The campaign checks source hashes
before each case and after completion. Earlier results remain preserved.

The first cached-replay attempt used the system Python without `lap`; the
second encountered its partial output directories. These attempts are retained.
The F2 environment also lacks `lap`. The successful retry uses
`perception_venv/bin/python` and writes `selected-memory3-perception/`.
It completed eighteen cases with zero wrong locks and held all 266 sampled
out-of-camera frames. These scores still precede the recorded-base-transform
acquisition correction described below.

## Heartbeat and playback-start correction

The memory campaign passed Isaac look-alike crossing (181 correct locked
frames, zero wrong), but Gazebo look-alike crossing selected the bystander
(147 wrong locked frames, zero correct). The campaign was stopped, and its
outputs remain preserved. Three-second retention alone did not fix acquisition.

Inspection found the tracker emitted a heartbeat before looking for a complete
queued RGB-D/TF bundle. That advanced `last_output` and could discard valid
frames. Heartbeats now run only when no usable bundle exists. A regression
includes the first-observation case; the package suite reports 102 passed and
two optional-dependency skips. Playback now starts paused and resumes through
its ROS service after sensor publishers and the resume service are discovered.
This replaces publisher-only startup as the replay readiness boundary.

Cached replay also assumed an identity odom-to-base transform for acquisition.
It now reads the recorded transform, matching the ROS node. Evaluation labels
remain separate; the stationary survey alignment applies only during scoring.

## Corrected fixed-gallery comparison

The stationary-anchor rescore covers five candidates on sixteen outfit-profile
cases. Nearby leader-pick accuracy includes rejections in its denominator.
These scores evaluate cached image recognition, not ROS lock acceptance.

| Candidate | Nearby accuracy | Wrong nearby picks | Median case embedding time (ms) |
|---|---:|---:|---:|
| osnet-ain | 79.7% | 29 | 4.65 |
| clip-reid | 74.5% | 0 | 6.20 |
| transreid | 77.2% | 63 | 4.89 |
| solider | 74.5% | 23 | 4.71 |
| dinov2 | 48.6% | 74 | 5.07 |

OSNet-AIN remains the selected candidate. The JSON evidence is preserved in
[comparison-stationary-anchor.json](assets/2026-11-03-f3-recognition/comparison-stationary-anchor.json).
The [source receipt](assets/2026-11-03-f3-recognition/source-startup-final.json)
and [source snapshot](assets/2026-11-03-f3-recognition/source-startup-final.tar.gz)
identify the dirty source used for the fresh startup-final ROS matrix.
The focused Gazebo heartbeat/paused-start replay reached 280 correct locked
frames with zero wrong locks; its first observation was at 0.264 seconds,
compared with 1.122 seconds in the failed memory-only replay.

The optional ByteTrack association check passed in `perception_venv`, and the
TrackEval metrics check passed in `f2-venv`, separately from the system-Python
suite. Documentation link checks pass. The removed-facts check for edited
leader-following references reports zero missing facts; the unrestricted check
reports generated Graphify community names and extraction fragments, which
are superseded generated graph content rather than removed project guidance.

## Nearby recognition accuracy by scenario

Fixed-gallery comparison defaults are similarity `.72` and separation `.05`;
the selected integrated profile is calibrated separately to `.65` and `.15`.
These percentages therefore describe the candidate comparison, not the lock
pipeline's tracking coverage. A low score includes abstentions as failures.

| Case | OSNet-AIN | CLIP-ReID | TransReID | SOLIDER | DINOv2 |
|---|---:|---:|---:|---:|---:|
| gz-crossing-4 | 96.8% | 96.8% | 96.8% | 96.8% | 84.1% |
| gz-crowd-2 | 96.8% | 96.8% | 96.8% | 96.8% | 82.5% |
| gz-crowd-4 | 96.8% | 96.8% | 96.8% | 96.8% | 82.5% |
| gz-crowd-8 | 98.4% | 98.4% | 98.4% | 98.4% | 69.8% |
| gz-leader_alone-4 | 96.8% | 96.8% | 96.8% | 96.8% | 85.7% |
| gz-leader_stop-4 | 97.1% | 97.1% | 97.1% | 97.1% | 37.3% |
| gz-lookalike_crossing-4 | 1.6% | 0.0% | 4.8% | 0.0% | 9.5% |
| gz-lookalike_parallel-4 | 54.0% | 0.0% | 41.3% | 0.0% | 34.9% |
| isaac-crossing-4 | 96.9% | 96.9% | 96.9% | 96.9% | 54.7% |
| isaac-crowd-2 | 96.9% | 96.9% | 96.9% | 96.9% | 21.9% |
| isaac-crowd-4 | 96.9% | 96.9% | 96.9% | 96.9% | 43.8% |
| isaac-crowd-8 | 96.9% | 96.9% | 96.9% | 96.9% | 28.1% |
| isaac-leader_alone-4 | 96.9% | 96.9% | 96.9% | 96.9% | 76.6% |
| isaac-leader_stop-4 | 97.1% | 97.1% | 97.1% | 97.1% | 46.1% |
| isaac-lookalike_crossing-4 | 3.2% | 0.0% | 0.0% | 0.0% | 4.8% |
| isaac-lookalike_parallel-4 | 29.7% | 0.0% | 0.0% | 0.0% | 25.0% |

## Lidar-only continuity revision

The startup-final diagnostic matrix passed its first six completed identity
checks without wrong locks, but the Isaac camera-exit case held only 48 of 83
outside-camera observations (57.8%). Empty messages repeatedly caused
`target_missing` and three-frame recovery. The matrix was stopped and retained.
The tracker previously required a synchronized camera bundle even when fresh
transformed lidar observations were available.

F3 now tries fresh front/rear scan bundles when RGB-D synchronization is
unavailable. The existing learned lidar and gated raw support measurements
update only camera-confirmed tracks. No appearance is synthesized, and no
unobserved predictions are published. A regression checks observed scan
publication and prevents repeating the same timestamp. The full system-Python
suite reports 103 passed, two optional skips; the package builds successfully.
`f3-live-validation-lidar-final/` is the fresh ROS campaign, beginning with both
camera-exit recordings and Gazebo look-alike crossing.

The first recorded-transform cached replay exposed a final-tick nanosecond
extrapolation error. Acquisition now queries TF at the actual RGB sensor stamp,
matching its input rather than a rounded evaluation-schedule timestamp.
`selected-lidar-final/` is the fresh offline output; earlier partial attempts
remain preserved and must not be cited as completed runs.

The lidar-revision [source receipt](assets/2026-11-03-f3-recognition/source-lidar-final.json)
and [snapshot](assets/2026-11-03-f3-recognition/source-lidar-final.tar.gz)
include the matrix script. Installed tracker/identity executables match their
source hashes. The aggregate acceptance command below reports incomplete
until all required evidence exists; publication success alone cannot pass it.

```bash
python3 -m ridgeback_leader_follow.benchmarking.reid_acceptance \
  --root artifacts/leader-following/f3-appearance-evaluation \
  --supplementary artifacts/leader-following/f3-camera-exit-evaluation \
  --ros-root artifacts/leader-following/f3-live-validation-qualified-profile \
  --lock-profile selected-lidar-final \
  --output artifacts/leader-following/f3-acceptance-qualified-profile.json
```

## Bounded observation timeout

The lidar-only diagnostic passed both camera-exit identity checks with zero
wrong locks. Isaac held every outside-camera observation; Gazebo held 166 of
172 (96.5%), losing the lock twice and recovering within 0.136 seconds.
Both losses began with empty heartbeats at 9.36 and 11.48 seconds, followed
12 ms and 4 ms later by valid observations of the original leader.

The selected F3 `observation_timeout` is now 250 ms, matching the stamped-TF
wait bound; F2 retains 150 ms. A regression verifies no heartbeat at 200 ms
and an empty heartbeat at 300 ms with this configuration. The suite reports
104 passed, two optional skips, and the package rebuild succeeds. The fresh
`f3-live-validation-qualified-profile/` campaign tests this profile in domains
60–77. The folder name identifies the candidate profile; qualification still
depends on its completed evidence. The lidar-final matrix was stopped and
retained as diagnostic evidence.

The final candidate's [source receipt](assets/2026-11-03-f3-recognition/source-qualified-profile.json)
and [snapshot](assets/2026-11-03-f3-recognition/source-qualified-profile.tar.gz)
include the 250 ms profile and campaign script. Installed node executables
were checked against source hashes. The cached eighteen-case lock replay
completed with zero wrong locks and 266/266 outside-camera samples held; it
uses recorded RGB stamps for acquisition TF. ROS evidence is assessed
separately because callback scheduling is absent from that cached replay.

Final verification also exercised the aggregate reviewer against a temporary
qualified-looking corpus with all ROS/offline reports missing. It returned
`incomplete_or_failed`, with both completeness checks false. The final Colcon
run reports 123 tests, zero errors/failures, and two optional skips. The optional
checks had passed separately in their dependency environments. Graphify was
rebuilt after the timeout and lidar changes; documentation links and the
removed-facts check for the edited topic references pass.

## Final simulation acceptance review

All eighteen actual ROS cases completed and passed. Each reached a correct
leader lock; no wrong locked frames were scored. The five-candidate comparison
is complete, and OSNet-AIN publishes 512-dimensional features. All aggregate
checks in [the acceptance report](assets/2026-11-03-f3-recognition/acceptance-qualified-profile.json)
pass. The [ROS summary](assets/2026-11-03-f3-recognition/ros-qualified-profile-summary.json)
preserves per-case states, recovery and coverage. The source hashes remained
unchanged throughout the completed campaign. Playback used rate 0.5 with three
independent workers; this is not a real-time or target-hardware measurement.

| Case | Correct locked frames | Wrong locked frames | Lock coverage | Outside-camera held / observed |
|---|---:|---:|---:|---:|
| gz-crossing-4 | 326 | 0 | 89.3% | — |
| gz-crowd-2 | 330 | 0 | 92.7% | — |
| gz-crowd-4 | 289 | 0 | 89.5% | — |
| gz-crowd-8 | 219 | 0 | 71.3% | — |
| gz-leader_alone-4 | 454 | 0 | 99.8% | — |
| gz-leader_out_of_view-4 | 367 | 0 | 100.0% | 117/117 |
| gz-leader_stop-4 | 317 | 0 | 85.0% | — |
| gz-lookalike_crossing-4 | 97 | 0 | 24.0% | — |
| gz-lookalike_parallel-4 | 269 | 0 | 66.7% | — |
| isaac-crossing-4 | 299 | 0 | 88.5% | — |
| isaac-crowd-2 | 322 | 0 | 91.2% | — |
| isaac-crowd-4 | 279 | 0 | 93.3% | — |
| isaac-crowd-8 | 217 | 0 | 68.9% | — |
| isaac-leader_alone-4 | 357 | 0 | 100.0% | — |
| isaac-leader_out_of_view-4 | 344 | 0 | 96.4% | 115/115 |
| isaac-leader_stop-4 | 144 | 0 | 40.4% | — |
| isaac-lookalike_crossing-4 | 276 | 0 | 81.2% | — |
| isaac-lookalike_parallel-4 | 202 | 0 | 57.9% | — |

Both camera-exit recordings held 232/232 actual ROS outside-camera observations
in total. Cached replay held 266/266 samples on its separate fixed schedule.
Those denominators must not be combined. The planned F3 simulation checks are
complete, but lock coverage is not uniformly high: Gazebo look-alike crossing
is approximately 24%, and Isaac leader-stop approximately 40%. These results
require future control to respond safely to uncertain or missing identity;
they do not establish reliable continuous following. One deterministic recording
per configuration and a shared character body limit generalization.

After review, only status/description metadata in `config/recognition.json`
changed to record acceptance; the tested model, thresholds and runtime
parameters remain unchanged. The plan, current recognition/tracker references
and explainer now reflect this bounded result. F4–F6 and robot/Thor timing and
GPU-memory qualification remain open. No commit, push or driving was performed.
