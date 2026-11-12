# Leader appearance recognition

F3’s planned simulation acceptance checks pass. OSNet-AIN is selected; the
[checked-in profile](../../src/ridgeback_leader_follow/config/recognition.json)
records its checkpoint, thresholds and validation status. All five adapters
remain available for comparison. All eighteen recorded ROS cases reached a
correct lock with zero wrong locks; both camera-exit cases held every sampled
outside-camera observation. Robot movement remains later-phase work.

This is bounded stationary-observer evidence, with one recording per case.
With the profile's continuity confidence floor, visible-frame lock coverage is
87–100% (mean 98%) in the latest ROS replay of all eighteen cases. Isaac crowd-8
and both look-alike crossings are the weakest cases. When the leader walks
directly behind an identical look-alike, the lock deliberately stays
uncertain, so coverage in those cases varies with tracker timing between runs.
These results do not establish continuous following or hardware performance.

## Implemented boundary

`benchmarking/reid.py` clips detected person boxes to RGB image bounds and
rejects invalid or undersized crops. Each model owns its resizing and
normalization. `AppearanceEncoder` checks batch dimensions and produces finite,
L2-normalized features. Encoder inputs contain crops and track IDs, never
simulator identities.

`appearance.py` compares a leader gallery with candidate embeddings. Explicit
similarity and separation thresholds reject weak or ambiguous matches,
including identical look-alikes. These thresholds require model-specific
calibration; an appearance proposal alone does not establish identity.

The separate `RecognitionScore` consumes evaluation-only identities. It counts
correct, wrong and rejected selections when the leader is eligible, and false
acceptances when the leader is absent. Eligibility and distance filtering must
be supplied by the evaluation harness. Rejections count against leader-pick
accuracy rather than disappearing from its denominator.

## Model adapters

The DINOv2-small adapter uses the CLS token of
`last_hidden_state`. Its official Hugging Face checkpoint is pinned to revision
`ed25f3a31f01632728cabb09d1542f84ab7b0056`; the same revision supplies its image
processor. Weights cache beneath the supplied model directory. Batching and
CPU/CUDA device selection are explicit. This is a generic visual-feature
baseline, not a person-ReID-trained checkpoint.

Sources: [official model](https://huggingface.co/facebook/dinov2-small) and
[image preprocessing](https://huggingface.co/facebook/dinov2-small/blob/ed25f3a31f01632728cabb09d1542f84ab7b0056/preprocessor_config.json).

The specialized person-ReID adapters load official OSNet-AIN x1.0 MSMT17, CLIP-ReID ViT-B/16
MSMT17, TransReID ViT baseline Market-1501 (without JPM/SIE), and SOLIDER Swin
Tiny Market-1501 checkpoints. Official inference resize/normalization is used.
The CLIP adapter extracts the official before-neck concatenation of 768 visual
and 512 projected features; it does not use text prompts for recognition.
Source revisions and checkpoint receipts accompany the preparation utility.
The upstream source licenses are MIT; checkpoint and source rights remain
separate from downstream dataset redistribution rights.

Legacy TransReID/SOLIDER imports of removed `torch._six` are replaced by
`collections.abc`; SOLIDER semantic weights follow the input tensor device.
The maintained compatibility patches live under `benchmark_native/`.

## Setup and replay

Use the existing perception environment. Install
`benchmark_native/requirements-f3.txt`, then run:

```bash
perception_venv/bin/python -m ridgeback_leader_follow.benchmarking.prepare_reid
ros2 launch ridgeback_leader_follow leader_recognition.launch.py appearance_candidate:=osnet-ain
```

Source ROS and the workspace first. The launch starts tracker and identity
nodes only. Rebuild the package after node-script changes: these installed
executables are copies even with `--symlink-install`. The launch loads the
selected profile: OSNet-AIN, similarity `.65`, recovery margin `.15`, and an
8 m, 30-degree front acquisition sector. The ordinary identity node retains
its original 2 m acquisition default outside this F3 launch. Comparison
results do not by themselves qualify the ROS runtime.

`benchmarking/reid_benchmark.py` evaluates a fixed gallery initialized from the
nearest sensor-detected person in the front sector. It receives cached detector
boxes and RGB-D; evaluation labels never choose the reference crop. Reports
separate nearby and all-visible recognition. `reid_lock_replay.py` instead
runs cached features through the actual image association, fusion and lock
cores. These recognition and integrated-lock scores have different meanings.

## Continuity and recovery

A verified lock can persist without appearance only for the same track ID,
with confidence at least `continuity_min_confidence`, unchanged frame, successive observations at most
750 ms apart, and position within `.75 m` of the previous velocity prediction.
A competing observation at or above that confidence, with prediction error
within `.1 m` of the selected track, makes continuity ambiguous and drops the
lock. The node default floor is `.5`; the F3 profile uses `.25`. Observations
within `duplicate_radius` of the selected track (profile `.3 m`, node default
`0`) are duplicates of the same body, not competing people, for continuity and
motion recovery. With `hold_on_appearance_mismatch` (profile `true`, node
default `false`), a visible appearance mismatch does not drop a lock that
continuity still confirms: a leader turning toward the camera changes
appearance, not identity. The gallery is not updated from such frames. Partly
occluded leaders often produce camera boxes scored around `.3` that still carry
a matching appearance. At `.5`, those single frames caused almost every lock
drop in the F3 cases. Appearance is still checked on every frame that has it.
This uses the physically gated lidar-supported tracker; it does not retain a
lock on an unobserved prediction. Missing observations or incompatible visible
appearance make identity uncertain. Gallery updates require visible appearance.
The selected F3 profile retains unobserved tracks internally for three seconds
to bridge short occlusions. It publishes only observed tracks; this memory
does not authorize a lock on an unobserved prediction. The F3 empty-heartbeat
timeout is 250 ms, allowing delayed sensor/TF bundles to arrive. The F2 tracker default
remains 750 ms.
Recovery requires both an unambiguous gallery match and a candidate within
`.75 m` of the last verified motion prediction, at most 3 seconds old. Two
motion candidates within `.1 m` of equal prediction error are rejected.
Appearance ambiguity is evaluated among motion-plausible candidates, rather
than rejecting a match because an identical-looking person is on an incompatible
trajectory. Candidates within `.1 m` of the best admissible prediction error
remain competing hypotheses; ambiguous matches among them cannot recover a
lost identity. Simulator clock
rollback resets identity acquisition.

Five-model comparison, threshold calibration and the selected profile’s
integrated simulation acceptance checks are complete.
Synthetic-crop loading checks do not establish leader-recognition accuracy.
The [plan](../plans/FOLLOWING_leader_in_crowds.md) owns F3 completion criteria.

## Appearance-corpus evaluation

After all sixteen `f3` profile recordings pass scene qualification, run:

```bash
perception_venv/bin/python -m ridgeback_leader_follow.benchmarking.f3_campaign \
  --corpus artifacts/leader-following/f3-appearance-corpus \
  --output artifacts/leader-following/f3-appearance-evaluation
```

The campaign refuses incomplete or mixed-profile corpora. It generates
profile-matched mesh annotations, cached OWLv2/LFE-PPN detections, five-model
recognition comparisons and integrated default lock replays for the best two
nearby-accuracy candidates. Complete outputs are reused on resume; partial
outputs require inspection. Evaluation completion does not grant F3 acceptance.
All stages retain a stationary observer and start no following controller.

## Supplementary camera-exit scenario

`--scene-profile f3 --scenario leader_out_of_view` adds a separate 20-second
recognition test. The leader starts at `(2, 0)`, remains still for acquisition,
walks to `(2, 2.3)`, pauses, then returns to the centre before moving forward.
A bystander remains at `(5, -1.3)`. The original six F1 families are unchanged.
Actual out-of-view duration must be established from recorded camera extrinsics
and mesh projections; path geometry alone is not qualification.

Lock reports separate correct locks without appearance from correct locks
while the leader's projected mesh is outside the camera. They also report
recovery durations and unrecovered losses. Missing camera-exit coverage remains
unknown rather than passing the continuity requirement. `reid_acceptance`
reviews per-case evidence without automatically declaring F3 complete.

For a newly recorded corpus, `reid_calibrate` accepts explicit `--corpus`,
`--comparison`, `--detections` and `--annotations` paths. Calibration uses
Gazebo cases and reports Isaac cases separately; only the two comparison
finalists are tuned. Calibrated lock replays must be evaluated again rather
than treating fixed-gallery calibration accuracy as integrated identity success.

## Appearance-assisted measurement association

With an appearance encoder configured, the tracker adds `.75 * (1 - cosine)`
to camera-association costs. In this F3 mode, matching camera IDs reduce the
position cost by 10%, acting as a tie preference rather than the tenfold
preference retained by the F2 profile. This helps
resolve camera-ID swaps between distinct-looking people. The physical distance
gate remains mandatory; appearance cannot link a measurement outside it.
The last valid feature is retained internally for association, while published
appearance remains empty on lidar-only observations. Without an encoder the
weight is zero, retaining the F2 default association behavior.

F3 camera measurements within `.25 m` of one another are deduplicated before
association, retaining the highest-confidence observation. This prevents
multiple boxes on one person from creating extra identity tracks. The F2
profile retains its original camera-measurement behavior.

During recovery, a track previously observed alongside a confirmed leader is
negative identity evidence. It can be promoted only when its current appearance
matches the leader and is separated from its own stored appearance history by
the recovery margin. Identical-looking known bystanders therefore cannot take
the lock merely by entering the predicted position. A distinct appearance can
still verify a leader after an actual tracking-ID swap. Session resets clear
this history; gallery history is bounded to eight features per tracked ID.

## Runtime appearance quality

The F3 runtime withholds embeddings when a detection touches the horizontal
image boundary (within two pixels). A partly exiting person's crop is treated
as unavailable appearance rather than a contradictory new identity. Position
measurements can still update the confirmed track. Fixed-gallery candidate
comparisons retain their declared crop policy; selected-runtime adaptation is
validated separately.

## Raw-scan support

When F3 appearance association is enabled, scan clusters up to `.4 m` wide can
support an already camera-confirmed track within `.3 m` of its prediction.
When RGB-D synchronization is unavailable, the F3 tracker can update from a
fresh lidar-only bundle with stamped TF. No image embedding is carried forward
into that observation. Raw clusters update only tracks not already observed by camera or learned lidar during
that tick. They cannot create people or update appearance galleries. The F2
profile does not consume these support measurements. This handles leg poses
missed by the learned detector without treating every scan cluster as a person.

## Exact-stamp TF readiness

The identity node queues `People` observations and retries exact-stamp TF on a
20 ms timer. Lookups are nonblocking, allowing TF subscription callbacks to
run on the same executor. `tf_wait_timeout` defaults to `.25` seconds of node
time. An expired observation is processed as empty with
`reason: no_base_transform`; the node never substitutes the latest transform
for a missing stamped transform. Clock rollback clears pending observations.

## Initial acquisition ties

The selected F3 profile attempts acquisition on each usable message
(`acquire_interval: 1`), rather than waiting eight messages after an empty
startup observation. Before its publisher becomes ready, the tracker warms
both camera and appearance inference on synthetic images. Replay starts only
after both tracker and identity publishers are discovered. Bag playback starts
paused and resumes through its service once sensor publishers are discovered.
The tracker processes an available complete bundle before considering an empty
heartbeat, preventing the heartbeat from invalidating queued sensor data. Validation cases
use separate ROS domains so a previous run cannot supply a stale clock or TF.

The selected F3 profile treats range estimates within `.25 m` of the nearest
candidate as tied. It first prefers the central candidates (absolute lateral
position within `.1 m` of the best), then the robot's right, negative Y. Final
ties use distance and track ID. This geometric policy is independent of
simulator names, roles, outfits and truth. It makes equal-range look-alike
starts deterministic despite measurement noise and callback timing. The base
acquisition helper retains strict nearest selection when both tolerances are
zero; the F3 launch supplies the profile-specific tolerances.

## Evaluation coordinates

The F3 fixtures declare a stationary observer at the surveyed world/base
planar origin. Evaluation uses recorded `odom -> base_link` TF to align
odometry-frame predictions with that stationary world reference and to project
mesh annotations. It does not fit a transform to pedestrian truth or change
runtime inference inputs. This alignment must not be used for a moving robot;
that requires an independently established world/map transform.

Leader-lock scoring labels the selected target against the nearest truth
position within `.75 m`, requiring `.1 m` separation from the next candidate.
Unselected duplicate tracks cannot steal its truth label. Unmatched and
ambiguous locked targets fail the correctness check. People-tracking duplicate
penalties remain separate. Camera-exit correctness and recovery reports are
separate from appearance availability and publication checks.

## Archived evidence

- [F3 recognition and ROS qualification investigation](../../archive/engineering/2026-11-03-f3-recognition.md) — comparison populations, evaluation corrections and actual ROS failures/fixes.

- [Lock continuity confidence floor](../../archive/engineering/2026-11-11-lock-continuity-floor.md) — drop-cause analysis, offline floor sweep and the eighteen-case ROS replay.
