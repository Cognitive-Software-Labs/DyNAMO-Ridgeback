# People-tracking benchmarks

F2's simulation implementation compares twenty candidate methods on the same
six scene families and three crowd densities in Gazebo and Isaac. The selected
quality-first profile is **OWLv2 + LFE-PPN + ByteTrack-assisted constant-velocity
Kalman fusion**, exposed by the [people tracker](people_tracker.md). RF-DETR is
the configurable faster camera alternative. Appearance/leader recognition is
F3; driving is F4. Robot/Thor performance checks remain deferred.

## Candidate results and selection

Every row has sixteen measured cases. The archived evidence below owns the
numerical tables, range/visibility strata, recovery events, timing definitions,
licenses and limitations. This is a comparison of the specified variants and
presets, not all sizes or all possible settings of each model family.

| Camera candidate | Tested variant | Result/decision |
|---|---|---|
| YOLO11 | YOLO11n | Measured; fewer detections than the selected camera |
| YOLO26 | YOLO26n | Measured; high precision, low recall on these assets |
| RT-DETRv2 | R18 | Measured; more false positives than selected |
| D-FINE | small COCO | Measured; lower balanced box score |
| DEIM | D-FINE small COCO | Measured; lower aggregate recall |
| RF-DETR | small | Measured; faster configurable alternative |
| OWLv2 | base patch16 ensemble, `a person` | Selected for highest box precision/recall balance |

| Lidar candidate | Tested implementation | Result/decision |
|---|---|---|
| DR-SPAAM | JRDB annotated checkpoint | Measured; lower balanced position score |
| DROW3 | JRDB annotated checkpoint | Measured; more false positives |
| LFE-PPN | published ONNX, CPU | Selected for balanced position score and small CPU cost |
| PeTra | original bootstrapping graph | Measured; poor recall without adaptation |
| ROS leg detector | original forest and cluster features, Jazzy/OpenCV bridge | Measured; upstream default permits many false positives |
| Scan clustering | in-repo contiguous clusters | Measured floor; no semantic person classification |

| Multi-person tracker | Tested comparison | Result/decision |
|---|---|---|
| ByteTrack | BoxMOT defaults, fixed YOLO11 boxes | Highest mean HOTA/IDF1 and low cost; algorithm selected |
| BoT-SORT | same boxes, default OSNet model | Measured; similar HOTA, greater cost |
| OC-SORT | same boxes | Measured; lower HOTA/IDF1 |
| Deep OC-SORT | same boxes, default OSNet model | Measured; lower HOTA/IDF1 |
| StrongSORT | same boxes, default OSNet model | Measured; lower HOTA/IDF1 |

The combined runtime uses the existing Ultralytics ByteTrack implementation and
is evaluated separately from the BoxMOT defaults. OWLv2's lower score scale
requires high/new thresholds 0.1, low threshold 0.05 and confidence-weighted IoU
disabled; the buffer is 30 updates and matching threshold 0.8. Camera identity
cues cannot bypass the physical position gate. This selected-profile adaptation
is not a parameter search over all candidates. No detector was fine-tuned.

| Single-target candidate | Tested variant | Result/decision |
|---|---|---|
| SAM 2 | tiny SAM2.1, SAMURAI mode off | Measured leader coverage and recovery |
| SAMURAI | same weights, motion-aware mode on | Greater leader coverage, greater cost; not an all-people detector |

The chosen combined profile has a simulator-truth-scored report for every case.
Dense crowds still exhibit ID switches and leader-track losses; F2 completion
establishes a measured, runnable baseline, not perfect identity continuity or
closed-loop success. The simulations use one human asset and deterministic
paths, with one recording per configuration. [F3 recognition](recognition.md) evaluates appearance separately on its outfit
corpus. Repeated closed-loop evaluation and hardware qualification remain
later-phase work.

## Input and scoring contract

Adapters receive sensor inputs. Only the evaluation time columns of an F1
`ground_truth.csv` determine the offline schedule; person positions and IDs
never enter detector or tracker updates. The evaluator reads those annotations
separately. SAM methods receive one explicitly recorded frame-zero leader click
as their initialization, with no subsequent ground-truth assistance.

Continuous recordings retain warmup sensors and static TF. The corpus checks
registered/rectified RGB-D freshness and all sensor-to-odometry transforms at
all 400 ticks of every 20-second case. Rejected recordings remain outside the
accepted corpus. Qualification snapshots alone are not complete datasets.

For each tick, replay selects preceding inputs no older than 75 ms. RGB-D skew
must not exceed 40 ms. Transform lookup uses each sensor's timestamp. A missing
input produces an explicit empty prediction; missing static extrinsics rejects
the bag. World scoring requires an independently known world/odom anchor; never
fit that transform to truth. F1's stationary observer declares coincident XY
origins. Native lidar detections are transformed and merged with overlap
suppression, preserving their distinct sensor origins.

Prediction JSONL has one world-frame record per evaluation tick:

```json
{"scenario_time": 0.05, "frame": "world", "people": [{"track_id": 1, "x": 1.525, "y": 0.0}]}
```

The position evaluator aligns ticks within 1 ms, rejects duplicates/non-finite
coordinates, and counts absent predictions as empty. Gated Hungarian matching
maximizes valid cardinality, then minimizes distance (default 0.75 m). It reports
precision, recall, position RMSE, ID switches across gaps, fragmentation,
track-loss and leader-track-loss events. Initial acquisition is not a switch.

Camera scoring uses IoU 0.5 with animated-mesh projected boxes and depth-supported
visible-fraction estimates. Range bands are 0–2, 2–4 and 4–8 m; visibility bins
have width 0.25. Per-stratum precision ignores detections matched to other
strata and retains unmatched false positives in every stratum, so FP counts are
not additive. Visibility is an in-image mesh estimate with sampled gait and
15 cm depth tolerance, not exact renderer instance segmentation.

Official TrackEval supplies box-IoU HOTA and IDF1. These are separate from
world-position diagnostics. Lidar results include position-gated distance bands
and coverage outside the camera. Single-target reports score the leader alone,
including recovery failures. Reports retain model/source/input provenance and
state timing scope; CPU/GPU memory and timing refer to this workstation, not
robot/Thor certification. See the archived record for measurement caveats.

## Candidate environments

The simulation benchmark uses the workspace's ROS 2 Jazzy environment and a
compatible PyTorch/CUDA installation. The tested workstation used PyTorch
2.11/CUDA 12.8; robot/Thor wheel/platform selection remains a deployment check.
After building and sourcing the workspace:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
perception_venv/bin/pip install --extra-index-url https://download.pytorch.org/whl/cu128 -r src/ridgeback_leader_follow/ridgeback_leader_follow/benchmarking/requirements-camera.txt
python3 -m venv --system-site-packages artifacts/leader-following/f2-venv
artifacts/leader-following/f2-venv/bin/pip install --extra-index-url https://download.pytorch.org/whl/cu128 -r src/ridgeback_leader_follow/ridgeback_leader_follow/benchmarking/requirements-tracking.txt
perception_venv/bin/python -m ridgeback_leader_follow.benchmarking.prepare_candidates
```

Use a separate tracking environment: BoxMOT/TrackEval require NumPy 2, while the
vision benchmark pins NumPy 1.26. Source revisions and compatibility patches are
pinned under the leader-follow package's `benchmarking/` directory. The helper
preserves mismatching/dirty dependencies rather than replacing them. It prepares
DROW/DR-SPAAM, DEIM, LFE, PeTra, SAM2/SAMURAI, the leg bridge and OSNet weights;
camera adapters download their remaining checkpoints into `model_dir` on first
use. Hugging Face revisions are pinned. Model hashes and the final source tree
are retained by the provenance command. Selected runtime dependencies do not
require TensorFlow or the separate BoxMOT environment.

## Recording and replay commands

Run from the workspace root with ROS and the built workspace sourced. Choose
unused ROS domains; stop only this task's processes.

```bash
ros2 run ridgeback_leader_follow record_people_corpus --root artifacts/leader-following/<new-corpus> --ros-domain 140
python3 -m ridgeback_leader_follow.benchmarking.qualify_corpus --root artifacts/leader-following/<new-corpus>
ros2 run ridgeback_leader_follow replay_people --run <recorded-F1-run> --output <new-replay-dir> --world-from-odom 0 0 0 0
ros2 run ridgeback_leader_follow evaluate_people --run <recorded-F1-run> --predictions <candidate.jsonl> --candidate <name> --output <new-report.json>
perception_venv/bin/python -m ridgeback_leader_follow.benchmarking.sweep --corpus <qualified-corpus> --output <new-results> --model-dir artifacts/leader-following/f2-models --world-from-odom 0 0 0 0
```

`record_people_corpus` resumes valid scene runs and records missing cases.
`qualify_corpus` additionally rejects incomplete TF/freshness and can record a
replacement without overwriting the rejected evidence. The lighter
`replay_people` executable retains the unselected clustering/CV plumbing
baseline; the selected node is launched through the
[tracker workflow](people_tracker.md#runtime).

The completed campaign's accepted-case index is
`artifacts/leader-following/f2-corpus/corpus.json`. Its final candidate matrix is
`f2-simulation-results-final.json` under the same leader-following artifact root;
the selected per-case reports are under `f2-fused-owlv2-lfe-ppn-selected/`.
Earlier diagnostics and failed attempts are excluded from those selections.
The [plan](../plans/FOLLOWING_leader_in_crowds.md) owns remaining phase work.

## Archived evidence

- [F2 simulation candidate comparison](../../archive/engineering/2026-11-02-f2-simulation.md) — all candidate rows, selected combination, per-scenario losses/switches, provenance and limits.
