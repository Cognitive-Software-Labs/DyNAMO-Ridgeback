# Detection

The detection component converts RGB frames into index-aligned target boxes.
It owns model inference, confidence filtering, duplicate suppression, box
normalization, and the `TargetDetections` message contract. It does not segment,
track, associate across time, estimate distance, or choose a fallback estimator.

## Current implementation

`target_detector_node` subscribes to the configured colour image. Its
startup-only `detector` selector chooses a backend through
`core/detection.py`; `owlv2` is the only registered production value.
`detection_model` is a separate checkpoint identifier, passed to that backend.
The current defaults are:

| Setting | Default |
|---|---|
| Detector implementation (`detector`) | `owlv2` |
| Checkpoint (`detection_model`) | `google/owlv2-base-patch16-ensemble` |
| Candidate label | `humanoid robot` |
| Confidence threshold | `0.55` |
| Maximum inference cadence (`detector_fps`) | `10 Hz`; see [cadence defaults](#detector-cadence-defaults) |
| Same-label NMS IoU threshold | `0.5` |
| Same-label containment threshold | `0.95` intersection-over-smaller |

OWLv2 is an implementation choice, not a downstream dependency. Consumers use
only the published image grid, boxes, labels, scores, and original image header.
The ROS-free `ridgeback_common.detector_config` module owns supported names,
defaults, and validation. It rejects empty/unknown selectors, blank checkpoints
or labels, nonfinite or out-of-range thresholds, and nonpositive or nonfinite
rates before any model is constructed. Comma-separated labels are stripped and
empty parts ignored. Changes to detector settings through ROS parameters are
rejected after startup; select a different configuration by restarting the node.

The factory in `ridgeback_localization.core.detection.create_detector_backend`
is the one registration point for a future implementation. A backend implements
`load`, `infer`, and `normalize`; `normalize` returns the existing
`DetectionBatch`. `OwlV2Backend` alone interprets OWLv2 pipeline dictionaries
and applies the existing clipping and duplicate-suppression policy. A new
backend must explicitly produce the same image-grid boxes, labels and scores;
it need not imitate OWLv2's private output shape. The factory accepts an
injected registry for tests, but the production registry contains only `owlv2`.
A direct ROS-parameter start of that baseline is:

```bash
# The direct executable inherits this shell's Python environment.
source perception_venv/bin/activate
ros2 run ridgeback_localization target_detector_node --ros-args \
  -p detector:=owlv2 \
  -p detection_model:=google/owlv2-base-patch16-ensemble \
  -p target_labels:="humanoid robot" -p detection_threshold:=0.55 \
  -p detector_fps:=10.0
```

## Detector cadence defaults

`detector_fps` is an upper bound on detection step starts, not a guaranteed
rate; every selected measurement row inherits the resulting cadence. Its
default depends on where the value comes from:

| Context | Default when unset | Source |
|---|---|---|
| `target_detector_node`, exploration, `localization.launch.py`, `target_distance_benchmark.launch.py`, and a sweep whose `defaults` omit it | `10.0` | `DETECTOR_FPS_DEFAULT` in `ridgeback_common.detector_config` |
| A live sweep authored in the [configurator](../target_distance_benchmarking/configurator.md#detector-settings) | `5.0` | the live-system `detector_fps` field default in `benchmarking/replay_profiles.py` |

The configurator writes its field value into the sweep it authors, so an
authored sweep runs at 5 Hz unless the operator changes the field. A referenced
sweep YAML and a direct launch keep `10.0` when they leave it unset. Offline
replay profiles reuse frozen detections and select no cadence.

## Frame and scheduling behavior

The node keeps one latest colour message rather than a frame queue. The worker
waits until its monotonic cadence permits another inference, then takes the
latest available frame. Frames superseded before processing are intentionally
dropped. This bounds queued work when inference is slower than the camera and
prevents latency from growing without limit.

The detector's RGB subscription uses best-effort, volatile, keep-last QoS
with depth **1**. This bounds the pending middleware queue as well as the
worker's single pending slot. It does not cancel inference already in progress
or bound camera/transport delay. Other consumers keep their own QoS.

Depth and LiDAR measurement inputs deliberately retain history: the mask worker
uses the exact depth timestamp of the detected RGB image and the nearest scan
within the configured tolerance (default 50 ms). Their sensor-data subscription
depth is 5; separate timestamp buffers retain 15 depth frames and 20 scans.
These are lookup histories, not a FIFO inference backlog. Replacing them with
only the newest sample would mix acquisition times or cause lookup misses.

The detector loads its model during node construction before creating the
publisher. Consequently, the raw-detections publisher is also the launch layer's
warm-model readiness signal. Starting target localization without the required
Torch/Transformers environment exits cleanly with one actionable error.

The output header is copied from the processed colour frame. All downstream
exact-stamp matching therefore refers to the image on which inference actually
ran, not callback receipt time or publish time.

## Box parsing and duplicate suppression

Model outputs below the configured confidence threshold are discarded. Each
remaining floating-point box is converted to a half-open integer rectangle:

```text
[x1, x2) x [y1, y2)
```

The bounds are clamped to the colour image, with `x2 > x1` and `y2 > y1`
guaranteed. This is the same convention consumed by mask rasterization and ROI
storage.

Candidates are sorted by descending score without mutating the model's list.
A candidate is suppressed only by a higher-scoring box with the same label when
either:

- their intersection-over-union exceeds `0.5`; or
- their intersection covers more than `0.95` of the smaller box.

The containment rule removes the observed near-full-frame duplicate surrounding
a tighter detection even when ordinary IoU is low. It deliberately does not
remove a lone oversized box; the mask node's accepted-detection size gate owns
that later safety decision. Cross-label overlap is retained.

## Published contract

The node publishes `TargetDetections` on the shared raw-detections topic. One
message describes one processed colour frame:

| Field | Meaning |
|---|---|
| `header` | Exact source-image header |
| `detected` | Whether any post-NMS detection remains |
| `count` | Number of post-NMS detections |
| `bbox_xyxy` | Flattened sequence of `count` box quadruples |
| `labels` | One label per box |
| `scores` | One confidence score per box |
| `image_width`, `image_height` | Colour grid on which every box is defined |

The arrays are index-aligned: box, label, and score at index `i` describe the
same detection. A successfully processed frame with no accepted detections still
publishes a valid message with `detected=false` and `count=0`. Downstream masks
and measurements preserve this index identity for the frame.

## Failure behavior

An image that cannot be decoded is skipped with a warning. Any other per-frame
inference or publish exception is caught by the worker boundary, logged at most
once per five seconds, and does not kill the worker. A failed frame publishes no
synthetic empty detection message because that would falsely claim inference
completed and found nothing. The cadence clock still advances, preventing a
persistent failure from becoming a hot loop.

There is no temporal tracking or stable object identity between messages. The
same physical target in consecutive frames is a new detection index each time.
Benchmark association is a separate scoring concern, and estimator outputs are
aligned only within a detection batch.

## Downstream boundaries and checks

The `box` mask gate rasterizes each accepted box directly. The `silhouette` gate
uses the same boxes as segmenter prompts. Both converge on the
[mask representation](mask_representation.md); model-specific segmentation
behavior is documented in [segmentation](segmentation.md).

[`test_detection.py`](../../src/ridgeback_autonomy/test/test_detection.py)
checks parsing, clamping, threshold forwarding, score ordering, IoU, containment,
label separation, and input immutability.
[`test_detector_node.py`](../../src/ridgeback_autonomy/test/test_detector_node.py)
checks model injection, configured-threshold forwarding, and worker survival
after inference failure. End-to-end hardware imagery and target-class expansion
remain separate validation questions.

## Timing interpretation

Result headers preserve source-image time. Subtracting that timestamp from
observer receipt time measures image age at that observer, including upstream
work and delivery. Cross-host differences require synchronized clocks; the
header alone does not establish physical exposure time. Output frequency is
not latency, and receipt of an overlay is not its appearance on a screen.

The constructor-only `detector_debug` switch enables bounded cold/warm stage
statistics: cadence wait, pending-slot age, image decode/preparation, the
detector adapter call (`inference`), parsing, publication, and total worker step.
The inference call includes adapter preprocessing/postprocessing, not only GPU
kernels. Diagnostic CUDA synchronization changes execution slightly, so keep
it off for ordinary operation. `depth_match_debug` on the mask worker reports
exact-depth availability, detection queue wait, depth preparation and estimator
reduction. Its `publication_age_ms` is elapsed time since detection receipt,
not age since the camera timestamp. Counters for stage execution and whole
batches can describe different populations, especially with no target present.
The first five samples of each timing statistic are excluded from warm figures.

Compare matched frames and equivalent scenes/configurations before attributing
an improvement. Do not sum independent stage medians as an exact frame trace.
Internal model time, camera/transport age, result delivery, and display delay
need separate measurements.

## Archived evidence

- [October 22 Intel–Thor frame timing and depth-one deployment](../../archive/engineering/2026-10-22-perception-frame-timing.md)
