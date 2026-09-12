# Detection

The detection component converts RGB frames into index-aligned target boxes.
It owns model inference, confidence filtering, duplicate suppression, box
normalization, and the `TargetDetections` message contract. It does not segment,
track, associate across time, estimate distance, or choose a fallback estimator.

## Current implementation

`target_detector_node` subscribes to the configured colour image and runs the
ROS-free `OwlV2Detector` adapter from `core/detection.py`. The current defaults
are:

| Setting | Default |
|---|---|
| Model | `google/owlv2-base-patch16-ensemble` |
| Candidate label | `humanoid robot` |
| Confidence threshold | `0.55` |
| Maximum inference cadence | `10 Hz` |
| Same-label NMS IoU threshold | `0.5` |
| Same-label containment threshold | `0.95` intersection-over-smaller |

OWLv2 is an implementation choice, not a downstream dependency. Consumers use
only the published image grid, boxes, labels, scores, and original image header.
A replacement detector must preserve that boundary or change it explicitly.

## Frame and scheduling behavior

The node keeps one latest colour message rather than a frame queue. The worker
waits until its monotonic cadence permits another inference, then takes the
latest available frame. Frames superseded before processing are intentionally
dropped. This bounds queued work when inference is slower than the camera and
prevents latency from growing without limit.

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
