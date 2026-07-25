from __future__ import annotations

from ridgeback_autonomy.common.models import Detection
from ridgeback_autonomy.perception.core.detection import (
    DETECTION_LABELS,
    DETECTION_THRESHOLD,
    NMS_IOU_THRESHOLD,
    OwlV2Detector,
    compute_iou,
    intersection_over_smaller,
    non_maximum_suppression,
    parse_owl_detections,
)


def test_compute_iou_handles_overlap() -> None:
    iou = compute_iou((0, 0, 10, 10), (5, 5, 15, 15))

    assert round(iou, 4) == round(25.0 / 175.0, 4)


def test_intersection_over_smaller_full_containment() -> None:
    # A small box fully inside a giant one: IoS is 1.0 while IoU is tiny.
    giant = (0, 0, 100, 100)
    small = (10, 10, 30, 30)

    assert intersection_over_smaller(giant, small) == 1.0
    assert compute_iou(giant, small) < NMS_IOU_THRESHOLD


def test_nms_containment_drops_giant_box_around_higher_score_box() -> None:
    # The outlier case: a near-full-frame box (low IoU with everything) that
    # contains a tighter, higher-scoring box must be suppressed by it.
    giant = Detection(bbox_xyxy=(0, 0, 100, 100), label='humanoid robot', score=0.60)
    tight = Detection(bbox_xyxy=(10, 10, 30, 30), label='humanoid robot', score=0.90)

    kept = non_maximum_suppression([giant, tight], NMS_IOU_THRESHOLD)

    assert kept == [tight]


def test_nms_keeps_lone_giant_box() -> None:
    # Limitation, encoded: with no competing box to outscore it, the giant
    # box survives -- the detector adds no size gate (lone-giant defense is the
    # mask node's MAX_BOX_FRAME_FRACTION gate, not here).
    giant = Detection(bbox_xyxy=(0, 0, 100, 100), label='humanoid robot', score=0.60)

    assert non_maximum_suppression([giant], NMS_IOU_THRESHOLD) == [giant]


def test_nms_keeps_overlapping_boxes_of_different_labels() -> None:
    # Suppression is per-label. Two identical boxes with different labels
    # (IoU 1.0) are not duplicates -- both are kept.
    robot = Detection(bbox_xyxy=(0, 0, 100, 100), label='humanoid robot', score=0.90)
    person = Detection(bbox_xyxy=(0, 0, 100, 100), label='person', score=0.80)

    kept = non_maximum_suppression([robot, person], NMS_IOU_THRESHOLD)

    assert len(kept) == 2
    assert {detection.label for detection in kept} == {'humanoid robot', 'person'}


def test_non_maximum_suppression_does_not_mutate_input() -> None:
    low = Detection(bbox_xyxy=(0, 0, 10, 10), label='humanoid robot', score=0.10)
    high = Detection(bbox_xyxy=(50, 50, 60, 60), label='humanoid robot', score=0.90)
    detections = [low, high]

    non_maximum_suppression(detections, NMS_IOU_THRESHOLD)

    assert detections == [low, high]


class _RecordingPipeline:
    def __init__(self) -> None:
        self.calls: list[dict] = []

    def __call__(self, image, **kwargs) -> list:
        self.calls.append(kwargs)
        return []


class _NullLogger:
    def info(self, *args, **kwargs) -> None:
        pass

    def warn(self, *args, **kwargs) -> None:
        pass


def _detector_with_recording_pipeline() -> tuple[OwlV2Detector, _RecordingPipeline]:
    detector = OwlV2Detector('model', _NullLogger())
    pipeline = _RecordingPipeline()
    detector._pipeline = pipeline  # skip load(); no torch/transformers needed
    return detector, pipeline


def test_detect_passes_threshold_and_top_k_to_pipeline() -> None:
    detector, pipeline = _detector_with_recording_pipeline()

    detector.detect('img', threshold=0.7, top_k=5)

    assert pipeline.calls[0]['threshold'] == 0.7
    assert pipeline.calls[0]['top_k'] == 5
    assert pipeline.calls[0]['candidate_labels'] == DETECTION_LABELS


def test_detect_defaults_pin_pipeline_behavior() -> None:
    detector, pipeline = _detector_with_recording_pipeline()

    detector.detect('img')

    assert pipeline.calls[0]['threshold'] == DETECTION_THRESHOLD
    assert pipeline.calls[0]['top_k'] is None


def test_parse_owl_detections_applies_threshold_and_nms() -> None:
    outputs = [
        {
            'score': 0.95,
            'label': 'humanoid robot',
            'box': {'xmin': 10.0, 'ymin': 20.0, 'xmax': 50.0, 'ymax': 80.0},
        },
        {
            'score': 0.80,
            'label': 'humanoid robot',
            'box': {'xmin': 12.0, 'ymin': 22.0, 'xmax': 48.0, 'ymax': 78.0},
        },
        {
            'score': 0.40,
            'label': 'humanoid robot',
            'box': {'xmin': 100.0, 'ymin': 100.0, 'xmax': 140.0, 'ymax': 170.0},
        },
    ]

    batch = parse_owl_detections(outputs, image_width=160, image_height=120, threshold=0.55)

    assert batch.detected is True
    assert batch.count == 1
    assert batch.detections[0].bbox_xyxy == (10, 20, 50, 80)
    assert batch.detections[0].score == 0.95
