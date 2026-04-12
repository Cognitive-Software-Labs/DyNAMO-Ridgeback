from __future__ import annotations

from ridgeback_autonomy.perception.core.detection import compute_iou, parse_owl_detections


def test_compute_iou_handles_overlap() -> None:
    iou = compute_iou((0, 0, 10, 10), (5, 5, 15, 15))

    assert round(iou, 4) == round(25.0 / 175.0, 4)


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
