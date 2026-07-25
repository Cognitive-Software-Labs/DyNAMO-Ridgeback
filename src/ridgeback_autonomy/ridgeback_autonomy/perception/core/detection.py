from __future__ import annotations

import importlib
import math

from ridgeback_autonomy.common.models import Detection, DetectionBatch


DETECTION_MODEL_DEFAULT = 'google/owlv2-base-patch16-ensemble'
DETECTION_LABELS = ['humanoid robot']
DETECTION_THRESHOLD = 0.55
NMS_IOU_THRESHOLD = 0.5
# A candidate almost entirely inside a kept box (high intersection-over-smaller)
# is suppressed even when its IoU is low -- the near-full-frame box overlapping a
# tight box is the outlier source. Conservative: near-total containment only.
# Near-total containment only: a full giant-box-over-real-box still scores 1.0
# and is suppressed, while a partly-occluded neighbour (~0.85-0.95 inside a
# nearer box) survives, preserving multi-instance recall in crowded scenes.
NMS_CONTAINMENT_THRESHOLD = 0.95


def resolve_torch_device() -> str:
    torch_spec = importlib.util.find_spec('torch')
    if torch_spec is None:
        return 'cpu'
    torch = importlib.import_module('torch')
    return 'cuda' if torch.cuda.is_available() else 'cpu'


class OwlV2Detector:
    def __init__(self, model_name: str, logger) -> None:
        self.model_name = model_name
        self.logger = logger
        self.device = resolve_torch_device()
        self._pipeline = None

    def load(self) -> None:
        try:
            transformers = importlib.import_module('transformers')
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "G1 perception needs 'transformers' (and torch). Set up the "
                "perception_venv — see README 'Set up G1 perception venv' — then "
                "launch with g1_perception_enabled:=true."
            ) from exc
        self.logger.info(f'Loading detection model: {self.model_name}')
        self.logger.info(f'Using detection device: {self.device}')
        self._pipeline = transformers.pipeline(
            task='zero-shot-object-detection',
            model=self.model_name,
            device=self.device,
        )
        self.logger.info('Detection model loaded.')

    def detect(
        self,
        image,
        candidate_labels: list[str] | None = None,
        threshold: float = DETECTION_THRESHOLD,
        top_k: int | None = None,
    ) -> list[dict]:
        if self._pipeline is None:
            self.load()
        labels = candidate_labels or DETECTION_LABELS
        # Pin threshold/top_k explicitly: pushes filtering into the pipeline so we
        # stop parsing throwaway low-score boxes, and stops behavior riding on
        # transformers-version defaults (5.10.2: threshold=0.1, top_k=None).
        return self._pipeline(
            image,
            candidate_labels=labels,
            threshold=threshold,
            top_k=top_k,
        )


def compute_iou(box_a: tuple[int, int, int, int], box_b: tuple[int, int, int, int]) -> float:
    x1 = max(box_a[0], box_b[0])
    y1 = max(box_a[1], box_b[1])
    x2 = min(box_a[2], box_b[2])
    y2 = min(box_a[3], box_b[3])
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    area_a = (box_a[2] - box_a[0]) * (box_a[3] - box_a[1])
    area_b = (box_b[2] - box_b[0]) * (box_b[3] - box_b[1])
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def intersection_over_smaller(
    box_a: tuple[int, int, int, int], box_b: tuple[int, int, int, int]
) -> float:
    x1 = max(box_a[0], box_b[0])
    y1 = max(box_a[1], box_b[1])
    x2 = min(box_a[2], box_b[2])
    y2 = min(box_a[3], box_b[3])
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    area_a = (box_a[2] - box_a[0]) * (box_a[3] - box_a[1])
    area_b = (box_b[2] - box_b[0]) * (box_b[3] - box_b[1])
    smaller = min(area_a, area_b)
    return inter / smaller if smaller > 0 else 0.0


def non_maximum_suppression(detections: list[Detection], iou_threshold: float) -> list[Detection]:
    # Copy before ordering -- never mutate the caller's list.
    ordered = sorted(detections, key=lambda detection: detection.score, reverse=True)
    keep: list[Detection] = []
    for detection in ordered:
        # Suppress only against a same-label kept box (cross-label overlap is
        # not a duplicate) that either overlaps enough (IoU) or nearly contains
        # this one (intersection-over-smaller -- the giant-box case).
        suppressed = any(
            detection.label == kept.label
            and (
                compute_iou(detection.bbox_xyxy, kept.bbox_xyxy) > iou_threshold
                or intersection_over_smaller(detection.bbox_xyxy, kept.bbox_xyxy)
                > NMS_CONTAINMENT_THRESHOLD
            )
            for kept in keep
        )
        if suppressed:
            continue
        keep.append(detection)
    return keep


def parse_owl_detections(
    outputs: list[dict],
    image_width: int,
    image_height: int,
    threshold: float,
) -> DetectionBatch:
    detections: list[Detection] = []
    for raw_detection in outputs:
        score = float(raw_detection['score'])
        if score < threshold:
            continue

        box = raw_detection['box']
        x1 = max(0, min(image_width - 1, int(box['xmin'])))
        y1 = max(0, min(image_height - 1, int(box['ymin'])))
        x2 = max(x1 + 1, min(image_width, int(math.ceil(box['xmax']))))
        y2 = max(y1 + 1, min(image_height, int(math.ceil(box['ymax']))))
        detections.append(
            Detection(
                bbox_xyxy=(x1, y1, x2, y2),
                label=str(raw_detection['label']),
                score=score,
            )
        )

    batch = DetectionBatch(
        image_width=image_width,
        image_height=image_height,
        detections=non_maximum_suppression(detections, NMS_IOU_THRESHOLD),
    )
    return batch
