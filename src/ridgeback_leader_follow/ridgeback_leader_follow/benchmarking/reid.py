"""F3 embedding boundary and identity scoring, independent of model frameworks.

Adapters receive RGB images and detected boxes only. Identity labels enter the
separate scorer, never the encoder or appearance matcher.
"""
from dataclasses import dataclass
import math

import numpy as np

from ridgeback_leader_follow.appearance import normalize_embedding

CANDIDATES = ('osnet-ain', 'clip-reid', 'transreid', 'solider', 'dinov2')


@dataclass(frozen=True)
class PersonCrop:
    track_id: int
    rgb: np.ndarray


def person_crops(rgb, detections, minimum_side=8, horizontal_margin=0):
    """Clip detected xyxy boxes to RGB bounds; reject invalid/tiny crops.

    Detections are (track_id, xyxy). No resize or normalization here: those
    are model-specific and must match each checkpoint's official pipeline.
    """
    rgb = np.asarray(rgb)
    if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
        raise ValueError('expected HxWx3 uint8 RGB')
    if minimum_side < 1:
        raise ValueError('minimum_side must be positive')
    height, width = rgb.shape[:2]
    crops, seen = [], set()
    for identity, box in detections:
        identity = int(identity)
        if identity in seen:
            raise ValueError('duplicate track id in one frame')
        seen.add(identity)
        box = np.asarray(box, dtype=float)
        if box.shape != (4,) or not np.isfinite(box).all():
            continue
        if horizontal_margin and (box[0] <= horizontal_margin or box[2] >= width-horizontal_margin):
            continue
        x1, y1 = np.floor(np.clip(box[:2], [0, 0], [width, height])).astype(int)
        x2, y2 = np.ceil(np.clip(box[2:], [0, 0], [width, height])).astype(int)
        if x2-x1 < minimum_side or y2-y1 < minimum_side:
            continue
        crops.append(PersonCrop(identity, rgb[y1:y2, x1:x2].copy()))
    return crops


class AppearanceEncoder:
    """Wrap an official model's batch inference callable in a checked interface."""
    def __init__(self, name, infer, dimension):
        if name not in CANDIDATES or dimension <= 0:
            raise ValueError('unknown candidate or invalid embedding dimension')
        self.name, self.infer, self.dimension = name, infer, int(dimension)

    def encode(self, crops):
        if not crops:
            return {}
        features = np.asarray(self.infer([crop.rgb for crop in crops]), dtype=np.float32)
        if features.shape != (len(crops), self.dimension):
            raise ValueError('model output does not match the declared batch/dimension')
        result = {}
        for crop, feature in zip(crops, features):
            if crop.track_id in result:
                raise ValueError('duplicate track id')
            normalized = normalize_embedding(feature)
            if normalized is not None:
                result[crop.track_id] = normalized
        return result


class RecognitionScore:
    """Count correct/wrong/rejected selections with an explicit eligible set.

    The caller defines eligibility from evaluation-only truth (e.g. nearby
    people within 3 m). Rejection counts as a miss when the leader is eligible;
    acceptance on an absent leader is always a false selection.
    """
    def __init__(self):
        self.frames = self.eligible = self.correct = self.wrong = self.rejected = 0
        self.absent_frames = self.absent_false_accepts = 0
        self.similarity = []

    def update(self, selected_track, truth_by_track, leader_identity, *, leader_eligible,
               similarity=None):
        self.frames += 1
        if similarity is not None and math.isfinite(similarity):
            self.similarity.append(float(similarity))
        if leader_eligible:
            self.eligible += 1
            if selected_track is None:
                self.rejected += 1
            elif truth_by_track.get(selected_track) == leader_identity:
                self.correct += 1
            else:
                self.wrong += 1
        else:
            self.absent_frames += 1
            if selected_track is not None:
                self.absent_false_accepts += 1

    def report(self):
        return {'frames': self.frames, 'leader_eligible_frames': self.eligible,
                'correct': self.correct, 'wrong': self.wrong, 'rejected': self.rejected,
                'leader_pick_accuracy': self.correct/self.eligible if self.eligible else None,
                'absent_frames': self.absent_frames,
                'absent_false_accepts': self.absent_false_accepts}
