"""Model-independent F3 appearance matching; no simulator identity inputs."""
from dataclasses import dataclass
from typing import Mapping, Sequence

import numpy as np


def normalize_embedding(value):
    """Return a finite unit embedding, or None for unavailable/invalid features."""
    feature = np.asarray(value, dtype=np.float32)
    if feature.ndim != 1 or feature.size == 0 or not np.isfinite(feature).all():
        return None
    norm = float(np.linalg.norm(feature))
    if not np.isfinite(norm) or norm < 1e-8:
        return None
    return feature / norm


@dataclass(frozen=True)
class AppearanceMatch:
    track_id: int | None
    similarity: float | None
    margin: float | None
    reason: str


def match_gallery(gallery: Sequence, candidates: Mapping[int, Sequence], *,
                  threshold: float, minimum_margin: float) -> AppearanceMatch:
    """Match a fixed leader gallery, rejecting ties and incompatible features.

    Thresholds are explicit: each embedding model needs measured calibration.
    This proposes an appearance match; motion continuity remains a separate cue.
    """
    if not np.isfinite(threshold) or not -1 <= threshold <= 1:
        raise ValueError('threshold must be finite and in [-1, 1]')
    if not np.isfinite(minimum_margin) or not 0 <= minimum_margin <= 2:
        raise ValueError('minimum_margin must be finite and in [0, 2]')
    references = [normalize_embedding(value) for value in gallery]
    references = [value for value in references if value is not None]
    if not references:
        return AppearanceMatch(None, None, None, 'empty_gallery')
    dimensions = {value.size for value in references}
    if len(dimensions) != 1:
        raise ValueError('gallery embeddings must use one model and dimension')
    scores = []
    for track_id, value in candidates.items():
        feature = normalize_embedding(value)
        if feature is None or feature.size not in dimensions:
            continue
        score = max(float(np.clip(np.dot(feature, ref), -1., 1.)) for ref in references)
        scores.append((score, int(track_id)))
    if not scores:
        return AppearanceMatch(None, None, None, 'no_valid_candidates')
    scores.sort(reverse=True)
    best, identity = scores[0]
    margin = best - scores[1][0] if len(scores) > 1 else None
    if best < threshold:
        return AppearanceMatch(None, best, margin, 'below_threshold')
    if margin is not None and (margin <= 1e-7 or margin < minimum_margin):
        return AppearanceMatch(None, best, margin, 'ambiguous')
    return AppearanceMatch(identity, best, margin, 'appearance_match')
