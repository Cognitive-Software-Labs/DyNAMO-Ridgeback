"""World-position association and tracking diagnostics, not HOTA or IDF1.

Predictions and annotations must share timestamps and world coordinates.
Unmatched truth is a miss, including frames with no predictions. A switch
compares a person's last matched ID across gaps; initial acquisition is not
a switch. Track IDs must be unique within a frame.
"""
from dataclasses import dataclass
import math

import numpy as np
from scipy.optimize import linear_sum_assignment


@dataclass(frozen=True)
class Position:
    identity: str
    x: float
    y: float
    role: str = 'unknown'

    def __post_init__(self):
        if not self.identity or not all(math.isfinite(v) for v in (self.x,self.y)):
            raise ValueError('Positions need a nonempty ID and finite coordinates')


def associate(truth, predictions, gate):
    """Maximum-cardinality gated association, then minimum total distance."""
    if not math.isfinite(gate) or gate <= 0:
        raise ValueError('Association gate must be positive and finite')
    for values in (truth,predictions):
        if len({p.identity for p in values}) != len(values):
            raise ValueError('Duplicate identity in frame')
    if not truth or not predictions:
        return []
    distance = np.array([[math.hypot(t.x-p.x,t.y-p.y) for p in predictions] for t in truth])
    # One forbidden match costs more than every possible allowed match,
    # so assignment maximizes valid cardinality before minimizing distance.
    penalty = (min(distance.shape)+1)*(gate+1)
    rows,cols = linear_sum_assignment(np.where(distance <= gate,distance,penalty))
    return [(int(i),int(j),float(distance[i,j])) for i,j in zip(rows,cols) if distance[i,j] <= gate]


def score_frames(frames, gate=.75):
    previous_ids = {}
    previous_matches = set()
    previously_seen = set()
    tp = fp = fn = switches = losses = leader_losses = fragments = 0
    errors = []
    last_time = -math.inf
    count = 0
    for stamp,truth,predictions in frames:
        if not math.isfinite(stamp) or stamp <= last_time:
            raise ValueError('Frame timestamps must be finite and strictly increasing')
        last_time = stamp
        count += 1
        matches = associate(truth,predictions,gate)
        current = {truth[i].identity for i,_,_ in matches}
        tp += len(matches); fp += len(predictions)-len(matches); fn += len(truth)-len(matches)
        for person in truth:
            if person.identity in previous_matches and person.identity not in current:
                losses += 1
                leader_losses += person.role == 'leader'
        for i,j,error in matches:
            identity,track = truth[i].identity,predictions[j].identity
            if identity in previous_ids and previous_ids[identity] != track:
                switches += 1
            if identity in previously_seen and identity not in previous_matches:
                fragments += 1
            previous_ids[identity] = track
            previously_seen.add(identity)
            errors.append(error)
        previous_matches = current
    if not count:
        raise ValueError('No truth frames to score')
    return {'frames':count,'true_positives':tp,'false_positives':fp,'false_negatives':fn,
            'precision':tp/(tp+fp) if tp+fp else None,
            'recall':tp/(tp+fn) if tp+fn else None,
            'id_switches':switches,'track_loss_events':losses,
            'leader_track_loss_events':leader_losses,'fragmentations':fragments,
            'position_rmse_m':math.sqrt(sum(x*x for x in errors)/len(errors)) if errors else None,
            'association_gate_m':gate,
            'scope':'world-position tracking diagnostics; not HOTA, IDF1, visibility or detector AP'}
