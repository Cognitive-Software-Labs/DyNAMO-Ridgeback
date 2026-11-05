"""Replay F3 recognition on cached detections; truth is used only for scoring."""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from scipy.optimize import linear_sum_assignment

from ridgeback_leader_follow.appearance import match_gallery
from ridgeback_leader_follow.fusion import box_measurement
from .dataset import SensorBag
from .image_metrics import box_iou
from .reid import CANDIDATES, RecognitionScore, person_crops
from .reid_models import official_encoder


def benchmark(run, detections, annotations, output, encoder, threshold=.72, margin=.05):
    output.mkdir(parents=True, exist_ok=False)
    bag = SensorBag(run)
    rows = [json.loads(line) for line in detections.read_text().splitlines()]
    truth = [json.loads(line) for line in annotations.read_text().splitlines()]
    if len(rows) != len(truth) or len(rows) != len(bag.schedule):
        raise ValueError('Incomplete or misaligned benchmark inputs')
    scores = RecognitionScore()
    all_visible = RecognitionScore()
    durations = []
    feature_records = []
    gallery = []
    reference_time = None
    import torch
    cuda = encoder.provenance['device'].startswith('cuda')
    if cuda:
        torch.cuda.reset_peak_memory_stats()
    with (output/'predictions.jsonl').open('w') as stream:
        for frame_index, (row, annotation, (scenario_time, _)) in enumerate(zip(rows, truth, bag.schedule, strict=True)):
            if abs(row['scenario_time']-scenario_time) > 1e-6 or abs(annotation['scenario_time']-scenario_time) > 1e-6:
                raise ValueError('Benchmark tick mismatch')
            boxes = row['boxes'] if row['input_error'] is None else []
            if boxes:
                _, rgb, depth, k, matrix = bag.camera(row['sim_time'])
                crops = person_crops(rgb, list(enumerate([box[:4] for box in boxes])))
                if cuda:
                    torch.cuda.synchronize()
                start = time.perf_counter()
                features = encoder.encode(crops)
                if cuda:
                    torch.cuda.synchronize()
                durations.append((time.perf_counter()-start)*1000.)
                positions = {}
                for identity, box in enumerate(boxes):
                    measurement = box_measurement(box[:4], box[4], depth, k, matrix)
                    if measurement is not None:
                        positions[identity] = (measurement.x, measurement.y)
            else:
                features, positions = {}, {}
            feature_records.extend((frame_index, identity, feature.copy()) for identity, feature in features.items())
            # Sensor-only initialization: nearest detected person within 30 degrees and 8 m.
            if not gallery:
                eligible = [(np.hypot(*pos), identity) for identity, pos in positions.items()
                            if pos[0] > 0 and abs(np.arctan2(pos[1],pos[0])) <= np.pi/6
                            and np.hypot(*pos) <= 8 and identity in features]
                if eligible:
                    _, identity = min(eligible)
                    gallery = [features[identity].copy()]
                    reference_time = scenario_time
            match = match_gallery(gallery, features, threshold=threshold, minimum_margin=margin)
            stream.write(json.dumps({'scenario_time': scenario_time, 'selected_detection': match.track_id,
                                     'similarity': match.similarity, 'margin': match.margin,
                                     'reason': match.reason, 'reference_time': reference_time})+'\n')
            # Evaluation boundary: labels below are never passed to inference/selection.
            people = annotation['people']
            labels = {}
            boxed_people=[p for p in people if p['box'] is not None]
            if boxed_people and boxes:
                ious = box_iou([p['box'] for p in boxed_people], [b[:4] for b in boxes])
                rr, cc = linear_sum_assignment(-ious)
                labels = {int(c): boxed_people[r]['id'] for r, c in zip(rr, cc) if ious[r, c] >= .5}
            leader = next((p for p in people if p['role'] == 'leader'), None)
            leader_visible = leader is not None and leader.get('visible_fraction', 0) >= .2
            all_visible.update(match.track_id, labels, 'lf_leader', leader_eligible=leader_visible)
            # Score nearby leader selection on frames where the visible leader is <= 3 m.
            # A farther visible leader is excluded, not treated as absent.
            if not leader_visible or leader['distance_m'] <= 3:
                nearby_selection = match.track_id
                if nearby_selection is not None and nearby_selection in positions:
                    if np.hypot(*positions[nearby_selection]) > 3:
                        nearby_selection = None
                scores.update(nearby_selection, labels, 'lf_leader', leader_eligible=leader_visible)
    np.savez_compressed(output/'embeddings.npz',
        frames=np.array([r[0] for r in feature_records],dtype=np.int32),
        detections=np.array([r[1] for r in feature_records],dtype=np.int32),
        features=np.stack([r[2] for r in feature_records]) if feature_records else np.empty((0,encoder.dimension),np.float32))
    report = {'candidate': encoder.name, 'nearby': scores.report(), 'all_visible': all_visible.report(),
              'reference_time': reference_time, 'parameters': {'threshold': threshold, 'minimum_margin': margin,
                             'acquire_range_m':8, 'acquire_half_angle_deg':30},
              'processing_median_ms': float(np.median(durations)) if durations else None,
              'processing_p95_ms': float(np.percentile(durations,95)) if durations else None,
              'timing_scope': 'crop preprocessing and embedding batches only; excludes decoding/detection/matching',
              'gpu_peak_allocated_bytes': torch.cuda.max_memory_allocated() if cuda else None,
              'provenance': encoder.provenance,
              'input_sha256': {str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (detections, annotations)},
              'scope': 'fixed sensor-initialized gallery; recognition isolation on detector boxes, not integrated lock qualification'}
    (output/'score.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('run','detections','annotations','output','model-dir'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--candidate', choices=CANDIDATES, required=True)
    parser.add_argument('--device', default='cuda')
    parser.add_argument('--threshold', type=float, default=.72)
    parser.add_argument('--margin', type=float, default=.05)
    args=parser.parse_args()
    encoder=official_encoder(args.candidate,args.model_dir,args.device)
    benchmark(args.run,args.detections,args.annotations,args.output,encoder,args.threshold,args.margin)


if __name__ == '__main__':
    main()
