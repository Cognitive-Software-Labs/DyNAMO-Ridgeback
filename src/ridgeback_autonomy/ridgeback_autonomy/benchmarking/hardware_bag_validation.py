"""Unscored, resumable hardware diagnostics using production measurement paths.

Hardware evidence intentionally has its own envelope: it has no simulator truth
or measured yaw and cannot accidentally enter the scored replay trial reader.
NumPy payloads, cached mask outcomes, and replay_sensor_event remain shared.
"""
from __future__ import annotations

from collections import Counter
from contextlib import contextmanager
import fcntl
import importlib.metadata
import json
import os
from pathlib import Path
import resource
import shutil
import subprocess
import time

import numpy as np

from ridgeback_autonomy.benchmarking.hardware_bag_source import (
    assemble_events, digest, file_hash, read_json, ros_messages, select_session,
)
from ridgeback_autonomy.benchmarking.layered_replay import replay_sensor_event
from ridgeback_autonomy.benchmarking.replay_artifacts import CachedMaskOutcome
from ridgeback_autonomy.benchmarking.replay_materialization import _box_outcomes, _segment_outcomes
from ridgeback_common.miss_reason import MissReason
from ridgeback_localization.core.mask import MaskPrecision, MaskRegion

MODELS = {
    'owl': ('google/owlv2-base-patch16-ensemble', 'cfd3195ba4ea9592eec887ded089f4c08eff231d', 'owlv2'),
    'slimsam': ('Zigeng/SlimSAM-uniform-50', 'd67c795ed0caaec7423d799da25cdeded48aac40', 'sam'),
    'monocular': ('depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf', '8078d68a9c75a972131914f6afd0c1723be0da7f', 'depth_anything'),
}
DEPTH_ESTIMATORS = ('projective_ranging', 'euclidean_reconstruction')
PATHS = tuple({'id': f'{estimator}__{mask}__{source}', 'estimator': estimator,
               'mask': mask, 'source': source}
              for mask in ('box', 'slimsam')
              for source, estimators in (('stereo', DEPTH_ESTIMATORS), ('monocular', DEPTH_ESTIMATORS),
                                          ('scan', ('polar_profiling',)))
              for estimator in estimators)


class InputFault(ValueError):
    """One bag's raw input is unusable; unaffected bags may continue."""


class SharedProcessingFault(RuntimeError):
    """Infrastructure/model failure; stop before multiplying invalid results."""


def atomic_json(path, document):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + '.tmp')
    with temp.open('w') as stream:
        json.dump(document, stream, sort_keys=True, indent=2, allow_nan=False)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temp, path)


@contextmanager
def output_lock(root):
    root = Path(root)
    root.mkdir(parents=True, exist_ok=True)
    with (root / '.lock').open('a') as stream:
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise ValueError(f'Another writer owns {root}') from exc
        try:
            yield
        finally:
            fcntl.flock(stream, fcntl.LOCK_UN)


def save_payload(path, value):
    """Atomic two-file commit, with a hashed NPZ and JSON commit marker."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    arrays = {}

    def pack(item):
        if isinstance(item, np.ndarray):
            key = f'array_{len(arrays)}'
            arrays[key] = item
            return {'__array__': key}
        if isinstance(item, np.generic):
            return item.item()
        if isinstance(item, dict):
            return {k: pack(v) for k, v in item.items()}
        if isinstance(item, (list, tuple)):
            return [pack(v) for v in item]
        return item
    document = pack(value)
    array_path = path.with_suffix('.npz')
    with array_path.with_suffix('.npz.tmp').open('wb') as stream:
        np.savez_compressed(stream, **arrays)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(array_path.with_suffix('.npz.tmp'), array_path)
    atomic_json(path, {'schema': 'dynamo-hardware-event/1', 'npz_sha256': file_hash(array_path), 'value': document})


def load_payload(path):
    path = Path(path)
    document = read_json(path)
    if document['schema'] != 'dynamo-hardware-event/1' or file_hash(path.with_suffix('.npz')) != document['npz_sha256']:
        raise ValueError(f'Invalid hardware payload: {path}')
    with np.load(path.with_suffix('.npz'), allow_pickle=False) as arrays:
        def unpack(item):
            if isinstance(item, dict):
                if set(item) == {'__array__'}:
                    return arrays[item['__array__']].copy()
                return {k: unpack(v) for k, v in item.items()}
            if isinstance(item, list):
                return [unpack(v) for v in item]
            return item
        return unpack(document['value'])


def effective_settings():
    from ridgeback_localization.core import depth_common as dc, isolation_2d as i2, isolation_3d as i3
    from ridgeback_localization.core import polar_profiling as polar, ranging_defaults as rd, segmentation as seg
    from ridgeback_localization.core.box_gate import MAX_BOX_FRAME_FRACTION
    from ridgeback_localization.core.depth_sources import MONOCULAR_USABLE_RANGE_FRACTION
    from ridgeback_localization.core.detection import NMS_IOU_THRESHOLD, NMS_CONTAINMENT_THRESHOLD
    arguments = {
        'isolation_2d': i2.ISOLATION_2D_DEFAULT,
        'isolation_2d_bin_width_m': dc.NEAREST_MODE_BIN_WIDTH_M_DEFAULT,
        'isolation_2d_band_m': i2.NEAREST_MODE_BAND_M_DEFAULT,
        'isolation_2d_min_bin_fraction': dc.NEAREST_MODE_MIN_BIN_FRACTION_DEFAULT,
        'isolation_3d': i3.ISOLATION_3D_DEFAULT,
        'isolation_3d_floor_margin_m': i3.FLOOR_MARGIN_M_DEFAULT,
        'isolation_3d_percentile': rd.FRONT_PERCENTILE,
        'isolation_3d_ahead_m': rd.INLIER_AHEAD_MARGIN_M,
        'isolation_3d_behind_m': rd.INLIER_BEHIND_MARGIN_M,
        'isolation_3d_bin_width_m': dc.NEAREST_MODE_BIN_WIDTH_M_DEFAULT,
        'isolation_3d_min_bin_fraction': dc.NEAREST_MODE_MIN_BIN_FRACTION_DEFAULT,
        'polar_range_jump_m': polar.RANGE_JUMP_M_DEFAULT,
        'polar_range_band_m': polar.RANGE_BAND_M_DEFAULT,
        'polar_min_valid_rays': polar.MIN_VALID_RAYS_DEFAULT,
        'min_valid_pixels': rd.MIN_VALID_SAMPLES, 'mask_depth_max_meters': 0.0,
    }
    return {'arguments': {k: str(v) for k, v in arguments.items()},
            'front_offset_m': 0.465, 'base_above_floor_m': i3.BASE_ABOVE_FLOOR_M_DEFAULT,
            'prompt': 'humanoid robot', 'threshold': 0.30,
            'nms_iou': NMS_IOU_THRESHOLD, 'nms_containment': NMS_CONTAINMENT_THRESHOLD,
            'max_box_frame_fraction': MAX_BOX_FRAME_FRACTION,
            'segmentation_prompt_padding_rel': seg.PROMPT_PADDING_REL_DEFAULT,
            'segmentation_min_iou': seg.SEGMENTATION_MIN_PREDICTED_IOU_DEFAULT,
            'monocular_usable_range_fraction': MONOCULAR_USABLE_RANGE_FRACTION,
            'stereo_usable_max_m': None,
            'limitations': ['monocular 0.9 fraction is an implementation default, not validated reliability',
                            'pinhole model retains recorded distortion without correcting it',
                            'published depth units used; capture-time driver binary not fully pinned',
                            'offline duration is not live throughput; numerical success is not accuracy',
                            'all nominal references provisional; no final accuracy aggregates']}


def code_identity():
    import ridgeback_common.models as models
    import ridgeback_localization.measurement_pipeline as pipeline
    roots = [Path(__file__).resolve().parent, Path(models.__file__).resolve().parent, Path(pipeline.__file__).resolve().parent]
    # Includes producer and measurement dependencies, excludes unrelated launch,
    # renderer, generated graphs, and documentation changes in this shared tree.
    files = {str(p.resolve()): file_hash(p) for root in roots for p in sorted(root.rglob('*.py'))}
    return {'files_sha256': files, 'signature': digest(files)}


def model_snapshots(cache):
    result = {}
    for role, (identifier, revision, model_type) in MODELS.items():
        path = Path(cache).expanduser().resolve() / ('models--'+identifier.replace('/', '--')) / 'snapshots' / revision
        if not path.is_dir():
            raise ValueError(f'Missing local {role} snapshot: {path}; no download/fallback allowed')
        config = read_json(path / 'config.json')
        if config.get('model_type') != model_type:
            raise ValueError(f'{role}: snapshot model_type mismatch')
        weights = list(path.glob('*.safetensors')) + list(path.glob('pytorch_model*.bin'))
        if not weights or any(not p.is_file() for p in weights):
            raise ValueError(f'{role}: local model weights unavailable')
        # Cache path namespace pins identifier, revision pins snapshot. Metadata
        # and weight digests pin contents including local replacements/symlinks.
        inventory = {p.name: {'bytes': p.stat().st_size, 'sha256': file_hash(p)}
                     for p in sorted(path.iterdir()) if p.is_file()}
        result[role] = {'id': identifier, 'revision': revision, 'path': str(path),
                        'model_type': model_type, 'files': inventory}
        if role == 'monocular':
            if config.get('depth_estimation_type') != 'metric' or not math_is_positive(config.get('max_depth')):
                raise ValueError('Monocular snapshot must declare metric max_depth')
            result[role]['declared_max_depth_m'] = float(config['max_depth'])
    return result


def math_is_positive(value):
    import math
    return isinstance(value, (float, int)) and math.isfinite(value) and value > 0


def describe(session, output, cache, *, device='cuda:0', base_frame='base_link'):
    if device != 'cpu' and not (device.startswith('cuda:') and device[5:].isdigit()):
        raise ValueError('Device must be explicit cpu or cuda:N; no auto device')
    settings = effective_settings()
    snapshots = model_snapshots(cache)
    settings['monocular_usable_max_m'] = snapshots['monocular']['declared_max_depth_m'] * settings['monocular_usable_range_fraction']
    versions = {}
    for name in ('numpy', 'torch', 'transformers', 'Pillow', 'scipy', 'opencv-python'):
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            versions[name] = None
    repo = Path(__file__).resolve().parents[4]
    revision = subprocess.check_output(['git', '-C', str(repo), 'rev-parse', 'HEAD'], text=True).strip()
    patch = subprocess.check_output(['git', '-C', str(repo), 'diff', '--binary'])
    job = {'schema': 'dynamo-hardware-job/1', 'selection': select_session(session),
           'output': str(Path(output).expanduser().resolve()), 'paths': list(PATHS),
           'settings': settings, 'models': snapshots, 'device': device, 'dtype': 'float32',
           'base_frame': base_frame, 'dependencies': versions, 'code': code_identity(),
           'checkout': {'revision': revision, 'tracked_patch_sha256': __import__('hashlib').sha256(patch).hexdigest()},
           'policy': {'interval': 'receive [start,end)', 'sampling': 'first RGB header stamp anchors 100 ms buckets; first source frame per occupied bucket; duplicates counted; regression is input fault',
                      'depth': 'exact RGB header equality, first source-order tie',
                      'scan': 'nearest within inclusive 50 ms; earlier stamp then source order',
                      'context': 'full-bag disk index; colour info latest header <= RGB; static TF all bag; dynamic TF bracketed interpolation only',
                      'memory': 'one bag metadata/TF/scans and a decoded frame pair; all depth candidates indexed on disk',
                      'scoring': 'unscored; no final accuracy or fabricated yaw'}}
    job['identity'] = digest(job)
    return job


def validate_job(job, *, current_sources=True):
    unsigned = {k: v for k, v in job.items() if k != 'identity'}
    if job.get('schema') != 'dynamo-hardware-job/1' or digest(unsigned) != job.get('identity'):
        raise ValueError('Job identity/schema mismatch; create a new output/job for a configuration change')
    if list(PATHS) != job['paths'] or effective_settings() != {k: v for k, v in job['settings'].items() if k != 'monocular_usable_max_m'}:
        raise ValueError('Resolved settings changed; describe into a new output')
    if code_identity() != job['code']:
        raise ValueError('Source/producer implementation changed; describe into a new output')
    if current_sources and select_session(job['selection']['session']) != job['selection']:
        raise ValueError('Raw source/selection/reference identity changed; refuse resume')
    for model in job['models'].values():
        path = Path(model['path'])
        for name, frozen in model['files'].items():
            p = path / name
            if not p.is_file() or p.stat().st_size != frozen['bytes'] or file_hash(p) != frozen['sha256']:
                raise ValueError(f'Local snapshot changed/unavailable: {p}')
    for name, version in job['dependencies'].items():
        try:
            current = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            current = None
        if current != version:
            raise ValueError(f'Dependency {name} changed: {version} -> {current}')


class ProductionProducers:
    def __init__(self, job):
        # Hard offline boundary, including eager model construction.
        os.environ['HF_HUB_OFFLINE'] = '1'
        os.environ['TRANSFORMERS_OFFLINE'] = '1'
        import torch
        import transformers
        from ridgeback_autonomy.benchmarking.replay_materialization import _PrintLogger
        from ridgeback_localization.core.detection import OwlV2Detector
        from ridgeback_localization.core.depth_sources import MonocularDepthSource
        from ridgeback_localization.core.segmentation import SamBoxSegmenter
        device = job['device']
        if device.startswith('cuda:') and (not torch.cuda.is_available() or int(device[5:]) >= torch.cuda.device_count()):
            raise SharedProcessingFault(f'Requested device {device} unavailable; no fallback')
        logger = _PrintLogger()
        snapshots = job['models']
        self.owl = OwlV2Detector(snapshots['owl']['path'], logger)
        self.owl.device = device
        self.owl._pipeline = transformers.pipeline(task='zero-shot-object-detection', model=snapshots['owl']['path'], device=device, dtype=torch.float32)
        self.segmenter = SamBoxSegmenter(snapshots['slimsam']['path'], logger, device=device, dtype='float32')
        self.segmenter.load()
        self.mono = MonocularDepthSource(snapshots['monocular']['path'], device, logger)
        self.mono._build_pipeline = lambda: transformers.pipeline(task='depth-estimation', model=snapshots['monocular']['path'], device=device, dtype=torch.float32)
        if not self.mono.load() or self.mono.usable_max_m != job['settings']['monocular_usable_max_m']:
            raise SharedProcessingFault('Monocular model failed to load or ceiling differs from frozen job')
        self.settings = job['settings']
        self.device = device
        for pipeline in (self.owl._pipeline, self.mono._pipeline):
            if str(pipeline.device) != device or next(pipeline.model.parameters()).dtype != torch.float32:
                raise SharedProcessingFault('Resolved model device/dtype differs from frozen job')
        if (str(next(self.segmenter._model.parameters()).device) != device
                or next(self.segmenter._model.parameters()).dtype != torch.float32):
            raise SharedProcessingFault('Resolved segmentation device differs from frozen job')

    def detect(self, rgb):
        from PIL import Image
        from ridgeback_localization.core.detection import parse_owl_detections
        batch = parse_owl_detections(self.owl.detect(Image.fromarray(rgb), [self.settings['prompt']], self.settings['threshold']),
                                     rgb.shape[1], rgb.shape[0], self.settings['threshold'])
        return [{'bbox_xyxy': list(d.bbox_xyxy), 'label': d.label, 'score': d.score} for d in batch.detections]

    def monocular(self, rgb):
        result = self.mono.produce_from_rgb(rgb, None)
        if result is None:
            raise SharedProcessingFault('Shared monocular inference failed; see producer warning')
        return result[0]


def freeze_inference(event, producers, settings):
    try:
        event = dict(event)
        event['detections'] = producers.detect(event['rgb'])
        event['count'] = len(event['detections'])
        event['detected'] = bool(event['count'])
        # A selected empty-detection frame still gets one monocular product and
        # an explicit ten-path result; it is not removed from denominators.
        monocular = np.asarray(producers.monocular(event['rgb']), dtype=np.float32)
        if monocular.shape != event['rgb'].shape[:2] or not np.any(np.isfinite(monocular) & (monocular > 0)):
            raise SharedProcessingFault('Monocular producer returned malformed/no-positive metric depth')
        outcomes = {'box': _box_outcomes(event), 'slimsam': _segment_outcomes(event, producers.segmenter,
                    prompt_padding_rel=settings['segmentation_prompt_padding_rel'], min_predicted_iou=settings['segmentation_min_iou'])}
        return {'event': event, 'monocular_m': monocular,
                'monocular_usable_max_m': settings['monocular_usable_max_m'],
                'masks': {name: [dict(status=o.status, duration_ms=o.duration_ms,
                         region=None if o.region is None else dict(data=o.region.data,
                         origin_u=o.region.origin_u, origin_v=o.region.origin_v,
                         image_width=o.region.image_width, image_height=o.region.image_height,
                         precision=o.region.precision.value)) for o in batch] for name, batch in outcomes.items()}}
    except SharedProcessingFault:
        raise
    except Exception as exc:
        raise SharedProcessingFault(f'Shared inference failure: {exc}') from exc


def mask_outcomes(evidence, mask):
    return [CachedMaskOutcome(None if item['region'] is None else MaskRegion(**{
                **item['region'], 'precision': MaskPrecision(item['region']['precision'])}),
                item['status'], item['duration_ms']) for item in evidence['masks'][mask]]


def measure_evidence(evidence, settings):
    event = evidence['event']
    rows = []
    # Distinct views: polar is executed only once per mask; stereo availability
    # never gates monocular or polar.
    for mask in ('box', 'slimsam'):
        outcomes = mask_outcomes(evidence, mask)
        for source, selected in (('stereo', DEPTH_ESTIMATORS), ('monocular', DEPTH_ESTIMATORS), ('scan', ('polar_profiling',))):
            view = dict(event, front_offset_m=0.465)
            if source == 'monocular':
                view.update(depth_m=evidence['monocular_m'], depth_usable_max_m=evidence['monocular_usable_max_m'])
            else:
                view['depth_usable_max_m'] = settings['stereo_usable_max_m']
            measured = replay_sensor_event(view, outcomes, settings['arguments'], selected)
            for estimator in selected:
                detections = []
                for index, detection in enumerate(measured.detections):
                    forward = getattr(detection, f'{estimator}_forward_m')
                    lateral = getattr(detection, f'{estimator}_lateral_m')
                    radial = getattr(detection, f'{estimator}_distance_m')
                    status = getattr(detection, f'{estimator}_status')
                    detections.append({'index': index, 'bbox_xyxy': list(detection.bbox_xyxy),
                        'label': detection.label, 'score': detection.score, 'status': status,
                        'status_name': MissReason(status).name if status is not None else 'UNSET',
                        'mask_status': outcomes[index].status,
                        'forward_F_m': forward, 'lateral_F_m': lateral, 'radial_F_m': radial,
                        'base_planar_point_m': None if forward is None else [forward+0.465, lateral],
                        'base_point_limitation': 'planar point only; height is not returned by shared measurement kernel'})
                rows.append({'path': f'{estimator}__{mask}__{source}', 'empty_detections': not detections,
                             'detections': detections})
    if len(rows) != 10 or len({r['path'] for r in rows}) != 10:
        raise ValueError('Expected ten distinct hardware paths')
    return {'event_id': event['event_id'], 'ordinal': event['ordinal'], 'stamp_ns': event['stamp_ns'],
            'rgb_receive_ns': event['rgb_receive_ns'], 'depth_identity': event.get('depth_identity'),
            'scan_stamp_ns': event['scan']['stamp_ns'] if event.get('scan') else None,
            'scan_delta_ns': event.get('scan_delta_ns'), 'missing_inputs': event['missing_inputs'],
            'rows': rows, 'final_scoring_eligible': False}


def receipt_tree(root):
    return {str(p.relative_to(root)): file_hash(p) for p in sorted(root.rglob('*'))
            if p.is_file() and not p.name.endswith('.tmp') and p.name != 'complete.json'}


def validate_completed(bag_root, identity):
    complete = read_json(bag_root / 'complete.json')
    if complete['job_identity'] != identity or receipt_tree(bag_root) != complete['files_sha256']:
        raise ValueError(f'Completed bag identity/hash mismatch: {bag_root}')
    summary = read_json(bag_root / 'summary.json')
    if len(summary['paths']) != 10 or summary['selected_events'] <= 0:
        raise ValueError(f'Incomplete ten-path summary: {bag_root}')
    return summary


def summarize_bag(selection, results):
    paths = {p['id']: {'selected_events': 0, 'empty_detection_events': 0, 'detections': 0,
                      'successful_estimates': 0, 'input_failures': 0, 'mask_failures': 0,
                      'estimator_failures': 0, 'statuses': Counter(), 'mask_statuses': Counter()} for p in PATHS}
    missing = Counter()
    for event in results:
        missing.update(event['missing_inputs'].keys())
        if {r['path'] for r in event['rows']} != set(paths):
            raise ValueError('Event missing a path')
        for row in event['rows']:
            summary = paths[row['path']]
            summary['selected_events'] += 1
            summary['empty_detection_events'] += int(row['empty_detections'])
            for detection in row['detections']:
                summary['detections'] += 1
                status = detection['status_name']
                summary['statuses'][status] += 1
                summary['mask_statuses'][detection['mask_status']] += 1
                if detection['radial_F_m'] is not None:
                    summary['successful_estimates'] += 1
                elif detection['mask_status'] != 'ok':
                    summary['mask_failures'] += 1
                elif status in ('NO_DEPTH_FRAME', 'NO_SCAN', 'NO_CAMERA_INFO', 'GRID_MISMATCH', 'TF_MISS_EXTRINSIC', 'TF_MISS_SCAN', 'SCAN_INVALID', 'NO_COLOR_FRAME'):
                    summary['input_failures'] += 1
                else:
                    summary['estimator_failures'] += 1
    classes = []
    if any(missing.get(key) for key in ('camera_info', 'camera_tf', 'scan_tf')):
        classes.append({'category': 'software issue recoverable remotely', 'evidence': dict(missing),
                        'action': 'Inspect cached context/matched stamps; repair adapter or use unaffected paths. Missing inputs remain in denominators.'})
    classes.append({'category': 'reference limitation requiring qualified claims',
                    'evidence': {'reference_id': selection['reference']['id'], 'assumed_unchanged': selection['reference']['assumed_unchanged']},
                    'action': 'Keep estimates unscored; review supplied reference qualifications. No inference repeat needed for label corrections.'})
    classes.insert(0, {'category': 'usable for the diagnostic', 'evidence': f'{len(results)} selected events retained; algorithm misses are observations',
                      'action': 'Inspect overlays and per-detection statuses; no re-record recommendation from model misses.'})
    return {'bag': selection['bag'], 'selected_events': len(results), 'paths': paths,
            'missing_input_events': dict(missing), 'reference': selection['reference'],
            'classifications': classes, 'final_scoring_eligible': False}


def render_overlays(bag_root, evidence_paths):
    """At most three RGB/box/silhouette/scan panels per bag."""
    import cv2
    from ridgeback_autonomy.benchmarking.replay import replay_scan_points
    from ridgeback_localization.core.intrinsics import CameraIntrinsics, project_points
    if not evidence_paths:
        return []
    paths = []
    for index in sorted({0, len(evidence_paths)//2, len(evidence_paths)-1}):
        evidence = load_payload(evidence_paths[index])
        event = evidence['event']
        rgb = event['rgb']
        panels = [rgb.copy()]
        for mask in ('box', 'slimsam'):
            overlay = rgb.copy()
            union = np.zeros(rgb.shape[:2], np.uint8)
            for outcome in mask_outcomes(evidence, mask):
                if outcome.region is not None:
                    outcome.region.blit_into(union, 1)
            overlay[union.astype(bool)] = (0.55*overlay[union.astype(bool)] + 0.45*np.array([0, 255, 0])).astype(np.uint8)
            for d in event['detections']:
                x1,y1,x2,y2 = d['bbox_xyxy']
                cv2.rectangle(overlay, (x1,y1),(x2,y2),(255,180,0),1)
            cv2.putText(overlay, mask, (6,18), cv2.FONT_HERSHEY_SIMPLEX, .5, (255,255,255),1)
            panels.append(overlay)
        scan_panel = rgb.copy()
        points, _ = replay_scan_points(event)
        if points is not None and event.get('intrinsics'):
            intr = CameraIntrinsics(**{k:event['intrinsics'][k] for k in ('fx','fy','cx','cy','width','height')})
            xyz, valid = points
            # project_points is the shared production projection.
            uv, in_grid = project_points(xyz[valid], intr)
            for x,y in uv[in_grid]:
                if np.isfinite(x) and np.isfinite(y) and 0 <= x < rgb.shape[1] and 0 <= y < rgb.shape[0]:
                    cv2.circle(scan_panel,(int(x),int(y)),1,(255,0,0),-1)
        panels.append(scan_panel)
        relative = f'overlay-{index:06d}.png'
        if not cv2.imwrite(str(bag_root / relative), cv2.cvtColor(np.concatenate(panels,axis=1),cv2.COLOR_RGB2BGR)):
            raise OSError('Failed to save inspection overlay')
        paths.append(relative)
    return paths


def write_report(root, job, progress):
    lines = ['# Hardware-bag diagnostic', '',
             'Unscored workstation processing. Numerical success is not accuracy. All references remain provisional.', '',
             f'Job: `{job["identity"]}`. Front datum: **0.465 m**. No live throughput claims.', '',
             '| Bag | State | Events | Stereo misses | Scan misses | Evidence / action |',
             '|---|---|---:|---:|---:|---|']
    for selection in job['selection']['bags']:
        name = selection['bag']
        state = progress['bags'].get(name, {})
        if state.get('state') == 'complete':
            summary = read_json(root / name / 'summary.json')
            lines.append(f'| {name} | diagnostic usable; reference qualified | {summary["selected_events"]} | {summary["missing_input_events"].get("stereo",0)} | {summary["missing_input_events"].get("scan",0)} | [summary]({name}/summary.json), [events]({name}/events.jsonl); inspect overlays |')
        else:
            lines.append(f'| {name} | {state.get("state", "pending")} | — | — | — | {state.get("reason", "")} |')
    lines += ['', 'Each completed summary retains event/detection denominators, ten paths, status histograms, reference flags and classifications.',
              'Input failures need cached-evidence inspection before deciding recovery versus limited use. OWL misses, empty masks, distance errors and assumed positions do not justify re-recording.',
              'A failed raw input is a possible re-record need only if inspection establishes that the needed observation cannot be recovered or qualified.', '',
              '## Resource accounting', '', f'Peak process RSS (Linux KiB): {resource.getrusage(resource.RUSAGE_SELF).ru_maxrss}.',
              'Scratch holds one bag of aligned depths and selected RGB; it is removed after source evidence is committed. Failures retain scratch for inspection.',
              'Event caches commit atomically. A crash may require a visible new source-index attempt or repeating the current uncommitted inference event. Completed bags are never overwritten.']
    (root / 'report.md').write_text('\n'.join(lines)+'\n')


def run_job(job, *, resume=False, producers_factory=ProductionProducers, message_reader=ros_messages,
            validate=True):
    if validate:
        validate_job(job)
    root = Path(job['output'])
    with output_lock(root):
        job_path = root / 'job.json'
        if job_path.exists():
            if read_json(job_path) != job:
                raise ValueError('Output already belongs to another frozen job')
            if not resume:
                raise ValueError('Output already initialized; use resume')
        else:
            if resume:
                raise ValueError('Cannot resume an uninitialized output')
            atomic_json(job_path, job)
        ledger_path = root / 'progress.json'
        progress = read_json(ledger_path) if ledger_path.exists() else {'job_identity': job['identity'], 'bags': {}}
        if progress['job_identity'] != job['identity']:
            raise ValueError('Progress belongs to incompatible configuration')
        producers = None
        for selection in job['selection']['bags']:
            name = selection['bag']
            bag_root = root / name
            if (bag_root / 'complete.json').exists():
                validate_completed(bag_root, job['identity'])
                progress['bags'][name] = {'state': 'complete'}
                atomic_json(ledger_path, progress)
                continue
            if progress['bags'].get(name, {}).get('state') == 'input_failed':
                # Explicit failed input is not silently re-read on every resume.
                continue
            started = time.monotonic()
            previous = progress['bags'].get(name, {})
            progress['bags'][name] = {'state': 'incomplete', 'attempt': previous.get('attempt',0)+1}
            atomic_json(ledger_path, progress)
            bag_root.mkdir(exist_ok=True)
            source_manifest = bag_root / 'source.json'
            try:
                if not source_manifest.exists():
                    attempt = progress['bags'][name]['attempt']
                    scratch = root / '.scratch' / f'{name}-attempt-{attempt}'
                    raw_paths = []
                    try:
                        source = message_reader(Path(job['selection']['session']) / 'bags' / name, selection['topics'])
                        for event in assemble_events(source, selection, scratch, base_frame=job['base_frame']):
                            event['base_above_floor_m'] = job['settings']['base_above_floor_m']
                            path = bag_root / 'source' / f'{event["ordinal"]:06d}.json'
                            save_payload(path, event)
                            raw_paths.append(str(path.relative_to(bag_root)))
                    except OSError as exc:
                        raise SharedProcessingFault(f'Source indexing storage/I/O failure: {exc}') from exc
                    except (ValueError, RuntimeError) as exc:
                        raise InputFault(f'Raw assembly failed: {exc}') from exc
                    for frozen in selection.get('files', []):
                        current = Path(frozen['path']).stat()
                        if current.st_size != frozen['bytes'] or current.st_mtime_ns != frozen['mtime_ns']:
                            raise InputFault(f'Raw input changed during read: {frozen["path"]}')
                    if not raw_paths:
                        raise InputFault('No eligible selected RGB events in settled interval')
                    atomic_json(source_manifest, {'job_identity': job['identity'], 'events': raw_paths,
                                                  'source_index_attempt': attempt})
                    shutil.rmtree(scratch)
                manifest = read_json(source_manifest)
                if manifest['job_identity'] != job['identity']:
                    raise ValueError('Cached source belongs to incompatible job')
                results, evidence_paths = [], []
                for relative in manifest['events']:
                    raw = load_payload(bag_root / relative)
                    evidence_path = bag_root / 'inference' / Path(relative).name
                    if evidence_path.exists():
                        evidence = load_payload(evidence_path)
                        if evidence['event']['event_id'] != raw['event_id']:
                            raise ValueError('Cached inference event identity mismatch')
                    else:
                        if producers is None:
                            producers = producers_factory(job)
                        try:
                            evidence = freeze_inference(raw, producers, job['settings'])
                        except SharedProcessingFault as exc:
                            atomic_json(bag_root / 'processing-failure.json', {'event_id': raw['event_id'],
                                'job_identity': job['identity'], 'failure': str(exc),
                                'classification': 'shared processing fault; batch stopped'})
                            raise
                        save_payload(evidence_path, evidence)
                    result = measure_evidence(evidence, job['settings'])
                    result['source_artifact'] = relative
                    result['inference_artifact'] = str(evidence_path.relative_to(bag_root))
                    results.append(result)
                    evidence_paths.append(evidence_path)
                summary = summarize_bag(selection, results)
                summary['source_index_resources'] = raw['resources']
                summary['evidence_disk_bytes_before_summary'] = sum(p.stat().st_size for p in bag_root.rglob('*') if p.is_file())
                summary['effective_configuration'] = job['settings']
                summary['job_identity'] = job['identity']
                summary['wall_time_sec'] = time.monotonic()-started
                summary['peak_process_rss_kib'] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
                summary['overlays'] = render_overlays(bag_root, evidence_paths)
                atomic_json(bag_root / 'summary.json', summary)
                temp = bag_root / 'events.jsonl.tmp'
                with temp.open('w') as stream:
                    for result in results:
                        stream.write(json.dumps(result, allow_nan=False)+'\n')
                    stream.flush()
                    os.fsync(stream.fileno())
                os.replace(temp, bag_root / 'events.jsonl')
                atomic_json(bag_root / 'complete.json', {'job_identity': job['identity'], 'files_sha256': receipt_tree(bag_root)})
                validate_completed(bag_root, job['identity'])
                progress['bags'][name] = {'state': 'complete'}
            except InputFault as exc:
                progress['bags'][name].update(state='input_failed', reason=str(exc),
                    category='software issue recoverable remotely', action='Inspect retained raw-index evidence; no re-record decision without concrete unrecoverable observation')
            except Exception as exc:
                progress['bags'][name].update(state='incomplete', reason=str(exc), failure_type=type(exc).__name__)
                atomic_json(ledger_path, progress)
                write_report(root, job, progress)
                raise
            atomic_json(ledger_path, progress)
            write_report(root, job, progress)
        write_report(root, job, progress)
        return progress


def remeasure_cached(evidence_root, output, *, references=None):
    """A new derivative from completed evidence; no raw reader or producers.

    References can change independently: a JSON mapping bag name to a qualified
    reference document. They never enter the measurement call or scored replay.
    The derivative freezes the current measurement code/settings and parent
    hashes; the original completed results are left intact.
    """
    evidence_root, output = Path(evidence_root).resolve(), Path(output).resolve()
    if output == evidence_root or output.is_relative_to(evidence_root):
        raise ValueError('Cached remeasurement needs a separate new output')
    parent = read_json(evidence_root / 'job.json')
    progress = read_json(evidence_root / 'progress.json')
    overrides = read_json(references) if references else {}
    current_settings = effective_settings()
    current_settings['monocular_usable_max_m'] = parent['settings']['monocular_usable_max_m']
    completed = []
    for selection in parent['selection']['bags']:
        name = selection['bag']
        if progress['bags'].get(name, {}).get('state') == 'complete':
            validate_completed(evidence_root / name, parent['identity'])
            completed.append(selection)
    if not completed:
        raise ValueError('No completed bags to remeasure')
    if set(overrides)-{s['bag'] for s in completed}:
        raise ValueError('Reference override names must be completed exact bags')
    with output_lock(output):
        if any(p.name != '.lock' for p in output.iterdir()):
            raise ValueError('Cached remeasurement requires a new empty output')
        derivative = {'schema': 'dynamo-hardware-remeasurement/1', 'parent': str(evidence_root),
                      'parent_job_identity': parent['identity'], 'settings': current_settings,
                      'code': code_identity(), 'reference_overrides': overrides,
                      'final_scoring_eligible': False, 'state': 'incomplete', 'bags': []}
        for selection in completed:
            selection = deepcopy_json(selection)
            name = selection['bag']
            if name in overrides:
                selection['reference'] = dict(selection['reference'], document=overrides[name],
                                               override_source=str(Path(references).resolve()),
                                               nominal_position={k:overrides[name][k] for k in ('x_F_m','y_F_m') if k in overrides[name]},
                                               assumed_unchanged=bool(overrides[name].get('assumed_unchanged',selection['reference']['assumed_unchanged'])),
                                               final_scoring_eligible=False)
            target = output/name
            target.mkdir()
            original = evidence_root/name
            manifest = read_json(original/'source.json')
            results = []
            for relative in manifest['events']:
                inference = original/'inference'/Path(relative).name
                evidence = load_payload(inference)
                result = measure_evidence(evidence,current_settings)
                result['inference_artifact'] = str(inference)
                result['parent_inference_sha256'] = file_hash(inference)
                results.append(result)
            summary = summarize_bag(selection,results)
            summary['effective_configuration'] = current_settings
            atomic_json(target/'summary.json',summary)
            (target/'events.jsonl').write_text(''.join(json.dumps(r,allow_nan=False)+'\n' for r in results))
            derivative['bags'].append({'bag':name,'parent_complete_sha256':file_hash(original/'complete.json'),
                                       'files_sha256':receipt_tree(target)})
            atomic_json(output/'derivative.json',derivative)
        derivative['state'] = 'complete'
        atomic_json(output/'derivative.json',derivative)
        (output/'report.md').write_text('# Cached hardware remeasurement\n\nUnscored derivative; references remain provisional. No raw reads or inference.\n\n'+
            '\n'.join(f'- [{b["bag"]}]({b["bag"]}/summary.json)' for b in derivative['bags'])+'\n')
        return derivative


def deepcopy_json(document):
    return json.loads(json.dumps(document))
