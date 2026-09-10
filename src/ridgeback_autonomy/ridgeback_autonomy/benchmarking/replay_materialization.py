"""Mask-cache materialization over immutable sensor captures."""

from __future__ import annotations

import math
from pathlib import Path
import time
from typing import Callable

from ridgeback_autonomy.benchmarking.replay_artifacts import (
    MASK_STATUS_EMPTY_SEGMENTATION,
    MASK_STATUS_NO_COLOR_FRAME,
    MASK_STATUS_OK,
    MASK_STATUS_OVERSIZED_BOX,
    CachedMaskOutcome,
    MaskCacheWriter,
    ReplayArtifact,
    SENSOR_CAPTURE_KIND,
    dependency_versions,
    load_sensor_trial,
)
from ridgeback_autonomy.perception.target_localization.core.box_gate import (
    box_within_frame_fraction,
)
from ridgeback_autonomy.perception.target_localization.core.mask import (
    MaskPrecision,
    region_from_bbox,
    region_from_blob,
)


PRODUCER_BOX = 'box'
PRODUCER_SLIMSAM = 'slimsam'
PRODUCERS = (PRODUCER_BOX, PRODUCER_SLIMSAM)


class _PrintLogger:
    def info(self, message: str) -> None:
        print(message)

    def warning(self, message: str) -> None:
        print(f'warning: {message}')

    warn = warning


def materializer_producer_document(
    producer: str,
    *,
    model: str | None,
    model_revision: str | None,
    prompt_padding_rel: float,
    min_predicted_iou: float,
    device: str,
    dtype: str,
    preprocessing: str,
    code_provenance: dict,
) -> dict:
    if producer not in PRODUCERS:
        raise ValueError(f'Unknown mask producer {producer!r}; expected one of: {", ".join(PRODUCERS)}.')
    if preprocessing != 'transformers-default':
        raise ValueError(
            'Only segmentation_preprocessing="transformers-default" is implemented.')
    if not math.isfinite(prompt_padding_rel) or prompt_padding_rel < 0.0:
        raise ValueError('Segmentation prompt padding must be finite and non-negative.')
    if not math.isfinite(min_predicted_iou) or not 0.0 <= min_predicted_iou <= 1.0:
        raise ValueError('Segmentation predicted-IoU floor must be finite and between 0 and 1.')
    if dtype not in ('auto', 'float32', 'float16', 'bfloat16'):
        raise ValueError(
            'Segmentation dtype must be auto, float32, float16, or bfloat16.')
    return {
        'commit': code_provenance.get('commit'),
        'branch': code_provenance.get('branch'),
        'dirty_count': int(code_provenance.get('dirty_count', 0)),
        'model': 'box-rasterizer' if producer == PRODUCER_BOX else model,
        'model_revision': model_revision,
        'dependencies': dependency_versions(
            ('numpy',) if producer == PRODUCER_BOX else ('numpy', 'torch', 'transformers')),
        'parameters': {
            'mask_producer': producer,
            'segmentation_model': model,
            'segmentation_model_revision': model_revision,
            'segmentation_prompt_padding_rel': float(prompt_padding_rel),
            'segmentation_min_iou': float(min_predicted_iou),
            'segmentation_device': device,
            'segmentation_dtype': dtype,
            'segmentation_preprocessing': preprocessing,
        },
    }


def _box_outcomes(event: dict) -> list[CachedMaskOutcome]:
    height, width = int(event['image_height']), int(event['image_width'])
    outcomes = []
    for detection in event['detections']:
        box = tuple(int(value) for value in detection['bbox_xyxy'])
        if not box_within_frame_fraction(box, height, width):
            outcomes.append(CachedMaskOutcome(None, MASK_STATUS_OVERSIZED_BOX))
        else:
            outcomes.append(CachedMaskOutcome(
                region_from_bbox(box, height, width), MASK_STATUS_OK, 0.0))
    return outcomes


def _segment_outcomes(
    event: dict,
    segmenter,
    *,
    prompt_padding_rel: float,
    min_predicted_iou: float,
) -> list[CachedMaskOutcome]:
    height, width = int(event['image_height']), int(event['image_width'])
    detections = event['detections']
    accepted = [
        box_within_frame_fraction(
            tuple(int(value) for value in detection['bbox_xyxy']), height, width)
        for detection in detections
    ]
    if event.get('rgb') is None:
        return [
            CachedMaskOutcome(
                None,
                MASK_STATUS_NO_COLOR_FRAME if keep else MASK_STATUS_OVERSIZED_BOX,
            )
            for keep in accepted
        ]
    boxes = [
        tuple(float(value) for value in detection['bbox_xyxy'])
        for detection, keep in zip(detections, accepted) if keep
    ]
    started = time.perf_counter()
    blobs = segmenter.segment_boxes(
        event['rgb'],
        boxes,
        pad_rel=prompt_padding_rel,
        min_predicted_iou=min_predicted_iou,
    ) if boxes else []
    elapsed_ms = (time.perf_counter() - started) * 1000.0
    if len(blobs) != len(boxes):
        raise ValueError(
            f'Segmenter returned {len(blobs)} masks for {len(boxes)} prompts.')
    per_detection_ms = elapsed_ms / len(boxes) if boxes else 0.0
    blob_iter = iter(blobs)
    outcomes = []
    for keep in accepted:
        if not keep:
            outcomes.append(CachedMaskOutcome(None, MASK_STATUS_OVERSIZED_BOX))
            continue
        blob = next(blob_iter)
        if blob is None:
            outcomes.append(CachedMaskOutcome(
                None, MASK_STATUS_EMPTY_SEGMENTATION, per_detection_ms))
        else:
            outcomes.append(CachedMaskOutcome(
                region_from_blob(blob, MaskPrecision.TIGHT),
                MASK_STATUS_OK,
                per_detection_ms,
            ))
    return outcomes


def materialize_masks(
    sensor: ReplayArtifact,
    output_dir: str | Path,
    *,
    producer: str,
    code_provenance: dict,
    model: str | None = None,
    model_revision: str | None = None,
    prompt_padding_rel: float = 0.05,
    min_predicted_iou: float = 0.5,
    device: str = 'auto',
    dtype: str = 'auto',
    preprocessing: str = 'transformers-default',
    segmenter_factory: Callable | None = None,
    logger=None,
) -> ReplayArtifact:
    """Stream one parent through one producer and atomically publish its cache.

    Ground truth is intentionally discarded at the loader boundary: producers
    receive only each sensor event's detections and RGB, never the trial truth.
    """

    if sensor.kind != SENSOR_CAPTURE_KIND:
        raise ValueError('Mask materialization requires a sensor-capture artifact.')
    producer_document = materializer_producer_document(
        producer,
        model=model,
        model_revision=model_revision,
        prompt_padding_rel=prompt_padding_rel,
        min_predicted_iou=min_predicted_iou,
        device=device,
        dtype=dtype,
        preprocessing=preprocessing,
        code_provenance=code_provenance,
    )
    writer = MaskCacheWriter(
        output_dir,
        parent=sensor,
        producer=producer_document,
        metadata={'claim_boundary': 'frozen mask output; materialization timing is diagnostic'},
    )
    segmenter = None
    try:
        if producer == PRODUCER_SLIMSAM:
            if not model:
                raise ValueError('The slimsam producer requires a segmentation model.')
            if segmenter_factory is None:
                from ridgeback_autonomy.perception.target_localization.core.segmentation import (
                    SamBoxSegmenter,
                )
                segmenter_factory = SamBoxSegmenter
            segmenter = segmenter_factory(
                model,
                logger or _PrintLogger(),
                revision=model_revision,
                device=device,
                dtype=dtype,
            )
            segmenter.load()
            resolved_revision = getattr(
                getattr(getattr(segmenter, '_model', None), 'config', None),
                '_commit_hash',
                None,
            )
            if resolved_revision:
                writer.manifest['producer']['model_revision'] = str(resolved_revision)
                writer.manifest['producer']['signature_sha256'] = ''
                from ridgeback_autonomy.benchmarking.replay_artifacts import producer_signature
                unsigned = dict(writer.manifest['producer'])
                unsigned.pop('signature_sha256', None)
                writer.manifest['producer']['signature_sha256'] = producer_signature(unsigned)

        for entry in sensor.trial_entries:
            _trial, events = load_sensor_trial(sensor, entry)
            outcomes = []
            for event in events:
                if producer == PRODUCER_BOX:
                    outcomes.append(_box_outcomes(event))
                else:
                    outcomes.append(_segment_outcomes(
                        event,
                        segmenter,
                        prompt_padding_rel=prompt_padding_rel,
                        min_predicted_iou=min_predicted_iou,
                    ))
            writer.write_trial(str(entry['trial_id']), outcomes)
        return writer.finalize()
    except Exception:
        writer.discard_empty()
        raise
