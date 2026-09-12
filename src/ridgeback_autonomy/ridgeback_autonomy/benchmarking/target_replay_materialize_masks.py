#!/usr/bin/env python3
"""Create one immutable mask cache from a sensor capture."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

from ridgeback_autonomy.benchmarking.process_utils import git_provenance
from ridgeback_autonomy.benchmarking.replay_artifacts import load_artifact
from ridgeback_autonomy.benchmarking.replay_materialization import (
    PRODUCERS,
    materialize_masks,
)
from ridgeback_autonomy.benchmarking.target_offline_replay_benchmark import (
    _source_repository,
)
from ridgeback_autonomy.perception.target_localization.core.segmentation import (
    PROMPT_PADDING_REL_DEFAULT,
    SEGMENTATION_MIN_PREDICTED_IOU_DEFAULT,
    SEGMENTATION_MODEL_DEFAULT,
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Materialize a box or segmentation mask cache from frozen sensor evidence.')
    parser.add_argument('sensor_capture', help='Completed sensor-capture artifact')
    parser.add_argument('--producer', choices=PRODUCERS, required=True)
    parser.add_argument('--output-dir', required=True, help='New immutable cache directory')
    parser.add_argument('--model', default=SEGMENTATION_MODEL_DEFAULT)
    parser.add_argument('--model-revision', default='')
    parser.add_argument('--prompt-padding-rel', type=float, default=PROMPT_PADDING_REL_DEFAULT)
    parser.add_argument(
        '--min-predicted-iou', type=float,
        default=SEGMENTATION_MIN_PREDICTED_IOU_DEFAULT)
    parser.add_argument('--device', default='auto')
    parser.add_argument('--dtype', default='auto')
    parser.add_argument('--preprocessing', default='transformers-default')
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.prompt_padding_rel < 0.0:
        raise ValueError('--prompt-padding-rel must be non-negative.')
    if not 0.0 <= args.min_predicted_iou <= 1.0:
        raise ValueError('--min-predicted-iou must be between 0 and 1.')
    sensor = load_artifact(args.sensor_capture, require_parent=False)
    cache = materialize_masks(
        sensor,
        args.output_dir,
        producer=args.producer,
        code_provenance=git_provenance(str(_source_repository())),
        model=args.model,
        model_revision=args.model_revision or None,
        prompt_padding_rel=args.prompt_padding_rel,
        min_predicted_iou=args.min_predicted_iou,
        device=args.device,
        dtype=args.dtype,
        preprocessing=args.preprocessing,
    )
    print(f'Mask cache {cache.id} written to {Path(args.output_dir).expanduser().resolve()}')
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except ValueError as exc:
        print(f'error: {exc}', file=sys.stderr)
        raise SystemExit(2)
