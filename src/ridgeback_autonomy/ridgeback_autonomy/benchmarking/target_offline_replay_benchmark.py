#!/usr/bin/env python3
"""Evaluate a projective-ranging sweep from a frozen replay dataset."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

from ridgeback_autonomy.benchmarking.process_utils import git_provenance
from ridgeback_autonomy.benchmarking.replay import (
    evaluate_dataset,
    load_dataset,
    sha256_file,
    write_replay_results,
)
from ridgeback_autonomy.benchmarking.sweep import SweepConfig, load_sweep
from ridgeback_autonomy.benchmarking.replay_profiles import (
    MEASUREMENT_AXES,
    PROFILE_MEASUREMENT,
    ProfileValidationError,
    validate_profile_axes,
)
from ridgeback_autonomy.perception.target_localization.core.depth_common import (
    DEPTH_GATE_DISABLED,
    NEAREST_MODE_BIN_WIDTH_M_DEFAULT,
    NEAREST_MODE_MIN_BIN_FRACTION_DEFAULT,
)
from ridgeback_autonomy.perception.target_localization.core.isolation_2d import (
    ISOLATION_2D_DEFAULT,
)
from ridgeback_autonomy.perception.target_localization.core.ranging_defaults import (
    MIN_VALID_SAMPLES,
    NEAR_SURFACE_BAND_M,
)


REPLAY_V1_ARGUMENT_DEFAULTS = {
    'estimators': 'projective_ranging',
    'mask_gate': 'box',
    'depth_source': 'stereoscopic',
    'mask_depth_max_meters': str(DEPTH_GATE_DISABLED),
    'isolation_2d': ISOLATION_2D_DEFAULT,
    'isolation_2d_bin_width_m': str(NEAREST_MODE_BIN_WIDTH_M_DEFAULT),
    'isolation_2d_band_m': str(NEAR_SURFACE_BAND_M),
    'isolation_2d_min_bin_fraction': str(NEAREST_MODE_MIN_BIN_FRACTION_DEFAULT),
    'min_valid_pixels': str(MIN_VALID_SAMPLES),
}
# Compatibility aliases for callers/tests that imported the original names.
REPLAY_V1_ARGUMENT_NAMES = frozenset(REPLAY_V1_ARGUMENT_DEFAULTS)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Evaluate a projective-ranging parameter sweep without ROS or Gazebo.')
    parser.add_argument('dataset', help='Completed replay dataset directory')
    parser.add_argument('sweep', help='Existing benchmark sweep YAML')
    parser.add_argument('--output-dir', required=True, help='New output directory')
    parser.add_argument('--workers', type=int, default=1, help='Process workers; each evaluates all variants for one trial')
    return parser


def _validated_v1_variants(spec) -> tuple[SweepConfig, ...]:
    if not spec.configs:
        raise ValueError('Replay sweep has no configurations.')
    variants = []
    for config in spec.configs:
        arguments = config.arguments
        try:
            validate_profile_axes(PROFILE_MEASUREMENT, config.explicit_keys)
        except ProfileValidationError as exc:
            raise ValueError(
                f'{exc.field} does not affect offline projective ranging; {exc}') from exc
        if arguments.get('estimators') != 'projective_ranging':
            raise ValueError(
                f'Measurement replay config "{config.name}" must select '
                'projective_ranging only for legacy evidence.')
        if arguments.get('mask_gate', 'box') != 'box':
            raise ValueError(
                f'Measurement replay config "{config.name}" requires the frozen box mask.')
        if arguments.get('depth_source', 'stereoscopic') != 'stereoscopic':
            raise ValueError(
                f'Measurement replay config "{config.name}" requires its frozen '
                'stereoscopic depth.')
        irrelevant = sorted(config.explicit_keys - MEASUREMENT_AXES)
        if irrelevant:
            raise ValueError(
                f'Measurement replay config "{config.name}" explicitly sets '
                f'"{irrelevant[0]}", which is frozen or irrelevant.')
        resolved = dict(REPLAY_V1_ARGUMENT_DEFAULTS)
        resolved.update({
            key: value for key, value in arguments.items()
            if key in REPLAY_V1_ARGUMENT_NAMES
        })
        variants.append(SweepConfig(
            name=config.name,
            arguments=resolved,
            explicit_keys=frozenset(config.explicit_keys & REPLAY_V1_ARGUMENT_NAMES),
        ))
    return tuple(variants)


def _source_repository() -> Path:
    """Repository containing the evaluator, independent of the caller's cwd."""

    source = Path(__file__).resolve()
    for candidate in source.parents:
        if (candidate / '.git').exists():
            return candidate
    return source.parent


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.workers < 1:
        raise ValueError('--workers must be at least 1.')
    spec = load_sweep(args.sweep)
    variants = _validated_v1_variants(spec)
    dataset = load_dataset(args.dataset)
    output = Path(args.output_dir).expanduser().resolve()
    if output.exists():
        raise ValueError(f'Replay output directory already exists: {output}')
    workspace_root = _source_repository()
    evaluation_started = time.strftime('%Y-%m-%d %H:%M:%S %z')
    evaluation_started_monotonic = time.monotonic()
    results = evaluate_dataset(dataset, variants, workers=args.workers)
    evaluation_wall_time_sec = time.monotonic() - evaluation_started_monotonic
    evaluation_finished = time.strftime('%Y-%m-%d %H:%M:%S %z')
    write_replay_results(
        output,
        dataset,
        variants,
        results,
        evaluation_provenance={
            **git_provenance(str(workspace_root)),
            'sweep': str(Path(args.sweep).expanduser().resolve()),
            'sweep_sha256': sha256_file(args.sweep),
            'started': evaluation_started,
            'finished': evaluation_finished,
        },
        worker_count=args.workers,
        evaluation_wall_time_sec=evaluation_wall_time_sec,
        sweep_name=spec.name,
        sweep_description=spec.description,
    )
    print(f'Offline replay written to {output}')
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except ValueError as exc:
        print(f'error: {exc}', file=sys.stderr)
        raise SystemExit(2)
