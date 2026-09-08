#!/usr/bin/env python3
"""Evaluate a projective-ranging sweep from a frozen replay dataset."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys

from ridgeback_autonomy.benchmarking.process_utils import git_provenance
from ridgeback_autonomy.benchmarking.replay import (
    evaluate_dataset,
    load_dataset,
    sha256_file,
    write_replay_results,
)
from ridgeback_autonomy.benchmarking.sweep import load_sweep


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Evaluate a projective-ranging parameter sweep without ROS or Gazebo.')
    parser.add_argument('dataset', help='Completed replay dataset directory')
    parser.add_argument('sweep', help='Existing benchmark sweep YAML')
    parser.add_argument('--output-dir', required=True, help='New output directory')
    parser.add_argument('--workers', type=int, default=1, help='Process workers; each evaluates all variants for one trial')
    return parser


def _validate_v1(spec) -> None:
    for config in spec.configs:
        arguments = config.arguments
        if arguments.get('estimators') != 'projective_ranging':
            raise ValueError(
                f'Replay V1 config "{config.name}" must select projective_ranging only.')
        if arguments.get('mask_gate', 'box') != 'box':
            raise ValueError(f'Replay V1 config "{config.name}" must use mask_gate:=box.')
        if arguments.get('depth_source', 'stereoscopic') != 'stereoscopic':
            raise ValueError(f'Replay V1 config "{config.name}" must use stereoscopic depth.')
    if not spec.configs:
        raise ValueError('Replay sweep has no configurations.')


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.workers < 1:
        raise ValueError('--workers must be at least 1.')
    spec = load_sweep(args.sweep)
    _validate_v1(spec)
    dataset = load_dataset(args.dataset)
    output = Path(args.output_dir).expanduser().resolve()
    if output.exists():
        raise ValueError(f'Replay output directory already exists: {output}')
    workspace_root = os.getcwd()
    results = evaluate_dataset(dataset, spec.configs, workers=args.workers)
    write_replay_results(
        output,
        dataset,
        spec.configs,
        results,
        evaluation_provenance={
            **git_provenance(workspace_root),
            'sweep': str(Path(args.sweep).expanduser().resolve()),
            'sweep_sha256': sha256_file(args.sweep),
        },
        worker_count=args.workers,
    )
    print(f'Offline replay written to {output}')
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except ValueError as exc:
        print(f'error: {exc}', file=sys.stderr)
        raise SystemExit(2)
