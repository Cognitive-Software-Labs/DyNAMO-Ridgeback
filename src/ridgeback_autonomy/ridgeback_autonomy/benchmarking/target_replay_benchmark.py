#!/usr/bin/env python3
"""Execute a validated layered-replay job without ROS or Gazebo."""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import sys
import time
import uuid

from ridgeback_autonomy.benchmarking.layered_replay import (
    evaluate_sensor_capture,
    resolve_expanded_baseline,
    write_layered_results,
)
from ridgeback_autonomy.benchmarking.process_utils import git_provenance
from ridgeback_autonomy.benchmarking.replay import (
    evaluate_dataset,
    load_dataset,
    sha256_file,
    write_replay_results,
)
from ridgeback_autonomy.benchmarking.replay_artifacts import load_artifact
from ridgeback_autonomy.benchmarking.replay_jobs import load_job
from ridgeback_autonomy.benchmarking.replay_profiles import (
    AXES,
    MEASUREMENT_AXES,
    PROFILE_LIVE_SYSTEM,
    PROFILE_MASK_MODEL,
    PROFILE_MASK_OUTPUT,
    PROFILE_MEASUREMENT,
    get_profile,
)
from ridgeback_autonomy.benchmarking.sweep import SweepConfig
from ridgeback_autonomy.benchmarking.target_offline_replay_benchmark import (
    _source_repository,
    _validated_v1_variants,
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Run a canonical layered replay job.')
    parser.add_argument('job', help='Replay job YAML or JSON')
    parser.add_argument('--output-dir', help='Override the job output directory')
    return parser


def _relative_to_job(job_path: Path, value: str) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (job_path.parent / path).resolve()


def _measurement_variants(job) -> tuple[SweepConfig, ...]:
    defaults = {
        name: str(AXES[name].default)
        for name in MEASUREMENT_AXES
        if AXES[name].default != ''
    }
    variants = []
    for config in job.sweep.configs:
        arguments = dict(defaults)
        arguments.update({
            key: value for key, value in config.arguments.items()
            if key in MEASUREMENT_AXES
        })
        variants.append(SweepConfig(
            name=config.name,
            arguments=arguments,
            explicit_keys=frozenset(config.explicit_keys & MEASUREMENT_AXES),
        ))
    return tuple(variants)


def _ordered_baseline(variants: tuple[SweepConfig, ...], baseline: str):
    return tuple(sorted(variants, key=lambda variant: variant.name != baseline))


def _sweep_provenance(job) -> tuple[str, str]:
    source = job.sweep.source
    path = Path(source)
    if path.is_file():
        return str(path.resolve()), sha256_file(path)
    encoded = json.dumps({
        'name': job.sweep.name,
        'description': job.sweep.description,
        'configs': [
            {'name': config.name, **config.arguments}
            for config in job.sweep.configs
        ],
    }, sort_keys=True).encode()
    import hashlib
    return source, hashlib.sha256(encoded).hexdigest()


def _evaluation_provenance(job) -> tuple[dict, float]:
    source, sweep_hash = _sweep_provenance(job)
    return {
        **git_provenance(str(_source_repository())),
        'sweep': source,
        'sweep_sha256': sweep_hash,
        'started': time.strftime('%Y-%m-%d %H:%M:%S %z'),
    }, time.monotonic()


def _materialization_kwargs(spec) -> dict:
    args = spec.arguments
    def resolved(name: str):
        return args.get(name, AXES[name].default)

    return {
        'producer': str(resolved('mask_producer')),
        'model': resolved('segmentation_model') or None,
        'model_revision': resolved('segmentation_model_revision') or None,
        'prompt_padding_rel': float(resolved('segmentation_prompt_padding_rel')),
        'min_predicted_iou': float(resolved('segmentation_min_iou')),
        'device': str(resolved('segmentation_device')),
        'dtype': str(resolved('segmentation_dtype')),
        'preprocessing': str(resolved('segmentation_preprocessing')),
    }


def _materialize_job_caches(job, sensor, cache_root: Path, provenance: dict):
    from ridgeback_autonomy.benchmarking.replay_materialization import materialize_masks

    tasks = []
    for spec in job.materializations:
        tasks.append((spec, _materialization_kwargs(spec)))
    if job.model_workers > 1:
        model_devices = [
            kwargs['device'] for _spec, kwargs in tasks
            if kwargs['producer'] != 'box'
        ]
        if any(device == 'auto' for device in model_devices) or len(set(model_devices)) != len(model_devices):
            raise ValueError(
                'model_workers > 1 requires one distinct explicit device per '
                'segmentation materialization; this prevents duplicate model loads on one GPU.')

    def run(task):
        spec, kwargs = task
        return materialize_masks(
            sensor,
            cache_root / spec.name,
            code_provenance=provenance,
            **kwargs,
        )

    if job.model_workers == 1 or len(tasks) == 1:
        return tuple(run(task) for task in tasks)
    with ThreadPoolExecutor(max_workers=job.model_workers) as executor:
        return tuple(executor.map(run, tasks))


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    job_path = Path(args.job).expanduser().resolve()
    job = load_job(job_path)
    if job.profile == PROFILE_LIVE_SYSTEM:
        raise ValueError(
            'Profile "live-system" routes to target_benchmark_sweep; it is not an offline replay.')
    output_value = args.output_dir or job.output_dir
    if not output_value:
        raise ValueError('Replay output directory is required in the job or --output-dir.')
    output = _relative_to_job(job_path, output_value)
    if output.exists():
        raise ValueError(f'Replay output directory already exists: {output}')
    output.parent.mkdir(parents=True, exist_ok=True)
    staging = output.with_name(f'.{output.name}.partial-{os.getpid()}-{uuid.uuid4().hex}')
    staging.mkdir()
    provenance, started = _evaluation_provenance(job)

    if job.profile == PROFILE_MEASUREMENT:
        variants = _ordered_baseline(
            _validated_v1_variants(job.sweep), job.comparison_baseline)
        dataset_path = _relative_to_job(job_path, job.inputs['measurement_dataset'])
        dataset = load_dataset(dataset_path)
        results = evaluate_dataset(dataset, variants, workers=job.measurement_workers)
        elapsed = time.monotonic() - started
        provenance['finished'] = time.strftime('%Y-%m-%d %H:%M:%S %z')
        write_replay_results(
            staging / 'results', dataset, variants, results,
            evaluation_provenance=provenance,
            worker_count=job.measurement_workers,
            evaluation_wall_time_sec=elapsed,
            sweep_name=job.sweep.name,
            sweep_description=job.sweep.description,
        )
    else:
        sensor_path = _relative_to_job(job_path, job.inputs['sensor_capture'])
        sensor = load_artifact(sensor_path, require_parent=False)
        if job.profile == PROFILE_MASK_OUTPUT:
            caches = tuple(
                load_artifact(_relative_to_job(job_path, path), parent=sensor)
                for path in job.inputs['mask_caches']
            )
        elif job.profile == PROFILE_MASK_MODEL:
            caches = _materialize_job_caches(
                job, sensor, staging / 'mask_caches', provenance)
        else:  # guarded by parse_job, retained as an execution invariant
            raise AssertionError(job.profile)
        measurement_variants = _measurement_variants(job)
        expanded, results = evaluate_sensor_capture(
            sensor, caches, measurement_variants,
            workers=job.measurement_workers)
        baseline = resolve_expanded_baseline(expanded, job.comparison_baseline)
        elapsed = time.monotonic() - started
        provenance['finished'] = time.strftime('%Y-%m-%d %H:%M:%S %z')
        write_layered_results(
            staging / 'results', sensor, caches, expanded, results,
            baseline_name=baseline,
            profile=get_profile(job.profile).as_dict(),
            evaluation_provenance=provenance,
            worker_count=job.measurement_workers,
            evaluation_wall_time_sec=elapsed,
            sweep_name=job.sweep.name,
            sweep_description=job.sweep.description,
        )

    (staging / 'job.json').write_text(
        json.dumps(job.document, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    os.replace(staging, output)
    print(f'Layered replay written to {output}')
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except ValueError as exc:
        print(f'error: {exc}', file=sys.stderr)
        raise SystemExit(2)
