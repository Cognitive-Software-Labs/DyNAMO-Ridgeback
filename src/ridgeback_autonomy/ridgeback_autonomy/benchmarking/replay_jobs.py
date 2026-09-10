"""Canonical layered-replay job specification and validation."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any

import yaml

from ridgeback_autonomy.benchmarking.replay_profiles import (
    MASK_AXES,
    PROFILE_LIVE_SYSTEM,
    PROFILE_MASK_MODEL,
    PROFILE_MASK_OUTPUT,
    PROFILE_MEASUREMENT,
    ProfileValidationError,
    get_profile,
    validate_axis_values,
    validate_profile_axes,
)
from ridgeback_autonomy.benchmarking.sweep import SweepConfig, SweepSpec, load_sweep, parse_sweep
from ridgeback_autonomy.perception.target_localization.estimator_registry import parse_estimators


JOB_VERSION = 1
TOP_LEVEL_FIELDS = frozenset({
    'job_version', 'question', 'profile', 'inputs', 'sweep', 'resources',
    'output_dir', 'comparison_baseline', 'materializations',
})
INPUT_FIELDS = frozenset({'measurement_dataset', 'sensor_capture', 'mask_caches'})
RESOURCE_FIELDS = frozenset({'measurement_workers', 'model_workers'})


@dataclass(frozen=True)
class MaskMaterializationSpec:
    name: str
    arguments: dict[str, Any]


@dataclass(frozen=True)
class ReplayJob:
    source: str
    question: str
    profile: str
    inputs: dict[str, Any]
    sweep: SweepSpec
    measurement_workers: int
    model_workers: int
    output_dir: str
    comparison_baseline: str
    materializations: tuple[MaskMaterializationSpec, ...]
    document: dict[str, Any]


def _mapping(value: Any, field: str) -> dict:
    if not isinstance(value, dict):
        raise ProfileValidationError(
            field=field,
            code='invalid_type',
            message=f'Job field "{field}" must be a mapping.',
        )
    return value


def _positive_int(value: Any, field: str) -> int:
    if isinstance(value, bool):
        value = None
    try:
        parsed = int(value)
    except (TypeError, ValueError) as exc:
        raise ProfileValidationError(
            field=field,
            code='invalid_value',
            message=f'Job field "{field}" must be a positive integer.',
        ) from exc
    if parsed < 1 or str(parsed) != str(value).strip():
        raise ProfileValidationError(
            field=field,
            code='invalid_value',
            message=f'Job field "{field}" must be a positive integer.',
        )
    return parsed


def _load_sweep_value(value: Any, *, source: str) -> SweepSpec:
    if isinstance(value, str) and value.strip():
        path = Path(value).expanduser()
        if not path.is_absolute() and source != '<memory>':
            path = Path(source).resolve().parent / path
        return load_sweep(str(path))
    if isinstance(value, dict):
        return parse_sweep(value, source=f'{source} field "sweep"')
    raise ProfileValidationError(
        field='sweep',
        code='invalid_type',
        message='Job field "sweep" must be a sweep mapping or YAML path.',
    )


def _validate_variants(profile_id: str, sweep: SweepSpec) -> tuple[SweepConfig, ...]:
    profile = get_profile(profile_id)
    validate_profile_axes(profile_id, sweep.defaults)
    validate_axis_values(sweep.defaults)
    variants = []
    for config in sweep.configs:
        validate_profile_axes(profile_id, config.explicit_keys)
        validate_axis_values({
            key: config.arguments[key]
            for key in config.explicit_keys
            if key in config.arguments
        })
        selected = parse_estimators(config.arguments.get('estimators'))
        unsupported = [
            estimator for estimator in selected
            if estimator not in profile.compatible_estimators
        ]
        if unsupported:
            raise ProfileValidationError(
                field='estimators',
                code='incompatible_estimator',
                profile=profile_id,
                suggested_profile=(
                    PROFILE_LIVE_SYSTEM if profile_id == PROFILE_MEASUREMENT else None),
                message=(
                    f'Profile "{profile_id}" cannot evaluate estimator '
                    f'"{unsupported[0]}".'
                ),
            )
        variants.append(config)
    return tuple(variants)


def _materializations(value: Any, profile_id: str) -> tuple[MaskMaterializationSpec, ...]:
    if value in (None, []):
        if profile_id == PROFILE_MASK_MODEL:
            raise ProfileValidationError(
                field='materializations',
                code='missing_required_field',
                profile=profile_id,
                message='Profile "mask-model" requires at least one mask materialization.',
            )
        return ()
    if profile_id != PROFILE_MASK_MODEL:
        raise ProfileValidationError(
            field='materializations',
            code='upstream_stage_frozen',
            profile=profile_id,
            suggested_profile=PROFILE_MASK_MODEL,
            message=(
                f'Mask materialization is frozen by profile "{profile_id}"; '
                'choose profile "mask-model" to vary it.'
            ),
        )
    if not isinstance(value, list):
        raise ProfileValidationError(
            field='materializations', code='invalid_type',
            message='Job field "materializations" must be a list.')
    result = []
    seen = set()
    for index, raw in enumerate(value):
        item = _mapping(raw, f'materializations[{index}]')
        unknown = sorted(set(item) - ({'name'} | MASK_AXES))
        if unknown:
            raise ProfileValidationError(
                field=unknown[0], code='unknown_axis', profile=profile_id,
                message=f'Unknown mask materialization field "{unknown[0]}".')
        name = item.get('name')
        if not isinstance(name, str) or not name.strip():
            raise ProfileValidationError(
                field=f'materializations[{index}].name', code='invalid_value',
                message='Every mask materialization needs a non-empty name.')
        name = name.strip()
        if name in seen:
            raise ProfileValidationError(
                field='materializations.name', code='duplicate_name',
                message=f'Duplicate mask materialization name "{name}".')
        seen.add(name)
        arguments = {key: value for key, value in item.items() if key != 'name'}
        validate_profile_axes(profile_id, arguments)
        validate_axis_values(arguments)
        result.append(MaskMaterializationSpec(name=name, arguments=arguments))
    return tuple(result)


def parse_job(document: Any, *, source: str = '<memory>') -> ReplayJob:
    raw = _mapping(document, '<root>')
    unknown = sorted(set(raw) - TOP_LEVEL_FIELDS)
    if unknown:
        raise ProfileValidationError(
            field=unknown[0], code='unknown_field',
            message=f'Unknown replay job field "{unknown[0]}".')
    if raw.get('job_version') != JOB_VERSION:
        raise ProfileValidationError(
            field='job_version', code='unsupported_version',
            message=(
                f'Unsupported replay job version {raw.get("job_version")!r}; '
                f'expected {JOB_VERSION}.'),
        )
    profile_id = str(raw.get('profile', ''))
    get_profile(profile_id)
    inputs = _mapping(raw.get('inputs', {}), 'inputs')
    unknown_inputs = sorted(set(inputs) - INPUT_FIELDS)
    if unknown_inputs:
        raise ProfileValidationError(
            field=f'inputs.{unknown_inputs[0]}', code='unknown_field',
            message=f'Unknown replay input "{unknown_inputs[0]}".')

    required_input = {
        PROFILE_MEASUREMENT: 'measurement_dataset',
        PROFILE_MASK_OUTPUT: 'sensor_capture',
        PROFILE_MASK_MODEL: 'sensor_capture',
    }.get(profile_id)
    if required_input and not inputs.get(required_input):
        raise ProfileValidationError(
            field=f'inputs.{required_input}', code='missing_required_field',
            profile=profile_id,
            message=f'Profile "{profile_id}" requires input "{required_input}".',
        )
    mask_caches = inputs.get('mask_caches', [])
    if isinstance(mask_caches, str):
        mask_caches = [mask_caches]
        inputs = {**inputs, 'mask_caches': mask_caches}
    if not isinstance(mask_caches, list) or not all(
        isinstance(path, str) and path.strip() for path in mask_caches
    ):
        raise ProfileValidationError(
            field='inputs.mask_caches', code='invalid_type',
            message='Job input "mask_caches" must be a list of paths.')
    allowed_inputs = {
        PROFILE_MEASUREMENT: frozenset({'measurement_dataset'}),
        PROFILE_MASK_OUTPUT: frozenset({'sensor_capture', 'mask_caches'}),
        PROFILE_MASK_MODEL: frozenset({'sensor_capture'}),
        PROFILE_LIVE_SYSTEM: frozenset(),
    }[profile_id]
    incompatible_inputs = sorted(
        name for name, value in inputs.items()
        if value not in (None, '', []) and name not in allowed_inputs
    )
    if incompatible_inputs:
        name = incompatible_inputs[0]
        raise ProfileValidationError(
            field=f'inputs.{name}', code='incompatible_input', profile=profile_id,
            message=f'Profile "{profile_id}" does not consume input "{name}".',
        )
    if profile_id == PROFILE_MASK_OUTPUT and not mask_caches:
        raise ProfileValidationError(
            field='inputs.mask_caches', code='missing_required_field',
            profile=profile_id,
            message='Profile "mask-output" requires at least one mask cache.',
        )

    sweep_value = raw.get('sweep')
    if profile_id == PROFILE_LIVE_SYSTEM and not (
        isinstance(sweep_value, str) and sweep_value.strip()
    ):
        raise ProfileValidationError(
            field='sweep', code='live_sweep_requires_path', profile=profile_id,
            message=(
                'Profile "live-system" requires a sweep YAML path because it '
                'routes to target_benchmark_sweep.'),
        )
    sweep = _load_sweep_value(sweep_value, source=source)
    variants = _validate_variants(profile_id, sweep)
    baseline = str(raw.get('comparison_baseline') or variants[0].name)
    baseline_variant = baseline.split(':', 1)[-1]
    if baseline_variant not in {variant.name for variant in variants}:
        raise ProfileValidationError(
            field='comparison_baseline', code='unknown_baseline',
            message=f'Comparison baseline "{baseline}" is not a sweep variant.')

    resources = _mapping(raw.get('resources', {}), 'resources')
    unknown_resources = sorted(set(resources) - RESOURCE_FIELDS)
    if unknown_resources:
        raise ProfileValidationError(
            field=f'resources.{unknown_resources[0]}', code='unknown_field',
            message=f'Unknown replay resource field "{unknown_resources[0]}".')
    measurement_workers = _positive_int(
        resources.get('measurement_workers', 1), 'resources.measurement_workers')
    model_workers = _positive_int(
        resources.get('model_workers', 1), 'resources.model_workers')
    if profile_id != PROFILE_MASK_MODEL and model_workers != 1:
        raise ProfileValidationError(
            field='resources.model_workers', code='unused_resource',
            message=f'Profile "{profile_id}" does not run mask models.')

    output_dir = raw.get('output_dir', '')
    if not isinstance(output_dir, str):
        raise ProfileValidationError(
            field='output_dir', code='invalid_type',
            message='Job field "output_dir" must be a path string.')
    materializations = _materializations(raw.get('materializations'), profile_id)
    return ReplayJob(
        source=source,
        question=str(raw.get('question', '')),
        profile=profile_id,
        inputs=dict(inputs),
        sweep=sweep,
        measurement_workers=measurement_workers,
        model_workers=model_workers,
        output_dir=output_dir,
        comparison_baseline=baseline,
        materializations=materializations,
        document=json.loads(json.dumps(raw)),
    )


def load_job(path: str | Path) -> ReplayJob:
    resolved = Path(path).expanduser().resolve()
    try:
        with open(resolved, encoding='utf-8') as handle:
            document = yaml.safe_load(handle)
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f'Cannot read replay job {resolved}: {exc}') from exc
    return parse_job(document, source=str(resolved))


def canonical_job_json(job: ReplayJob) -> str:
    return json.dumps(job.document, indent=2, sort_keys=True) + '\n'


def command_argv(job_path: str, job: ReplayJob, *, output_dir: str | None = None) -> list[str]:
    if job.profile == PROFILE_LIVE_SYSTEM:
        return ['ros2', 'run', 'ridgeback_autonomy', 'target_benchmark_sweep', job.sweep.source]
    output = output_dir or job.output_dir
    argv = ['ros2', 'run', 'ridgeback_autonomy', 'target_replay_benchmark', job_path]
    if output:
        argv.extend(['--output-dir', output])
    return argv
