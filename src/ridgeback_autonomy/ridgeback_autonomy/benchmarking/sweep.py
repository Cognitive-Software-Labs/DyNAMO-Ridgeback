"""ROS-free loader and validator for benchmark sweep YAML files."""

from __future__ import annotations

from dataclasses import dataclass
import os
import re

import yaml

from ridgeback_autonomy.perception.target_localization.estimator_registry import (
    DEPTH_PATH_ESTIMATORS,
    MASK_ESTIMATORS,
    parse_estimators,
)
from ridgeback_autonomy.perception.target_localization.launch import (
    CONFIG_LAUNCH_ARGUMENT_NAMES,
    ENV_LAYER_CONFIG_KEYS,
)
from ridgeback_autonomy.benchmarking.scenarios import load_scenarios


PATH_SAFE_NAME = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._-]*$')
TOP_LEVEL_KEYS = frozenset({'sweep', 'defaults', 'configs'})
SWEEP_METADATA_KEYS = frozenset({'name', 'description'})
# These belong only to the environment launch, but are legal as sweep-wide
# defaults: an installation selects its Clearpath setup once, and the detector
# rate and its diagnostic are fixed for the life of the persistent layer.
ENVIRONMENT_ONLY_DEFAULT_NAMES = frozenset({
    'setup_path',
    'detector_fps',
    'detector_debug',
})
SWEEP_DEFAULT_ARGUMENT_NAMES = (
    CONFIG_LAUNCH_ARGUMENT_NAMES | ENVIRONMENT_ONLY_DEFAULT_NAMES)

# Only explicit per-config keys are checked. A defaults block may intentionally
# spell every axis once while legacy configurations inherit knobs they never
# consume; those inherited values do not create duplicate runs.
KNOB_ESTIMATORS = {
    'depth_source': DEPTH_PATH_ESTIMATORS,
    'mask_depth_max_meters': DEPTH_PATH_ESTIMATORS,
    'mask_gate': MASK_ESTIMATORS,
    'isolation_2d': frozenset({'projective_ranging'}),
    'isolation_3d': frozenset({'euclidean_reconstruction'}),
}


@dataclass(frozen=True)
class SweepConfig:
    name: str
    arguments: dict[str, str]
    explicit_keys: frozenset[str]


@dataclass(frozen=True)
class SweepSpec:
    name: str
    description: str
    defaults: dict[str, str]
    configs: tuple[SweepConfig, ...]
    source: str


def _path_safe_name(value, *, field: str, source: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f'Sweep file {source}: "{field}" must be a non-empty string.')
    name = value.strip()
    if name in {'.', '..'} or PATH_SAFE_NAME.fullmatch(name) is None:
        raise ValueError(
            f'Sweep file {source}: "{field}" must be path-safe '
            f'(letters, digits, dot, dash, underscore), got {value!r}.'
        )
    return name


def _launch_value(value, *, owner: str, field: str) -> str:
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, (str, int, float)) and not isinstance(value, complex):
        return str(value)
    raise ValueError(
        f'{owner}: field "{field}" must be a scalar launch value, got {value!r}.'
    )


def _validate_keys(
    raw: dict,
    *,
    legal: frozenset[str],
    owner: str,
) -> None:
    unknown = sorted(set(raw) - legal)
    if unknown:
        raise ValueError(f'{owner}: unknown field "{unknown[0]}".')


def _scenario_to_validate(
    scenario: str,
    *,
    source: str,
    default_scenario_path: str | None,
) -> str | None:
    if scenario.strip():
        path = os.path.expanduser(scenario.strip())
        if not os.path.isabs(path):
            base = os.getcwd() if source == '<memory>' else os.path.dirname(os.path.abspath(source))
            path = os.path.join(base, path)
        return os.path.abspath(path)
    if default_scenario_path:
        return os.path.abspath(os.path.expanduser(default_scenario_path))
    return None


def _validate_config(
    name: str,
    arguments: dict[str, str],
    explicit_keys: frozenset[str],
    *,
    source: str,
    default_scenario_path: str | None,
) -> dict[str, str]:
    owner = f'Sweep config "{name}"'
    try:
        selected = parse_estimators(arguments.get('estimators'))
    except ValueError as exc:
        raise ValueError(f'{owner}: field "estimators": {exc}') from exc

    resolved = dict(arguments)
    resolved['estimators'] = ','.join(selected)

    for key in sorted(explicit_keys):
        allowed_estimators = KNOB_ESTIMATORS.get(key)
        if allowed_estimators is None:
            continue
        if not any(estimator in allowed_estimators for estimator in selected):
            supported = ', '.join(sorted(allowed_estimators))
            raise ValueError(
                f'{owner}: field "{key}" does not apply to estimators '
                f'"{resolved["estimators"]}"; it requires one of: {supported}.'
            )

    repeats_raw = resolved.get('repeats', '5')
    try:
        repeats = int(repeats_raw)
    except ValueError as exc:
        raise ValueError(f'{owner}: field "repeats" must be a positive integer.') from exc
    if repeats < 1 or str(repeats) != str(repeats_raw).strip():
        raise ValueError(f'{owner}: field "repeats" must be a positive integer.')
    resolved['repeats'] = str(repeats)

    scenario = resolved.get('scenario', '')
    scenario_path = _scenario_to_validate(
        scenario,
        source=source,
        default_scenario_path=default_scenario_path,
    )
    if scenario_path is not None:
        if not os.path.isfile(scenario_path):
            raise ValueError(
                f'{owner}: field "scenario" does not name a file: {scenario_path}'
            )
        try:
            load_scenarios(scenario_path)
        except (OSError, ValueError) as exc:
            raise ValueError(f'{owner}: field "scenario" is invalid: {exc}') from exc
        # Preserve empty: the runner resolves its packaged default itself.
        if scenario.strip():
            resolved['scenario'] = scenario_path

    return resolved


def parse_sweep(
    document,
    *,
    source: str = '<memory>',
    default_scenario_path: str | None = None,
) -> SweepSpec:
    """Parse and validate a loaded sweep document."""

    if not isinstance(document, dict):
        raise ValueError(f'Sweep file {source} must be a mapping at the top level.')
    _validate_keys(document, legal=TOP_LEVEL_KEYS, owner=f'Sweep file {source}')

    metadata = document.get('sweep')
    if not isinstance(metadata, dict):
        raise ValueError(f'Sweep file {source}: "sweep" must be a mapping.')
    _validate_keys(metadata, legal=SWEEP_METADATA_KEYS, owner=f'Sweep file {source} field "sweep"')
    name = _path_safe_name(metadata.get('name'), field='sweep.name', source=source)
    description = metadata.get('description', '')
    if not isinstance(description, str):
        raise ValueError(f'Sweep file {source}: "sweep.description" must be a string.')

    defaults_raw = document.get('defaults') or {}
    if not isinstance(defaults_raw, dict):
        raise ValueError(f'Sweep file {source}: "defaults" must be a mapping when present.')
    _validate_keys(
        defaults_raw,
        legal=SWEEP_DEFAULT_ARGUMENT_NAMES,
        owner=f'Sweep file {source} field "defaults"',
    )
    defaults = {
        key: _launch_value(value, owner=f'Sweep file {source} field "defaults"', field=key)
        for key, value in defaults_raw.items()
    }

    configs_raw = document.get('configs')
    if not isinstance(configs_raw, list) or not configs_raw:
        raise ValueError(f'Sweep file {source} must define a non-empty "configs" list.')

    configs: list[SweepConfig] = []
    seen: set[str] = set()
    for raw in configs_raw:
        if not isinstance(raw, dict):
            raise ValueError(f'Sweep file {source}: each config must be a mapping, got {raw!r}.')
        config_name = _path_safe_name(raw.get('name'), field='config.name', source=source)
        if config_name in seen:
            raise ValueError(f'Sweep file {source}: duplicate config name "{config_name}".')
        seen.add(config_name)

        config_values = {key: value for key, value in raw.items() if key != 'name'}
        env_keys = sorted(set(config_values) & ENV_LAYER_CONFIG_KEYS)
        if env_keys:
            raise ValueError(
                f'Sweep config "{config_name}": field "{env_keys[0]}" belongs to the '
                'persistent environment and may only be set in sweep defaults.'
            )
        _validate_keys(
            config_values,
            legal=CONFIG_LAUNCH_ARGUMENT_NAMES,
            owner=f'Sweep config "{config_name}"',
        )
        explicit = {
            key: _launch_value(value, owner=f'Sweep config "{config_name}"', field=key)
            for key, value in config_values.items()
        }
        merged = {
            key: value for key, value in defaults.items()
            if key in CONFIG_LAUNCH_ARGUMENT_NAMES
        }
        merged.update(explicit)
        resolved = _validate_config(
            config_name,
            merged,
            frozenset(explicit),
            source=source,
            default_scenario_path=default_scenario_path,
        )
        configs.append(SweepConfig(
            name=config_name,
            arguments=resolved,
            explicit_keys=frozenset(explicit),
        ))

    return SweepSpec(
        name=name,
        description=description.strip(),
        defaults=defaults,
        configs=tuple(configs),
        source=source,
    )


def load_sweep(
    path: str,
    *,
    default_scenario_path: str | None = None,
) -> SweepSpec:
    """Load and validate a sweep YAML file."""

    resolved_path = os.path.abspath(os.path.expanduser(path))
    with open(resolved_path, 'r', encoding='utf-8') as handle:
        document = yaml.safe_load(handle)
    return parse_sweep(
        document,
        source=resolved_path,
        default_scenario_path=default_scenario_path,
    )
