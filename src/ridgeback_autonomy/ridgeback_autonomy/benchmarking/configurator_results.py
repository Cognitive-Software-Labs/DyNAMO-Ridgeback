"""Read and rename what benchmarks leave under ``artifacts/benchmarks``.

Two unlike things share that directory. Typed artifacts are evidence fed *into*
a run; run outputs are results read *out* of one. They are listed apart and
renamed under different rules, because a directory name is load-bearing for some
of them and inert for others.
"""

from __future__ import annotations

import json
import os
from pathlib import Path

import yaml

from ridgeback_autonomy.benchmarking.configurator_errors import ConfiguratorError
from ridgeback_autonomy.benchmarking.paths import (
    BENCHMARK_ROOT_RELATIVE,
    JOB_ROOT_RELATIVE,
    anchored_path,
    write_json_atomic,
)
from ridgeback_autonomy.benchmarking.replay_artifacts import ReplayArtifact, load_replay_input


KIND_SWEEP = 'sweep'
KIND_SWEEP_CONFIG = 'sweep-config'
KIND_REPLAY_OUTPUT = 'replay-output'
KIND_TRIAL_OUTPUT = 'trial-output'
KIND_ARTIFACT = 'artifact'
KIND_STAGING = 'staging'
KIND_CONTAINER = 'container'

# Evidence you feed in versus results you read out. The browser groups on this
# because it decides which list an entry belongs in, not how it is rendered.
RESULT_KINDS = frozenset({KIND_SWEEP, KIND_REPLAY_OUTPUT, KIND_TRIAL_OUTPUT})

# `os.replace` of a finished staging directory is what publishes a replay
# result; an interrupted writer leaves its staging directory behind instead.
STAGING_MARKER = '.partial-'
# Sweeps land one level down when an operator groups them, so a plain top-level
# listing would miss them.
CONTAINER_DEPTH = 2
MAX_LISTED_ENTRIES = 200


def classify(path: Path) -> str:
    """What kind of directory this is, decided by what it contains.

    Order matters. A sweep's per-config directory carries its own ``run.json``
    and is indistinguishable from a standalone trial output by content alone, so
    membership of a sweep is tested before anything else.
    """

    if path.name.startswith('.') and STAGING_MARKER in path.name:
        return KIND_STAGING
    if (path.parent / 'sweep.json').is_file():
        return KIND_SWEEP_CONFIG
    if (path / 'sweep.json').is_file():
        return KIND_SWEEP
    if (path / 'job.json').is_file() and (path / 'results').is_dir():
        return KIND_REPLAY_OUTPUT
    if (path / 'run.json').is_file():
        return KIND_TRIAL_OUTPUT
    if (path / 'manifest.json').is_file():
        return KIND_ARTIFACT
    return KIND_CONTAINER


def _load_json(path: Path) -> dict:
    try:
        payload = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return {}
    return payload if isinstance(payload, dict) else {}


def _sweep_summary(path: Path) -> dict:
    manifest = _load_json(path / 'sweep.json')
    configs = manifest.get('configs') or []
    sweep = manifest.get('sweep') or {}
    return {
        'name': sweep.get('name'),
        'status': sweep.get('status'),
        'started': sweep.get('started'),
        'finished': sweep.get('finished'),
        'configs': len(configs),
        'completed': sum(1 for item in configs if item.get('status') == 'success'),
        'software_gl': (manifest.get('gl') or {}).get('software'),
    }


def _replay_summary(path: Path) -> dict:
    job = _load_json(path / 'job.json')
    sweep = job.get('sweep')
    variants = sweep.get('configs') if isinstance(sweep, dict) else None
    return {
        'profile': job.get('profile'),
        'question': job.get('question'),
        'variants': len(variants) if isinstance(variants, list) else None,
    }


def _trial_summary(path: Path) -> dict:
    run = _load_json(path / 'run.json').get('run') or {}
    return {
        'label': run.get('label'),
        'started': run.get('started'),
        'commit': run.get('commit'),
        'trials': run.get('trials_included'),
    }


def artifact_summary(value) -> dict:
    """The operator-facing shape of one typed artifact or legacy dataset."""

    if isinstance(value, ReplayArtifact):
        trials = value.trial_entries
        return {
            'path': str(value.root), 'kind': value.kind, 'state': value.manifest['artifact']['state'],
            'id': value.id, 'trials': len(trials),
            'events': sum(int(item.get('event_count', 0)) for item in trials),
            'detections': sum(int(item.get('detection_count', 0)) for item in trials),
            'manifest_version': value.manifest.get('manifest_version'),
        }
    trials = value.trial_entries
    return {
        'path': str(value.root), 'kind': 'legacy-measurement', 'state': value.manifest.get('state'),
        'trials': len(trials), 'events': sum(int(item.get('event_count', 0)) for item in trials),
        'schema_version': value.manifest.get('schema_version'),
    }


def inspect_path(value: str, workspace_root: Path) -> dict:
    """Describe one operator-supplied path, anchored exactly as execution will."""

    path = Path(anchored_path(str(workspace_root), str(value)))
    if not path.exists():
        raise ConfiguratorError('path', 'missing_path', f'Path does not exist: {path}')
    try:
        return {**artifact_summary(load_replay_input(path)), 'issues': []}
    except ValueError as exc:
        return {'path': str(path), 'kind': 'unknown', 'issues': [str(exc)]}


def discover_artifacts(workspace_root: Path) -> list[dict]:
    """Typed evidence an operator can select as a job input."""

    root = workspace_root / BENCHMARK_ROOT_RELATIVE
    if not root.is_dir():
        return []
    return [
        inspect_path(str(path.parent), workspace_root)
        for path in sorted(root.rglob('manifest.json'))[:100]
        # An interrupted writer's staging directory carries a manifest but is
        # not a published artifact; offering it would hand out a partial input.
        if not any(part.startswith('.') and STAGING_MARKER in part for part in path.parent.parts)
    ]


def _jobs_referencing(workspace_root: Path, target: Path) -> list[str]:
    """Saved jobs whose inputs name this exact directory.

    Artifact lineage is content-addressed and survives a rename, but a job
    refers to its inputs by path, so these are the files a rename would break.
    """

    directory = workspace_root / JOB_ROOT_RELATIVE
    if not directory.is_dir():
        return []
    referencing = []
    for path in sorted(directory.glob('*.y*ml')):
        try:
            document = yaml.safe_load(path.read_text(encoding='utf-8'))
        except (OSError, yaml.YAMLError):
            continue
        inputs = (document or {}).get('inputs') if isinstance(document, dict) else None
        if not isinstance(inputs, dict):
            continue
        values = [inputs.get('measurement_dataset'), inputs.get('sensor_capture')]
        values.extend(inputs.get('mask_caches') or [])
        if any(
            isinstance(value, str) and value.strip()
            and Path(anchored_path(str(workspace_root), value)) == target
            for value in values
        ):
            referencing.append(str(path))
    return referencing


def rename_policy(workspace_root: Path, path: Path, kind: str) -> dict:
    """Whether this directory's name is safe to change, and what it costs."""

    if kind == KIND_SWEEP_CONFIG:
        return {'allowed': False, 'reason': (
            'A sweep addresses this directory as <sweep>/<config name>, so resume '
            'and reporting would both stop finding it.')}
    if kind == KIND_STAGING:
        return {'allowed': False, 'reason': (
            'This is an interrupted writer\'s staging directory, not a published '
            'result.')}
    if kind == KIND_CONTAINER:
        return {'allowed': False, 'reason': 'Not a benchmark artifact or run output.'}
    if kind == KIND_SWEEP:
        return {'allowed': True, 'note': (
            'The sweep manifest stores this directory as an absolute path for '
            'every config; renaming rewrites those entries.')}
    if kind == KIND_ARTIFACT:
        referencing = _jobs_referencing(workspace_root, path)
        warning = (
            'Artifact lineage is content-addressed and survives this, but saved '
            'jobs name inputs by path.')
        if referencing:
            warning += ' These jobs reference the current path: ' + ', '.join(
                Path(item).name for item in referencing)
        return {'allowed': True, 'warning': warning, 'referencing_jobs': referencing}
    # Replay and trial outputs record their provenance and their run label
    # independently of where they sit, so the directory name carries nothing.
    return {'allowed': True}


def describe(workspace_root: Path, path: Path, kind: str) -> dict:
    summary = {
        KIND_SWEEP: _sweep_summary,
        KIND_REPLAY_OUTPUT: _replay_summary,
        KIND_TRIAL_OUTPUT: _trial_summary,
    }.get(kind)
    entry = {
        'path': str(path),
        'name': path.name,
        'parent': str(path.parent),
        'kind': kind,
        'category': 'result' if kind in RESULT_KINDS else 'artifact',
        'modified': path.stat().st_mtime,
        'summary': summary(path) if summary else {},
        'rename': rename_policy(workspace_root, path, kind),
    }
    if kind == KIND_ARTIFACT:
        try:
            entry['summary'] = artifact_summary(load_replay_input(path))
        except ValueError as exc:
            entry['summary'] = {'issues': [str(exc)]}
    return entry


def discover_results(workspace_root: Path) -> list[dict]:
    """Everything under ``artifacts/benchmarks`` an operator can open or rename."""

    root = workspace_root / BENCHMARK_ROOT_RELATIVE
    if not root.is_dir():
        return []
    entries: list[dict] = []

    def walk(directory: Path, depth: int) -> None:
        for path in sorted(directory.iterdir(), reverse=True):
            if len(entries) >= MAX_LISTED_ENTRIES or not path.is_dir():
                continue
            kind = classify(path)
            if kind == KIND_STAGING:
                continue
            if kind == KIND_CONTAINER:
                if depth < CONTAINER_DEPTH:
                    walk(path, depth + 1)
                continue
            entries.append(describe(workspace_root, path, kind))

    walk(root, 1)
    return entries


def _validated_name(value: str) -> str:
    candidate = str(value).strip()
    separators = {os.sep, os.altsep} - {None}
    if (not candidate or candidate.startswith('.')
            or candidate != Path(candidate).name
            or any(separator in candidate for separator in separators)):
        raise ConfiguratorError(
            'name', 'invalid_name',
            'A new name must be one path component: no separators, no traversal, '
            'and no leading dot.')
    return candidate


def _validated_target(workspace_root: Path, value: str) -> Path:
    root = (workspace_root / BENCHMARK_ROOT_RELATIVE).resolve()
    path = Path(anchored_path(str(workspace_root), str(value)))
    if path == root or root not in path.parents:
        raise ConfiguratorError(
            'path', 'outside_benchmarks',
            f'Only directories under {root} can be renamed.')
    if not path.is_dir():
        raise ConfiguratorError('path', 'missing_path', f'Not a directory: {path}')
    return path


def _rewrite_sweep_output_paths(sweep_dir: Path) -> None:
    """Point every config's recorded output directory at the sweep's new name.

    The sweep writes this key as the sweep directory itself for every config, so
    reassigning it is a restatement of what it already means -- which also
    repairs entries left stale by an earlier move.
    """

    manifest_path = sweep_dir / 'sweep.json'
    manifest = _load_json(manifest_path)
    for config in manifest.get('configs') or []:
        arguments = config.get('arguments')
        if isinstance(arguments, dict) and 'output_dir' in arguments:
            arguments['output_dir'] = str(sweep_dir)
    write_json_atomic(manifest_path, manifest)


def rename_result(workspace_root: Path, value: str, name: str) -> dict:
    """Rename one result directory, refusing the cases where the name is used."""

    path = _validated_target(workspace_root, value)
    candidate = _validated_name(name)
    kind = classify(path)
    policy = rename_policy(workspace_root, path, kind)
    if not policy['allowed']:
        raise ConfiguratorError('path', 'rename_refused', policy['reason'])
    destination = path.parent / candidate
    if destination.exists():
        raise ConfiguratorError(
            'name', 'name_taken', f'A directory named "{candidate}" is already there.')
    os.rename(path, destination)
    if kind == KIND_SWEEP:
        try:
            _rewrite_sweep_output_paths(destination)
        except OSError:
            os.rename(destination, path)
            raise ConfiguratorError(
                'path', 'rename_failed',
                'The sweep manifest could not be rewritten, so the directory was '
                'put back under its original name.') from None
    return describe(workspace_root, destination, kind)
