"""Loopback-only benchmark job configurator.

The browser deliberately owns presentation and draft state only. This module
converts that state to the canonical replay-job format and validates it with
the same parser and command renderer that benchmark execution uses.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import secrets
import shlex
import sys
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import webbrowser

import yaml

from ridgeback_autonomy.benchmarking.replay_artifacts import (
    SENSOR_CAPTURE_KIND,
    ReplayArtifact,
    load_replay_input,
)
from ridgeback_autonomy.benchmarking.replay_jobs import command_argv, parse_job
from ridgeback_autonomy.benchmarking.replay_profiles import (
    describe_capabilities,
    ProfileValidationError,
)


ARTIFACT_ROOT_RELATIVE = Path('artifacts/benchmarks')


@dataclass(frozen=True)
class ConfiguratorError(Exception):
    """One field-oriented error suitable for the browser response."""

    field: str
    code: str
    message: str
    suggested_profile: str | None = None

    def as_dict(self) -> dict[str, str]:
        result = {'field': self.field, 'code': self.code, 'message': self.message}
        if self.suggested_profile:
            result['suggested_profile'] = self.suggested_profile
        return result

def _safe_variant_name(value, index: int) -> str:
    name = str(value or f'variant_{index + 1}').strip()
    if not name or any(char not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789._-' for char in name):
        raise ConfiguratorError('variants', 'invalid_name', 'Variant names use letters, digits, dot, dash, and underscore.')
    return name

def _stage_labels(stages: list[str]) -> list[str]:
    return [stage.replace('-', ' ').title() for stage in stages]


def capabilities() -> dict:
    """The canonical capability contract with small presentation adaptations."""

    contract = describe_capabilities()
    axes = {axis['name']: axis for axis in contract['axes']}
    profiles = {}
    for profile in contract['profiles']:
        fields = []
        materialization_fields = []
        for name in profile['allowed_axes']:
            axis = axes[name]
            if axis['stage'] not in {'measurement', 'mask-production'}:
                continue
            rendered = {
                'key': name,
                'label': axis['label'],
                'type': (
                    'select' if axis.get('choices') else
                    'integer' if axis['type'] == 'integer' else
                    'number' if axis['type'] == 'number' else 'text'),
                'default': axis.get('default', ''),
                'options': axis.get('choices', []),
                'min': axis.get('minimum'), 'max': axis.get('maximum'),
                'estimators': axis.get('estimators', []),
            }
            (fields if axis['stage'] == 'measurement' else materialization_fields).append(rendered)
        profiles[profile['id']] = {
            'label': profile['label'], 'available': True,
            'held_constant': _stage_labels(profile['frozen_stages']),
            'rerun': _stage_labels(profile['rerun_stages']),
            'claims': profile['supported_claims'],
            'limitations': profile['limitations'],
            'variant_fields': fields,
            'materialization_fields': materialization_fields,
        }
    descriptions = {
        'measurement': 'Rerun measurement against frozen legacy evidence.',
        'mask-output': 'Compare the selected immutable mask caches.',
        'mask-model': 'Materialize candidate masks from frozen sensor evidence.',
        'live-system': 'Use the existing live sweep for system-level evidence.',
    }
    return {
        'contract': contract,
        'questions': [{**question, 'description': descriptions[question['profile']]}
                      for question in contract['questions']],
        'profiles': profiles,
        'defaults': {'workers': 1, 'model_workers': 1},
    }


def _issue(error: ProfileValidationError) -> dict:
    return error.as_dict()


def _ui_sweep(raw: dict, profile: str) -> dict | str:
    if profile == 'live-system':
        return str(raw.get('sweep_path', '')).strip()
    descriptor = capabilities()['profiles'][profile]
    variants = raw.get('variants') or [{'name': 'baseline', 'arguments': {}}]
    configs = []
    for index, item in enumerate(variants):
        if not isinstance(item, dict):
            raise ConfiguratorError('variants', 'invalid_variant', 'Every variant must be an object.')
        name = _safe_variant_name(item.get('name'), index)
        supplied = item.get('arguments') or {}
        if not isinstance(supplied, dict):
            raise ConfiguratorError('variants', 'invalid_variant', 'Variant arguments must be an object.')
        arguments = {}
        for field in descriptor['variant_fields']:
            key = field['key']
            arguments[key] = supplied.get(key, field['default'])
        selected_estimators = {
            value.strip() for value in str(arguments.get('estimators', '')).split(',')
            if value.strip()
        }
        for field in descriptor['variant_fields']:
            compatible = set(field.get('estimators', ()))
            if compatible and not compatible.intersection(selected_estimators):
                arguments.pop(field['key'], None)
        configs.append({'name': name, **arguments})
    return {
        'sweep': {'name': f'gui_{profile}', 'description': 'Generated by target_benchmark_configurator.'},
        'configs': configs,
    }


def _canonical_document(raw: object) -> dict:
    if not isinstance(raw, dict):
        raise ConfiguratorError('job', 'invalid_job', 'The job must be a JSON object.')
    profile = str(raw.get('profile', 'measurement'))
    inputs = raw.get('inputs') or {}
    if not isinstance(inputs, dict):
        raise ConfiguratorError('inputs', 'invalid_inputs', 'Inputs must be an object.')
    selected_inputs: dict[str, object]
    if profile == 'measurement':
        selected_inputs = {'measurement_dataset': str(inputs.get('dataset', inputs.get('measurement_dataset', ''))).strip()}
    elif profile == 'mask-output':
        caches = inputs.get('mask_caches', [])
        if isinstance(caches, str):
            caches = [item.strip() for item in caches.split(',') if item.strip()]
        selected_inputs = {
            'sensor_capture': str(inputs.get('sensor_capture', '')).strip(),
            'mask_caches': caches,
        }
    elif profile == 'mask-model':
        selected_inputs = {'sensor_capture': str(inputs.get('sensor_capture', '')).strip()}
    else:
        selected_inputs = {}
    materializations = raw.get('materializations')
    if profile == 'mask-model' and not materializations:
        materializations = [{'name': 'candidate', 'mask_producer': 'slimsam'}]
    return {
        'job_version': 1,
        'question': str(raw.get('question', '')),
        'profile': profile,
        'inputs': selected_inputs,
        'sweep': _ui_sweep(raw, profile),
        'resources': {
            'measurement_workers': raw.get('workers', 1),
            'model_workers': raw.get('model_workers', 1),
        },
        'output_dir': str(raw.get('output_dir', '')).strip(),
        'comparison_baseline': str(raw.get('baseline', '')).strip(),
        **({'materializations': materializations} if materializations else {}),
    }


def _artifact_summary(value) -> dict:
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


def _inspect_job_inputs(job) -> dict[str, dict]:
    if job.profile == 'live-system':
        return {}
    if job.profile == 'measurement':
        return {'measurement_dataset': _artifact_summary(
            load_replay_input(job.inputs['measurement_dataset']))}
    sensor = load_replay_input(job.inputs['sensor_capture'])
    if not isinstance(sensor, ReplayArtifact) or sensor.kind != SENSOR_CAPTURE_KIND:
        raise ValueError('The selected sensor capture is not a typed sensor-capture artifact.')
    inspected = {'sensor_capture': _artifact_summary(sensor)}
    if job.profile == 'mask-output':
        inspected['mask_caches'] = [
            _artifact_summary(load_replay_input(path, parent=sensor))
            for path in job.inputs['mask_caches']
        ]
    return inspected


def _estimate(job, artifacts: dict[str, dict]) -> dict:
    root = artifacts.get('measurement_dataset') or artifacts.get('sensor_capture')
    if not root:
        return {'trials': None, 'events': None, 'measurement_work': None,
                'capture_duration': 'Live sweep; use its --dry-run estimate.',
                'hard_timeout': 'Defined by the referenced sweep.',
                'assumptions': ['Live execution is delegated to target_benchmark_sweep.']}
    variants = len(job.sweep.configs)
    events = root['events']
    return {
        'trials': root['trials'], 'events': events, 'measurement_work': events * variants,
        'capture_duration': 'Frozen evidence; no live capture.',
        'hard_timeout': 'Not applicable to replay evaluation.',
        'assumptions': ['Every selected variant consumes the same frozen evidence.'],
    }


def validate_job(raw: object) -> dict:
    """Resolve the browser draft through the canonical replay-job validator."""

    try:
        document = _canonical_document(raw)
        job = parse_job(document)
    except ConfiguratorError as exc:
        return {'job': raw, 'issues': [exc.as_dict()], 'valid': False}
    except ProfileValidationError as exc:
        return {'job': document, 'issues': [_issue(exc)], 'valid': False}
    except ValueError as exc:
        return {'job': document, 'issues': [{
            'field': 'job', 'code': 'cli_validation', 'message': str(exc),
        }], 'valid': False}
    issues = []
    try:
        artifacts = _inspect_job_inputs(job)
    except ValueError as exc:
        artifacts = {}
        issues.append({'field': 'inputs', 'code': 'invalid_artifact', 'message': str(exc)})
    return {'job': job.document, 'issues': issues, 'artifacts': artifacts, 'valid': not issues,
            'estimates': _estimate(job, artifacts)}


def render_job(raw: object) -> dict:
    outcome = validate_job(raw)
    if not outcome['valid']:
        return outcome
    job = parse_job(outcome['job'])
    filename = f'benchmark-{job.profile}.yaml'
    argv = command_argv(f'./{filename}', job)
    return {**outcome, 'yaml': yaml.safe_dump(job.document, sort_keys=False, allow_unicode=True),
            'filename': filename, 'argv': argv, 'command': shlex.join(argv)}


def _ui_draft(document: dict) -> dict:
    job = parse_job(document)
    fields = {
        field['key']: field
        for field in capabilities()['profiles'][job.profile].get('variant_fields', [])
    }

    def ui_arguments(arguments: dict[str, object]) -> dict[str, object]:
        rendered = {}
        for key, value in arguments.items():
            field = fields.get(key, {})
            if field.get('type') == 'integer':
                rendered[key] = int(value)
            elif field.get('type') == 'number':
                rendered[key] = float(value)
            else:
                rendered[key] = value
        return rendered

    raw = {
        'question': job.question, 'profile': job.profile,
        'inputs': {}, 'workers': job.measurement_workers,
        'model_workers': job.model_workers, 'output_dir': job.output_dir,
        'baseline': job.comparison_baseline,
        'variants': [
            {'name': item.name, 'arguments': ui_arguments(item.arguments)}
            for item in job.sweep.configs
        ],
    }
    if job.profile == 'measurement':
        raw['inputs']['dataset'] = job.inputs['measurement_dataset']
    elif job.profile == 'live-system':
        raw['sweep_path'] = str(document['sweep'])
    else:
        raw['inputs']['sensor_capture'] = job.inputs['sensor_capture']
        raw['inputs']['mask_caches'] = job.inputs.get('mask_caches', [])
    if job.materializations:
        raw['materializations'] = [
            {'name': item.name, **item.arguments} for item in job.materializations]
    return raw


def import_job(text: str) -> dict:
    try:
        document = yaml.safe_load(text)
        draft = _ui_draft(document)
        outcome = validate_job(draft)
        return {**outcome, 'job': draft}
    except (yaml.YAMLError, ProfileValidationError, ValueError) as exc:
        raise ConfiguratorError('import', 'invalid_job', f'Cannot import replay job: {exc}') from exc


def inspect_path(value: str) -> dict:
    path = Path(value).expanduser().resolve()
    if not path.exists():
        raise ConfiguratorError('path', 'missing_path', f'Path does not exist: {path}')
    try:
        return {**_artifact_summary(load_replay_input(path)), 'issues': []}
    except ValueError as exc:
        return {'path': str(path), 'kind': 'unknown', 'issues': [str(exc)]}


def discover_artifacts(workspace_root: Path) -> list[dict]:
    root = workspace_root / ARTIFACT_ROOT_RELATIVE
    if not root.is_dir():
        return []
    return [inspect_path(str(path.parent)) for path in sorted(root.rglob('manifest.json'))[:100]]


def _assets_directory() -> Path:
    local = Path(__file__).with_name('configurator_assets')
    if local.is_dir():
        return local.resolve()
    try:
        from ament_index_python.packages import get_package_share_directory
        shared = Path(get_package_share_directory('ridgeback_autonomy')) / 'configurator_assets'
        if shared.is_dir():
            return shared.resolve()
    except ImportError:
        pass
    raise RuntimeError('Configurator static assets are not installed.')


def _workspace_root() -> Path:
    for candidate in Path(__file__).resolve().parents:
        if (candidate / '.git').exists():
            return candidate
    return Path.cwd()


def make_server(*, port: int = 0, token: str | None = None, workspace_root: Path | None = None) -> ThreadingHTTPServer:
    """Create an unstarted loopback server for tests and the executable."""

    expected_token = token or secrets.token_urlsafe(32)
    assets = _assets_directory().resolve()
    root = workspace_root or _workspace_root()

    class Handler(BaseHTTPRequestHandler):
        server_version = 'RidgebackBenchmarkConfigurator/1.0'

        def log_message(self, _format, *_args):
            return

        def _json(self, status: int, payload: object) -> None:
            body = json.dumps(payload).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def _authorised(self) -> bool:
            return self.headers.get('X-Configurator-Token') == expected_token

        def _body(self):
            length = int(self.headers.get('Content-Length', '0'))
            if length > 2_000_000:
                raise ConfiguratorError('request', 'too_large', 'Request body exceeds 2 MB.')
            try:
                return json.loads(self.rfile.read(length).decode('utf-8'))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise ConfiguratorError('request', 'invalid_json', 'Request body must be JSON.') from exc

        def _static(self, relative: str) -> None:
            # Assets are symlink-installed by colcon. Resolving an individual
            # file follows that link back to the source tree, which is valid
            # but no longer lexically below the installed asset directory.
            # This server exposes a fixed set of basenames, so reject every
            # other lexical path before reading it instead of resolving it.
            if relative not in {'index.html', 'app.js', 'app.css'} or Path(relative).name != relative:
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            candidate = assets / relative
            if not candidate.is_file():
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            content_type = 'text/html; charset=utf-8' if candidate.suffix == '.html' else 'text/css; charset=utf-8' if candidate.suffix == '.css' else 'application/javascript; charset=utf-8'
            body = candidate.read_bytes()
            self.send_response(HTTPStatus.OK)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            self.server.last_request = time.monotonic()  # type: ignore[attr-defined]
            path = self.path.split('?', 1)[0]
            if path == '/':
                return self._static('index.html')
            if path == '/app.js':
                return self._static('app.js')
            if path == '/app.css':
                return self._static('app.css')
            if path.startswith('/api/'):
                if not self._authorised():
                    return self._json(HTTPStatus.FORBIDDEN, {'error': 'Invalid configurator token.'})
                if path == '/api/capabilities':
                    return self._json(HTTPStatus.OK, capabilities())
                if path == '/api/artifacts':
                    return self._json(HTTPStatus.OK, {'artifacts': discover_artifacts(root)})
            self.send_error(HTTPStatus.NOT_FOUND)

        def do_POST(self):
            self.server.last_request = time.monotonic()  # type: ignore[attr-defined]
            if not self._authorised():
                return self._json(HTTPStatus.FORBIDDEN, {'error': 'Invalid configurator token.'})
            try:
                request = self._body()
                if self.path == '/api/inspect':
                    response = inspect_path(str(request.get('path', '')))
                elif self.path == '/api/validate':
                    response = validate_job(request.get('job'))
                elif self.path == '/api/render':
                    response = render_job(request.get('job'))
                elif self.path == '/api/import':
                    response = import_job(str(request.get('yaml', '')))
                else:
                    self.send_error(HTTPStatus.NOT_FOUND)
                    return
                self._json(HTTPStatus.OK, response)
            except ConfiguratorError as exc:
                self._json(HTTPStatus.UNPROCESSABLE_ENTITY, {'issues': [exc.as_dict()]})

    server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    server.token = expected_token  # type: ignore[attr-defined]
    server.last_request = time.monotonic()  # type: ignore[attr-defined]
    return server


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Open the local benchmark configuration UI.')
    parser.add_argument('--bind', default='127.0.0.1', help='Loopback bind address; only 127.0.0.1 is accepted.')
    parser.add_argument('--port', type=int, default=0, help='Loopback TCP port (0 chooses an available port).')
    parser.add_argument('--no-open', action='store_true', help='Print the URL without opening a browser.')
    parser.add_argument('--idle-timeout', type=float, default=0.0, help='Optional idle shutdown in seconds (0 disables it).')
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.bind != '127.0.0.1':
        raise ValueError('Only loopback binding 127.0.0.1 is allowed.')
    if not 0 <= args.port <= 65535:
        raise ValueError('--port must be between 0 and 65535.')
    if args.idle_timeout < 0:
        raise ValueError('--idle-timeout must be zero or positive.')
    server = make_server(port=args.port)
    host, port = server.server_address[:2]
    url = f'http://{host}:{port}/?token={server.token}'  # type: ignore[attr-defined]
    print(url, flush=True)
    if not args.no_open:
        webbrowser.open(url)
    server.timeout = 0.5
    try:
        while True:
            server.handle_request()
            if args.idle_timeout and time.monotonic() - server.last_request >= args.idle_timeout:  # type: ignore[attr-defined]
                break
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except ValueError as exc:
        print(f'error: {exc}', file=sys.stderr)
        raise SystemExit(2)
