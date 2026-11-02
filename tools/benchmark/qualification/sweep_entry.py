"""Adapter around the existing sweep owner: strict checks between configurations."""
import os
from pathlib import Path
import sys
import time

from campaign import atomic_json, check_frozen, read_json
from evidence import processing_diagnostics, validate_run


def main():
    from ridgeback_autonomy.benchmarking import target_benchmark_sweep as sweep
    bundle = Path(os.environ['QUALIFICATION_BUNDLE'])
    stage = os.environ['QUALIFICATION_STAGE']
    end = float(os.environ['QUALIFICATION_STOP_AT'])
    admission = float(os.environ['QUALIFICATION_ADMISSION_SEC'])
    expected_trials = 1 if stage.startswith('check_') else 109
    checked = set()
    guard = {'stage': stage, 'checked': {}, 'error': None}
    write_original = sweep._write_manifest
    command_original = sweep._config_command
    wait_original = sweep._wait_for_config

    def write(sweep_dir, manifest):
        write_original(sweep_dir, manifest)
        for entry in manifest['configs']:
            name, status = entry['name'], entry['status']
            if name in checked or status not in ('success', 'skipped', 'failed', 'timeout'):
                continue
            checked.add(name)
            try:
                if status != 'success':
                    raise RuntimeError(f'{name}: {status}; stop rather than continue or retry')
                check_frozen(bundle)
                renderer = manifest.get('gl') or {}
                if renderer.get('software') is not False or not renderer.get('renderer'):
                    raise RuntimeError('Missing hardware-renderer evidence or software rendering')
                result = validate_run(Path(sweep_dir)/name, entry, expected_trials,
                                      bundle/'capture_windows.jsonl')
                result['processing_stages'] = processing_diagnostics(bundle/f'{stage}.log', name)
                required = []
                if 'silhouette' in name:
                    required.append('slimsam')
                if 'monocular' in name:
                    required.append('depth_anything')
                if any(result['processing_stages'].get(key, {}).get('warm_n', 0) < 3 for key in required):
                    raise RuntimeError('Requested models did not produce enough warm inference samples')
                guard['checked'][name] = result
            except Exception as error:
                guard['error'] = f'{type(error).__name__}: {error}'
                atomic_json(bundle/f'{stage}_guard.json', guard)
                raise RuntimeError(guard['error']) from error
            atomic_json(bundle/f'{stage}_guard.json', guard)

    def command(arguments):
        check_frozen(bundle)
        if time.monotonic()+admission >= end:
            raise RuntimeError('Insufficient remaining budget to admit another configuration')
        return command_original(arguments)

    def wait(process, environment, timeout_sec):
        return wait_original(process, environment, min(timeout_sec, 2400, max(0, end-time.monotonic())))

    sweep._write_manifest = write
    sweep._config_command = command
    sweep._wait_for_config = wait
    return sweep.main()


if __name__ == '__main__':
    raise SystemExit(main())
