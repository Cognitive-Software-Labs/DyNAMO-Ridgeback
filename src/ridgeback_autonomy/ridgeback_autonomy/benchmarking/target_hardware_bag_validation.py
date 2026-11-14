#!/usr/bin/env python3
"""Plan, run, resume or inspect the unscored saved hardware session."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    plan = commands.add_parser('describe', help='metadata and local snapshot validation only; no raw MCAP read/inference')
    plan.add_argument('--session', required=True)
    plan.add_argument('--output', required=True, help='new diagnostic output under artifacts/benchmarks')
    plan.add_argument('--job', required=True, help='write immutable job JSON')
    plan.add_argument('--model-cache', default=os.environ.get('HF_HUB_CACHE', '~/.cache/huggingface/hub'))
    plan.add_argument('--device', default='cuda:0', help='explicit cpu or cuda:N; float32, no fallback')
    plan.add_argument('--base-frame', default='base_link', help='recorded TF identity, never inferred from namespace')
    for command in ('run', 'resume', 'check-models'):
        sub = commands.add_parser(command)
        sub.add_argument('--job', required=True)
    cached = commands.add_parser('remeasure', help='new unscored derivative from completed caches; no inference/raw reads')
    cached.add_argument('--evidence', required=True)
    cached.add_argument('--output', required=True)
    cached.add_argument('--references', help='JSON mapping exact bag names to qualified reference documents')
    report = commands.add_parser('report', help='validate committed evidence and regenerate report, without inference')
    report.add_argument('--output', required=True)
    args = parser.parse_args(argv)
    try:
        from ridgeback_autonomy.benchmarking.hardware_bag_validation import (
            ProductionProducers, atomic_json, describe, output_lock, read_json, run_job,
            validate_completed, validate_job, write_report, remeasure_cached,
        )
        from ridgeback_autonomy.benchmarking.paths import anchored_path
        from ridgeback_autonomy.benchmarking.target_offline_replay_benchmark import _source_repository
        # Workspace helper differs across installed/source layouts; paths are
        # canonicalized once here and absolute in the saved job.
        workspace = str(_source_repository())
        if args.command == 'describe':
            destination = Path(anchored_path(workspace, args.job))
            if destination.exists():
                raise ValueError(f'Job already exists: {destination}; choose a new job path')
            output = anchored_path(workspace, args.output)
            if Path(output).exists() and any(Path(output).iterdir()):
                raise ValueError('Describe requires a new/empty output directory')
            job = describe(anchored_path(workspace, args.session), output,
                           anchored_path(workspace, args.model_cache), device=args.device, base_frame=args.base_frame)
            atomic_json(destination, job)
            print(json.dumps({'job': str(destination), 'identity': job['identity'],
                              'bags': [s['bag'] for s in job['selection']['bags']],
                              'paths': job['paths'], 'settings': job['settings'],
                              'output': job['output']}, indent=2))
        elif args.command == 'remeasure':
            output = anchored_path(workspace, args.output)
            remeasure_cached(anchored_path(workspace, args.evidence), output,
                             references=anchored_path(workspace, args.references) if args.references else None)
            print(Path(output) / 'report.md')
        elif args.command == 'report':
            output = Path(anchored_path(workspace, args.output))
            with output_lock(output):
                job = read_json(output / 'job.json')
                progress = read_json(output / 'progress.json')
                for name, state in progress['bags'].items():
                    if state['state'] == 'complete':
                        validate_completed(output / name, job['identity'])
                write_report(output, job, progress)
            print(output / 'report.md')
        else:
            job = read_json(anchored_path(workspace, args.job))
            if args.command == 'check-models':
                validate_job(job)
                ProductionProducers(job)
                print('All three local models loaded on frozen device/dtype; no inference performed.')
            else:
                result = run_job(job, resume=args.command == 'resume')
                failed = [name for name, state in result['bags'].items() if state['state'] != 'complete']
                print(json.dumps({'report': str(Path(job['output'])/'report.md'), 'failed_inputs': failed}))
                return 2 if failed else 0
    except Exception as exc:
        print(f'hardware-bag validation: {type(exc).__name__}: {exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
