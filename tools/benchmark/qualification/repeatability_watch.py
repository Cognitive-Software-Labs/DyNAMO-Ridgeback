#!/usr/bin/env python3
"""Read-only health snapshot for a receiving agent's periodic heartbeat.

This command does not schedule itself, send a notification, launch a run, stop
a process, or mutate campaign evidence. The agent's heartbeat owns wakeups.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import time


def read(path):
    return json.loads(Path(path).read_text())


def inspect(bundle, *, now=None, unit_state=None):
    bundle = Path(bundle)
    now = time.time() if now is None else now
    state = read(bundle / 'campaign.json')
    status = state['status']
    issues = list(state.get('errors', []))
    if 'failed' in status or 'stopped' in status:
        issues.append('Campaign ended with status ' + status)
    for path in sorted(bundle.glob('*_guard.json')):
        if error := read(path).get('error'):
            issues.append(path.name + ': ' + error)
    prepared = status == 'prepared-not-launched'
    finished = status in ('live-complete-offline-pending', 'execution-complete-review-required')
    active = not prepared and not finished and not issues
    if active:
        service_path = bundle / ('service_offline.json' if status == 'processing-offline'
                                 else 'service.json')
        if unit_state is None and service_path.exists():
            unit = read(service_path)['unit']
            result = subprocess.run(
                ['systemctl', '--user', 'show', unit, '--property=ActiveState', '--value'],
                capture_output=True, text=True, timeout=10)
            if result.returncode:
                issues.append('Cannot inspect owned unit: ' + result.stderr.strip())
            else:
                unit_state = result.stdout.strip()
        started = state.get('stage_started_utc') or state.get('started_utc')
        started_at = datetime.fromisoformat(started).timestamp() if started else now
        # Stage starts reset the allowance. Diagnostic log chatter is deliberately
        # not progress: a wedged worker can keep printing counters forever.
        timings = bundle / 'capture_windows.jsonl'
        progress_at = max(started_at, timings.stat().st_mtime if timings.exists() else 0)
        if status != 'processing-offline' and now - progress_at > 300:
            issues.append('No completed capture window for more than 300 seconds')
        if unit_state is not None and unit_state not in ('active', 'activating') and now-started_at > 30:
            issues.append('Owned unit is ' + unit_state + ' while campaign claims to be running')
        if not service_path.exists() and now - started_at > 30:
            issues.append('No owned-service receipt after launch')
        samples = bundle / 'host_samples.jsonl'
        if now - started_at > 60 and (not samples.exists() or now - samples.stat().st_mtime > 60):
            issues.append('Host sampler has no fresh record for more than 60 seconds')
    review = []
    compared = {}
    if (bundle / 'comparisons.json').exists():
        compared = read(bundle / 'comparisons.json')
    for key, result in compared.items():
        if result.get('error'):
            issues.append(key + ': ' + result['error'])
        elif result.get('outcome_or_reason_changes'):
            review.append(key + ': changed outcome/miss reason')
        else:
            # Operational review triggers only, not scientific pass/fail margins.
            unusual = [p for p in result.get('pairs', [])
                       if (p.get('abs_delta_m') or 0) > .02
                       or (p.get('relative_delta') or 0) > .01]
            if unusual:
                review.append(key + f': {len(unusual)} pairs exceed 20 mm or 1% review trigger')
    if finished and (len(compared) != 12 or any(r.get('error') for r in compared.values())):
        issues.append('Completed Isaac collection has missing/unreadable estimator comparisons')
    return {'status': status, 'stage': state.get('current_stage'),
            'checked_utc': datetime.fromtimestamp(now, timezone.utc).isoformat(),
            'attention': bool(issues or review), 'completed': finished,
            'issues': issues, 'review': review,
            'next_action': ('preserve evidence; diagnose before any repair; relaunch only from a fresh bundle'
                            if issues or review else 'report completion once' if finished
                            else 'stay quiet; never start from this heartbeat'),
            'report': str(bundle / 'REPORT.md')}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bundle', type=Path)
    args = parser.parse_args()
    try:
        result = inspect(args.bundle)
    except (OSError, ValueError, KeyError, subprocess.SubprocessError) as error:
        result = {'attention': True, 'issues': [f'{type(error).__name__}: {error}']}
    print(json.dumps(result, indent=2))
    return 1 if result['attention'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
