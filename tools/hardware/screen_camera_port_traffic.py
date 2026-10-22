#!/usr/bin/env python3
"""Compare physical-port traffic before/during guarded local camera readers.

No commands, service changes or network rules. Root is needed only for short
header packet captures; physical e-stop/stationary evidence is checked first.
The camera worker independently guards scan continuity throughout its trial.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
PORTS = ('eno1', 'enp2s0', 'wlp3s0')


def counters():
    return {name: {direction: int(Path('/sys/class/net', name, 'statistics',
                                      direction + '_bytes').read_text())
                   for direction in ('rx', 'tx')} for name in PORTS}


def rates(before, after, elapsed):
    if elapsed <= 0:
        raise ValueError('positive measurement interval required')
    return {name: {direction + '_mbit_s': (after[name][direction] - before[name][direction])
                   * 8 / elapsed / 1e6 for direction in ('rx', 'tx')} for name in PORTS}


def captures(root, phase):
    workers = []
    try:
        for name in PORTS:
            stderr = (root / (phase + '-' + name + '.stderr')).open('w')
            try:
                worker = subprocess.Popen([shutil.which('tcpdump'), '-p', '-i', name,
                                           '-nn', '-s', '96', '-U', '-w',
                                           str(root / (phase + '-' + name + '.pcap'))],
                                          stdout=subprocess.DEVNULL, stderr=stderr)
            finally:
                stderr.close()
            workers.append(worker)
        time.sleep(.5)
        if any(p.poll() is not None for p in workers):
            raise RuntimeError('packet capture could not start; inspect capture stderr')
        return workers
    except BaseException:
        stop_captures(workers)
        raise


def stop_captures(workers):
    for worker in workers:
        if worker.poll() is None:
            worker.send_signal(signal.SIGINT)
    for worker in workers:
        try:
            worker.wait(timeout=3)
        except subprocess.TimeoutExpired:
            worker.kill()
            worker.wait(timeout=3)


def excess_wired_tx(current, baseline):
    return [name for name in ('eno1', 'enp2s0')
            if current[name]['tx_mbit_s'] - baseline[name]['tx_mbit_s'] > 20]


def camera_trial(root, baseline):
    command = ['/usr/bin/python3', str(ROOT / 'tools/camera_contract_check'),
               '--backend', 'hardware', '--guard', '--warmup', '3',
               '--duration', '12', '--output', str(root / 'camera.json')]
    samples, reason, high = [], None, dict.fromkeys(('eno1', 'enp2s0'), 0)
    with (root / 'camera.stdout').open('w') as stdout, (root / 'camera.stderr').open('w') as stderr:
        worker = subprocess.Popen(command, stdout=stdout, stderr=stderr)
        started = interval_start = time.monotonic()
        before = counters()
        try:
            while worker.poll() is None:
                time.sleep(.1)
                now = time.monotonic()
                if now - interval_start >= 1:
                    after = counters()
                    current = rates(before, after, now - interval_start)
                    samples.append({'t_s': now - started, 'ports': current})
                    excessive = excess_wired_tx(current, baseline)
                    for name in high:
                        high[name] = high[name] + 1 if name in excessive else 0
                    if any(count >= 2 for count in high.values()):
                        reason = 'incremental wired TX above 20 Mbit/s for two consecutive samples'
                        break
                    before, interval_start = after, now
                if now - started > 30:
                    reason = 'camera checker exceeded 30 seconds'
                    break
        finally:
            if worker.poll() is None:
                worker.terminate()
                try:
                    worker.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    worker.kill()
                    worker.wait(timeout=3)
        return worker.returncode, reason, samples


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--operator-confirmed', action='store_true')
    args = p.parse_args()
    if os.geteuid() != 0 or not args.operator_confirmed:
        p.error('sudo and operator confirmation required; keep e-stop pressed')
    if not shutil.which('tcpdump'):
        p.error('tcpdump must already be installed')
    if importlib.util.find_spec('rclpy') is None:
        if os.environ.get('DYNAMO_TRAFFIC_ROS_SOURCED'):
            p.error('ROS Jazzy Python unavailable')
        os.environ['DYNAMO_TRAFFIC_ROS_SOURCED'] = '1'
        os.execv('/bin/bash', ['/bin/bash', '-c',
                             'source /opt/ros/jazzy/setup.bash && exec /usr/bin/python3 "$@"',
                             'camera-traffic-screen', str(Path(__file__).resolve()), *sys.argv[1:]])
    os.environ.update(ROS_DOMAIN_ID='0', RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
                      CYCLONEDDS_URI='file:///etc/clearpath/cyclonedds.xml', ROS_LOG_DIR='/tmp')
    from install_commissioning_caps import identity, run_probe
    root = ROOT / 'artifacts/hardware' / ('camera-port-screen-' +
        datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
    root.mkdir(parents=True)
    result = {'status': 'not-started', 'source_sha256': hashlib.sha256(
        Path(__file__).read_bytes()).hexdigest(),
        'limits': 'Short port-level comparison; packet destinations must explain traffic. No long soak or causal LiDAR-fault claim.'}
    workers = []
    try:
        print('Checking pressed stop and stationary routing. No joystick needed.', flush=True)
        result['preflight'] = run_probe(identity(), root, 'before')
        if not result['preflight']['ok']:
            raise RuntimeError('pressed-stop/stationary preflight failed')
        print('Recording five-second quiet traffic baseline.', flush=True)
        workers = captures(root, 'baseline')
        before, started = counters(), time.monotonic()
        time.sleep(5)
        result['baseline'] = rates(before, counters(), time.monotonic() - started)
        stop_captures(workers)
        workers = []
        print('Recording guarded camera trial and packet headers.', flush=True)
        workers = captures(root, 'camera')
        before, started = counters(), time.monotonic()
        code, stop_reason, samples = camera_trial(root, result['baseline'])
        result['camera_trial_ports'] = rates(before, counters(), time.monotonic() - started)
        stop_captures(workers)
        workers = []
        result.update(camera_returncode=code, reader_stop_reason=stop_reason,
                      one_second_network_samples=samples)
        result['incremental_tx_mbit_s'] = {name: result['camera_trial_ports'][name]['tx_mbit_s']
            - result['baseline'][name]['tx_mbit_s'] for name in PORTS}
        result['status'] = ('traffic-captured-camera-passed' if code == 0 and stop_reason is None
                            else 'traffic-captured-camera-failed')
        result['postflight'] = run_probe(identity(), root, 'after')
        if not result['postflight']['ok']:
            result['status'] = 'postflight-failed'
    except BaseException as exc:
        result.update(status='screen-failed', error=str(exc) or type(exc).__name__)
    finally:
        stop_captures(workers)
        result['capture_sha256'] = {f.name: hashlib.sha256(f.read_bytes()).hexdigest()
                                    for f in root.glob('*.pcap')}
        (root / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'status': result['status'], 'report': str(root / 'result.json'),
                      'incremental_tx_mbit_s': result.get('incremental_tx_mbit_s'),
                      'reader_stop_reason': result.get('reader_stop_reason'),
                      'error': result.get('error')}, indent=2), flush=True)
    return 0 if result['status'] == 'traffic-captured-camera-passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
