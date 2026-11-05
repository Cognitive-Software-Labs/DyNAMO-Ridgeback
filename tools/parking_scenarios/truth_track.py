"""Gazebo ground truth of the robot at about 1 Hz, for validation only (never fed back)."""
import os, re, subprocess, sys, time
out = sys.argv[1]
env = dict(os.environ, GZ_PARTITION=f'ridgeback-parking-{os.getuid()}-83', GZ_IP='127.0.0.1')
num = r'(-?[\d.]+(?:e-?\d+)?)'
with open(f'{out}/truth_track.csv', 'a') as f:
    while True:
        t = time.time()
        rows = []
        for model in ('r100_0001/robot', 'bench_parking_demo_g1'):
            try:
                txt = subprocess.run(['gz', 'model', '-m', model, '--pose'], env=env, capture_output=True,
                                     text=True, timeout=5).stdout
                v = re.findall(r'\[\s*' + r'\s+'.join([num] * 3) + r'\s*\]', txt)
                rows.append(f'{v[0][0]},{v[0][1]},{v[1][2]}' if len(v) >= 2 else ',,')
            except Exception:
                rows.append(',,')
        f.write(f'{t:.2f},' + ','.join(rows) + '\n'); f.flush()
        time.sleep(max(0.0, 1.0 - (time.time() - t)))
