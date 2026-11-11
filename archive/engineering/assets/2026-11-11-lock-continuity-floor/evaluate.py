"""Replay all 18 F3 cases through LeaderIdentity and score them with ros_identity_score.

usage: evaluate.py LABEL [variant-module]   (variant module may define LOCK_KWARGS / IDENTITY_CLS)
"""
import importlib.util, json, os, sys
from pathlib import Path
sys.path.insert(0, os.path.dirname(__file__))
from replay import replay
from ridgeback_leader_follow.benchmarking.ros_identity_score import score

ROOT = Path('artifacts/leader-following'); SRC = ROOT / 'f3-live-validation-qualified-profile'
label = sys.argv[1]; kw = {}; cls = None
if len(sys.argv) > 2:
    spec = importlib.util.spec_from_file_location('variant', sys.argv[2]); v = importlib.util.module_from_spec(spec); spec.loader.exec_module(v)
    kw = getattr(v, 'LOCK_KWARGS', {}); cls = getattr(v, 'IDENTITY_CLS', None)
cases = []
for prefix in ['f3-appearance', 'f3-camera-exit']:
    ev = ROOT / (prefix + '-evaluation')
    for c in json.loads((ev / 'corpus.json').read_text())['results']:
        run = Path(c['run']); cases.append((run, ev / 'annotations-stationary-anchor' / (run.name + '.jsonl')))
out = Path(os.path.dirname(__file__)) / label
res = {}
for run, ann in sorted(cases, key=lambda p: p[0].name):
    d = out / run.name
    replay(SRC / run.name, d, kw, **({'identity_cls': cls} if cls else {}))
    (d / 'people.jsonl').unlink(missing_ok=True); (d / 'people.jsonl').symlink_to((SRC / run.name / 'people.jsonl').resolve())
    score(run, d, d / 'identity_score.json', ann)
    res[run.name] = json.loads((d / 'identity_score.json').read_text())
tot_c = tot_w = 0
for k, v in res.items():
    tot_c += v['correct_locked_frames']; tot_w += v['wrong_locked_frames']
    oc = v['outside_camera_lock_fraction']
    print(f"{k:28s} cov={v['correct_lock_fraction']*100:5.1f}% wrong={v['wrong_locked_frames']:3d} out={'-' if oc is None else f'{oc:.2f}'} unrec={v['unrecovered_loss']} {v['status']}")
print(f"TOTAL correct={tot_c} wrong={tot_w} min_cov={min(v['correct_lock_fraction'] for v in res.values())*100:.1f}% mean_cov={sum(v['correct_lock_fraction'] for v in res.values())/len(res)*100:.1f}%")
json.dump(res, open(out / 'summary.json', 'w'), indent=1)
