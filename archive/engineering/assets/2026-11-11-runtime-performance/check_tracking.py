from pathlib import Path
import json
from ridgeback_leader_follow.benchmarking.dataset import SensorBag
from ridgeback_leader_follow.benchmarking.sweep import run_candidate
root=Path(__file__).resolve().parent
runs=['gz-reference-r3','gz-balanced-r2','isaac-reference-r3','isaac-balanced-r3']
results={}
for name in runs:
    bag=SensorBag(root/name)
    out=root/'tracking'/name/'yolo11'
    receipt=run_candidate(bag,'yolo11','camera',out,Path('artifacts/leader-following/f2-models').resolve(),[0.,0.,0.,0.],'cuda')
    results[name]={'receipt':receipt,'score':json.loads((out/'score.json').read_text())}
    print(name,results[name]['score'],flush=True)
(root/'tracking-comparison.json').write_text(json.dumps(results,indent=2)+'\n')
