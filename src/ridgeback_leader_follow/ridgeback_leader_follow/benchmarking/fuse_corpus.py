#!/usr/bin/env python3
"""Score the selected sensor combination in every qualified simulation case."""
import argparse
import json
import time
from pathlib import Path
from ridgeback_leader_follow.benchmarking.dataset import SensorBag
from ridgeback_leader_follow.benchmarking.fused import replay_fusion


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,required=True);p.add_argument('--camera',required=True);p.add_argument('--lidar',required=True);p.add_argument('--suffix',default='')
    a=p.parse_args();corpus=json.loads((a.root/'f2-corpus/corpus.json').read_text())
    if corpus.get('sensor_status') != 'passed':raise ValueError('Require sensor-qualified corpus')
    scores=[]
    for case in corpus['results']:
        run=Path(case['run']);paths=[]
        for name in (a.camera,a.lidar):
            deadline=time.monotonic()+7200
            collections=('f2-lfe-runtime-defaults',) if name == 'lfe-ppn' else ('f2-owl-defaults',) if name == 'owlv2' else ('f2-extra-defaults','f2-defaults')
            while True:
                matches=[a.root/collection/run.name/name for collection in collections
                         if (a.root/collection/run.name/name/'score.json').exists()]
                if matches:paths.append(matches[0]/'detections.jsonl');break
                if time.monotonic() > deadline:raise TimeoutError(f'No candidate output: {run.name}/{name}')
                time.sleep(2)
        output=a.root/f'f2-fused-{a.camera}-{a.lidar}{a.suffix}'/run.name
        if not (output/'score.json').exists():
            print('Fusion',run.name,flush=True);replay_fusion(SensorBag(run),*paths,output)
        scores.append({'run':run.name,**json.loads((output/'score.json').read_text())})
    (a.root/f'f2-fused-{a.camera}-{a.lidar}{a.suffix}'/'summary.json').write_text(json.dumps({'status':'complete','results':scores},indent=2)+'\n')


if __name__ == '__main__':main()
