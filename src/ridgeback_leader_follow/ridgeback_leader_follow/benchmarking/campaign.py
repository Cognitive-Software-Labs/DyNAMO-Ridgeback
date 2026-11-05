#!/usr/bin/env python3
"""Resume annotations and auxiliary candidate comparisons on a completed corpus."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from ridgeback_leader_follow.benchmarking.dataset import SensorBag
from ridgeback_leader_follow.benchmarking.sweep import run_candidate


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,required=True)
    p.add_argument('--stage',choices=('extra-detectors','mot','single'),required=True)
    a=p.parse_args();root=a.root
    deadline=time.monotonic()+3600
    while json.loads((root/'f2-corpus/corpus.json').read_text())['status'] != 'complete':
        if time.monotonic() > deadline: raise TimeoutError('Corpus did not complete')
        time.sleep(2)
    cases=json.loads((root/'f2-corpus/corpus.json').read_text())['results']
    for case in cases:
        run=Path(case['run']);annotations=root/'f2-annotations'/(run.name+'.jsonl')
        print(a.stage,run.name,flush=True)
        if a.stage in ('mot','single') and not annotations.exists():
            from ridgeback_leader_follow.benchmarking.annotations import annotate
            try:annotate(SensorBag(run),annotations,[0,0,0,0])
            except FileExistsError:pass
        if a.stage == 'extra-detectors':
            bag=SensorBag(run)
            for name,family in [('deim','camera'),('petra','lidar'),('leg-detector','lidar')]:
                output=root/'f2-extra-defaults'/run.name/name
                if (output/'score.json').exists():continue
                run_candidate(bag,name,family,output,root/'f2-models',[0,0,0,0],'cuda')
        elif a.stage == 'mot':
            output=root/'f2-mot-defaults'/run.name
            detections=root/'f2-defaults'/run.name/'yolo11/detections.jsonl'
            deadline=time.monotonic()+3600
            while not (detections.parent/'score.json').exists():
                if time.monotonic() > deadline:raise TimeoutError('Detector results not ready')
                time.sleep(2)
            command=[str(root/'f2-venv/bin/python'),'-m','ridgeback_leader_follow.benchmarking.mot',
                '--run',str(run),'--annotations',str(annotations),'--detections',str(detections),'--output',str(output)]
            subprocess.run(command,check=True)
        else:
            # Explicit simulator evaluation initialization: one leader-box click
            # from frame zero only; no subsequent annotations enter propagation.
            row=json.loads(annotations.read_text().splitlines()[0])
            leader=next(v for v in row['people'] if v['role'] == 'leader')
            if leader['box'] is None:raise ValueError('Leader not visible for initial click')
            x1,y1,x2,y2=leader['box'];click=[(x1+x2)/2,y1+.35*(y2-y1)]
            for mode in ('sam2','samurai'):
                output=root/'f2-single-defaults'/run.name/mode
                if (output/'score.json').exists():continue
                subprocess.run([sys.executable,'-m','ridgeback_leader_follow.benchmarking.single_target',
                    '--run',str(run),'--output',str(output),'--checkpoint',str(root/'f2-models/sam2.1_hiera_tiny.pt'),
                    '--mode',mode,'--initial-click',*map(str,click)],check=True)


if __name__ == '__main__':main()
