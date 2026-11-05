"""Resumable F3 appearance-corpus evaluation; each model runs in isolation."""
import argparse
import gc
import json
from pathlib import Path
import subprocess
import sys

from .reid import CANDIDATES


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--corpus',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--model-dir',type=Path,default=Path('artifacts/leader-following/f3-models'))
    parser.add_argument('--detector-model-dir',type=Path,default=Path('artifacts/leader-following/f2-models'))
    parser.add_argument('--device',default='cuda')
    parser.add_argument('--plan',action='store_true')
    parser.add_argument('--supplementary',action='store_true')
    args=parser.parse_args()
    qualification=json.loads((args.corpus/'qualification.json').read_text())
    cases=qualification.get('results',[])
    if args.plan:
        print(json.dumps({'qualified_cases':len(cases),'recording_status':qualification['status'],
                          'candidates':CANDIDATES,'stages':['mesh annotations','OWLv2/LFE-PPN detections',
                          'five-model recognition comparison','integrated lock replays']},indent=2));return
    expected=2 if args.supplementary else 16
    if qualification.get('status') != 'passed' or len(cases) != expected:
        raise ValueError(f'Require all {expected} qualified F3 recordings before evaluation')
    if args.supplementary and (any(case['scenario'] != 'leader_out_of_view' for case in cases)
                              or {case['backend'] for case in cases} != {'gz','isaac'}):
        raise ValueError('Supplementary corpus must contain the camera-exit case in both simulators')
    if any(case.get('scene_profile') != 'f3' for case in cases):
        raise ValueError('Refuse to mix baseline and F3 appearance cases')
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'corpus.json').write_text(json.dumps(qualification,indent=2)+'\n')
    from .dataset import SensorBag
    from .annotations import annotate
    from .sweep import run_candidate
    from .reid_report import summarize
    for case in cases:
        run=Path(case['run']);key=run.name
        bag=None
        for family,name in [('camera','owlv2'),('lidar','lfe-ppn')]:
            destination=args.output/'detections'/key/name
            if (destination/'receipt.json').exists() and (destination/'score.json').exists():continue
            if destination.exists():
                raise ValueError(f'Inspect incomplete detection output before resuming: {destination}')
            if bag is None:bag=SensorBag(run)
            print('Detect',key,name,flush=True)
            run_candidate(bag,name,family,destination,args.detector_model_dir,(0.,0.,0.,0.),args.device)
        annotation=args.output/'annotations'/(key+'.jsonl')
        if not annotation.exists():
            if bag is None:bag=SensorBag(run)
            print('Annotate',key,flush=True)
            method=annotate(bag,annotation,(0.,0.,0.,0.),stationary_base=True)
            annotation.with_suffix('.json').write_text(json.dumps(method,indent=2)+'\n')
        del bag;gc.collect()
    for candidate in CANDIDATES:
        for case in cases:
            run=Path(case['run']);key=run.name
            destination=args.output/'comparison'/candidate/key
            if (destination/'score.json').exists():continue
            if destination.exists():
                raise ValueError(f'Inspect incomplete recognition output before resuming: {destination}')
            destination.parent.mkdir(parents=True,exist_ok=True)
            command=[sys.executable,'-m','ridgeback_leader_follow.benchmarking.reid_benchmark',
                     '--run',str(run),'--detections',str(args.output/'detections'/key/'owlv2/detections.jsonl'),
                     '--annotations',str(args.output/'annotations'/(key+'.jsonl')),
                     '--output',str(destination),'--model-dir',str(args.model_dir),
                     '--candidate',candidate,'--device',args.device]
            print('Recognize',candidate,key,flush=True)
            with destination.with_suffix('.log').open('w') as log:
                subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
    summary=summarize(args.output/'comparison',args.output/'corpus.json',args.output/'comparison.json')
    # Initial defaults are evaluated before any candidate-specific tuning.
    for candidate in summary['ranking_by_nearby_accuracy'][:2]:
        for case in cases:
            run=Path(case['run']);key=run.name
            destination=args.output/'lock'/candidate/key
            if (destination/'score.json').exists():continue
            if destination.exists():
                raise ValueError(f'Inspect incomplete lock output before resuming: {destination}')
            destination.parent.mkdir(parents=True,exist_ok=True)
            command=[sys.executable,'-m','ridgeback_leader_follow.benchmarking.reid_lock_replay',
                     '--run',str(run),'--camera',str(args.output/'detections'/key/'owlv2/detections.jsonl'),
                     '--lidar',str(args.output/'detections'/key/'lfe-ppn/detections.jsonl'),
                     '--embeddings',str(args.output/'comparison'/candidate/key/'embeddings.npz'),
                     '--output',str(destination),'--annotations',str(args.output/'annotations'/(key+'.jsonl'))]
            print('Lock replay',candidate,key,flush=True)
            with destination.with_suffix('.log').open('w') as log:
                subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
    status={'status':'evaluation_complete','f3_qualified':False,
            'reason':'review per-case correct locks, recovery and appearance-free continuity before acceptance'}
    (args.output/'status.json').write_text(json.dumps(status,indent=2)+'\n')


if __name__ == '__main__':main()
