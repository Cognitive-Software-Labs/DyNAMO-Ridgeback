#!/usr/bin/env python3
"""Assemble complete F2 simulation evidence; refuse missing candidate/scenario rows."""
import argparse
import csv
import hashlib
import json
import math
import time
from pathlib import Path
import numpy as np
from ridgeback_leader_follow.benchmarking.detectors import CAMERA_MODELS
from ridgeback_leader_follow.benchmarking.image_metrics import detection_metrics
from ridgeback_leader_follow.benchmarking.mot import TRACKERS
from ridgeback_leader_follow.benchmarking.scoring import Position,score_frames

LIDAR=('clustering','drow3','dr-spaam','lfe-ppn','petra','leg-detector')
LICENSES={'yolo11':'AGPL-3.0','yolo26':'AGPL-3.0','rtdetrv2':'Apache-2.0','dfine':'Apache-2.0',
          'deim':'Apache-2.0','owlv2':'Apache-2.0','rfdetr':'Apache-2.0','clustering':'BSD-3-Clause',
          'drow3':'GPL-3.0','dr-spaam':'GPL-3.0','lfe-ppn':'MIT','petra':'LGPL-3.0',
          'leg-detector':'BSD source headers; package license undeclared',
          **{k:'AGPL-3.0 (BoxMOT implementation)' for k in TRACKERS},'sam2':'Apache-2.0','samurai':'Apache-2.0'}


def read_lines(path):return [json.loads(line) for line in path.read_text().splitlines()]


def truth_frames(run):
    frames={}
    with (run/'ground_truth.csv').open() as stream:
        for row in csv.DictReader(stream):
            frames.setdefault(float(row['scenario_time']),[]).append(Position(row['person'],float(row['x']),float(row['y']),row['role']))
    return frames


def world_strata(truth,predictions,annotations):
    predicted={f['scenario_time']:[Position(str(p['track_id']),p['x'],p['y']) for p in f['people']] for f in predictions}
    bands={}
    for low,high in ((0,2),(2,4),(4,8)):
        def inside(p):return low <= math.hypot(p.x,p.y) < high
        bands[f'{low}-{high}m']=score_frames([(t,[p for p in people if inside(p)],[p for p in predicted[t] if inside(p)]) for t,people in sorted(truth.items())])
    # Coverage outside the recorded camera's projected/visible silhouettes.
    outside_total=outside_matched=0
    from scipy.optimize import linear_sum_assignment
    for (t,people),annotation in zip(sorted(truth.items()),annotations,strict=True):
        preds=predicted[t]
        outside={p['id'] for p in annotation['people'] if p['box'] is None or p['visible_fraction'] == 0}
        outside_total+=sum(p.identity in outside for p in people)
        if people and preds:
            d=np.array([[math.hypot(g.x-p.x,g.y-p.y) for p in preds] for g in people])
            rows,cols=linear_sum_assignment(np.where(d <= .75,d,(min(d.shape)+1)*2))
            outside_matched+=sum(d[i,j] <= .75 and people[i].identity in outside for i,j in zip(rows,cols))
    return {'distance_bands':bands,'outside_camera':{'matched':int(outside_matched),'total':int(outside_total),
            'coverage':outside_matched/outside_total if outside_total else None}}


def leader_only(truth,predictions,annotations):
    frames=[];covered=[]
    for (t,people),prediction in zip(sorted(truth.items()),predictions,strict=True):
        leader=[p for p in people if p.role == 'leader']
        pred=[Position(str(p['track_id']),p['x'],p['y']) for p in prediction['people']]
        frames.append((t,leader,pred))
        covered.append(bool(leader and any(math.hypot(p.x-leader[0].x,p.y-leader[0].y) <= .75 for p in pred)))
    events=[];occluded=False;recovery_start=None
    for index,annotation in enumerate(annotations):
        person=next(p for p in annotation['people'] if p['role'] == 'leader')
        invisible=person['box'] is None or person['visible_fraction'] < .25
        if invisible:
            if recovery_start is not None:events.append({'visible_again':recovery_start,'recovery_s':None,'recovered':False})
            occluded=True;recovery_start=None
        elif occluded and person['visible_fraction'] >= .5:
            if recovery_start is None:recovery_start=annotation['scenario_time']
            if covered[index]:
                events.append({'visible_again':recovery_start,'recovery_s':annotation['scenario_time']-recovery_start,'recovered':True})
                occluded=False;recovery_start=None
    if recovery_start is not None:events.append({'visible_again':recovery_start,'recovery_s':None,'recovered':False})
    return {**score_frames(frames),'leader_coverage':sum(covered)/len(covered),'recovery_events':events}


def assemble(root,output):
    corpus=json.loads((root/'f2-corpus/corpus.json').read_text())
    if corpus.get('sensor_status') != 'passed' or len(corpus['results']) != 16:raise ValueError('Require all sixteen sensor-qualified runs')
    results=[]
    for case in corpus['results']:
        run=Path(case['run']);truth=truth_frames(run)
        annotations=read_lines(root/'f2-annotations'/(run.name+'.jsonl'))
        if len(annotations) != 400:raise ValueError('Incomplete image annotations')
        for category,names in [('camera',CAMERA_MODELS),('lidar',LIDAR),('mot',TRACKERS),('single',('sam2','samurai'))]:
            for name in names:
                if category in ('camera','lidar'):
                    candidates=[root/collection/run.name/name for collection in ('f2-drow-defaults','f2-lfe-runtime-defaults','f2-owl-defaults','f2-leg-defaults','f2-extra-defaults','f2-defaults')]
                    folders=[p for p in candidates if (p/'score.json').exists()]
                    if not folders:raise ValueError(f'Missing {category}: {run.name}/{name}')
                    folder=folders[0]
                else:folder=root/('f2-mot-defaults' if category == 'mot' else 'f2-single-defaults')/run.name/name
                score=json.loads((folder/'score.json').read_text())
                receipt=json.loads((folder/('metrics.json' if category == 'mot' else 'receipt.json')).read_text())
                predictions=read_lines(folder/'predictions.jsonl')
                if score['prediction_frames'] != 400:raise ValueError('Incomplete candidate predictions')
                if receipt.get('missing_input_frames',0):raise ValueError('Candidate used incomplete sensor inputs')
                row={'case':run.name,'backend':case['backend'],'scenario':case['scenario'],'crowd_size':case['crowd_size'],
                     'category':category,'candidate':name,'license':LICENSES[name],'score':score,'receipt':receipt,'evidence':str(folder)}
                if category == 'camera':row['image_detection']=detection_metrics(annotations,read_lines(folder/'detections.jsonl'))
                if category == 'lidar':row['strata']=world_strata(truth,predictions,annotations)
                if category == 'single':row['leader']=leader_only(truth,predictions,annotations)
                results.append(row)
    summaries=[]
    for category,names in [('camera',CAMERA_MODELS),('lidar',LIDAR),('mot',TRACKERS),('single',('sam2','samurai'))]:
        for name in names:
            rows=[r for r in results if r['category'] == category and r['candidate'] == name]
            counts={k:sum(r['score'][k] for r in rows) for k in ('true_positives','false_positives','false_negatives','id_switches','leader_track_loss_events')}
            tp,fp,fn=[counts[k] for k in ('true_positives','false_positives','false_negatives')]
            summary={'category':category,'candidate':name,'cases':len(rows),'license':LICENSES[name],**counts,
                'position_precision':tp/(tp+fp) if tp+fp else None,'position_recall':tp/(tp+fn) if tp+fn else None,
                'median_case_processing_ms':float(np.median([r['receipt']['processing_median_ms'] for r in rows])),
                'worst_case_p95_ms':max(r['receipt']['processing_p95_ms'] for r in rows),
                'gpu_peak_allocated_bytes':max((r['receipt'].get('gpu_peak_allocated_bytes',0) for r in rows),default=0)}
            if category == 'lidar' and name in ('clustering','lfe-ppn','petra','leg-detector'):summary['gpu_peak_allocated_bytes']=0
            if category == 'camera':
                totals={k:sum(r['image_detection'][k] for r in rows) for k in ('TP','FP','FN')}
                summary['image_precision']=totals['TP']/(totals['TP']+totals['FP']) if totals['TP']+totals['FP'] else None
                summary['image_recall']=totals['TP']/(totals['TP']+totals['FN'])
            if category == 'mot':summary.update({k:float(np.mean([r['receipt'][k] for r in rows])) for k in ('HOTA','IDF1')})
            if category == 'single':summary['leader_coverage']=float(np.mean([r['leader']['leader_coverage'] for r in rows]))
            summaries.append(summary)
    payload={'status':'complete','scope':'F2 simulation candidate evidence; hardware performance deferred; no F3/F4 claims',
             'cases':16,'candidate_runs':len(results),'summaries':summaries,'results':results,
             'timing_scope':'concurrent workstation runs; medians of per-case medians and worst per-case p95; not robot/Thor certification',
             'annotation_scope':'mesh-projected clipped boxes, sampled gait, depth-supported visible fraction estimate',
             'sources_sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in Path(__file__).resolve().parent.glob('*.py')}}
    with output.open('x') as stream:json.dump(payload,stream,indent=2,allow_nan=False);stream.write('\n')
    return payload


def missing_inputs(root):
    corpus=json.loads((root/'f2-corpus/corpus.json').read_text())
    missing=[]
    for case in corpus['results']:
        run=Path(case['run']).name
        for category,names in [('camera',CAMERA_MODELS),('lidar',LIDAR),('mot',TRACKERS),('single',('sam2','samurai'))]:
            for name in names:
                if category in ('camera','lidar'):
                    collections=('f2-drow-defaults',) if name in ('drow3','dr-spaam') else ('f2-lfe-runtime-defaults',) if name == 'lfe-ppn' else ('f2-owl-defaults',) if name == 'owlv2' else ('f2-leg-defaults',) if name == 'leg-detector' else ('f2-extra-defaults','f2-defaults')
                else:collections=('f2-mot-defaults',) if category == 'mot' else ('f2-single-defaults',)
                if not any((root/c/run/name/'score.json').exists() for c in collections):missing.append(run+'/'+name)
    return missing


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--wait',action='store_true')
    a=p.parse_args()
    deadline=time.monotonic()+7200
    if a.wait:
        while missing_inputs(a.root):
            if time.monotonic() > deadline:raise TimeoutError('Candidate matrix incomplete')
            time.sleep(3)
    result=assemble(a.root,a.output);print(json.dumps(result['summaries'],indent=2))


if __name__ == '__main__':main()
