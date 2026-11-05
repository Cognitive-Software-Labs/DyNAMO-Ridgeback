"""Calibrate only selected finalists on Gazebo; report Isaac separately."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.optimize import linear_sum_assignment
from ridgeback_leader_follow.appearance import match_gallery
from ridgeback_leader_follow.fusion import box_measurement
from .dataset import SensorBag
from .image_metrics import box_iou


def observations(run, detections, annotations, embeddings):
    bag=SensorBag(run)
    rows=[json.loads(line) for line in detections.read_text().splitlines()]
    truth=[json.loads(line) for line in annotations.read_text().splitlines()]
    with np.load(embeddings) as cache:
        features={}
        for f,d,e in zip(cache['frames'],cache['detections'],cache['features'],strict=True):
            features.setdefault(int(f),{})[int(d)]=e
    gallery=[];records=[]
    for index,(row,annotation) in enumerate(zip(rows,truth,strict=True)):
        if abs(row['scenario_time']-annotation['scenario_time']) > 1e-6:
            raise ValueError('Calibration timestamps differ')
        current=features.get(index,{})
        boxes=row['boxes'] if row['input_error'] is None else []
        if not gallery and boxes:
            _,_,depth,k,matrix=bag.camera(row['sim_time'])
            eligible=[]
            for d,box in enumerate(boxes):
                m=box_measurement(box[:4],box[4],depth,k,matrix)
                if m is not None and d in current and m.x > 0 and np.hypot(m.x,m.y) <= 8 and abs(np.arctan2(m.y,m.x)) <= np.pi/6:
                    eligible.append((np.hypot(m.x,m.y),d))
            if eligible:
                gallery=[current[min(eligible)[1]]]
        match=match_gallery(gallery,current,threshold=-1,minimum_margin=0)
        people=annotation['people'];labels={}
        boxed_people=[p for p in people if p['box'] is not None]
        if boxed_people and boxes:
            ious=box_iou([p['box'] for p in boxed_people],[b[:4] for b in boxes])
            rr,cc=linear_sum_assignment(-ious)
            labels={int(c):boxed_people[r]['role'] for r,c in zip(rr,cc) if ious[r,c] >= .5}
        leader=next((p for p in people if p['role']=='leader'),None)
        visible=leader is not None and leader.get('visible_fraction',0) >= .2
        records.append({'eligible':visible,'selected':match.track_id,'score':match.similarity,
                        'margin':match.margin,'correct':labels.get(match.track_id)=='leader'})
    return records


def counts(records,threshold,margin):
    result={'eligible':0,'correct':0,'wrong':0,'rejected':0,'absent_false_accepts':0}
    for row in records:
        accepted=(row['selected'] is not None and row['score'] >= threshold
                  and (row['margin'] is None or row['margin'] >= margin))
        if row['eligible']:
            result['eligible']+=1
            result['correct' if accepted and row['correct'] else 'wrong' if accepted else 'rejected']+=1
        elif accepted:
            result['absent_false_accepts']+=1
    result['accuracy']=result['correct']/result['eligible'] if result['eligible'] else None
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--candidates',nargs=2,required=True)
    p.add_argument('--root',type=Path,default=Path('artifacts/leader-following'))
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--corpus',type=Path)
    p.add_argument('--comparison',type=Path)
    p.add_argument('--detections',type=Path)
    p.add_argument('--annotations',type=Path)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    corpus=a.corpus or a.root/'f2-corpus/corpus.json'
    comparison=a.comparison or a.root/'f3-comparison'
    detections=a.detections or a.root/'f2-owl-defaults'
    annotations=a.annotations or a.root/'f2-annotations'
    cases=json.loads(corpus.read_text())['results']
    results=[]
    for candidate in a.candidates:
        train=[];test=[]
        for case in cases:
            run=Path(case['run']);key=run.name
            records=observations(run,detections/key/'owlv2/detections.jsonl',
                                 annotations/(key+'.jsonl'),
                                 comparison/candidate/key/'embeddings.npz')
            (train if case['backend']=='gz' else test).extend(records)
        rows=[]
        for threshold in (.65,.72,.8,.85,.9,.95):
            for margin in (.02,.05,.1,.15):
                rows.append({'threshold':threshold,'margin':margin,'train':counts(train,threshold,margin)})
        # Wrong identity is costlier than a rejection. Declare this conservative
        # objective explicitly rather than claiming a universally optimal threshold.
        selected=max(rows,key=lambda r:(r['train']['correct']-5*(r['train']['wrong']+r['train']['absent_false_accepts']),r['train']['correct']))
        result={'candidate':candidate,'selected':selected,
                'isaac_check':counts(test,selected['threshold'],selected['margin']),
                'grid':rows,'objective':'correct - 5*(wrong + absent false accepts)',
                'scope':'conditional cross-backend check of already chosen finalists; not independent generalization'}
        (a.output/(candidate+'.json')).write_text(json.dumps(result,indent=2)+'\n');results.append(result)
    (a.output/'summary.json').write_text(json.dumps(results,indent=2)+'\n')


if __name__ == '__main__':main()
