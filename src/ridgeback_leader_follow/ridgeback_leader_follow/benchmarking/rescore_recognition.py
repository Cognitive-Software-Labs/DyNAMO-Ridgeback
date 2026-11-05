"""Refresh evaluation labels from corrected annotations, without model inference."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.optimize import linear_sum_assignment
from ridgeback_leader_follow.fusion import box_measurement
from .dataset import SensorBag
from .image_metrics import box_iou
from .reid import CANDIDATES,RecognitionScore
from .stationary_anchor import world_from_odom


def rescore(root,annotations):
    cases=json.loads((root/'corpus.json').read_text())['results']
    for case in cases:
        run=Path(case['run']);key=run.name
        truth=[json.loads(line) for line in (annotations/(key+'.jsonl')).read_text().splitlines()]
        detections=[json.loads(line) for line in (root/'detections'/key/'owlv2/detections.jsonl').read_text().splitlines()]
        bag=SensorBag(run);scoring=[]
        for row,annotation in zip(detections,truth,strict=True):
            if abs(row['scenario_time']-annotation['scenario_time'])>1e-6:raise ValueError('Mismatched annotation tick')
            boxes=row['boxes'] if row['input_error'] is None else []
            boxed=[person for person in annotation['people'] if person['box'] is not None]
            labels={};distances={}
            if boxed and boxes:
                ious=box_iou([p['box'] for p in boxed],[b[:4] for b in boxes])
                rr,cc=linear_sum_assignment(-ious)
                labels={int(c):boxed[r]['id'] for r,c in zip(rr,cc) if ious[r,c]>=.5}
            if boxes:
                at,_,depth,k,matrix=bag.camera(row['sim_time'])
                anchor=world_from_odom(bag,at)
                for index,box in enumerate(boxes):
                    m=box_measurement(box[:4],box[4],depth,k,matrix)
                    if m is not None:
                        point=anchor@np.array([m.x,m.y,0.,1.]);distances[index]=float(np.linalg.norm(point[:2]))
            leader=next((p for p in annotation['people'] if p['role']=='leader'),None)
            visible=leader is not None and leader.get('visible_fraction',0)>=.2
            scoring.append((labels,distances,visible,leader['distance_m'] if leader is not None else None))
        for candidate in CANDIDATES:
            folder=root/'comparison'/candidate/key;score=folder/'score.json'
            old=json.loads(score.read_text())
            predictions=[json.loads(line) for line in (folder/'predictions.jsonl').read_text().splitlines()]
            nearby=RecognitionScore();visible_score=RecognitionScore()
            for prediction,(labels,distances,visible,distance) in zip(predictions,scoring,strict=True):
                selected=prediction['selected_detection']
                visible_score.update(selected,labels,'lf_leader',leader_eligible=visible)
                if not visible or distance<=3:
                    if selected is not None and distances.get(selected,0)>3:selected=None
                    nearby.update(selected,labels,'lf_leader',leader_eligible=visible)
            backup=folder/'score.before-stationary-anchor.json'
            if not backup.exists():backup.write_text(score.read_text())
            old['nearby']=nearby.report();old['all_visible']=visible_score.report()
            old['annotation_sha256']=hashlib.sha256((annotations/(key+'.jsonl')).read_bytes()).hexdigest()
            old['evaluation_revision']='stationary base_link planar alignment via recorded TF; cached image predictions unchanged'
            score.write_text(json.dumps(old,indent=2)+'\n')
        print('Updated labels',key,flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--annotations',type=Path,required=True)
    a=parser.parse_args();rescore(a.root,a.annotations)


if __name__ == '__main__':main()
