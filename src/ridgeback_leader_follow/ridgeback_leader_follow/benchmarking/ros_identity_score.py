"""Score recorded real ROS identity outputs against simulation-only truth."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from leader_follow_interfaces.msg import LeaderState
from .dataset import SensorBag
from .lock_metrics import score_locks
from .stationary_anchor import align_records


def score(run,validation,output,annotations_path=None):
    bag=SensorBag(run)
    people={round(row['sim_time'],6):row for row in
            (json.loads(line) for line in (validation/'people.jsonl').read_text().splitlines())}
    leaders=[json.loads(line) for line in (validation/'leader.jsonl').read_text().splitlines()]
    truth={}
    with (run/'ground_truth.csv').open() as stream:
        for row in csv.DictReader(stream):truth.setdefault(row['person'],[]).append(row)
    start=max(float(rows[0]['sim_time']) for rows in truth.values())
    end=min(float(rows[-1]['sim_time']) for rows in truth.values())
    predictions=[];groups={};annotations={}
    visibility=[] if annotations_path is None else [json.loads(line) for line in annotations_path.read_text().splitlines()]
    visibility_times=np.array([row['sensor_time'] for row in visibility])
    state_names={LeaderState.SEARCHING:'SEARCHING',LeaderState.LOCKED:'LOCKED',
                 LeaderState.UNCERTAIN:'UNCERTAIN',LeaderState.LOST:'LOST'}
    for leader in leaders:
        at=leader['sim_time'];key=round(at,6)
        if not start <= at <= end:continue
        row=people.get(key)
        if row is None:continue
        if row['frame']!='odom':raise ValueError('ROS scorer currently requires odom outputs')
        points=[{'id':p['track_id'],'x':p['x'],'y':p['y'],
                 'appearance_available':bool(p['appearance'])} for p in row['people']]
        predictions.append({'scenario_time':at,'state':state_names[leader['state']],
                            'selected':leader['track_id'] if leader['has_leader'] else None,'people':points})
        groups[key]=[]
        for samples in truth.values():
            times=[float(sample['sim_time']) for sample in samples]
            groups[key].append({'role':samples[0]['role'],
                'x':float(np.interp(at,times,[float(sample['x']) for sample in samples])),
                'y':float(np.interp(at,times,[float(sample['y']) for sample in samples]))})
        annotations[key]={'sensor_time':at}
        if visibility:
            nearest=int(np.argmin(np.abs(visibility_times-at)))
            annotations[key]['people']=visibility[nearest]['people'] if abs(visibility_times[nearest]-at)<=.075 else []
    aligned=align_records(predictions,bag,annotations)
    result=score_locks(aligned,groups,annotations if visibility else None)
    result['status']='passed' if result['correct_locked_frames']>0 and result['wrong_locked_frames']==0 else 'failed'
    result['scope']='real ROS tracker plus identity node, recorded sensors, stationary observer; no driving or hardware timing claim'
    output.write_text(json.dumps(result,indent=2)+'\n')
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('run','validation','output'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--annotations',type=Path)
    a=p.parse_args();score(a.run,a.validation,a.output,a.annotations)


if __name__ == '__main__':main()
