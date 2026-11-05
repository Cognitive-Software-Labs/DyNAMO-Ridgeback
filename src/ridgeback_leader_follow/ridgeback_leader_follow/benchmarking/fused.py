#!/usr/bin/env python3
"""Evaluate RGB-D-confirmed CV Kalman fusion using cached sensor detections."""
import argparse
import json
from pathlib import Path
from ridgeback_leader_follow.benchmarking.dataset import SensorBag
from ridgeback_leader_follow.benchmarking.evaluate import evaluate
from ridgeback_leader_follow.image_association import ImageAssociation
from ridgeback_leader_follow.fusion import Measurement,PeopleFusion,box_measurement


def replay_fusion(bag,camera,lidar,output):
    cam=[json.loads(line) for line in camera.read_text().splitlines()]
    laser=[json.loads(line) for line in lidar.read_text().splitlines()]
    if len(cam) != len(bag.schedule) or len(laser) != len(cam):raise ValueError('Incomplete fusion inputs')
    output.mkdir(parents=True,exist_ok=False);fusion=PeopleFusion()
    association=ImageAssociation(.1 if camera.parent.name == 'owlv2' else .25)
    with (output/'predictions.jsonl').open('w') as stream:
        for (st,sim),c,l in zip(bag.schedule,cam,laser,strict=True):
            if abs(st-c['scenario_time']) > 1e-6 or abs(st-l['scenario_time']) > 1e-6:raise ValueError('Fusion tick mismatch')
            measurements=[]
            if c['input_error'] is None:
                _,rgb,depth,k,matrix=bag.camera(sim)
                for camera_id,box in association.update(c['boxes'],rgb.shape[:2]):
                    m=box_measurement(box[:4],box[4],depth,k,matrix)
                    if m is not None:
                        m.camera_id=camera_id
                        measurements.append(m)
            if l['input_error'] is None:
                measurements.extend(Measurement(x,y,.09,.5,'lidar') for x,y in l['points'])
            tracks=fusion.update(st,measurements)
            people=[{'track_id':t.identity,'x':float(t.state[0]),'y':float(t.state[1]),
                     'vx':float(t.state[2]),'vy':float(t.state[3])} for t in tracks]
            stream.write(json.dumps({'scenario_time':st,'frame':'world','people':people})+'\n')
    receipt={'candidate':'RGB-D-confirmed CV Kalman fusion','camera':str(camera),'lidar':str(lidar),
             'parameters':{'association_gate_m':fusion.gate,'maximum_age_s':fusion.max_age,'acceleration_noise':fusion.acceleration},
             'world_anchor':'requires stationary F1 world/odom XY origins to coincide',
             'truth_access':'evaluation time columns only; scoring separate',
             'image_association':'Ultralytics ByteTrack; high/new threshold equals detector threshold, low equals half; buffer 30, match 0.8; score fusion off for OWLv2 low-confidence scale',
             'scope':'camera creates person tracks; lidar sustains/refines confirmed tracks; no appearance embedding (F3)'}
    (output/'receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
    evaluate(bag.run,output/'predictions.jsonl',output/'score.json','fused-people-tracker')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('run','camera','lidar','output'):p.add_argument('--'+key,type=Path,required=True)
    a=p.parse_args();replay_fusion(SensorBag(a.run),a.camera,a.lidar,a.output)


if __name__ == '__main__':main()
