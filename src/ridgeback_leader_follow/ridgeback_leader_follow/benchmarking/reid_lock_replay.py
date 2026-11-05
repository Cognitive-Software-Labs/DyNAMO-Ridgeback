"""Replay cached embeddings through actual fusion and leader-lock cores."""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.optimize import linear_sum_assignment
from leader_follow_interfaces.msg import People, Person
from ridgeback_leader_follow.fusion import Measurement, PeopleFusion, box_measurement
from ridgeback_leader_follow.image_association import ImageAssociation
from ridgeback_leader_follow.leader_identity_node import LeaderIdentity
from ridgeback_leader_follow.lock_on import LeaderLock
from .dataset import SensorBag
from .adapters import ScanClustering, ScanInput
from .dataset import stamp


def replay(run, camera, lidar, embeddings, output, threshold=.72, margin=.05, annotations=None):
    output.mkdir(parents=True, exist_ok=False)
    bag = SensorBag(run)
    cam = [json.loads(line) for line in camera.read_text().splitlines()]
    laser = [json.loads(line) for line in lidar.read_text().splitlines()]
    # Evaluation truth is deliberately loaded after inference and lock updates.
    with np.load(embeddings) as archive:
        features = {(int(f), int(d)): e for f, d, e in
                    zip(archive['frames'], archive['detections'], archive['features'], strict=True)}
    if len(cam) != len(bag.schedule) or len(laser) != len(cam):
        raise ValueError('Incomplete replay inputs')
    fusion = PeopleFusion(max_age=3.,appearance_weight=.75)
    association = ImageAssociation(.1)
    support_detector=ScanClustering(maximum_width=.4)
    identity = LeaderIdentity(LeaderLock(acquire_interval=1,match_threshold=threshold,recovery_margin=margin),
                              8., np.pi/6, acquire_distance_tolerance=.25, acquire_lateral_tolerance=.1)
    predictions = []
    with (output/'lock.jsonl').open('w') as stream:
        for index, ((st, sim), c, l) in enumerate(zip(bag.schedule, cam, laser, strict=True)):
            if abs(st-c['scenario_time']) > 1e-6 or abs(st-l['scenario_time']) > 1e-6:
                raise ValueError('Replay timestamp mismatch')
            measurements = []
            if c['input_error'] is None:
                _, rgb, depth, k, matrix = bag.camera(c['sim_time'])
                for camera_id, box in association.update(c['boxes'], rgb.shape[:2]):
                    m = box_measurement(box[:4], box[4], depth, k, matrix)
                    if m is not None:
                        distances = [np.max(np.abs(np.asarray(b)-box)) for b in c['boxes']]
                        detection = int(np.argmin(distances))
                        if distances[detection] > .01:
                            raise ValueError('Cannot map associated box to cached embedding')
                        if box[0] > 2 and box[2] < rgb.shape[1]-2:
                            m.appearance = tuple(map(float, features.get((index, detection), ())))
                        m.camera_id = camera_id
                        measurements.append(m)
            if l['input_error'] is None:
                measurements.extend(Measurement(x,y,.09,.5,'lidar') for x,y in l['points'])
            support_points=[]
            for sensor_name in ('lidar2d_0','lidar2d_1'):
                scan_time,scan=bag.latest('/'+sensor_name+'/scan',sim)
                transform=bag.transform('odom',scan.header.frame_id,scan_time)
                data=ScanInput(scan_time,scan.header.frame_id,scan.angle_min,scan.angle_increment,
                               tuple(scan.ranges),scan.range_min,scan.range_max)
                for x,y in support_detector.detect(data):
                    point=(transform@np.array([x,y,0.,1.]))[:2]
                    if all(np.linalg.norm(point-old)>.25 for old in support_points):support_points.append(point)
            measurements.extend(Measurement(float(x),float(y),.09,.5,'support') for x,y in support_points)
            tracks = fusion.update(st, measurements)
            msg = People()
            msg.header.frame_id = 'odom'
            msg.header.stamp.sec = int(sim)
            msg.header.stamp.nanosec = round((sim-int(sim))*1e9)
            for track in tracks:
                p = Person(id=track.identity, confidence=float(track.confidence),
                           appearance=list(track.appearance))
                p.position.x, p.position.y = map(float, track.state[:2])
                p.velocity.x, p.velocity.y = map(float, track.state[2:])
                msg.people.append(p)
            # Acquisition uses the recorded sensor transform, including odometry drift.
            to_base=bag.transform('base_link','odom',bag.latest('/camera_0/color/image',sim)[0])
            update = identity.update(msg, (float(to_base[0,3]),float(to_base[1,3]),
                                           float(np.arctan2(to_base[1,0],to_base[0,0]))))
            record = {'scenario_time':st, 'state':identity.lock.state, 'reason':update.reason,
                      'selected':int(update.track_id) if update.has_leader else None,
                      'people':[{'id':t.identity,'x':float(t.state[0]),'y':float(t.state[1]),'vx':float(t.state[2]),'vy':float(t.state[3]),
                                 'appearance_available':bool(t.appearance)} for t in tracks]}
            stream.write(json.dumps(record)+'\n')
            predictions.append(record)
    import csv
    truth = list(csv.DictReader((Path(run)/'ground_truth.csv').open()))
    # Use the existing evaluation time grouping and 0.75 m assignment gate.
    groups = {}
    for row in truth:
        key = round(float(row['scenario_time']),6)
        groups.setdefault(key,[]).append(row)
    from .lock_metrics import score_locks
    annotation_map = None
    if annotations is not None:
        annotation_map = {round(row['scenario_time'],6): row for row in
                          (json.loads(line) for line in annotations.read_text().splitlines())}
        if len(annotation_map) != len(predictions):
            raise ValueError('Incomplete camera visibility annotations')
    scoring_predictions=predictions
    manifest=json.loads((Path(run)/'manifest.json').read_text())
    if annotation_map is not None and manifest.get('scene_profile')=='f3':
        from .stationary_anchor import align_records
        scoring_predictions=align_records(predictions,bag,annotation_map)
    report = score_locks(scoring_predictions, groups, annotation_map)
    report['world_anchor']='stationary base_link planar identity via recorded TF' if scoring_predictions is not predictions else 'constant world/odom assumption' 
    report.update({'parameters':{'threshold':threshold,'recovery_margin':margin,'appearance_association_weight':fusion.appearance_weight},
                   'scope':'offline core replay; stationary observer; no ROS or motion qualification',
                   'inputs':{'camera':str(camera),'lidar':str(lidar),'embeddings':str(embeddings),
                             'annotations':str(annotations) if annotations is not None else None}})
    (output/'score.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ('run','camera','lidar','embeddings','output'):
        parser.add_argument('--'+key,type=Path,required=True)
    parser.add_argument('--annotations',type=Path)
    parser.add_argument('--threshold',type=float,default=.72)
    parser.add_argument('--margin',type=float,default=.05)
    args=parser.parse_args()
    replay(args.run,args.camera,args.lidar,args.embeddings,args.output,args.threshold,args.margin,args.annotations)


if __name__ == '__main__':
    main()
