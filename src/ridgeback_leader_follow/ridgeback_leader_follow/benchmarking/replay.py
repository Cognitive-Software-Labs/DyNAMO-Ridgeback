#!/usr/bin/env python3
"""Replay the lidar plumbing baseline from an F1 sensor bag without ROS playback."""
import argparse
import bisect
import csv
import hashlib
import json
import math
from pathlib import Path
import shutil
import time

import numpy as np

from ridgeback_leader_follow.benchmarking.adapters import ScanInput, ScanClustering
from ridgeback_leader_follow.benchmarking.tracking import ConstantVelocityTracker, transform_points


def load_schedule(run):
    """Use only evaluation time columns; annotations never reach adapters."""
    timing = {}
    with (run/'ground_truth.csv').open() as stream:
        for row in csv.DictReader(stream):
            scenario,simulation = float(row['scenario_time']),float(row['sim_time'])
            if not all(map(math.isfinite,(scenario,simulation))):
                raise ValueError('Non-finite evaluation timing')
            if scenario in timing and abs(timing[scenario]-simulation) > 1e-6:
                raise ValueError('Inconsistent frame timestamps')
            timing[scenario] = simulation
    if not timing:
        raise ValueError('Empty evaluation schedule')
    return sorted(timing.items())


def replay(run, output, world_from_odom, max_scan_age=.075):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from rclpy.duration import Duration
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException
    run,output = Path(run).resolve(),Path(output).resolve()
    manifest = json.loads((run/'manifest.json').read_text())
    if manifest['status'] != 'complete':
        raise ValueError('Replay requires a complete recording')
    if not math.isfinite(max_scan_age) or max_scan_age <= 0:
        raise ValueError('Scan age must be positive and finite')
    if len(world_from_odom) != 4 or not all(map(math.isfinite,world_from_odom)):
        raise ValueError('An explicit finite world-from-odom x,y,z,yaw is required')
    if not (run/'sensors/metadata.yaml').exists():
        raise ValueError('No continuous bag: record an F1 scene with --record first')
    schedule = load_schedule(run)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(run/'sensors'),storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('',''))
    types = {t.name:t.type for t in reader.get_all_topics_and_types()}
    scans_by_topic = {t:[] for t in types if t.endswith(('/lidar2d_0/scan','/lidar2d_1/scan'))}
    tf_topics = [t for t in types if t.endswith(('/tf','/tf_static'))]
    if len(scans_by_topic) != 2 or not tf_topics:
        raise ValueError('Bag needs both raw scans and TF')
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(scans_by_topic)+tf_topics))
    buffer = Buffer(cache_time=Duration(seconds=120.))
    static_count = 0
    while reader.has_next():
        topic,data,_ = reader.read_next()
        message = deserialize_message(data,get_message(types[topic]))
        if topic in tf_topics:
            for transform in message.transforms:
                if topic.endswith('/tf_static'):
                    buffer.set_transform_static(transform,'recorded-bag')
                    static_count += 1
                else:
                    buffer.set_transform(transform,'recorded-bag')
        else:
            stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
            scans_by_topic[topic].append(ScanInput(stamp,message.header.frame_id,
                message.angle_min,message.angle_increment,tuple(message.ranges),
                message.range_min,message.range_max))
    if not static_count:
        raise ValueError('Bag has no recorded static transforms; cannot infer sensor extrinsics')
    for scans in scans_by_topic.values():
        scans.sort(key=lambda s:s.stamp)
        if not scans:
            raise ValueError('A required scan topic has no messages')
    indexes = {t:[s.stamp for s in scans] for t,scans in scans_by_topic.items()}
    detector,tracker = ScanClustering(),ConstantVelocityTracker()
    output.mkdir(parents=True,exist_ok=False)
    durations = []
    missing_scans = missing_tf = 0
    with (output/'predictions.jsonl').open('w') as stream:
        for scenario_stamp,simulation_stamp in schedule:
            start = time.perf_counter()
            points = []
            reason = None
            for topic,scans in scans_by_topic.items():
                index = bisect.bisect_right(indexes[topic],simulation_stamp+1e-9)-1
                if index < 0 or simulation_stamp-scans[index].stamp > max_scan_age:
                    missing_scans += 1; reason = 'missing_or_stale_scan'; break
                scan = scans[index]
                try:
                    tf = buffer.lookup_transform('odom',scan.frame,
                        Time(nanoseconds=round(scan.stamp*1e9)))
                except TransformException:
                    missing_tf += 1; reason = 'missing_transform'; break
                p,q = tf.transform.translation,tf.transform.rotation
                points.extend(transform_points(detector.detect(scan),(p.x,p.y,p.z),
                                               (q.x,q.y,q.z,q.w),world_from_odom))
            # Reject partial 360-degree input explicitly; do not hide missing data.
            people = tracker.update(scenario_stamp,points if reason is None else [])
            durations.append(time.perf_counter()-start)
            stream.write(json.dumps({'scenario_time':scenario_stamp,'frame':'world',
                'people':people,'input_error':reason},allow_nan=False)+'\n')
    sources = Path(__file__).resolve().parent
    (output/'source').mkdir()
    for name in ('replay.py','adapters.py','tracking.py'):
        shutil.copyfile(sources/name,output/'source'/name)

    def digest(path):
        with path.open('rb') as stream:
            return hashlib.file_digest(stream,'sha256').hexdigest()

    receipt = {'candidate':'scan-clustering-cv-plumbing','status':'complete',
        'scope':'unselected CPU plumbing baseline; clusters are not person classifications',
        'world_from_odom':list(world_from_odom),'max_scan_age_s':max_scan_age,
        'frames':len(schedule),'missing_scan_frames':missing_scans,'missing_tf_frames':missing_tf,
        'processing_median_ms':float(np.median(durations)*1000),
        'processing_p95_ms':float(np.percentile(durations,95)*1000),
        'timing_scope':'clustering, transforms and association; excludes bag loading and scoring',
        'parameters':{'cluster_split_m':detector.split_distance,
                      'cluster_minimum_points':detector.minimum_points,
                      'cluster_maximum_width_m':detector.maximum_width,
                      'association_gate_m':tracker.gate,'track_maximum_age_s':tracker.max_age},
        'source_sha256':{p.name:digest(p) for p in (output/'source').glob('*.py')},
        'timing_schedule_sha256':digest(run/'ground_truth.csv'),
        'input_sha256':{str(p.relative_to(run)):digest(p)
                        for p in sorted((run/'sensors').glob('*')) if p.is_file()},
        'ground_truth_access':'evaluation timestamps only; no truth positions or identities consumed'}
    (output/'replay.json').write_text(json.dumps(receipt,indent=2)+'\n')
    return receipt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--world-from-odom',required=True,nargs=4,type=float,metavar=('X','Y','Z','YAW'))
    args = parser.parse_args()
    print(json.dumps(replay(args.run,args.output,args.world_from_odom),indent=2))


if __name__ == '__main__':
    main()
