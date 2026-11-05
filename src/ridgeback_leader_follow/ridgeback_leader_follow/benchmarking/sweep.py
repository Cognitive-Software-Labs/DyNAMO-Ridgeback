#!/usr/bin/env python3
"""Run sensor-only candidate adapters on a shared recorded simulation corpus."""
import argparse
import gc
import shutil
import hashlib
import json
import platform
import time
from pathlib import Path
import numpy as np
from ridgeback_leader_follow.benchmarking.dataset import SensorBag,stamp
from ridgeback_leader_follow.benchmarking.detectors import CameraDetector,CAMERA_MODELS
from ridgeback_leader_follow.benchmarking.lidar_detectors import lidar_detector
from ridgeback_leader_follow.benchmarking.adapters import ScanInput
from ridgeback_leader_follow.benchmarking.tracking import ConstantVelocityTracker
from ridgeback_leader_follow.benchmarking.evaluate import evaluate
from ridgeback_leader_follow.fusion import box_measurement


def digest(path):
    with Path(path).open('rb') as stream: return hashlib.file_digest(stream,'sha256').hexdigest()


def run_candidate(bag, name, family, output, model_dir, world_from_odom, device):
    import torch
    from tf2_ros import TransformException
    gc.collect()
    if device.startswith('cuda'): torch.cuda.empty_cache()
    output.mkdir(parents=True,exist_ok=False)
    if family == 'camera': detector=CameraDetector(name,model_dir,device)
    else: detector={k:lidar_detector(name,model_dir,device) for k in ('lidar2d_0','lidar2d_1')}
    tracker=ConstantVelocityTracker()
    times=[]; missing=0; cache={}
    if device.startswith('cuda'): torch.cuda.reset_peak_memory_stats()
    with (output/'predictions.jsonl').open('w') as predictions,(output/'detections.jsonl').open('w') as detections:
        for scenario_time,sim_time in bag.schedule:
            points=[]; boxes=[]; error=None
            if device.startswith('cuda'): torch.cuda.synchronize()
            start=time.perf_counter()
            try:
                if family == 'camera':
                    timestamp,rgb,depth,k,matrix=bag.camera(sim_time)
                    if timestamp not in cache: cache={timestamp:detector.detect(rgb)}
                    boxes=cache[timestamp].tolist()
                    for box in boxes:
                        m=box_measurement(box[:4],box[4],depth,k,matrix)
                        if m is not None: points.append((m.x,m.y))
                else:
                    for sensor in ('lidar2d_0','lidar2d_1'):
                        timestamp,scan=bag.latest(f'/{sensor}/scan',sim_time)
                        tf=bag.transform('odom',scan.header.frame_id,timestamp)
                        inp=ScanInput(timestamp,scan.header.frame_id,scan.angle_min,scan.angle_increment,
                                      tuple(scan.ranges),scan.range_min,scan.range_max)
                        for x,y in detector[sensor].detect(inp):
                            point=(tf@np.array([x,y,0.,1.]))[:2]
                            if all(np.linalg.norm(point-old) > .25 for old in points): points.append(tuple(point))
            except (ValueError,TransformException) as exc:
                missing+=1; error=str(exc);points=[]
            # The caller supplies the surveyed anchor; no fitting to truth.
            ox,oy,oz,yaw=world_from_odom; c,s=np.cos(yaw),np.sin(yaw)
            points=[(float(ox+c*x-s*y),float(oy+s*x+c*y)) for x,y in points]
            people=tracker.update(scenario_time,points)
            if device.startswith('cuda'): torch.cuda.synchronize()
            times.append((time.perf_counter()-start)*1000)
            predictions.write(json.dumps({'scenario_time':scenario_time,'frame':'world','people':people,'input_error':error})+'\n')
            detections.write(json.dumps({'scenario_time':scenario_time,'sim_time':sim_time,'boxes':boxes,'points':points,'input_error':error})+'\n')
    uses_gpu=device.startswith('cuda') and not (family == 'lidar' and name in ('clustering','lfe-ppn','petra','leg-detector'))
    receipt={'candidate':name,'family':family,'confidence_threshold':detector.threshold if family == 'camera' else None,'status':'complete','frames':len(times),'missing_input_frames':missing,
             'processing_median_ms':float(np.median(times)),'processing_p95_ms':float(np.percentile(times,95)),
             'gpu_peak_allocated_bytes':torch.cuda.max_memory_allocated() if uses_gpu else 0,
             'device':torch.cuda.get_device_name() if uses_gpu else platform.processor(),
             'requested_device':device,'backend':'CPU' if not uses_gpu else 'CUDA',
             'timing_scope':'adapter, sensor decoding, transforms and CV association; cache reused for repeated RGB stamps; model initialization excluded',
             'world_from_odom':world_from_odom,'predictions_sha256':digest(output/'predictions.jsonl'),
             'truth_access':'schedule time columns only; scoring is separate',
             'bag_sha256':{p.name:digest(p) for p in sorted((bag.run/'sensors').glob('*')) if p.is_file()},
             'source_sha256':{str(p.relative_to(Path(__file__).resolve().parent.parent)):digest(p) for p in Path(__file__).resolve().parent.parent.rglob('*.py')}}
    (output/'source').mkdir()
    for path in Path(__file__).resolve().parent.parent.rglob('*.py'):
        target=output/'source'/path.relative_to(Path(__file__).resolve().parent.parent)
        target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(path,target)
    (output/'receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
    evaluate(bag.run,output/'predictions.jsonl',output/'score.json',name)
    return receipt


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--corpus',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--model-dir',type=Path,required=True)
    parser.add_argument('--camera',nargs='*',default=list(CAMERA_MODELS))
    parser.add_argument('--lidar',nargs='*',default=['clustering','drow3','dr-spaam','lfe-ppn'])
    parser.add_argument('--device',default='cuda')
    parser.add_argument('--cases',nargs='*')
    parser.add_argument('--summary-name',default='summary.json')
    parser.add_argument('--world-from-odom',nargs=4,type=float,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    corpus=json.loads((args.corpus/'corpus.json').read_text())
    results=[]
    for case in corpus['results']:
        run=Path(case['run'])
        if args.cases is not None and run.name not in args.cases:continue
        bag=SensorBag(run)
        for family,names in [('camera',args.camera),('lidar',args.lidar)]:
            for name in names:
                output=args.output/run.name/name
                if (output/'score.json').exists():
                    results.append(json.loads((output/'receipt.json').read_text()));continue
                print(run.name,name,flush=True)
                try: result=run_candidate(bag,name,family,output,args.model_dir,args.world_from_odom,args.device)
                except Exception as exc:
                    result={'candidate':name,'case':run.name,'status':'failed','error':repr(exc)}
                    output.mkdir(parents=True,exist_ok=True)
                    (output/'failure.json').write_text(json.dumps(result,indent=2)+'\n')
                    print(result,flush=True)
                results.append(result)
                (args.output/args.summary_name).write_text(json.dumps(results,indent=2)+'\n')
        del bag


if __name__ == '__main__': main()
