#!/usr/bin/env python3
"""Compare the five planned MOT algorithms on identical cached detector boxes."""
import argparse
import json
import time
from pathlib import Path
import numpy as np
from ridgeback_leader_follow.benchmarking.dataset import SensorBag
from ridgeback_leader_follow.benchmarking.image_metrics import tracking_metrics
from ridgeback_leader_follow.benchmarking.evaluate import evaluate
from ridgeback_leader_follow.fusion import box_measurement


TRACKERS=('bytetrack','botsort','ocsort','deepocsort','strongsort')


def compare(bag, detections, annotations, output, names=TRACKERS):
    import torch
    from boxmot import create_tracker
    from boxmot.reid.specs import ReIDConfig
    rows=[json.loads(line) for line in Path(detections).read_text().splitlines()]
    truth=[json.loads(line) for line in Path(annotations).read_text().splitlines()]
    if len(rows) != len(bag.schedule) or len(truth) != len(rows): raise ValueError('Incomplete MOT inputs')
    results={}
    for name in names:
        folder=output/name
        if (folder/'metrics.json').exists():
            results[name]=json.loads((folder/'metrics.json').read_text());continue
        folder.mkdir(parents=True,exist_ok=False)
        tracker=create_tracker(name) if name in ('bytetrack','ocsort') else create_tracker(name,reid=ReIDConfig(model=Path.cwd()/'artifacts/leader-following/f2-models/osnet_x0_25_msmt17.pt',device='cuda'))
        frames=[];durations=[]
        torch.cuda.reset_peak_memory_stats()
        with (folder/'predictions.jsonl').open('w') as stream,(folder/'tracks.jsonl').open('w') as boxes_out:
            for (scenario_time,sim_time),row,annotation in zip(bag.schedule,rows,truth,strict=True):
                if abs(row['scenario_time']-scenario_time) > 1e-6 or abs(annotation['scenario_time']-scenario_time) > 1e-6:
                    raise ValueError('MOT inputs not aligned')
                at,rgb,depth,k,matrix=bag.camera(sim_time)
                boxes=np.asarray(row['boxes'],dtype=np.float32).reshape(-1,5)
                data=np.column_stack((boxes,np.zeros(len(boxes),dtype=np.float32)))
                torch.cuda.synchronize();start=time.perf_counter()
                tracks=np.asarray(tracker.update(data,rgb[:,:,::-1].copy()))
                torch.cuda.synchronize();durations.append((time.perf_counter()-start)*1000)
                people=[];pred=[]
                for track in tracks:
                    # BoxMOT legacy ndarray rows: xyxy,id,confidence,class,index.
                    identity=int(track[4]); pred.append((identity,track[:4].tolist()))
                    m=box_measurement(track[:4],float(track[5]),depth,k,matrix)
                    if m is not None: people.append({'track_id':identity,'x':m.x,'y':m.y})
                gt=[(p['id'],p['box']) for p in annotation['people'] if p['box'] is not None and p['visible_fraction'] > 0]
                frames.append((gt,pred))
                stream.write(json.dumps({'scenario_time':scenario_time,'frame':'world','people':people})+'\n')
                boxes_out.write(json.dumps({'scenario_time':scenario_time,'tracks':pred})+'\n')
        result=tracking_metrics(frames)
        result.update({'tracker':name,'gpu_peak_allocated_bytes':torch.cuda.max_memory_allocated() if name not in ('bytetrack','ocsort') else 0,
                       'device':torch.cuda.get_device_name() if name not in ('bytetrack','ocsort') else 'CPU','frames':len(frames),'processing_median_ms':float(np.median(durations)),
                       'processing_p95_ms':float(np.percentile(durations,95)),'detector':str(detections),
                       'timing_scope':'tracker only; detector, RGB-D decoding and world projection excluded',
                       'world_anchor':'stationary F1 world/odom XY origins coincide'})
        (folder/'metrics.json').write_text(json.dumps(result,indent=2)+'\n')
        evaluate(bag.run,folder/'predictions.jsonl',folder/'score.json',name)
        results[name]=result
    return results


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for arg in ('run','detections','annotations','output'): parser.add_argument('--'+arg,type=Path,required=True)
    args=parser.parse_args()
    print(json.dumps(compare(SensorBag(args.run),args.detections,args.annotations,args.output),indent=2))


if __name__ == '__main__': main()
