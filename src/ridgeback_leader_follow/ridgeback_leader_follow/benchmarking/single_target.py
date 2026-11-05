#!/usr/bin/env python3
"""SAM2/SAMURAI replay, using only one declared initial leader click."""
import argparse
import json
import time
from pathlib import Path
import numpy as np
from PIL import Image
from ridgeback_leader_follow.benchmarking.dataset import SensorBag
from ridgeback_leader_follow.fusion import box_measurement
from ridgeback_leader_follow.benchmarking.evaluate import evaluate


def run(bag,output,checkpoint,mode,initial_click):
    import torch
    from sam2.build_sam import build_sam2_video_predictor
    output.mkdir(parents=True,exist_ok=False)
    images=output/'images';images.mkdir()
    for i,(st,sim) in enumerate(bag.schedule):
        _,rgb,*_=bag.camera(sim)
        Image.fromarray(rgb).save(images/f'{i:05d}.jpg',quality=95)
    config='configs/samurai/sam2.1_hiera_t.yaml' if mode == 'samurai' else 'configs/sam2.1/sam2.1_hiera_t.yaml'
    predictor=build_sam2_video_predictor(config,str(checkpoint),device='cuda')
    durations=[];coverage=0
    torch.cuda.reset_peak_memory_stats()
    with torch.inference_mode(),torch.autocast('cuda',dtype=torch.bfloat16):
        state=predictor.init_state(str(images),offload_video_to_cpu=True)
        predictor.add_new_points_or_box(state,frame_idx=0,obj_id=1,points=np.array([initial_click],dtype=np.float32),labels=np.array([1],dtype=np.int32))
        generator=predictor.propagate_in_video(state)
        with (output/'predictions.jsonl').open('w') as stream:
            for index,(scenario_time,simulation_time) in enumerate(bag.schedule):
                torch.cuda.synchronize();start=time.perf_counter()
                frame,ids,logits=next(generator)
                torch.cuda.synchronize();durations.append((time.perf_counter()-start)*1000)
                if frame != index: raise ValueError('SAM output frame mismatch')
                mask=(logits[0,0] > 0).cpu().numpy()
                v,u=np.nonzero(mask);people=[]
                if len(u):
                    at,rgb,depth,k,matrix=bag.camera(simulation_time)
                    box=[u.min(),v.min(),u.max()+1,v.max()+1]
                    m=box_measurement(box,1.,depth,k,matrix)
                    if m is not None: people=[{'track_id':1,'x':m.x,'y':m.y}];coverage+=1
                stream.write(json.dumps({'scenario_time':scenario_time,'frame':'world','people':people})+'\n')
    receipt={'candidate':mode,'device':torch.cuda.get_device_name(),'gpu_peak_allocated_bytes':torch.cuda.max_memory_allocated(),'frames':len(durations),'nonempty_prediction_frames':coverage,
             'initial_click':initial_click,'initialization':'one supplied image-pixel click at frame zero; no further truth input',
             'processing_median_ms':float(np.median(durations)),'processing_p95_ms':float(np.percentile(durations,95)),
             'timing_scope':'video propagation only; excludes initialization, decoding, depth projection',
             'scope':'single-target coverage; score unmatched bystanders separately from leader diagnostics'}
    (output/'receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
    evaluate(bag.run,output/'predictions.jsonl',output/'score.json',mode)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('run','output','checkpoint'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--mode',choices=('sam2','samurai'),required=True)
    p.add_argument('--initial-click',nargs=2,type=float,required=True)
    a=p.parse_args();run(SensorBag(a.run),a.output,a.checkpoint,a.mode,a.initial_click)


if __name__ == '__main__':main()
