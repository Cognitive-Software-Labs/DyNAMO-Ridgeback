"""Learned lidar adapters using upstream inference and published checkpoints."""
import math
from pathlib import Path
import numpy as np


class DrowDetector:
    def __init__(self, name, model_dir, device='cuda', threshold=.5):
        from dr_spaam.detector import Detector
        filename='ckpt_jrdb_ann_drow3_e40.pth' if name == 'drow3' else 'ckpt_jrdb_ann_dr_spaam_e20.pth'
        self.model=Detector(str(Path(model_dir)/'drow/self_supervised_person_detection'/filename),
                            model='DROW3' if name == 'drow3' else 'DR-SPAAM',gpu=device.startswith('cuda'))
        self.threshold=threshold
        self.last_stamp=None; self.last_points=[]

    def detect(self, scan):
        if self.last_stamp == scan.stamp: return self.last_points
        self.model.set_laser_fov(math.degrees(abs(scan.angle_increment)*(len(scan.ranges)-1)))
        # Official preprocessing expects a symmetric increasing-angle scan.
        ranges=np.array(scan.ranges,dtype=np.float32)
        ranges[~np.isfinite(ranges) | (ranges < scan.range_min)]=29.99
        if scan.angle_increment < 0: ranges=ranges[::-1].copy()
        xy,confidence,_=self.model(ranges)
        offset=scan.angle_min+scan.angle_increment*(len(ranges)-1)/2
        c,s=math.cos(offset),math.sin(offset)
        points=[(float(c*x-s*y),float(s*x+c*y)) for (x,y),score in zip(xy,confidence) if score >= self.threshold]
        self.last_stamp,self.last_points=scan.stamp,points
        return points


class LfeDetector:
    """LFE-PPN normalization/anchor decoding follows the official ROS2 host."""
    def __init__(self, model_dir, device='cuda', threshold=.937660):
        import onnxruntime as ort
        # CPU is explicit: this avoids misreporting a failed CUDA-provider fallback.
        options=ort.SessionOptions()
        options.intra_op_num_threads=2
        options.inter_op_num_threads=1
        self.session=ort.InferenceSession(str(Path(model_dir)/'LFE-PPN.onnx'),sess_options=options,providers=['CPUExecutionProvider'])
        self.threshold=threshold

    def detect(self, scan):
        ranges=np.array(scan.ranges,dtype=np.float32)
        ranges[~np.isfinite(ranges)]=10.
        data=(1-np.clip(ranges/10.,0,1))[None,:,None]
        grid=self.session.run(['grid'],{'scan':data})[0][0]
        sectors,anchors,_=grid.shape
        anchor_depth=(10.-.02)/anchors
        sector_angle=scan.angle_increment*(len(ranges)-1)/sectors
        people=[]
        for flat in np.argsort(-grid[:,:,0].ravel(),kind='stable'):
            sector,anchor=divmod(int(flat),anchors)
            score,dr,da=grid[sector,anchor]
            if score < self.threshold: break
            radius=.02+(.5+anchor)*anchor_depth
            distance=radius+dr*anchor_depth
            angle=scan.angle_min+(.5+sector)*sector_angle+da*anchor_depth/radius
            point=(float(distance*math.cos(angle)),float(distance*math.sin(angle)))
            if all(math.dist(point,old) >= .8 for old in people): people.append(point)
        return people


def lidar_detector(name, model_dir, device='cuda'):
    if name in ('drow3','dr-spaam'): return DrowDetector(name,model_dir,device)
    if name == 'lfe-ppn': return LfeDetector(model_dir,device)
    if name == 'petra': return PetraDetector(model_dir,device)
    if name == 'leg-detector': return LegDetector(model_dir,device)
    if name == 'clustering':
        from ridgeback_leader_follow.benchmarking.adapters import ScanClustering
        return ScanClustering()
    raise ValueError(f'Unknown lidar candidate {name}')


class PetraDetector:
    """Original PeTra occupancy-image model and greedy leg pairing, CPU only."""
    def __init__(self, model_dir, device='cpu'):
        import os
        os.environ['TF_ENABLE_ONEDNN_OPTS']='0'
        import tensorflow as tf
        self.tf=tf
        raw=(Path(model_dir).parent/'f2-vendor/petra/petra/model/model_bootstrapping.pb').read_bytes()
        definition=tf.compat.v1.GraphDef();definition.ParseFromString(raw)
        self.graph=tf.Graph()
        with self.graph.as_default():tf.import_graph_def(definition,name='')
        config=tf.compat.v1.ConfigProto(intra_op_parallelism_threads=4,inter_op_parallelism_threads=2)
        config.graph_options.rewrite_options.disable_meta_optimizer=True
        self.session=tf.compat.v1.Session(graph=self.graph,config=config)
        self.input=self.graph.get_tensor_by_name('input_1:0')
        self.output=self.graph.get_tensor_by_name('conv2d_19_1/concat:0')

    def detect(self,scan):
        import cv2
        image=np.zeros((256,256),np.float32)
        for i,r in enumerate(scan.ranges):
            if not math.isfinite(r) or r <= 0 or r > 5.12:continue
            # Retain the official implementation's one-beam angle offset.
            angle=scan.angle_min+(i+1)*scan.angle_increment
            row=int(math.floor(r*math.cos(angle)*50));col=128-int(math.floor(r*math.sin(angle)*50))
            if 0 <= row < 256 and 0 <= col < 256:image[row,col]=1
        image=cv2.normalize(image,None,0,255,cv2.NORM_MINMAX)
        result=self.session.run(self.output,{self.input:image[None,:,:,None]})[0,:,:,0]
        result=cv2.normalize(result,None,0,255,cv2.NORM_MINMAX).astype(np.uint8)
        _,binary=cv2.threshold(result,200,255,cv2.THRESH_BINARY)
        contours,_=cv2.findContours(binary,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        legs=[]
        for contour in contours:
            m=cv2.moments(contour);u=m['m10']/(m['m00']+1e-5);v=m['m01']/(m['m00']+1e-5)
            legs.append((v*.02,-(u-128)*.02))
        people=[]
        while len(legs) > 1:
            pairs=[(math.dist(a,b),i,j) for i,a in enumerate(legs) for j,b in enumerate(legs) if j > i and .1 < math.dist(a,b) < .5]
            if not pairs:break
            _,i,j=min(pairs);people.append(tuple((a+b)/2 for a,b in zip(legs[i],legs[j])))
            legs.pop(j);legs.pop(i)
        return people


class LegDetector:
    """Original ROS leg-cluster forest through the standalone C++ bridge."""
    def __init__(self, model_dir, device='cpu'):
        import subprocess
        root=Path(model_dir).resolve().parent
        self.process=subprocess.Popen([str(root/'f2-leg-build/leg_cli'),
            str(root/'f2-vendor/leg/src/leg_detector/config/trained_leg_detector_res=0.33.yaml')],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)

    def detect(self,scan):
        values=[r if math.isfinite(r) and r > scan.range_min else scan.range_max for r in scan.ranges]
        row=[len(values),scan.angle_min,scan.angle_increment,scan.range_min,scan.range_max,*values]
        self.process.stdin.write(' '.join(map(str,row))+'\n');self.process.stdin.flush()
        line=self.process.stdout.readline()
        if not line:raise RuntimeError('Classic leg detector exited')
        fields=list(map(float,line.split()));count=int(fields[0])
        if len(fields) != 1+2*count:raise ValueError('Invalid leg detector reply')
        return [(fields[1+2*i],fields[2+2*i]) for i in range(count)]

    def __del__(self):
        if hasattr(self,'process'):
            self.process.terminate()
            try:self.process.wait(timeout=2)
            except Exception:self.process.kill()
