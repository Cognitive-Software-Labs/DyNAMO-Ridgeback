#!/usr/bin/env python3
"""Stationary, evaluation-only G1 viewpoints; never publishes motion or goals.

Requires an already running stationary simulation. Only the newly spawned
evaluation humanoid/table are moved or removed. Runtime estimators are reused.
"""
import argparse
from collections import Counter, OrderedDict
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import struct
import subprocess
import time
import xml.etree.ElementTree as ET

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from action_msgs.msg import GoalStatusArray
from tf2_ros import Buffer, TransformListener, TransformException
from ament_index_python.packages import get_package_share_directory

from ridgeback_interfaces.msg import TargetDetections, TargetMeasurements
from ridgeback_autonomy.mission.target_localization import TargetAssociation, TargetLocalizationAdapter
from ridgeback_autonomy.benchmarking.simulation import (
    spawn_model_command, despawn_model_command, pose_snapshot_command, pose_snapshot_from_payload,
)


CASES = [
    ('headon_3m',3.,0.,math.pi), ('headon_2m',2.,0.,math.pi),
    ('headon_4m',4.,0.,math.pi), ('profile_3m',3.,0.,math.pi/2),
    ('back_3m',3.,0.,0.), ('lateral_3m',3.,.7,math.atan2(-.7,-3.)),
    ('edge_3m',3.,2.4,math.atan2(-2.4,-3.)), ('table_3m',3.,0.,math.pi),
    ('oblique_3m',3.,0.,3*math.pi/4),
    ('lateral_right_3m',3.,-.7,math.atan2(.7,-3.)),
    ('table_low_3m',3.,0.,math.pi),
]


def command(args):
    result=subprocess.run(args,capture_output=True,text=True,timeout=15)
    if result.returncode:
        raise RuntimeError(f'{args}: {result.stderr[-500:]}')
    return result.stdout


def foot_reference(model):
    root=ET.parse(model).getroot()
    centers=[]
    for side in ('left','right'):
        visual=root.find(f'.//visual[@name="{side}_ankle_roll_link_visual"]')
        pose=list(map(float,visual.findtext('pose').split()))
        if any(abs(v)>1e-8 for v in pose[3:]):
            raise ValueError('evaluation helper requires original unrotated foot visual')
        data=(model.parent/visual.findtext('geometry/mesh/uri')).read_bytes()
        n=struct.unpack_from('<I',data,80)[0]
        if len(data)!=84+50*n: raise ValueError('expected original binary STL')
        triangles=np.frombuffer(data,offset=84,count=n,dtype=[('normal','<f4',(3,)),
            ('vertices','<f4',(3,3)),('attr','<u2')])['vertices'].reshape(-1,3)
        centers.append(((triangles.min(0)+triangles.max(0))/2+np.asarray(pose[:3])).tolist())
    return np.mean(centers,axis=0).tolist()


def xy_in_frame(world_xy,world_base,map_base):
    q=world_base['orientation']; z,w=q['z'],q['w']; x,y=q['x'],q['y']
    yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    dx=world_xy[0]-world_base['position']['x']; dy=world_xy[1]-world_base['position']['y']
    bx,by=math.cos(yaw)*dx+math.sin(yaw)*dy,-math.sin(yaw)*dx+math.cos(yaw)*dy
    return [map_base[0]+math.cos(map_base[2])*bx-math.sin(map_base[2])*by,
            map_base[1]+math.sin(map_base[2])*bx+math.cos(map_base[2])*by]


class Recorder:
    def __init__(self, output):
        self.node=rclpy.create_node('parking_localization_qualification',
            parameter_overrides=[Parameter('use_sim_time',value=True)],
            cli_args=['--ros-args','-r','__ns:=/r100_0001','-r','/tf:=tf','-r','/tf_static:=tf_static'])
        self.buffer=Buffer(node=self.node); self.listener=TransformListener(self.buffer,self.node)
        self.adapter=TargetLocalizationAdapter(self.buffer,map_frame='map',base_frame='base_link')
        self.log=(output/'observations.jsonl').open('w',buffering=1)
        self.detections=OrderedDict(); self.pending=[]; self.case=None; self.image=None
        self.active_goals=set(); self.max_command=0.; self.max_speed=0.; self.fail=None
        self.positive=Counter(); self.rejections=Counter(); self.samples=[]; self.batches=[]
        self.bridge=CvBridge()
        self.node.create_subscription(TargetDetections,'detections/target/raw',self.detect,30)
        self.node.create_subscription(TargetMeasurements,'measurements/target/mask',self.measure,30)
        self.node.create_subscription(Image,'sensors/camera_0/color/image',lambda m:setattr(self,'image',m),qos_profile_sensor_data)
        latched=QoSProfile(depth=20,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(String,'parking_mission/state',lambda m:self.write('mission_state',json.loads(m.data)),latched)
        for topic in ('navigate_to_pose/_action/status','navigate_through_poses/_action/status'):
            self.node.create_subscription(GoalStatusArray,topic,self.actions,latched)
        for topic in ('cmd_vel_nav','cmd_vel_smoothed','cmd_vel','platform/cmd_vel'):
            self.node.create_subscription(TwistStamped,topic,self.velocity,qos_profile_sensor_data)
        self.node.create_subscription(Odometry,'platform/odom/filtered',self.odom,qos_profile_sensor_data)

    def write(self,kind,data):
        self.log.write(json.dumps(dict(kind=kind,case=self.case,wall_s=time.time(),data=data))+'\n')

    def actions(self,msg):
        self.write('navigation_status',message_to_ordereddict(msg))
        if any(s.status in (1,2,3) for s in msg.status_list): self.fail='unexpected active navigation goal'

    def velocity(self,msg):
        speed=max(math.hypot(msg.twist.linear.x,msg.twist.linear.y),abs(msg.twist.angular.z))
        self.max_command=max(self.max_command,speed)
        if speed>1e-6: self.fail='unexpected nonzero velocity command'

    def odom(self,msg):
        speed=math.hypot(msg.twist.twist.linear.x,msg.twist.twist.linear.y)
        self.max_speed=max(self.max_speed,speed)
        if speed>.005 or abs(msg.twist.twist.angular.z)>.005: self.fail='robot not stationary'

    def detect(self,msg):
        stamp=Time.from_msg(msg.header.stamp).nanoseconds
        self.detections[stamp]=msg
        while len(self.detections)>200: self.detections.popitem(last=False)
        if self.case:
            self.positive['frames']+=1; self.positive['positive']+=int(msg.detected)
            self.write('detection',message_to_ordereddict(msg))

    def measure(self,msg):
        if self.case:
            self.write('measurement',message_to_ordereddict(msg))
            self.pending.append(msg); self.pending=self.pending[-50:]

    def spin(self,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(self.node,timeout_sec=.02)
            if self.fail: raise RuntimeError(self.fail)
            self.consume()

    def consume(self):
        waiting=[]
        for msg in self.pending:
            stamp=Time.from_msg(msg.header.stamp).nanoseconds
            if stamp<self.minimum_stamp: continue
            det=self.detections.get(stamp)
            if det is None:
                waiting.append(msg); continue
            candidates=[i for i,label in enumerate(det.labels) if label.strip().lower()=='humanoid robot']
            if len(candidates)!=1:
                self.rejections['no_unique_humanoid_detection']+=1; continue
            i=candidates[0]
            association=TargetAssociation(stamp,det.labels[i],tuple(det.bbox_xyxy[4*i:4*i+4]))
            result=self.adapter.observe(msg,association=association,now_ns=self.node.get_clock().now().nanoseconds)
            if result.reason=='transform_unavailable' and self.node.get_clock().now().nanoseconds-stamp<1_000_000_000:
                waiting.append(msg); continue
            self.write('adapter_result',asdict(result))
            if result.accepted:
                obs=asdict(result.observation)
                transform=self.buffer.lookup_transform('map','base_link',Time(nanoseconds=stamp))
                q=transform.transform.rotation
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                mb=[transform.transform.translation.x,transform.transform.translation.y,yaw]
                gt_model=xy_in_frame(self.world_g1[:2],self.world_robot,mb)
                fx=self.world_g1[0]+math.cos(self.world_g1[2])*self.foot[0]-math.sin(self.world_g1[2])*self.foot[1]
                fy=self.world_g1[1]+math.sin(self.world_g1[2])*self.foot[0]+math.cos(self.world_g1[2])*self.foot[1]
                gt_foot=xy_in_frame((fx,fy),self.world_robot,mb)
                obs.update(map_base=mb,gt_model_map=gt_model,gt_foot_map=gt_foot,
                    model_error_m=math.hypot(obs['map_x_m']-gt_model[0],obs['map_y_m']-gt_model[1]),
                    foot_error_m=math.hypot(obs['map_x_m']-gt_foot[0],obs['map_y_m']-gt_foot[1]))
                self.samples.append(obs); self.write('evaluated_sample',obs)
            else: self.rejections[result.reason]+=1
            if result.stabilized:
                self.batches.append(asdict(result.stabilized)); self.adapter.reset()
        self.pending=waiting

    def begin(self,name,robot,g1,foot):
        self.case=name; self.world_robot=robot; self.world_g1=g1; self.foot=foot
        self.minimum_stamp=self.node.get_clock().now().nanoseconds
        self.adapter.reset(); self.pending=[]; self.samples=[]; self.batches=[]
        self.positive=Counter(); self.rejections=Counter()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True); parser.add_argument('--world',default='warehouse')
    parser.add_argument('--seconds',type=float,default=12.)
    parser.add_argument('--cases',nargs='+',choices=[c[0] for c in CASES])
    args=parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='42': parser.error('domain 42 required')
    output=Path(args.output).resolve(); output.mkdir(parents=True,exist_ok=False)
    share=Path(get_package_share_directory('ridgeback_autonomy_gz'))
    model=share/'sim/models/g1/model.sdf'; foot=foot_reference(model)
    name='g1_parking_qualification'; table='table_parking_qualification'
    def snapshot():
        payload=json.loads(command(pose_snapshot_command(f'/world/{args.world}/pose/info')))
        return pose_snapshot_from_payload(payload)
    first=snapshot()
    if any('g1' in n.lower() for n in first): raise RuntimeError('existing G1; do not create duplicates')
    rclpy.init(); recorder=Recorder(output); recorder.minimum_stamp=0
    spawned=[]; results=[]
    try:
        recorder.spin(3.)
        for case,x,y,yaw in CASES:
            if args.cases and case not in args.cases: continue
            recorder.case=None
            if not spawned:
                command(spawn_model_command(args.world,str(model),name,x,y,0.,yaw)); spawned.append(name)
            else:
                req=f'name: "{name}" position: {{x:{x},y:{y},z:0}} orientation: {{z:{math.sin(yaw/2)},w:{math.cos(yaw/2)}}}'
                reply=command(['gz','service','-s',f'/world/{args.world}/set_pose/blocking',
                    '--reqtype','gz.msgs.Pose','--reptype','gz.msgs.Boolean','--timeout','5000','--req',req])
                if 'true' not in reply: raise RuntimeError('set_pose not acknowledged')
            if case in ('table_3m','table_low_3m'):
                asset='table' if case=='table_3m' else 'table_low'
                command(spawn_model_command(args.world,str(share/f'sim/models/{asset}/model.sdf'),table,2.1,0.,0.,math.pi/2)); spawned.append(table)
            recorder.pending=[]; recorder.spin(2.)
            before=snapshot()
            pose=before[name]; q=pose['orientation']
            gyaw=math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))
            gt=[pose['position']['x'],pose['position']['y'],gyaw]
            recorder.begin(case,before['r100_0001/robot'],gt,foot)
            recorder.spin(args.seconds)
            after=snapshot()
            record=dict(case=case,world_g1=gt,foot_midpoint_in_model=foot,
                world_robot_before=before['r100_0001/robot'],world_robot_after=after['r100_0001/robot'],
                samples=recorder.samples,batches=recorder.batches,detections=dict(recorder.positive),
                rejected=dict(recorder.rejections),max_command=recorder.max_command,max_odom_speed=recorder.max_speed)
            results.append(record)
            (output/'results.json').write_text(json.dumps(results,indent=2))
            print(json.dumps(dict(case=case,samples=len(recorder.samples),batches=len(recorder.batches),
                detections=dict(recorder.positive),rejected=dict(recorder.rejections))),flush=True)
            if recorder.image is not None:
                image=recorder.bridge.imgmsg_to_cv2(recorder.image,desired_encoding='bgr8')
                if recorder.samples:
                    box=recorder.samples[-1]['bbox_xyxy']; x1,y1,x2,y2=map(int,box)
                    cv2.rectangle(image,(x1,y1),(x2,y2),(0,255,255),2)
                cv2.imwrite(str(output/f'{case}.png'),image)
            if table in spawned:
                command(despawn_model_command(args.world,table)); spawned.remove(table)
    finally:
        recorder.case=None
        for owned in reversed(spawned):
            try: command(despawn_model_command(args.world,owned))
            except Exception as exc: print(f'cleanup {owned}: {exc}',flush=True)
        recorder.log.close(); recorder.node.destroy_node(); rclpy.shutdown()


if __name__=='__main__': main()
