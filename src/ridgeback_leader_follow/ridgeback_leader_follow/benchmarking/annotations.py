#!/usr/bin/env python3
"""Evaluation-only mesh projection and depth-supported visibility annotations."""
import argparse
import csv
import json
import math
import os
import uuid
from pathlib import Path
import numpy as np
from numba import njit
from ridgeback_leader_follow.benchmarking.dataset import SensorBag
from ridgeback_leader_follow.crowd_geometry import asset_directory
from ridgeback_leader_follow.crowd_scenarios import load_scenario
from ridgeback_leader_follow.fusion import rigid_matrix


@njit(cache=True)
def rasterize(vertices,faces,k,h,w):
    depth=np.full((h,w),np.inf)
    for face in faces:
        tri=vertices[face]
        if np.min(tri[:,2]) <= .05: continue
        uv=np.empty((3,2))
        for i in range(3):
            uv[i,0]=k[0,0]*tri[i,0]/tri[i,2]+k[0,2]
            uv[i,1]=k[1,1]*tri[i,1]/tri[i,2]+k[1,2]
        left=max(0,int(math.floor(np.min(uv[:,0])))); right=min(w-1,int(math.ceil(np.max(uv[:,0]))))
        top=max(0,int(math.floor(np.min(uv[:,1])))); bottom=min(h-1,int(math.ceil(np.max(uv[:,1]))))
        ax,ay=uv[0];bx,by=uv[1];cx,cy=uv[2]
        denom=(by-cy)*(ax-cx)+(cx-bx)*(ay-cy)
        if abs(denom) < 1e-10: continue
        for v in range(top,bottom+1):
            for u in range(left,right+1):
                a=((by-cy)*(u+.5-cx)+(cx-bx)*(v+.5-cy))/denom
                b=((cy-ay)*(u+.5-cx)+(ax-cx)*(v+.5-cy))/denom
                c=1-a-b
                if a >= 0 and b >= 0 and c >= 0:
                    z=1/(a/tri[0,2]+b/tri[1,2]+c/tri[2,2])
                    if z < depth[v,u]: depth[v,u]=z
    return depth


def annotate(bag, output, world_from_odom, stationary_base=False):
    manifest=json.loads((bag.run/'manifest.json').read_text())
    if manifest['scenario']['name'] == 'leader_out_of_view' and manifest.get('scene_profile') == 'f3':
        from ridgeback_leader_follow.recognition_scenarios import load_recognition_scenario
        scenario=load_recognition_scenario(manifest['scenario']['name'])
    else:
        scenario=load_scenario(manifest['scenario']['name'],manifest['crowd_size'])
    with (bag.run/'ground_truth.csv').open() as stream: rows=list(csv.DictReader(stream))
    by_person={p.name:[r for r in rows if r['person'] == p.name] for p in scenario.people}
    directory=asset_directory()
    if manifest.get('scene_profile') in ('f3','lab'):
        from ridgeback_leader_follow.lab_scene import asset_directory as maya_directory
        directory=maya_directory()
    with np.load(directory/'walk.npz') as cache:
        data={key:cache[key] for key in cache.files}
    ox,oy,oz,yaw=world_from_odom
    world_odom=rigid_matrix((ox,oy,oz),(0,0,math.sin(yaw/2),math.cos(yaw/2)))
    output.parent.mkdir(parents=True,exist_ok=True)
    temporary=output.with_name(output.name+'.'+uuid.uuid4().hex+'.tmp')
    with temporary.open('x') as stream:
        for scenario_time,sim_time in bag.schedule:
            rt,rgb,depth,k,odom_color=bag.camera(sim_time)
            anchor=world_odom
            if stationary_base:
                from .stationary_anchor import world_from_odom as stationary_anchor
                anchor=world_odom@stationary_anchor(bag,rt)
            color_world=np.linalg.inv(anchor@odom_color)
            people=[]; masks=[]
            for person in scenario.people:
                samples=by_person[person.name]; times=np.array([float(r['sim_time']) for r in samples])
                pose=[float(np.interp(rt,times,[float(r[key]) for r in samples])) for key in ('x','y','z')]
                # Unwrap yaw before interpolation across the -pi/pi boundary.
                angle=float(np.interp(rt,times,np.unwrap([float(r['yaw']) for r in samples])))
                command=float(np.interp(rt,times,[float(r['command_time']) for r in samples]))
                speed=float(data['gait_speed_mps']) if 'gait_speed_mps' in data else .5
                phase=(person.distance(command)/speed)%float(data['cycle_seconds'])
                frame=int(phase*float(data['fps']))%len(data['points'])
                world_person=rigid_matrix(pose,(0,0,math.sin(angle/2),math.cos(angle/2)))
                xyz=np.column_stack((data['points'][frame],np.ones(len(data['points'][frame]))))
                vertices=(color_world@world_person@xyz.T).T[:,:3]
                mesh_depth=rasterize(vertices,data['faces'],k,rgb.shape[0],rgb.shape[1])
                mask=np.isfinite(mesh_depth); masks.append(mesh_depth)
                v,u=np.nonzero(mask)
                box=[int(u.min()),int(v.min()),int(u.max()+1),int(v.max()+1)] if len(u) else None
                people.append({'id':person.name,'role':person.role,'box':box,'distance_m':float(np.linalg.norm((color_world@np.array([*pose,1]))[:3])),
                               'projected_pixels':int(mask.sum())})
            nearest=np.minimum.reduce(masks)
            for person,mesh_depth in zip(people,masks):
                mask=np.isfinite(mesh_depth)
                # Invalid stereo samples do not imply invisibility. Valid nearer
                # surfaces account for robot/fixtures, and mesh z-buffer for people.
                visible=mask & (mesh_depth <= nearest+.02) & (~np.isfinite(depth) | (depth <= 0) | (mesh_depth <= depth+.15))
                person['visible_fraction']=float(visible.sum()/max(1,mask.sum()))
            stream.write(json.dumps({'scenario_time':scenario_time,'sensor_time':rt,'people':people})+'\n')
    try:
        os.link(temporary,output)
    finally:
        temporary.unlink()
    return {'world_anchor':('surveyed stationary base_link planar origin, recorded TF removes odometry drift' if stationary_base else 'constant supplied world-from-odom'),
            'method':'animated mesh rasterization; inter-person z-buffer and recorded-depth occlusion support',
            'scope':'in-image silhouette visible fraction estimate; excludes out-of-image area; 15 cm depth tolerance; sampled gait',
            'truth_access':'evaluation-only; never supplied to detector/tracker adapters'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--world-from-odom',type=float,nargs=4,required=True)
    args=parser.parse_args()
    print(json.dumps(annotate(SensorBag(args.run),args.output,args.world_from_odom),indent=2))


if __name__ == '__main__': main()
