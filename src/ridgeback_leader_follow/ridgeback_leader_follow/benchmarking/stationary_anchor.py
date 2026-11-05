"""Evaluation-only planar alignment for the explicitly stationary F3 observer."""
import math
import numpy as np


def world_from_odom(bag,at):
    transform=bag.transform('odom','base_link',at)
    yaw=math.atan2(transform[1,0],transform[0,0])
    c,s=math.cos(yaw),math.sin(yaw)
    planar=np.eye(4);planar[:2,:2]=[[c,-s],[s,c]];planar[:2,3]=transform[:2,3]
    return np.linalg.inv(planar)


def align_records(records,bag,annotations):
    """Known world-from-base planar identity; never estimate from person truth."""
    aligned=[]
    for record in records:
        annotation=annotations[round(record['scenario_time'],6)]
        matrix=world_from_odom(bag,annotation['sensor_time'])
        people=[]
        for person in record['people']:
            point=matrix@np.array([person['x'],person['y'],0.,1.])
            people.append({**person,'x':float(point[0]),'y':float(point[1])})
        aligned.append({**record,'people':people})
    return aligned
