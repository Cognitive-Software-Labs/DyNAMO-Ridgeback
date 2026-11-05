import numpy as np
from types import SimpleNamespace
from ridgeback_leader_follow.benchmarking.stationary_anchor import align_records,world_from_odom


def test_anchor_uses_robot_tf_not_pedestrian_labels_and_preserves_raw_records():
    matrix=np.eye(4);matrix[0,3]=-.8
    bag=SimpleNamespace(transform=lambda target,source,at:matrix)
    records=[{'scenario_time':20.,'people':[{'id':1,'x':6.7,'y':0.}]}]
    annotations={20.:{'sensor_time':20.}}
    result=align_records(records,bag,annotations)
    assert result[0]['people'][0]['x']==7.5
    assert records[0]['people'][0]['x']==6.7


def test_planar_anchor_removes_translation_and_yaw():
    matrix=np.eye(4);matrix[:2,:2]=[[0.,-1.],[1.,0.]];matrix[:2,3]=[1.,2.]
    bag=SimpleNamespace(transform=lambda *args:matrix)
    result=world_from_odom(bag,1.)@np.array([1.,3.,0.,1.])
    np.testing.assert_allclose(result,[1.,0.,0.,1.],atol=1e-12)
