import numpy as np
import pytest
from ridgeback_leader_follow.fusion import Measurement, PeopleFusion, depth_in_color, box_measurement


def test_lidar_does_not_invent_people_and_keeps_confirmed_id():
    fusion=PeopleFusion()
    assert fusion.update(0.,[Measurement(2,0,.04,.5,'lidar')]) == []
    first=fusion.update(.1,[Measurement(2,0,.04,.9,'camera')])[0]
    seen=fusion.update(.2,[Measurement(2.1,0,.02,.7,'lidar')])[0]
    assert seen.identity == first.identity
    assert 2 < seen.state[0] < 2.1
    assert seen.state[2] > 0
    assert np.linalg.eigvalsh(seen.covariance).min() > 0
    assert fusion.update(.3,[]) == []
    assert fusion.update(1.2,[Measurement(2.2,0,.02,.7,'lidar')]) == []


def test_depth_registration_baseline_and_nearest_surface():
    k=np.array([[10,0,0],[0,10,0],[0,0,1]])
    depth=np.ones((4,4))*2
    matrix=np.eye(4); matrix[0,3]=.2
    registered=depth_in_color(depth,k,k,(4,4),matrix)
    assert np.isinf(registered[:,0]).all()
    assert np.allclose(registered[:,1:],2)


def test_box_depth_invalid_rejected_and_metric_projection():
    k=np.array([[10,0,5],[0,10,5],[0,0,1]])
    depth=np.full((20,20),np.inf)
    assert box_measurement([0,0,20,20],.8,depth,k,np.eye(4)) is None
    depth[:]=2
    m=box_measurement([0,0,20,20],.8,depth,k,np.eye(4))
    assert (m.x,m.y) == (1.,1.)
    with pytest.raises(ValueError): Measurement(0,0,-1,.5,'camera')
    fusion=PeopleFusion(); fusion.update(1,[])
    with pytest.raises(ValueError): fusion.update(1,[])


def test_camera_identity_cue_preserves_crossing_but_cannot_teleport():
    fusion=PeopleFusion()
    first=fusion.update(0.,[Measurement(0,0,.04,.9,'camera',camera_id=10),
                            Measurement(.4,0,.04,.9,'camera',camera_id=20)])
    original={t.camera_id:t.identity for t in first}
    crossing=fusion.update(.1,[Measurement(.3,0,.04,.9,'camera',camera_id=10),
                               Measurement(.1,0,.04,.9,'camera',camera_id=20)])
    assert {t.camera_id:t.identity for t in crossing} == original
    far=fusion.update(.2,[Measurement(4,0,.04,.9,'camera',camera_id=10)])
    assert far[0].identity != original[10]


def test_appearance_resolves_swapped_camera_ids_inside_physical_gate():
    first=(1.,0.);second=(0.,1.)
    from ridgeback_leader_follow.fusion import Measurement,PeopleFusion
    def run(weight):
        fusion=PeopleFusion(appearance_weight=weight)
        fusion.update(0.,[Measurement(0.,0.,.01,.9,'camera',first,1),
                          Measurement(.6,0.,.01,.9,'camera',second,2)])
        tracks=fusion.update(.05,[Measurement(.15,0.,.01,.9,'camera',first,2),
                                 Measurement(.45,0.,.01,.9,'camera',second,1)])
        return next(t for t in tracks if t.identity==1)
    assert run(0.).appearance==second
    assert run(.75).appearance==first


def test_lookalike_camera_id_swap_does_not_override_nearer_motion_match():
    from ridgeback_leader_follow.fusion import Measurement,PeopleFusion
    feature=(1.,0.)
    fusion=PeopleFusion(appearance_weight=.75)
    fusion.update(0.,[Measurement(0.,0.,.01,.9,'camera',feature,1),
                      Measurement(.6,0.,.01,.9,'camera',feature,2)])
    tracks=fusion.update(.05,[Measurement(.15,0.,.01,.9,'camera',feature,2),
                             Measurement(.45,0.,.01,.9,'camera',feature,1)])
    assert next(t for t in tracks if t.identity==1).state[0]<.15
    tracks=fusion.update(.1,[Measurement(4.,0.,.01,.9,'camera',feature,2)])
    assert len(tracks)==1 and tracks[0].identity not in (1,2)


def test_f3_deduplicates_camera_measurements_but_keeps_f2_default():
    from ridgeback_leader_follow.fusion import Measurement,PeopleFusion
    measurements=[Measurement(1.,0.,.01,.9,'camera',(1.,0.),1),
                  Measurement(1.1,0.,.01,.3,'camera',(1.,0.),2)]
    assert len(PeopleFusion().update(0.,measurements))==2
    assert len(PeopleFusion(appearance_weight=.75).update(0.,measurements))==1


def test_raw_support_never_creates_people_and_is_tightly_gated():
    from ridgeback_leader_follow.fusion import Measurement,PeopleFusion
    fusion=PeopleFusion(appearance_weight=.75)
    support=lambda x:Measurement(x,0.,.09,.5,'support')
    assert fusion.update(0.,[support(1.)])==[]
    tracks=fusion.update(.05,[Measurement(1.,0.,.01,.9,'camera',(1.,0.),1)])
    identity=tracks[0].identity
    tracks=fusion.update(.1,[support(1.1)])
    assert len(tracks)==1 and tracks[0].identity==identity and not tracks[0].appearance
    assert fusion.update(.15,[support(1.7)])==[]
    baseline=PeopleFusion()
    assert baseline.update(0.,[support(1.)])==[]


def test_f3_track_memory_survives_occlusion_without_publishing_predictions():
    from ridgeback_leader_follow.fusion import Measurement,PeopleFusion
    fusion=PeopleFusion(max_age=3.,appearance_weight=.75)
    identity=fusion.update(0.,[Measurement(1.,0.,.01,.9,'camera',(1.,0.),1)])[0].identity
    assert fusion.update(1.,[])==[]
    restored=fusion.update(1.5,[Measurement(1.,0.,.01,.9,'camera',(1.,0.),7)])
    assert len(restored)==1 and restored[0].identity==identity
    assert fusion.update(5.,[])==[] and not fusion.tracks
