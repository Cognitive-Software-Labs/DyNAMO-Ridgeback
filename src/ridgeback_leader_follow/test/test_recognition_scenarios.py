import math
import pytest
from ridgeback_leader_follow.recognition_scenarios import load_recognition_scenario
from ridgeback_leader_follow.crowd_scenarios import SCENARIOS


def test_supplementary_camera_exit_keeps_original_f1_matrix_unchanged():
    assert 'leader_out_of_view' not in SCENARIOS
    scene=load_recognition_scenario('leader_out_of_view')
    leader,other=scene.people
    assert leader.sample(0)[:2]==leader.sample(3)[:2]==(2.,0.)
    assert leader.sample(7)[:2]==leader.sample(12)[:2]==(2.,2.3)
    assert leader.sample(16)[:2]==(2.,0.)
    # Production pinhole half-angle is atan(320/384). Actual visibility must
    # still be confirmed from recorded extrinsics and mesh annotations.
    assert math.atan2(2.3,2.) > math.atan2(320,384)
    for i in range(401):
        at=i*.05;x,y,_=leader.sample(at);ox,oy,_=other.sample(at)
        assert abs(y)<=2.3
        assert math.hypot(x-ox,y-oy)>1
    with pytest.raises(ValueError):load_recognition_scenario('unknown')
