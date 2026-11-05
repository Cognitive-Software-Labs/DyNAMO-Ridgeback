import numpy as np
from ridgeback_leader_follow.benchmarking.image_metrics import box_iou,detection_metrics


def test_iou_empty_and_overlap():
    assert box_iou([],[[0,0,1,1]]).shape == (0,1)
    assert np.allclose(box_iou([[0,0,2,2]],[[1,0,3,2]]),1/3)


def test_detection_strata_count_misses_and_false_positives():
    gt=[{'people':[{'id':'a','box':[0,0,2,2],'distance_m':3.,'visible_fraction':.8}]}]
    result=detection_metrics(gt,[{'boxes':[[10,10,20,20,.9]]}])
    assert (result['TP'],result['FP'],result['FN']) == (0,1,1)
    assert result['strata']['2-4m']['recall'] == 0
