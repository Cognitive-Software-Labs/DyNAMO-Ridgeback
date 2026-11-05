import json
import math
import pytest

from ridgeback_leader_follow.benchmarking.scoring import Position, score_frames
from ridgeback_leader_follow.benchmarking.adapters import ScanInput, ScanClustering
from ridgeback_leader_follow.benchmarking.evaluate import evaluate
from ridgeback_leader_follow.benchmarking.tracking import ConstantVelocityTracker, transform_points


def test_switches_and_loss_across_occlusion():
    truth = [Position('leader',1.,0.,'leader')]
    result = score_frames([(0.,truth,[Position('1',1.,0.)]),
                           (1.,truth,[]),(2.,truth,[Position('2',1.,0.)])])
    assert result['id_switches'] == 1
    assert result['leader_track_loss_events'] == 1
    assert result['fragmentations'] == 1
    assert result['false_negatives'] == 1


def test_invalid_frames_and_duplicate_ids_rejected():
    with pytest.raises(ValueError):
        score_frames([(0.,[],[Position('1',0,0),Position('1',1,0)])])
    with pytest.raises(ValueError):
        score_frames([(1.,[],[]),(0.,[],[])])
    with pytest.raises(ValueError):
        score_frames([])


def test_gated_assignment_does_not_force_distant_match():
    result = score_frames([(0.,[Position('a',0,0)],[Position('b',10,10)])],gate=.5)
    assert result['true_positives'] == 0
    assert result['false_negatives'] == result['false_positives'] == 1


def test_clustering_splits_invalid_beams_and_keeps_sensor_frame():
    scan = ScanInput(0.,'lidar',-.03,.01,(1.,1.,1.,math.inf,1.,1.,1.),.1,10.)
    result = ScanClustering().detect(scan)
    assert len(result) == 2
    assert result[0][1] < result[1][1]


def test_missing_prediction_frames_count_as_misses_and_no_overwrite(tmp_path):
    run = tmp_path/'run'; run.mkdir()
    (run/'manifest.json').write_text(json.dumps({'status':'complete','backend':'gz','scenario':{'name':'crossing'}}))
    (run/'ground_truth.csv').write_text('scenario_time,person,x,y,role,frame\n0,leader,1,0,leader,world\n1,leader,1,0,leader,world\n')
    predictions = tmp_path/'predictions.jsonl'
    predictions.write_text(json.dumps({'scenario_time':0,'frame':'world','people':[{'track_id':1,'x':1,'y':0}]})+'\n')
    output = tmp_path/'score.json'
    result = evaluate(run,predictions,output,'synthetic-test')
    assert result['recall'] == .5
    with pytest.raises(FileExistsError):
        evaluate(run,predictions,output,'synthetic-test')


def test_tracking_survives_short_gap_but_expires_stale_ids():
    tracker = ConstantVelocityTracker()
    first = tracker.update(0.,[(0.,0.)])[0]['track_id']
    assert tracker.update(.1,[(.1,0.)])[0]['track_id'] == first
    assert tracker.update(.2,[]) == []
    assert tracker.update(.3,[(.3,0.)])[0]['track_id'] == first
    assert tracker.update(1.,[(.3,0.)])[0]['track_id'] != first


def test_transform_applies_sensor_rotation_then_explicit_world_anchor():
    point = transform_points([(1.,0.)],(2.,0.,0.),(0.,0.,math.sqrt(.5),math.sqrt(.5)),
                             (10.,20.,0.,math.pi/2))[0]
    assert point == pytest.approx((9.,22.))
    with pytest.raises(ValueError):
        transform_points([(1.,0.)],(0.,0.,0.),(0.,0.,0.,0.),(0.,0.,0.,0.))


def test_replay_schedule_reads_timing_and_rejects_conflicts(tmp_path):
    from ridgeback_leader_follow.benchmarking.replay import load_schedule
    path = tmp_path/'ground_truth.csv'
    path.write_text('scenario_time,sim_time\n0.05,1.05\n0.05,1.05\n0.1,1.1\n')
    assert load_schedule(tmp_path) == [(.05,1.05),(.1,1.1)]
    path.write_text('scenario_time,sim_time\n0.05,1.05\n0.05,2.05\n')
    with pytest.raises(ValueError):
        load_schedule(tmp_path)


def test_recording_preserves_pre_epoch_scans_for_first_replay_tick(tmp_path):
    from sensor_msgs.msg import LaserScan
    from ridgeback_leader_follow.crowd_sensors import SensorEvidence
    class Writer:
        def __init__(self): self.messages = []
        def write(self,*args): self.messages.append(args)
    evidence = SensorEvidence.__new__(SensorEvidence)
    evidence.stats = {}; evidence.epoch = None; evidence.writer = Writer()
    evidence.output = tmp_path
    scan = LaserScan()
    scan.header.stamp.sec = 1
    scan.ranges = [1.,1.,1.]
    evidence.receive('front_scan','/scan',scan)
    assert len(evidence.writer.messages) == 1
    assert evidence.writer.messages[0][2] == 1_000_000_000


def test_drow_invalid_beams_use_upstream_padding():
    import numpy as np
    from ridgeback_leader_follow.benchmarking.lidar_detectors import DrowDetector
    seen=[]
    class Model:
        def set_laser_fov(self,fov):pass
        def __call__(self,ranges):
            seen.append(ranges.copy())
            return np.empty((0,2)),np.empty(0),None
    detector=DrowDetector.__new__(DrowDetector)
    detector.model=Model();detector.threshold=.5
    detector.last_stamp=None;detector.last_points=[]
    scan=ScanInput(1.,'laser',-.15,.1,(math.inf,math.nan,0.,.2),.05,10.)
    assert detector.detect(scan) == []
    assert np.allclose(seen[0],[29.99,29.99,29.99,.2])
    detector.detect(scan)
    assert len(seen) == 1
