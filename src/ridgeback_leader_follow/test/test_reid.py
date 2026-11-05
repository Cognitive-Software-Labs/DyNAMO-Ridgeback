import numpy as np
import pytest
from ridgeback_leader_follow.benchmarking.reid import person_crops, AppearanceEncoder, RecognitionScore


def test_crops_clip_reject_invalid_and_preserve_rgb():
    image = np.zeros((40, 60, 3), np.uint8)
    image[:, :, 0] = 200
    crops = person_crops(image, [(4, [-10, 5, 30, 35]), (5, [5, 5, 4, 4]),
                                (6, [np.nan, 0, 20, 20])])
    assert len(crops) == 1
    assert crops[0].rgb.shape == (30, 30, 3)
    assert crops[0].rgb[0, 0].tolist() == [200, 0, 0]
    with pytest.raises(ValueError):
        person_crops(image, [(1, [0, 0, 20, 20]), (1, [0, 0, 20, 20])])


def test_encoder_normalizes_and_never_calls_inference_for_empty_batch():
    calls = []
    def infer(images):
        calls.append(len(images))
        return [[3., 4.]] * len(images)
    encoder = AppearanceEncoder('dinov2', infer, 2)
    assert encoder.encode([]) == {}
    crops = person_crops(np.zeros((20, 20, 3), np.uint8), [(7, [0, 0, 20, 20])])
    np.testing.assert_allclose(encoder.encode(crops)[7], [.6, .8])
    assert calls == [1]


def test_score_penalizes_rejections_and_absent_leader_false_matches():
    score = RecognitionScore()
    score.update(7, {7: 'leader'}, 'leader', leader_eligible=True)
    score.update(8, {8: 'other'}, 'leader', leader_eligible=True)
    score.update(None, {}, 'leader', leader_eligible=True)
    score.update(8, {8: 'other'}, 'leader', leader_eligible=False)
    report = score.report()
    assert report['leader_pick_accuracy'] == pytest.approx(1/3)
    assert report['wrong'] == report['rejected'] == report['absent_false_accepts'] == 1
    assert RecognitionScore().report()['leader_pick_accuracy'] is None


def test_benchmark_scores_camera_exit_without_passing_null_boxes_to_iou(tmp_path,monkeypatch):
    import json
    from types import SimpleNamespace
    pytest.importorskip('torch')
    from ridgeback_leader_follow.benchmarking import reid_benchmark
    bag=SimpleNamespace(schedule=[(.05,.05)],camera=lambda at:(at,
        np.zeros((32,16,3),np.uint8),np.ones((32,16)),
        np.array([[10.,0.,8.],[0.,10.,16.],[0.,0.,1.]]),np.eye(4)))
    monkeypatch.setattr(reid_benchmark,'SensorBag',lambda run:bag)
    detections=tmp_path/'detections.jsonl';annotations=tmp_path/'annotations.jsonl'
    detections.write_text(json.dumps({'scenario_time':.05,'sim_time':.05,
        'input_error':None,'boxes':[[0,0,16,32,.9]]})+'\n')
    annotations.write_text(json.dumps({'scenario_time':.05,'people':[
        {'id':'lf_leader','role':'leader','box':None,'visible_fraction':0.,'distance_m':2.},
        {'id':'other','role':'bystander','box':[0,0,16,32],'visible_fraction':1.,'distance_m':1.}]})+'\n')
    encoder=SimpleNamespace(name='osnet-ain',dimension=2,provenance={'device':'cpu'},
        encode=lambda crops:{crop.track_id:np.array([1.,0.]) for crop in crops})
    report=reid_benchmark.benchmark(tmp_path,detections,annotations,tmp_path/'result',encoder)
    assert report['all_visible']['leader_eligible_frames']==0
    assert report['all_visible']['absent_frames']==1


def test_runtime_rejects_horizontally_truncated_appearance_crops():
    image=np.zeros((40,60,3),np.uint8)
    detections=[(1,[0,0,20,40]),(2,[10,0,40,40]),(3,[40,0,60,40])]
    assert len(person_crops(image,detections))==3
    assert [crop.track_id for crop in person_crops(image,detections,horizontal_margin=2)]==[2]
