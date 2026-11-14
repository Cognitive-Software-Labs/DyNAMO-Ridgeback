"""Synthetic ROS/MCAP, fake inference, parity, persistence and selection proofs."""
from __future__ import annotations

from copy import deepcopy
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_msgs.msg import TFMessage

from ridgeback_autonomy.benchmarking import hardware_bag_source as source
from ridgeback_autonomy.benchmarking import hardware_bag_validation as bridge
from ridgeback_localization.core.image_utils import convert_depth_to_meters_message
from ridgeback_localization.core.mask import region_from_bbox
from ridgeback_common.models import Detection, DetectionBatch
from ridgeback_common.miss_reason import MissReason
from ridgeback_localization.core.intrinsics import CameraIntrinsics
from ridgeback_localization.core.isolation_2d import build_isolation_2d
from ridgeback_localization.core.isolation_3d import build_isolation_3d, camera_floor_geometry
from ridgeback_localization.measurement_pipeline import fill_path_measurements

T = 10_000_000_000
W, H = 40, 30
CAMERA_TO_BASE = np.array([[0,0,1],[-1,0,0],[0,-1,0]], dtype=float)


def header(msg, stamp=T, frame='camera'):
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
    msg.header.frame_id = frame
    return msg


def image(stamp=T, *, depth=False, big=False, padding=0):
    msg = header(Image(), stamp)
    msg.width, msg.height = W, H
    msg.encoding = '16UC1' if depth else 'rgb8'
    msg.is_bigendian = big
    if depth:
        pixels = np.full((H,W), 2000, dtype='>u2' if big else '<u2')
    else:
        pixels = np.full((H,W,3), 100, dtype=np.uint8)
    msg.step = len(pixels[0].tobytes())+padding
    msg.data = b''.join(row.tobytes()+bytes(padding) for row in pixels)
    return msg


def info(stamp=T):
    msg = header(CameraInfo(), stamp)
    msg.width, msg.height = W,H
    msg.k = [100.,0.,W/2,0.,100.,H/2,0.,0.,1.]
    msg.r = np.eye(3).ravel().tolist()
    msg.p = [100.,0.,W/2,0.,0.,100.,H/2,0.,0.,0.,1.,0.]
    msg.distortion_model = 'plumb_bob'
    msg.d = [.1,0.,0.,0.,0.]
    return msg


def transform(parent, child, stamp=T, *, camera=False, x=0.):
    msg = header(TransformStamped(), stamp, parent)
    msg.child_frame_id = child
    if camera:
        # quaternion for optical -> FLU
        msg.transform.rotation.x = -.5
        msg.transform.rotation.y = .5
        msg.transform.rotation.z = -.5
        msg.transform.rotation.w = .5
    else:
        msg.transform.rotation.w = 1.
    msg.transform.translation.x = x
    msg.transform.translation.z = .8 if camera else .3
    return msg


def scan(stamp=T):
    msg = header(LaserScan(), stamp, 'laser')
    msg.angle_min = -.2
    msg.angle_increment = .01
    msg.angle_max = .2
    msg.range_min, msg.range_max = .05,30.
    msg.ranges = [2.]*41
    return msg


def selection(name=source.FIRST_BAG):
    return {'bag': name, 'interval_ns': [T,T+400_000_000], 'topics': {},
            'reference': {'id': 'nominal.json', 'document': {'x_F_m':2.,'y_F_m':-.2},
                          'assumed_unchanged': True, 'final_scoring_eligible': False}}


def messages(*, stereo=True, scans=True, calibration=True, tf=True):
    rows = []
    if tf:
        rows.append(('static', T-1_000_000_000, TFMessage(transforms=[
            transform('base_link','camera', camera=True), transform('base_link','laser')])))
    if calibration:
        rows.append(('info',T-100,info(T-100)))
    rows += [('rgb',T-1,image(T-100)), ('rgb',T,image()),
             ('rgb',T+1,image()), # duplicate stamp
             ('rgb',T+50_000_000,image(T+50_000_000)),
             ('rgb',T+100_000_000,image(T+100_000_000)),
             ('rgb',T+300_000_000,image(T+300_000_000)), # empty bucket 2
             ('rgb',T+400_000_000,image(T+400_000_000))] # outside interval
    if stereo:
        # Receive later, exact header match required. Equal-stamp duplicate
        # picks source order. No match for selected ordinal 1.
        rows += [('depth',T+450_000_000,image(T,depth=True,big=True,padding=1)),
                 ('depth',T+451_000_000,image(T,depth=True)),
                 ('depth',T+452_000_000,image(T+300_000_000,depth=True)),
                 ('depth',T+453_000_000,image(T+100_000_001,depth=True))]
    if scans:
        rows += [('scan',T+500_000_000,scan(T+50_000_000)),
                 ('scan',T+501_000_000,scan(T-50_000_000)), # tie: earlier header wins, future receive
                 ('scan',T+502_000_000,scan(T+350_000_001))] # 50 ms+1 miss
    return rows


class FakeSegmenter:
    def __init__(self, *, empty=False):
        self.calls = 0
        self.empty = empty
    def segment_boxes(self, rgb, boxes, **kwargs):
        self.calls += 1
        if self.empty:
            return [None for box in boxes]
        result = []
        for x1,y1,x2,y2 in boxes:
            mask = np.zeros(rgb.shape[:2], bool)
            mask[int(y1):int(y2),int(x1):int(x2)] = True
            result.append(mask)
        return result


class FakeProducers:
    def __init__(self, *, empty=False, empty_mask=False):
        self.detection_calls = self.mono_calls = 0
        self.segmenter = FakeSegmenter(empty=empty_mask)
        self.empty = empty
    def detect(self, rgb):
        self.detection_calls += 1
        return [] if self.empty else [dict(bbox_xyxy=[11,3,27,26], label='humanoid robot', score=.8),
                                       dict(bbox_xyxy=[1,2,9,25], label='humanoid robot',score=.5)]
    def monocular(self,rgb):
        self.mono_calls += 1
        return np.full(rgb.shape[:2],3.,np.float32)


def settings():
    return dict(bridge.effective_settings(), monocular_usable_max_m=18.)


def assembled(tmp_path, **kwargs):
    return list(source.assemble_events(messages(**kwargs), selection(),tmp_path/'index'))


def test_sampling_boundaries_occupied_buckets_duplicates_and_speed(tmp_path):
    first = assembled(tmp_path/'a')
    second = assembled(tmp_path/'b')
    assert [e['stamp_ns'] for e in first] == [T,T+100_000_000,T+300_000_000]
    assert [e['event_id'] for e in first] == [e['event_id'] for e in second]
    assert first[0]['sampling'] == {'anchor_ns':T,'duplicate_rgb_stamps':1}
    assert first[0]['rgb_receive_ns'] == T
    sampler = source.Sampler()
    assert sampler.keep(T)
    with pytest.raises(ValueError,match='Out-of-order'):
        sampler.keep(T-1)
    bad = image()
    bad.header.stamp.nanosec = 1_000_000_000
    with pytest.raises(ValueError,match='Malformed'):
        source.stamp_ns(bad.header)


def test_future_depth_exact_pair_scan_tie_boundary_and_preinterval_static(tmp_path):
    events = assembled(tmp_path)
    event = events[0]
    assert event['depth_identity']['stamp_ns'] == T
    assert event['depth_identity']['duplicate_matches'] == 1
    np.testing.assert_allclose(event['depth_m'],2.)
    assert event['scan']['stamp_ns'] == T-50_000_000
    assert event['scan_delta_ns'] == -50_000_000
    assert events[1]['depth_m'] is None
    assert events[1]['scan_delta_ns'] == -50_000_000
    assert events[2]['scan'] is None
    np.testing.assert_allclose(event['camera_rotation'],CAMERA_TO_BASE)
    assert event['context']['camera_tf'][0]['receive_ns'] < T
    assert event['intrinsics']['d'][0] == .1
    assert events[2]['depth_m'] is not None


@pytest.mark.parametrize('missing,reason', [('calibration','NO_CAMERA_INFO'),('tf','TF_MISS_EXTRINSIC')])
def test_missing_context_is_explicit(tmp_path, missing, reason):
    event = assembled(tmp_path, **{missing:False})[0]
    evidence = bridge.freeze_inference(event,FakeProducers(),settings())
    result = bridge.measure_evidence(evidence,settings())
    assert {d['status_name'] for row in result['rows'] for d in row['detections']} == {reason}


@pytest.mark.parametrize('field,value', [('k',[0.]*9),('width',99),('header',None)])
def test_malformed_calibration_is_not_used(tmp_path, field,value):
    rows = messages()
    if field == 'header':
        rows[1][2].header.frame_id = 'wrong_optical'
    else:
        setattr(rows[1][2],field,value)
    event = list(source.assemble_events(rows,selection(),tmp_path/'index'))[0]
    assert event.get('intrinsics') is None
    assert 'camera_info' in event['missing_inputs']


def test_tf_dynamic_brackets_no_extrapolation_and_conflicting_static():
    tf = source.RecordedTF()
    tf.add(TFMessage(transforms=[transform('base','moving',T,x=0.)]),T,0,False)
    tf.add(TFMessage(transforms=[transform('base','moving',T+100,x=1.)]),T+100,1,False)
    matrix,chain = tf.resolve('base','moving',T+50)
    assert matrix[0,3] == .5 and len(chain) == 2
    assert tf.resolve('base','moving',T-1)[0] is None
    tf.add(TFMessage(transforms=[transform('base','fixed',x=0.)]),T,2,True)
    tf.add(TFMessage(transforms=[transform('base','fixed',x=1.)]),T,3,True)
    with pytest.raises(ValueError,match='Conflicting'):
        tf.resolve('base','fixed',T)
    invalid = transform('base','bad')
    invalid.transform.rotation.w = 0.
    with pytest.raises(ValueError,match='quaternion'):
        source.rigid_matrix(invalid.transform)


@pytest.mark.parametrize('big', [False,True])
def test_depth_padded_odd_stride_endian_units_and_no_return(big):
    msg = Image()
    msg.height,msg.width = 2,3
    msg.encoding,msg.step,msg.is_bigendian = '16UC1',7,big
    dtype = '>u2' if big else '<u2'
    msg.data = np.array([0,1000,65535],dtype=dtype).tobytes()+b'x'+np.array([2000,2500,3000],dtype=dtype).tobytes()+b'y'
    np.testing.assert_allclose(convert_depth_to_meters_message(msg),[[0.,1.,65.535],[2.,2.5,3.]],rtol=1e-6)
    msg.encoding,msg.width,msg.height,msg.step = '32FC1',3,1,12
    msg.data = np.array([0,np.nan,np.inf],dtype='>f4' if big else '<f4').tobytes()
    result = convert_depth_to_meters_message(msg)
    assert result[0,0] == 0 and np.isnan(result[0,1]) and np.isinf(result[0,2])
    msg.data = b'bad'
    with pytest.raises(ValueError,match='data length'):
        convert_depth_to_meters_message(msg)


def test_shared_calls_ten_paths_empty_events_and_independent_failures(tmp_path):
    events = assembled(tmp_path)
    producers = FakeProducers()
    results = [bridge.measure_evidence(bridge.freeze_inference(e,producers,settings()),settings()) for e in events]
    assert producers.detection_calls == producers.mono_calls == producers.segmenter.calls == 3
    assert len(results[0]['rows']) == 10
    assert sum('polar_profiling' in r['path'] for r in results[0]['rows']) == 2
    assert [d['score'] for d in results[0]['rows'][0]['detections']] == [.8,.5]
    for row in results[1]['rows']:
        if row['path'].endswith('__stereo'):
            assert {d['status_name'] for d in row['detections']} == {'NO_DEPTH_FRAME'}
        elif row['path'].endswith('__monocular'):
            assert all(d['radial_F_m'] is not None for d in row['detections'])
    for row in results[2]['rows']:
        if row['path'].endswith('__scan'):
            assert {d['status_name'] for d in row['detections']} == {'NO_SCAN'}
        else:
            assert all(d['radial_F_m'] is not None for d in row['detections'])
    empty = FakeProducers(empty=True)
    no_detection = bridge.measure_evidence(bridge.freeze_inference(events[0],empty,settings()),settings())
    assert all(row['empty_detections'] and row['detections'] == [] for row in no_detection['rows'])
    assert empty.detection_calls == empty.mono_calls == 1 and empty.segmenter.calls == 0
    summary = bridge.summarize_bag(selection(),[no_detection])
    assert all(p['selected_events'] == p['empty_detection_events'] == 1 for p in summary['paths'].values())
    assert summary['reference']['assumed_unchanged'] and summary['final_scoring_eligible'] is False


def test_segmentation_failure_no_box_fallback(tmp_path):
    event = assembled(tmp_path)[0]
    evidence = bridge.freeze_inference(event,FakeProducers(empty_mask=True),settings())
    result = bridge.measure_evidence(evidence,settings())
    for row in result['rows']:
        if '__slimsam__' in row['path']:
            assert {d['status_name'] for d in row['detections']} == {'MASK_EMPTY_SEGMENTATION'}
            assert all(d['radial_F_m'] is None for d in row['detections'])
        elif row['path'].endswith('__stereo'):
            assert all(d['radial_F_m'] is not None for d in row['detections'])


def test_adapter_matches_direct_measurement_and_off_axis_translation(tmp_path):
    from ridgeback_autonomy.benchmarking.replay import replay_scan_points, polar_isolation_settings
    from ridgeback_localization.core.vehicle_frame import ROBOT_FRONT_OFFSET_M
    event = assembled(tmp_path)[0]
    evidence = bridge.freeze_inference(event,FakeProducers(),settings())
    rows = {r['path']:r for r in bridge.measure_evidence(evidence,settings())['rows']}
    args = settings()['arguments']
    intr = CameraIntrinsics(**{k:event['intrinsics'][k] for k in ('fx','fy','cx','cy','width','height')})
    rotation,translation = np.array(event['camera_rotation']),np.array(event['camera_translation'])
    height,down = camera_floor_geometry(rotation,translation,.026)
    for mask in ('box','slimsam'):
        regions = [o.region for o in bridge.mask_outcomes(evidence,mask)]
        for depth_source, depth in [('stereo',event['depth_m']),('monocular',evidence['monocular_m'])]:
            batch = DetectionBatch(W,H,[Detection(tuple(d['bbox_xyxy']),d['label'],d['score']) for d in evidence['event']['detections']])
            fill_path_measurements(batch,regions,intr,depth,replay_scan_points(event)[0],camera_rotation=rotation,
                camera_translation=translation,front_offset_m=.465,
                isolation_2d=build_isolation_2d(args['isolation_2d']),
                isolation_3d=build_isolation_3d(args['isolation_3d'],height,down),
                polar_isolation=polar_isolation_settings(args),depth_max=18. if depth_source=='monocular' else math.inf)
            for estimator in ('projective_ranging','euclidean_reconstruction','polar_profiling'):
                source_name = 'scan' if estimator == 'polar_profiling' else depth_source
                for actual,direct in zip(rows[f'{estimator}__{mask}__{source_name}']['detections'],batch.detections):
                    assert actual['status'] == getattr(direct,f'{estimator}_status')
                    assert actual['radial_F_m'] == getattr(direct,f'{estimator}_distance_m')
                    if actual['radial_F_m'] is not None:
                        x,y = actual['base_planar_point_m']
                        assert actual['radial_F_m'] == pytest.approx(math.hypot(x-.465,y))
                        if abs(y) > .01:
                            assert actual['radial_F_m'] != pytest.approx(math.hypot(x,y)-.465,abs=1e-6)
    assert ROBOT_FRONT_OFFSET_M == .25


def test_atomic_payload_roundtrip_and_incomplete_marker(tmp_path):
    event = assembled(tmp_path/'raw')[0]
    evidence = bridge.freeze_inference(event,FakeProducers(),settings())
    path = tmp_path/'cache.json'
    bridge.save_payload(path,evidence)
    loaded = bridge.load_payload(path)
    np.testing.assert_array_equal(loaded['event']['rgb'],event['rgb'])
    assert bridge.measure_evidence(loaded,settings()) == bridge.measure_evidence(evidence,settings())
    path.with_suffix('.npz').write_bytes(b'incomplete')
    with pytest.raises(ValueError,match='Invalid hardware payload'):
        bridge.load_payload(path)
    orphan = tmp_path/'orphan.npz'
    orphan.write_bytes(b'crash before JSON commit')
    assert not orphan.with_suffix('.json').exists()


def fake_job(tmp_path, count=1):
    job = {'schema':'dynamo-hardware-job/1','selection':{'session':str(tmp_path/'raw'),
        'bags':[selection(source.FIRST_BAG if i==0 else f'bag{i}') for i in range(count)]},
        'output':str(tmp_path/'output'),'settings':settings(),'paths':list(bridge.PATHS),'base_frame':'base_link'}
    job['identity'] = source.digest(job)
    return job


def test_resume_reuses_inference_completed_skip_and_incompatible_refusal(tmp_path):
    job = fake_job(tmp_path)
    producers = FakeProducers()
    n = {'calls':0}
    def factory(_):
        n['calls'] += 1
        return producers
    def fail_measure(*args):
        raise RuntimeError('interruption after inference commit')
    original = bridge.measure_evidence
    with pytest.MonkeyPatch.context() as patch:
        patch.setattr(bridge,'measure_evidence',fail_measure)
        with pytest.raises(RuntimeError,match='interruption'):
            bridge.run_job(job,validate=False,producers_factory=factory,message_reader=lambda *a:iter(messages()))
    root = Path(job['output'])
    assert bridge.read_json(root/'progress.json')['bags'][source.FIRST_BAG]['state'] == 'incomplete'
    assert producers.detection_calls == 1
    def no_raw(*args):
        raise AssertionError('cached sources must not reread raw bag')
    result = bridge.run_job(job,resume=True,validate=False,producers_factory=factory,message_reader=no_raw)
    assert result['bags'][source.FIRST_BAG]['state'] == 'complete'
    assert producers.detection_calls == producers.mono_calls == 3
    assert n['calls'] == 2
    summary = bridge.validate_completed(root/source.FIRST_BAG,job['identity'])
    assert len(summary['paths']) == 10
    assert len(summary['overlays']) == 3
    bridge.run_job(job,resume=True,validate=False,producers_factory=lambda *a:pytest.fail('completed inference'),message_reader=no_raw)
    changed = dict(job,device='cpu')
    with pytest.raises(ValueError,match='another frozen job'):
        bridge.run_job(changed,resume=True,validate=False)
    (root/source.FIRST_BAG/'events.jsonl').write_text('tampered')
    with pytest.raises(ValueError,match='hash mismatch'):
        bridge.validate_completed(root/source.FIRST_BAG,job['identity'])


def test_shared_model_failure_stops_batch_and_raw_fault_continues(tmp_path):
    job = fake_job(tmp_path,2)
    class Broken(FakeProducers):
        def monocular(self,rgb):
            raise RuntimeError('injected OOM')
    seen = []
    def reader(bag,topics):
        seen.append(bag.name)
        return iter(messages())
    with pytest.raises(bridge.SharedProcessingFault,match='OOM'):
        bridge.run_job(job,validate=False,producers_factory=lambda _:Broken(),message_reader=reader)
    assert seen == [source.FIRST_BAG]
    assert not (Path(job['output'])/source.FIRST_BAG/'complete.json').exists()
    job = fake_job(tmp_path/'next',2)
    def reader(bag,topics):
        if bag.name == source.FIRST_BAG:
            raise ValueError('corrupt single input')
        return iter(messages())
    result = bridge.run_job(job,validate=False,producers_factory=lambda _:FakeProducers(),message_reader=reader)
    assert result['bags'][source.FIRST_BAG]['state'] == 'input_failed'
    assert result['bags']['bag1']['state'] == 'complete'


def create_session(root, count=48):
    import yaml
    (root/'quality').mkdir(parents=True)
    (root/'ground-truth').mkdir()
    (root/'setup').mkdir()
    (root/'setup/sensor-contract.json').write_text('{}')
    (root/'ground-truth/reference.json').write_text(json.dumps({'x_F_m':2.,'y_F_m':-.2,'position_reference':'old.json','source':'assumed unchanged through turn'}))
    manifest,segments = [],[]
    names = [source.FIRST_BAG]+[f'{i+3:02}_arrangement' for i in range(count-1)]+sorted(source.REJECTED)
    for name in names:
        bag = root/'bags'/name
        bag.mkdir(parents=True)
        (bag/'raw.mcap').write_bytes(b'synthetic receipt fixture only')
        meta = {'rosbag2_bagfile_information':{'storage_identifier':'mcap','relative_file_paths':['raw.mcap'],
                'topics_with_message_count':[{'topic_metadata':{'name':'/ns'+suffix,'type':source.EXPECTED_TYPES[kind]}}
                                            for kind,suffix in source.SUFFIXES.items()]}}
        (bag/'metadata.yaml').write_text(yaml.safe_dump(meta))
        receipt = {'files':{p.name:{'bytes':p.stat().st_size,'sha256':source.file_hash(p)} for p in bag.iterdir()}}
        (root/'quality'/f'{name}.receipt.json').write_text(json.dumps(receipt))
        manifest.append({'bag':name,'row_id':name,'capture_complete':name not in source.REJECTED,
                         'ground_truth_record':'ground-truth/reference.json','file_inventory_and_sha256_receipt':f'quality/{name}.receipt.json',
                         'scene_accepted':False})
        segments.append({'record_type':'attempt','bag_name':name,'row_id':name,'settled_interval_start_ns':T,
                         'settled_interval_end_ns':T+20_000_000_000,'benchmark_ids':['alias_a','alias_b']})
    segments.append({'record_type':'coordinate_correction','bag_name':source.FIRST_BAG,'x_F_m':2.1,'y_F_m':.2})
    (root/'segments.jsonl').write_text('\n'.join(json.dumps(s) for s in segments))
    (root/'quality/recording-plan-closeout-audit-20261101.json').write_text(json.dumps({'bag_manifest':manifest}))
    return root


def test_selection_48_unique_excludes_rejected_starts02_aliases_not_trials(tmp_path):
    root = create_session(tmp_path/'session')
    selected = source.select_session(root)
    assert len(selected['bags']) == len({b['bag'] for b in selected['bags']}) == 48
    assert selected['bags'][0]['bag'] == source.FIRST_BAG
    assert not source.REJECTED.intersection(b['bag'] for b in selected['bags'])
    first = selected['bags'][0]
    assert first['reference']['nominal_position'] == {'x_F_m':2.1,'y_F_m':.2}
    assert first['reference']['assumed_unchanged']
    (root/'bags'/source.FIRST_BAG/'raw.mcap').write_bytes(b'wrong size')
    with pytest.raises(ValueError,match='receipt size'):
        source.select_session(root)


def test_real_metadata_selection_only_no_raw_read():
    # Optional local acceptance: reads JSON/YAML, receipts and file stats only.
    root = Path('/home/stefi/ScenesForDetectionTesting/20261029T141724Z-distance-benchmark-raw')
    if not root.is_dir():
        pytest.skip('operator session is not present')
    selected = source.select_session(root)
    assert len(selected['bags']) == 48
    assert selected['bags'][0]['bag'] == source.FIRST_BAG
    assert next(s for s in selected['bags'] if s['bag']=='34_P01-48_attempt1')['reference']['assumed_unchanged']
    assert selected['bags'][-1]['reference']['nominal_position']['y_F_m'] == .01


def test_synthetic_mcap_forward_reader(tmp_path):
    import rosbag2_py
    from rclpy.serialization import serialize_message
    bag = tmp_path/'mcap'
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='mcap'),rosbag2_py.ConverterOptions('cdr','cdr'))
    topics = {k:'/ns'+v for k,v in source.SUFFIXES.items()}
    for index,(kind,topic) in enumerate(topics.items()):
        writer.create_topic(rosbag2_py.TopicMetadata(id=index,name=topic,type=source.EXPECTED_TYPES[kind],serialization_format='cdr'))
    for kind,receive,msg in messages():
        writer.write(topics[kind],serialize_message(msg),receive)
    del writer
    events = list(source.assemble_events(source.ros_messages(bag,topics),selection(),tmp_path/'index'))
    assert len(events) == 3
    np.testing.assert_allclose(events[0]['depth_m'],2.)
    assert events[0]['scan_delta_ns'] == -50_000_000


def test_cached_remeasurement_and_reference_revision_no_inference(tmp_path, monkeypatch):
    job = fake_job(tmp_path)
    bridge.run_job(job,validate=False,producers_factory=lambda _:FakeProducers(),message_reader=lambda *a:iter(messages()))
    output = Path(job['output'])
    before = bridge.receipt_tree(output)
    overrides = tmp_path/'references.json'
    overrides.write_text(json.dumps({source.FIRST_BAG:{'x_F_m':2.1,'y_F_m':.15,'assumed_unchanged':True}}))
    monkeypatch.setattr(bridge,'ProductionProducers',lambda *a:pytest.fail('no inference'))
    monkeypatch.setattr(source,'ros_messages',lambda *a:pytest.fail('no raw reads'))
    derived = bridge.remeasure_cached(output,tmp_path/'derived',references=overrides)
    assert derived['final_scoring_eligible'] is False
    assert bridge.receipt_tree(output) == before
    summary = bridge.read_json(tmp_path/'derived'/source.FIRST_BAG/'summary.json')
    assert summary['reference']['document']['x_F_m'] == 2.1
    assert summary['reference']['assumed_unchanged']
    assert not summary['final_scoring_eligible']
    with pytest.raises(ValueError,match='empty output'):
        bridge.remeasure_cached(output,tmp_path/'derived')


def test_job_validation_freezes_models_sources_settings_and_code(tmp_path, monkeypatch):
    session = create_session(tmp_path/'session')
    cache = tmp_path/'hub'
    for role,(name,revision,kind) in bridge.MODELS.items():
        path = cache/('models--'+name.replace('/','--'))/'snapshots'/revision
        path.mkdir(parents=True)
        config = {'model_type':kind}
        if role == 'monocular':
            config.update(depth_estimation_type='metric',max_depth=20.)
        (path/'config.json').write_text(json.dumps(config))
        (path/'model.safetensors').write_bytes(b'fake weights, never loaded')
    job = bridge.describe(session,tmp_path/'out',cache,device='cpu')
    assert job['settings']['front_offset_m'] == .465
    assert job['settings']['monocular_usable_max_m'] == 18.
    bridge.validate_job(job)
    changed = deepcopy(job)
    changed['settings']['front_offset_m'] = .48
    with pytest.raises(ValueError,match='identity/schema'):
        bridge.validate_job(changed)
    code = bridge.code_identity()
    with monkeypatch.context() as patch:
        patch.setattr(bridge,'code_identity',lambda:dict(code,signature='other'))
        with pytest.raises(ValueError,match='implementation changed'):
            bridge.validate_job(job)
    weights = Path(job['models']['owl']['path'])/'model.safetensors'
    weights.write_bytes(b'changed')
    with pytest.raises(ValueError,match='snapshot changed'):
        bridge.validate_job(job)
    with pytest.raises(ValueError,match='explicit cpu'):
        bridge.describe(session,tmp_path/'out',cache,device='auto')


def test_cli_anchors_paths_and_describe_never_constructs_producers(tmp_path, monkeypatch):
    from ridgeback_autonomy.benchmarking import target_hardware_bag_validation as cli
    from ridgeback_autonomy.benchmarking import target_offline_replay_benchmark as old
    monkeypatch.setattr(old,'_source_repository',lambda:tmp_path)
    monkeypatch.setattr(bridge,'describe',lambda session,output,cache,**kw:{'identity':'test','selection':{'bags':[]},'paths':[],
                      'settings':{},'output':output,'session':session})
    monkeypatch.setattr(bridge,'ProductionProducers',lambda *a:pytest.fail('describe has no model construction'))
    assert cli.main(['describe','--session','inputs','--output','artifacts/benchmarks/new','--job','artifacts/benchmark-jobs/test.json']) == 0
    job = bridge.read_json(tmp_path/'artifacts/benchmark-jobs/test.json')
    assert job['session'] == str(tmp_path/'inputs')
    assert job['output'] == str(tmp_path/'artifacts/benchmarks/new')
    assert cli.main(['describe','--session','inputs','--output','artifacts/benchmarks/new','--job','artifacts/benchmark-jobs/test.json']) == 1


def test_scan_duplicate_stamp_source_order_and_late_static_context(tmp_path):
    rows = messages()
    static = rows.pop(0)
    later_duplicate = scan(T-50_000_000)
    later_duplicate.ranges = [4.]*41
    rows += [('scan',T+600_000_000,later_duplicate), ('static',T+700_000_000,static[2])]
    events = list(source.assemble_events(rows,selection(),tmp_path/'index'))
    assert events[0]['scan']['ranges'][0] == 2.
    assert events[0]['scan']['stamp_ns'] == T-50_000_000
    np.testing.assert_allclose(events[0]['camera_rotation'],CAMERA_TO_BASE)


def test_interrupted_npz_without_commit_is_retried_visibly(tmp_path, monkeypatch):
    job = fake_job(tmp_path)
    producers = FakeProducers()
    save = bridge.save_payload
    def crash_before_marker(path,value):
        if Path(path).parent.name == 'inference':
            Path(path).parent.mkdir(exist_ok=True)
            Path(path).with_suffix('.npz').write_bytes(b'uncommitted write')
            raise OSError('injected interrupted write')
        return save(path,value)
    with monkeypatch.context() as patch:
        patch.setattr(bridge,'save_payload',crash_before_marker)
        with pytest.raises(OSError,match='interrupted'):
            bridge.run_job(job,validate=False,producers_factory=lambda _:producers,message_reader=lambda *a:iter(messages()))
    assert producers.detection_calls == 1
    bridge.run_job(job,resume=True,validate=False,producers_factory=lambda _:producers,
                   message_reader=lambda *a:pytest.fail('source cache must survive'))
    assert producers.detection_calls == 4 # current uncommitted event repeated, explicit retry
    root = Path(job['output'])/source.FIRST_BAG
    assert bridge.read_json(root/'source.json')['source_index_attempt'] == 1
    bridge.validate_completed(root,job['identity'])


def test_storage_failure_during_source_index_stops_instead_of_failing_every_bag(tmp_path, monkeypatch):
    import errno
    job = fake_job(tmp_path,2)
    def full(*args):
        raise OSError(errno.ENOSPC,'injected full output disk')
    seen = []
    def reader(bag,topics):
        seen.append(bag.name)
        return iter(messages())
    monkeypatch.setattr(bridge,'save_payload',full)
    with pytest.raises(bridge.SharedProcessingFault,match='storage/I/O failure'):
        bridge.run_job(job,validate=False,producers_factory=lambda _:pytest.fail('no inference'),message_reader=reader)
    assert seen == [source.FIRST_BAG]
    state = bridge.read_json(Path(job['output'])/'progress.json')['bags'][source.FIRST_BAG]
    assert state['state'] == 'incomplete' and state['failure_type'] == 'SharedProcessingFault'
