"""Read-only raw hardware selection and a disk-indexed, single-pass ROS adapter.

No ROS nodes, playback, model inference, or simulator truth. The temporary index
keeps all depth candidates on disk until EOF, so late messages cannot turn an
exact match into a false miss. Only one decoded image pair is assembled at once.
"""
from __future__ import annotations

from collections import defaultdict, deque
import hashlib
import json
import math
from pathlib import Path
import sqlite3

import numpy as np
import yaml

FIRST_BAG = '02_P01-03_attempt2'
REJECTED = {'01_P01-03_attempt1', '25_P01-31_attempt1'}
PERIOD_NS = 100_000_000
SCAN_TOLERANCE_NS = 50_000_000
SUFFIXES = {
    'rgb': '/sensors/camera_0/color/image',
    'depth': '/sensors/camera_0/depth/image',
    'info': '/sensors/camera_0/color/camera_info',
    'scan': '/sensors/lidar2d_0/scan',
    'static': '/tf_static', 'tf': '/tf',
}
EXPECTED_TYPES = {'rgb': 'sensor_msgs/msg/Image', 'depth': 'sensor_msgs/msg/Image',
                  'info': 'sensor_msgs/msg/CameraInfo', 'scan': 'sensor_msgs/msg/LaserScan',
                  'static': 'tf2_msgs/msg/TFMessage', 'tf': 'tf2_msgs/msg/TFMessage'}


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def file_hash(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text())


def under(root, relative):
    path = (root / relative).resolve()
    if not path.is_relative_to(root):
        raise ValueError(f'Input path escapes session: {relative}')
    if not path.is_file():
        raise ValueError(f'Missing input: {path}')
    return path


def select_session(session, *, expected_count=48):
    root = Path(session).expanduser().resolve()
    audit_path = under(root, 'quality/recording-plan-closeout-audit-20261101.json')
    segment_path = under(root, 'segments.jsonl')
    audit = read_json(audit_path)
    segments = [json.loads(line) for line in segment_path.read_text().splitlines() if line.strip()]
    accepted = [entry for entry in audit['bag_manifest'] if entry['capture_complete'] is True]
    names = [entry['bag'] for entry in accepted]
    if len(names) != expected_count or len(set(names)) != len(names) or REJECTED.intersection(names):
        raise ValueError(f'Expected {expected_count} unique capture-complete bags, excluding {REJECTED}')
    if FIRST_BAG not in names:
        raise ValueError(f'Missing first accepted bag {FIRST_BAG}')
    result = []
    for entry in sorted(accepted, key=lambda e: (e['bag'] != FIRST_BAG, e['bag'])):
        name = entry['bag']
        attempts = [s for s in segments if s.get('record_type') == 'attempt' and s.get('bag_name') == name]
        if len(attempts) != 1:
            raise ValueError(f'{name}: expected one exact attempt association')
        attempt = attempts[0]
        start, end = (attempt[f'settled_interval_{key}_ns'] for key in ('start', 'end'))
        if not isinstance(start, int) or not isinstance(end, int) or start >= end:
            raise ValueError(f'{name}: invalid settled interval')
        truth_path = under(root, entry['ground_truth_record'])
        reference = read_json(truth_path)
        associations = [s for s in segments if s.get('bag_name', s.get('bag')) == name
                        or s.get('ground_truth_record', s.get('ground_truth')) == entry['ground_truth_record']]
        # Preserve chronological corrections separately; supplied current labels
        # remain available verbatim. Qualitative yaw is never converted to truth.
        corrections = [s for s in associations if s.get('record_type', '').startswith('coordinate_')]
        nominal = {k: reference[k] for k in ('x_F_m', 'y_F_m') if k in reference}
        for correction in corrections:
            nominal.update({k: correction[k] for k in ('x_F_m', 'y_F_m') if isinstance(correction.get(k), (int, float))})
        receipt_path = under(root, entry['file_inventory_and_sha256_receipt'])
        receipt = read_json(receipt_path)
        metadata_path = under(root, f'bags/{name}/metadata.yaml')
        metadata = yaml.safe_load(metadata_path.read_text())['rosbag2_bagfile_information']
        if metadata['storage_identifier'] != 'mcap':
            raise ValueError(f'{name}: expected MCAP storage')
        files = []
        for relative in ['metadata.yaml', *metadata['relative_file_paths']]:
            path = under(root, f'bags/{name}/{relative}')
            st = path.stat()
            saved = receipt['files'].get(relative)
            if saved is None or st.st_size != saved['bytes']:
                raise ValueError(f'{name}: receipt size mismatch for {relative}')
            if relative == 'metadata.yaml' and file_hash(path) != saved['sha256']:
                raise ValueError(f'{name}: metadata receipt hash mismatch')
            files.append({'path': str(path), 'bytes': st.st_size, 'mtime_ns': st.st_mtime_ns,
                          'receipt_sha256': saved['sha256']})
        topics = {}
        for kind, suffix in SUFFIXES.items():
            matches = [x['topic_metadata'] for x in metadata['topics_with_message_count']
                       if x['topic_metadata']['name'].endswith(suffix)]
            if len(matches) != 1 or matches[0]['type'] != EXPECTED_TYPES[kind]:
                raise ValueError(f'{name}: missing/ambiguous/wrong type {kind} topic')
            topics[kind] = matches[0]['name']
        result.append({'bag': name, 'row_id': entry['row_id'], 'attempt': attempt,
                       'interval_ns': [start, end], 'topics': topics, 'files': files,
                       'reference': {'id': entry['ground_truth_record'], 'document': reference,
                                     'nominal_position': nominal, 'corrections': corrections,
                                     'assumed_unchanged': 'assum' in json.dumps(reference).lower()
                                     or 'turn' in json.dumps(reference).lower(),
                                     'final_scoring_eligible': False},
                       'associations': associations,
                       'receipt': {'path': str(receipt_path), 'sha256': file_hash(receipt_path)}})
    return {'session': str(root), 'selection_policy': 'closeout bag_manifest capture_complete == true; exact bag association; aliases are labels',
            'metadata_sha256': {str(p.relative_to(root)): file_hash(p) for p in
                                [audit_path, segment_path, under(root, 'setup/sensor-contract.json'),
                                 *sorted((root / 'ground-truth').glob('*.json'))]},
            'raw_hash_policy': 'saved offload SHA256 receipts; current size/mtime frozen; metadata hashes verified; MCAP not rehashed in describe',
            'bags': result}


def stamp_ns(header):
    sec, nsec = header.stamp.sec, header.stamp.nanosec
    if sec < 0 or not 0 <= nsec < 1_000_000_000 or sec * 1_000_000_000 + nsec <= 0:
        raise ValueError(f'Malformed header timestamp: {sec}:{nsec}')
    return int(sec) * 1_000_000_000 + int(nsec)


class Sampler:
    """First source frame in each occupied bucket; reject timestamp regression.

    Duplicate stamps are explicitly counted and kept only once. Rejecting a
    regression is deliberate: anchoring to the first eligible stamp must not
    silently change when the receive/header clock relationship is malformed.
    """
    def __init__(self):
        self.anchor = self.last = None
        self.bucket = -1
        self.duplicates = 0

    def keep(self, stamp):
        if stamp <= 0:
            raise ValueError('RGB header timestamp must be positive')
        if self.last is not None and stamp < self.last:
            raise ValueError('Out-of-order eligible RGB header timestamp')
        if stamp == self.last:
            self.duplicates += 1
            return False
        self.last = stamp
        if self.anchor is None:
            self.anchor = stamp
        bucket = (stamp - self.anchor) // PERIOD_NS
        if bucket == self.bucket:
            return False
        self.bucket = bucket
        return True


def rigid_matrix(transform):
    q = transform.rotation
    quaternion = np.asarray([q.x, q.y, q.z, q.w], dtype=float)
    if not np.isfinite(quaternion).all() or not np.isclose(np.linalg.norm(quaternion), 1, atol=1e-5):
        raise ValueError('Invalid TF unit quaternion')
    x, y, z, w = quaternion
    matrix = np.eye(4)
    matrix[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                      [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                      [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    t = transform.translation
    matrix[:3, 3] = [t.x, t.y, t.z]
    if not np.isfinite(matrix).all():
        raise ValueError('Nonfinite TF translation')
    return matrix


class RecordedTF:
    def __init__(self):
        self.edges = defaultdict(list)

    def add(self, message, receive, order, static):
        for index, transform in enumerate(message.transforms):
            parent, child = transform.header.frame_id, transform.child_frame_id
            if not parent or not child or parent == child:
                raise ValueError('Invalid TF frame identities')
            self.edges[parent, child].append({'stamp_ns': 0 if static else stamp_ns(transform.header),
                'receive_ns': receive, 'source_order': order, 'transform_index': index,
                'static': static, 'matrix': rigid_matrix(transform.transform).tolist()})

    def resolve(self, target, source, stamp):
        if target == source:
            return np.eye(4), []
        adjacency = defaultdict(list)
        for (parent, child), records in self.edges.items():
            statics = [r for r in records if r['static']]
            dynamic = sorted([r for r in records if not r['static']], key=lambda r: (r['stamp_ns'], r['source_order']))
            chosen = None
            if statics:
                # A changing static mount is contradictory evidence, never last-wins.
                if any(not np.allclose(statics[0]['matrix'], r['matrix'], atol=1e-8) for r in statics):
                    raise ValueError(f'Conflicting recorded static TF {parent}->{child}')
                chosen = np.asarray(statics[0]['matrix']), [statics[0]]
            elif dynamic:
                exact = next((r for r in dynamic if r['stamp_ns'] == stamp), None)
                if exact:
                    chosen = np.asarray(exact['matrix']), [exact]
                else:
                    before = [r for r in dynamic if r['stamp_ns'] < stamp]
                    after = [r for r in dynamic if r['stamp_ns'] > stamp]
                    if before and after:
                        # ROS TF interpolation, bracketed only; never extrapolate.
                        from scipy.spatial.transform import Rotation, Slerp
                        a, b = before[-1], after[0]
                        fraction = (stamp-a['stamp_ns'])/(b['stamp_ns']-a['stamp_ns'])
                        ma, mb = np.array(a['matrix']), np.array(b['matrix'])
                        m = np.eye(4)
                        m[:3, :3] = Slerp([0, 1], Rotation.from_matrix([ma[:3, :3], mb[:3, :3]]))([fraction]).as_matrix()[0]
                        m[:3, 3] = ma[:3, 3]*(1-fraction)+mb[:3, 3]*fraction
                        chosen = m, [a, b]
            if chosen:
                matrix, provenance = chosen
                provenance = [dict(r, parent=parent, child=child) for r in provenance]
                adjacency[child].append((parent, matrix, provenance))
                adjacency[parent].append((child, np.linalg.inv(matrix), provenance))
        queue = deque([(source, np.eye(4), [])])
        seen = {source}
        while queue:
            frame, matrix, provenance = queue.popleft()
            for other, edge, records in adjacency[frame]:
                if other in seen:
                    continue
                combined = edge @ matrix
                if other == target:
                    return combined, provenance + records
                seen.add(other)
                queue.append((other, combined, provenance + records))
        return None, []


def ros_messages(bag, topics):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    kinds = {topic: kind for kind, topic in topics.items()}
    classes = {topic: get_message(EXPECTED_TYPES[kind]) for topic, kind in kinds.items()}
    for topic, kind in kinds.items():
        if types.get(topic) != EXPECTED_TYPES[kind]:
            raise ValueError(f'Recorded topic type changed: {topic}')
    while reader.has_next():
        topic, raw, receive = reader.read_next()
        if topic in kinds:
            yield kinds[topic], int(receive), deserialize_message(raw, classes[topic])


def camera_context(message, frame, width, height):
    k = list(message.k)
    if (message.header.frame_id, message.width, message.height) != (frame, width, height):
        raise ValueError('Colour CameraInfo grid/frame mismatch')
    if len(k) != 9 or not np.isfinite(k).all() or k[0] <= 0 or k[4] <= 0:
        raise ValueError('Invalid colour CameraInfo focal lengths/matrix')
    return {'fx': k[0], 'fy': k[4], 'cx': k[2], 'cy': k[5], 'width': width, 'height': height,
            'distortion_model': message.distortion_model, 'd': list(message.d),
            'k': k, 'r': list(message.r), 'p': list(message.p), 'frame_id': frame,
            'stamp_ns': stamp_ns(message.header),
            'limitation': 'production pinhole projection; distortion retained, no recalibration/undistortion'}


def assemble_events(messages, selection, scratch, *, base_frame='base_link'):
    """Index one forward traversal; yield small event views after EOF.

    Disk is bounded by one bag's aligned depths plus selected RGB. Metadata/TF
    memory is bounded by one bag. Scratch is deliberately retained on failure.
    """
    from ridgeback_localization.core.image_utils import decode_color_to_rgb, convert_depth_to_meters_message
    scratch = Path(scratch)
    scratch.mkdir(parents=True, exist_ok=False)
    db = sqlite3.connect(scratch / 'index.sqlite')
    db.execute('create table frames(kind text, stamp integer, ordinal integer, receive integer, frame text, path text)')
    tf = RecordedTF()
    infos, scans = [], []
    sampler = Sampler()
    start, end = selection['interval_ns']
    selected = []
    try:
        for order, (kind, receive, message) in enumerate(messages):
            if kind in ('static', 'tf'):
                tf.add(message, receive, order, kind == 'static')
                continue
            stamp = stamp_ns(message.header)
            if kind == 'info':
                infos.append((stamp, order, receive, message))
            elif kind == 'scan':
                scan = {key: float(getattr(message, key)) for key in
                        ('angle_min', 'angle_increment', 'range_min', 'range_max')}
                scan.update(ranges=np.asarray(message.ranges, dtype=np.float32), stamp_ns=stamp,
                            frame_id=message.header.frame_id, receive_ns=receive, source_order=order)
                scans.append(scan)
            elif kind in ('rgb', 'depth'):
                if kind == 'rgb' and not (start <= receive < end and sampler.keep(stamp)):
                    continue
                array = (decode_color_to_rgb(message) if kind == 'rgb'
                         else convert_depth_to_meters_message(message))
                path = scratch / f'{order}.npy'
                np.save(path, array, allow_pickle=False)
                db.execute('insert into frames values(?,?,?,?,?,?)',
                           (kind, stamp, order, receive, message.header.frame_id, str(path)))
                if kind == 'rgb':
                    selected.append((stamp, order, receive, message.header.frame_id, str(path)))
        db.execute('create index stamps on frames(kind,stamp,ordinal)')
        db.commit()
        scratch_bytes = sum(p.stat().st_size for p in scratch.iterdir() if p.is_file())
        for ordinal, (stamp, order, receive, frame, rgb_path) in enumerate(selected):
            rgb = np.load(rgb_path, allow_pickle=False)
            height, width = rgb.shape[:2]
            event = {'event_id': f'{selection["bag"]}:{ordinal:06d}:{stamp}:{order}',
                     'ordinal': ordinal, 'stamp_ns': stamp, 'rgb_receive_ns': receive,
                     'rgb_source_order': order, 'rgb_frame_id': frame, 'rgb': rgb,
                     'image_width': width, 'image_height': height, 'front_offset_m': 0.465,
                     'base_frame': base_frame, 'base_above_floor_m': 0.026,
                     'resources': {'index_disk_bytes': scratch_bytes, 'selected_events': len(selected),
                                   'scan_candidates': len(scans), 'camera_info_records': len(infos),
                                   'tf_records': sum(len(records) for records in tf.edges.values())},
                     'depth_m': None, 'scan': None, 'missing_inputs': {}, 'context': {},
                     'sampling': {'anchor_ns': sampler.anchor, 'duplicate_rgb_stamps': sampler.duplicates}}
            pairs = db.execute("select * from frames where kind='depth' and stamp=? order by ordinal", (stamp,)).fetchall()
            if pairs:
                _, ds, do, dr, df, path = pairs[0]
                depth = np.load(path, allow_pickle=False)
                event['depth_identity'] = {'stamp_ns': ds, 'receive_ns': dr, 'source_order': do,
                                           'frame_id': df, 'duplicate_matches': len(pairs)-1,
                                           'units': 'published 16UC1 mm / 1000 or 32FC1 metres; no device scaling'}
                if df == frame and depth.shape == (height, width):
                    event['depth_m'] = depth
                    valid = np.isfinite(depth) & (depth > 0)
                    event['depth_content'] = {'valid_pixels': int(valid.sum()), 'total_pixels': int(depth.size),
                                             'min_positive_m': float(depth[valid].min()) if valid.any() else None,
                                             'max_positive_m': float(depth[valid].max()) if valid.any() else None}
                else:
                    event['missing_inputs']['stereo'] = 'aligned depth grid/frame mismatch'
            else:
                event['missing_inputs']['stereo'] = 'no exact header-stamp depth match'
            candidates = [s for s in scans if abs(s['stamp_ns']-stamp) <= SCAN_TOLERANCE_NS]
            if candidates:
                scan = min(candidates, key=lambda s: (abs(s['stamp_ns']-stamp), s['stamp_ns'], s['source_order']))
                event['scan'] = scan
                event['scan_delta_ns'] = scan['stamp_ns']-stamp
                # Transform at scan time. Moving camera cross-time compensation
                # requires a fixed frame and is intentionally not invented.
                scan_to_base, scan_chain = tf.resolve(base_frame, scan['frame_id'], scan['stamp_ns'])
                camera_at_scan, camera_chain = tf.resolve(base_frame, frame, scan['stamp_ns'])
                camera_at_rgb, _ = tf.resolve(base_frame, frame, stamp)
                if (scan_to_base is not None and camera_at_scan is not None and camera_at_rgb is not None
                        and np.allclose(camera_at_scan, camera_at_rgb, atol=1e-8)):
                    matrix = np.linalg.inv(camera_at_scan) @ scan_to_base
                    event.update(scan_rotation=matrix[:3, :3].tolist(), scan_translation=matrix[:3, 3].tolist())
                    event['context']['scan_tf'] = scan_chain+camera_chain
                else:
                    event['missing_inputs']['scan_tf'] = 'recorded transform unavailable or camera mount moved between scan/RGB'
            else:
                event['missing_inputs']['scan'] = 'no front scan within inclusive 50 ms'
            eligible_info = [item for item in infos if item[0] <= stamp]
            if eligible_info:
                _, io, ir, info = max(eligible_info, key=lambda item: (item[0], item[1]))
                try:
                    event['intrinsics'] = camera_context(info, frame, width, height)
                    event['context']['camera_info'] = {'source_order': io, 'receive_ns': ir}
                except ValueError as exc:
                    event['missing_inputs']['camera_info'] = str(exc)
            else:
                event['missing_inputs']['camera_info'] = 'no colour calibration at or before RGB stamp'
            matrix, chain = tf.resolve(base_frame, frame, stamp)
            if matrix is not None:
                event.update(camera_rotation=matrix[:3, :3].tolist(), camera_translation=matrix[:3, 3].tolist())
                event['context']['camera_tf'] = chain
            else:
                event['missing_inputs']['camera_tf'] = 'recorded base-to-colour chain unavailable at RGB stamp'
            yield event
    finally:
        db.close()
