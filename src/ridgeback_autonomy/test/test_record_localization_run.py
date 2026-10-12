"""Segmenting a recorded localization run by scene marks."""
from importlib.machinery import SourceFileLoader
import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
loader = SourceFileLoader('record_localization_run', str(ROOT / 'tools/intel_thor/record_localization_run'))
spec = importlib.util.spec_from_loader(loader.name, loader)
TOOL = importlib.util.module_from_spec(spec)
loader.exec_module(TOOL)
S = 1_000_000_000


def measurement(at_s, forward):
    entry = {'kind': 'measurement', 'received_ns': int(at_s * S), 'stamp_ns': int((at_s - .7) * S), 'count': 1}
    for name in TOOL.ESTIMATORS:
        entry.update({f'{name}_forward_m': [forward], f'{name}_lateral_m': [0.1], f'{name}_distance_m': [forward]})
    return entry


def test_segments_split_at_marks_after_settling_and_shift_to_camera_face():
    samples = [measurement(1, 9.0),                       # before settling: ignored
               measurement(5, 1.54), measurement(6, 1.56),
               {'kind': 'detection', 'received_ns': 6 * S, 'stamp_ns': int(5.4 * S), 'count': 1},
               {'kind': 'health', 'received_ns': 6 * S, 'state': 'processing'},
               measurement(9, 7.0),                       # walking: after the 3 s + 4 s window
               measurement(12, 2.54)]
    marks = [{'label': '1.5', 'wall_ns': 0}, {'label': '2.5', 'wall_ns': 10 * S}]
    report = TOOL.summarize(samples, marks, {'camera_x_in_base_m': .28, 'front_offset_m': .25}, window_s=4)
    first, second = report['segments']
    assert report['camera_forward_shift_m'] == .03
    assert first['projective_ranging']['forward_from_camera_m']['median'] == 1.52
    assert first['health_states'] == {'processing': 1}
    assert first['detection_delay_ms']['median'] == 600
    assert first['measurement_delay_ms']['median'] == 700
    assert second['euclidean_reconstruction']['forward_from_camera_m'] is None  # 12 s is before the 13 s settle
