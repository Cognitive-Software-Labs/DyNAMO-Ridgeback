import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import driving_qualification as qualification


class QualificationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.patch = mock.patch.object(qualification, 'EVIDENCE', self.root)
        self.patch.start()

    def tearDown(self):
        self.patch.stop()
        self.temp.cleanup()

    def make_run(self, status='passed', passed=None):
        path = self.root / 'trial'
        path.mkdir()
        record = {'status': status}
        if passed is not None:
            record['passed'] = passed
        (path / 'result.json').write_text(json.dumps(record))
        return path

    def baseline(self):
        path = self.root / 'baseline.json'
        path.write_text(json.dumps({'schema': 'dynamo-driving-baseline-v1'}))
        return path

    def test_missing_automatic_pass_downgrades_to_inconclusive(self):
        path = self.make_run('aborted')
        record = qualification.finalize(path, 'D2', 'deadman', 'passed', 'stopped')
        self.assertEqual(record['verdict'], 'inconclusive')
        self.assertTrue(record['verdict_reasons'])

    def test_d6_needs_video_and_recovery(self):
        path = self.make_run('passed', True)
        record = qualification.finalize(path, 'D6', 'feedback-loss', 'passed', 'all wheels stopped')
        self.assertEqual(record['verdict'], 'inconclusive')
        video = path / 'video.mp4'
        video.write_bytes(b'video')
        record = qualification.finalize(path, 'D6', 'feedback-loss', 'passed', 'all wheels stopped',
                                        video=video, recovery='recovered-stationary', baseline=self.baseline())
        self.assertEqual(record['verdict'], 'passed')

    def test_d8_requires_speed_distance_uncertainty_duration_and_video(self):
        path = self.make_run('passed', True)
        video = path / 'video.mp4'
        video.write_bytes(b'video')
        incomplete = qualification.finalize(path, 'D8', 'deadman-forward', 'passed', 'stopped', video=video,
                                             measurements={'target_speed': .2, 'actual_speed': .1})
        self.assertEqual(incomplete['verdict'], 'inconclusive')
        complete = qualification.finalize(path, 'D8', 'deadman-forward', 'passed', 'stopped', video=video,
                                           conditions={'test_series': 'dry-empty',
                                                       'motion_class': 'translation',
                                                       'direction': 'forward', 'floor': 'dry concrete',
                                                       'payload': 'empty', 'battery_state': '80%',
                                                       'tire_condition': 'normal',
                                                       'available_clearance_m': 1.0},
                                           measurements={'target_speed': .2, 'actual_speed': .2,
                                                         'stopping_distance_m': .12,
                                                         'distance_uncertainty_m': .01,
                                                         'stop_duration_s': .4}, baseline=self.baseline())
        self.assertEqual(complete['verdict'], 'inconclusive')
        self.assertTrue(any('lower speed' in reason for reason in complete['verdict_reasons']))

    def test_first_d8_speed_can_pass_with_complete_conditions(self):
        path = self.make_run('passed', True)
        video = path / 'video.mp4'
        video.write_bytes(b'video')
        record = qualification.finalize(
            path, 'D8', 'deadman-forward', 'passed', 'stopped', video=video,
            conditions={'test_series': 'dry-empty', 'motion_class': 'translation',
                        'direction': 'forward', 'floor': 'dry concrete', 'payload': 'empty',
                        'battery_state': '80%', 'tire_condition': 'normal',
                        'available_clearance_m': 1.0},
            measurements={'target_speed': .05, 'actual_speed': .05,
                          'stopping_distance_m': .12, 'distance_uncertainty_m': .01,
                          'stop_duration_s': .4}, baseline=self.baseline())
        self.assertEqual(record['verdict'], 'passed')

    def test_matrix_never_infers_missing_runs(self):
        path = self.make_run('passed', True)
        qualification.finalize(path, 'D4', 'handover', 'passed', 'forward and clean stop')
        matrix = qualification.build_matrix(self.root)
        self.assertEqual(matrix['summary']['D4']['passed'], 1)
        self.assertEqual(matrix['summary']['D6']['passed'], 0)


if __name__ == '__main__':
    unittest.main()
