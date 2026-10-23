"""Check that observed gaps cannot be hidden by other wheels in the flag model."""
import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).parent))
from replay_puma_freshness import replay,replay_motion_fields,receive_field,receive_field_update,field_receive_gaps,DEVICES


def trace(hole=False):
    events=[]
    for n in range(81):
        t=n*.05
        for device in DEVICES:
            for field in range(5):
                if hole and device==3 and field==4 and 2.05<=t<=2.35:continue
                events.append((t,device,field))
    return sorted(events)


class ReplayTests(unittest.TestCase):
    def test_verbose_and_compact_can_logs_preserve_rx_direction(self):
        verbose=' (1791804446.707109)  vcan0  RX - -  02021545   [4]  00 00 00 00'
        compact='(1791896516.956271) vcan0 02021505#00000000 R'
        self.assertEqual(receive_field(verbose)[1:],(5,3))
        self.assertEqual(receive_field(compact)[1:],(5,2))
        self.assertIsNone(receive_field(compact[:-1]+'T'))
        self.assertIsNone(receive_field('(1791896516.956271) vcan0 02021505# R'))

    def test_replay_rejects_wrong_field_width_in_both_candump_formats(self):
        stamp='(1791896516.956271) vcan0 '
        # Speed is four bytes; current is two bytes in the deployed speed mode.
        self.assertIsNone(receive_field(stamp+'02021545#0000 R'))
        self.assertIsNone(receive_field(stamp+'02021485#00000000 R'))
        self.assertIsNone(receive_field(stamp+'RX - - 02021545 [2] 00 00'))
        self.assertIsNone(receive_field(stamp+'RX - - 02021485 [4] 00 00 00 00'))
        self.assertEqual(receive_field(stamp+'02021485#0000 R')[1:],(5,1))

    def test_replay_rejects_truncated_or_malformed_verbose_payload(self):
        stamp='(1791896516.956271) vcan0 RX - - 02021545 '
        self.assertIsNone(receive_field(stamp+'[4] 00 00 00'))
        self.assertIsNone(receive_field(stamp+'[4] 00 00 00 GG'))
        self.assertIsNone(receive_field(stamp+'[4] 00 00 00 00 00'))
        self.assertEqual(receive_field(stamp+'[4] 00 00 00 00')[1:],(5,3))

    def test_malformed_relevant_rx_invalidates_candidate_cache(self):
        invalid=receive_field_update('(2.100000) vcan0 02021543#0000 R')
        self.assertEqual(invalid,(2.1,3,3,False))
        events=[(t,d,f,True) for t,d,f in trace() if not
                (d==3 and f==3 and 2.1<=t<2.4)]
        events.append(invalid);events.sort()
        result=replay_motion_fields(events,.025)
        self.assertGreater(result['max_publication_gap_s'],.25)

    def test_gap_report_keeps_each_wheel_and_missing_field_distinct(self):
        rows=field_receive_gaps([(1.,5,3),(1.04,5,3),(1.09,5,3),(1.02,4,3)])
        speed5=next(row for row in rows if (row['wheel'],row['field'])==(5,3))
        speed4=next(row for row in rows if (row['wheel'],row['field'])==(4,3))
        self.assertEqual(speed5['frames'],3)
        self.assertAlmostEqual(speed5['max_gap_ms'],50.)
        self.assertAlmostEqual(speed5['median_gap_ms'],45.)
        self.assertIsNone(speed4['max_gap_ms'])
        self.assertEqual(len(rows),20)

    def test_motion_policy_keeps_complete_trace_at_timer_rate_and_bounded_age(self):
        result=replay_motion_fields(trace(),.025)
        self.assertGreaterEqual(result['projected_hz'],19)
        self.assertLess(result['max_publication_gap_s'],.051)
        self.assertLessEqual(result['max_cached_field_age_s'],.2)

    def test_motion_policy_exposes_one_missing_wheel_speed(self):
        events=[row for row in trace() if not (row[1]==3 and row[2]==3 and 2.05<=row[0]<=2.35)]
        result=replay_motion_fields(events,.025)
        self.assertGreater(result['max_publication_gap_s'],.25)

    def test_motion_policy_counts_end_of_capture_silence(self):
        events=[row for row in trace() if not (row[1]==3 and row[2]==3 and row[0]>=3.55)]
        result=replay_motion_fields(events,.025)
        self.assertGreater(result['max_silence_including_edges_s'],.25)

    def test_motion_policy_rejects_aged_nonmotion_field_without_waiting_for_new_each_cycle(self):
        events=[row for row in trace() if not (row[1]==3 and row[2]==4 and 2.05<=row[0]<=2.4)]
        result=replay_motion_fields(events,.025)
        self.assertGreater(result['max_publication_gap_s'],.2)
        self.assertLessEqual(result['max_cached_field_age_s'],.2)

    def test_one_wheel_gap_reduces_rate_instead_of_borrowing_other_wheels(self):
        baseline=replay(trace(True),.025,False)
        candidate=replay(trace(True),.025,True)
        self.assertGreater(baseline['batches_with_unreceived_wheel_fields'],0)
        self.assertEqual(candidate['batches_with_unreceived_wheel_fields'],0)
        self.assertGreater(candidate['max_publication_gap_s'],.25)
        self.assertGreater(baseline['projected_hz'],candidate['projected_hz'])

    def test_complete_input_is_not_rejected_by_the_candidate(self):
        baseline=replay(trace(),.025,False)
        candidate=replay(trace(),.025,True)
        self.assertEqual(candidate['batches_with_unreceived_wheel_fields'],0)
        self.assertGreaterEqual(candidate['projected_hz'],baseline['projected_hz'])
        self.assertLess(candidate['max_publication_gap_s'],.051)


if __name__=='__main__':unittest.main()
