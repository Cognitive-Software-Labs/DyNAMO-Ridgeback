import unittest
from check_clock_ready import evaluate

class ClockTests(unittest.TestCase):
    def report(self,correction='.001',skew='2',reference='B97DBE38',stamp='1000',leap='Normal'):
        return ','.join([reference,'185.125.190.56','3',stamp,correction,'.0001','.001','2','0',skew,'.03','.001','64',leap])
    def ready(self,text):return evaluate(text,.005,10,180,1010)
    def test_current_small_last_offset_cannot_hide_system_correction(self):
        self.assertFalse(self.ready(self.report(correction='.17'))['ready'])
        self.assertTrue(self.ready(self.report())['ready'])
    def test_unsynced_local_and_old_references_refused(self):
        for text in [self.report(reference='7F7F0101'),self.report(reference='0'),self.report(stamp='0'),self.report(stamp='1011'),self.report(leap='Not synchronised'),self.report(skew='11')]:self.assertFalse(self.ready(text)['ready'])
    def test_nonfinite_and_bad_schema_refused(self):
        self.assertRaises(ValueError,self.ready,self.report(correction='nan'))
        self.assertRaises(ValueError,self.ready,'broken')
    def test_error_bound_includes_network_uncertainty(self):
        self.assertAlmostEqual(self.ready(self.report())['estimated_utc_error_bound_s'],.017)
