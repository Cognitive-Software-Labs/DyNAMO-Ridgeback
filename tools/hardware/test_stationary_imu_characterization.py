import unittest
from characterize_stationary_imu import summarize

class StatisticsTests(unittest.TestCase):
    def test_reference_vectors_preserve_mean_and_cross_covariance(self):
        rows=[{'gyro':[v,2*v,3*v],'accel':[0.,0.,9.8]} for v in [-1.,0.,1.]]
        result=summarize(rows)
        self.assertEqual(result['gyro']['mean'],[0.,0.,0.])
        self.assertEqual(result['gyro']['covariance'],[[1.,2.,3.],[2.,4.,6.],[3.,6.,9.]])
        self.assertEqual(result['accel']['covariance'],[[0.,0.,0.],[0.,0.,0.],[0.,0.,0.]])
    def test_halves_report_change_instead_of_claiming_stable_bias(self):
        rows=[{'gyro':[v,0.,0.],'accel':[0.,0.,9.8]} for v in [0.,0.,1.,1.]]
        result=summarize(rows)
        self.assertEqual(result['gyro']['first_to_second_half_mean_change'],[1.,0.,0.])
