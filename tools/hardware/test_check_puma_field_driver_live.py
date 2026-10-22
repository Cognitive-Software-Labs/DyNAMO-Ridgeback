import unittest

from check_puma_field_driver_live import evaluate

DRIVER = '/opt/dynamo-puma-field/abc/lib/puma_motor_driver/multi_puma_node'


def good(**changes):
    values = dict(arrivals_s=[i * .05 for i in range(1201)],
                  ages=[(f'puma {n}', 40.0, 300.0) for n in range(4)] * 60,
                  queue=[(3000, 1.2, 8.0, 0)] * 12, stops=[True] * 100, speeds=[0.0] * 100,
                  driver=DRIVER, duration=60.0)
    values.update(changes)
    return evaluate(**values)


class EvaluateTests(unittest.TestCase):
    def test_passes_healthy_stationary_window(self):
        self.assertTrue(good()['passed'])

    def test_each_requirement_can_fail(self):
        cases = {'driver': '/opt/dynamo-puma/x/lib/puma_motor_driver/multi_puma_node',
                 'stops': [True] * 99 + [False], 'speeds': [0.0, 0.01],
                 'arrivals_s': [i * .06 for i in range(1001)],
                 'ages': [(f'puma {n}', 151.0, 300.0) for n in range(4)],
                 'queue': [(3000, 1.2, 76.0, 0)], 'duration': 61.0 * 2}
        for key, value in cases.items():
            with self.subTest(key=key):
                self.assertFalse(good(**{key: value})['passed'])
        self.assertFalse(good(ages=[(f'puma {n}', -1.0, 300.0) for n in range(4)])['passed'])
        self.assertFalse(good(ages=[(f'puma {n}', 40.0, 300.0) for n in range(3)])['passed'])
        self.assertFalse(good(queue=[])['passed'])
        self.assertFalse(good(queue=[(3000, 1.2, 8.0, 1)])['passed'])
        gap = [i * .05 for i in range(600)] + [30.2 + i * .05 for i in range(600)]
        self.assertFalse(good(arrivals_s=gap)['passed'])


if __name__ == '__main__':
    unittest.main()
