import unittest
from bound_clock_poll import candidate,OLD,NEW

class PollTests(unittest.TestCase):
    def test_only_pool_poll_bound_changes(self):
        old=OLD+'\nallow 192.168.131.0/24\nlocal stratum 10\nmakestep 1 3\nmaxslewrate 500\n'
        self.assertEqual(candidate(old),old.replace(OLD,NEW))
    def test_refuses_unexpected_profile(self):
        for text in (NEW+'\nmaxslewrate 500\n',OLD+'\n',OLD+'\n'+OLD+'\nmaxslewrate 500\n'):
            with self.assertRaises(RuntimeError):candidate(text)
