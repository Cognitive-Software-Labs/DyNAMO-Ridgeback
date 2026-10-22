import unittest
from expand_clock_sources import candidate, OLD, NEW

class SourcesTest(unittest.TestCase):
    def test_only_source_changes(self):
        text = '# preserved\n' + OLD + '\nallow 192.168.131.0/24\nlocal stratum 10\nmakestep 1 3\nmaxslewrate 500\n'
        self.assertEqual(candidate(text), text.replace(OLD, NEW))

    def test_rejects_unexpected_profile(self):
        for text in (OLD + '\n', OLD + '\n' + OLD + '\nmaxslewrate 500\n', NEW + '\nmaxslewrate 500\n'):
            with self.assertRaises(RuntimeError): candidate(text)

if __name__ == '__main__': unittest.main()
