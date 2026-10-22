import tempfile
from pathlib import Path
import unittest
from controller_autoconnect import controllers, poll

class AutoconnectTests(unittest.TestCase):
    def test_addresses_are_validated_and_deduplicated(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)/'controllers';p.write_text('ac:36:1b:89:a1:0a # paired\nAC:36:1B:89:A1:0A\n')
            self.assertEqual(controllers(p),['AC:36:1B:89:A1:0A'])
            p.write_text('$(touch /tmp/bad)');self.assertRaises(ValueError,controllers,p)
    def test_connected_never_restarts_or_reconnects(self):
        calls=[]
        def run(args):calls.append(args);return 0,'Paired: yes\nTrusted: yes\nConnected: yes\n'
        self.assertEqual(poll(['AC:36:1B:89:A1:0A'],run)[0]['status'],'connected-link-only')
        self.assertEqual(calls,[['info','AC:36:1B:89:A1:0A']])
    def test_disconnected_trusted_device_gets_only_connect_request(self):
        calls=[]
        def run(args):calls.append(args);return 0,'Paired: yes\nTrusted: yes\nConnected: no\n'
        self.assertEqual(poll(['AC:36:1B:89:A1:0A'],run)[0]['status'],'connect-request-sent')
        self.assertEqual([x[0] for x in calls],['info','connect'])
    def test_untrusted_or_unpaired_is_not_automatically_enrolled(self):
        for field in ['Paired','Trusted']:
            calls=[]
            def run(args):calls.append(args);return 0,f'Paired: yes\nTrusted: yes\nConnected: no\n'.replace(field+': yes',field+': no')
            self.assertEqual(poll(['AC:36:1B:89:A1:0A'],run)[0]['status'],'manual-pair-and-trust-required')
            self.assertEqual(len(calls),1)
    def test_failed_query_cannot_connect(self):
        calls=[]
        def run(args):calls.append(args);return 1,''
        self.assertEqual(poll(['AC:36:1B:89:A1:0A'],run)[0]['status'],'manual-pair-and-trust-required')
        self.assertEqual(len(calls),1)
