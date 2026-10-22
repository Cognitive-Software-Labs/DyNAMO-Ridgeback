"""Exercise delayed device/address availability and a bounded startup failure."""
import sys
from pathlib import Path
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).parent))
from wait_dds_interfaces import requirements, missing, wait


class ReadinessTests(unittest.TestCase):
    def profile(self,root,transport='udp'):
        path=root/'dds.xml'
        path.write_text('<CycloneDDS xmlns="https://cdds.io/config"><Domain id="any">'
                        '<General><Transport>'+transport+'</Transport><Interfaces>'
                        '<NetworkInterface name="br0"/><NetworkInterface name="wlp3s0"/>'
                        '</Interfaces></General></Domain></CycloneDDS>')
        return path

    def interface(self,name,family='inet',up=True):
        return {'ifname':name,'flags':['UP'] if up else [],
                'addr_info':[{'family':family,'local':'10.0.0.1'}]}

    def test_presence_without_usable_address_is_not_ready(self):
        required=[('br0','inet',None),('wlp3s0','inet',None)]
        self.assertEqual(missing(required,[self.interface('br0')]),['wlp3s0'])
        self.assertEqual(missing(required,[self.interface('br0'),
                                          self.interface('wlp3s0','inet6')]),['wlp3s0'])
        self.assertEqual(missing(required,[self.interface('br0'),
                                          self.interface('wlp3s0',up=False)]),['wlp3s0'])
        self.assertEqual(missing(required,[self.interface('br0'),self.interface('wlp3s0')]),[])

    def test_wait_exits_on_readiness_and_times_out_on_missing_device(self):
        with tempfile.TemporaryDirectory() as temp:
            profile=self.profile(Path(temp));clock=[0.];reads=[0]
            def sleep(seconds):clock[0]+=seconds
            def read():
                reads[0]+=1
                return [self.interface('br0')]+([self.interface('wlp3s0')] if reads[0]>=3 else [])
            self.assertEqual(wait(profile,1,lambda:clock[0],sleep,read)['status'],'interfaces-ready')
            self.assertAlmostEqual(clock[0],.4)
            with self.assertRaisesRegex(RuntimeError,'wlp3s0'):
                wait(profile,.5,lambda:clock[0],sleep,lambda:[self.interface('br0')])
            self.assertAlmostEqual(clock[0],.9)

    def test_ipv6_and_unknown_profile_fail_closed(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            self.assertEqual(requirements(self.profile(root,'udp6'))[0][1],'inet6')
            with self.assertRaisesRegex(ValueError,'unsupported'):
                requirements(self.profile(root,'raweth'))
            profile=root/'auto.xml';profile.write_text('<CycloneDDS><Domain id="0">'
                '<General><Interfaces><NetworkInterface autodetermine="true"/>'
                '</Interfaces></General></Domain></CycloneDDS>')
            with self.assertRaisesRegex(ValueError,'explicit'):
                requirements(profile)


if __name__=='__main__':unittest.main()
