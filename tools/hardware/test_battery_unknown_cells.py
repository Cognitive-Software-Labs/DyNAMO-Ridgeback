"""Execute matching source update_cells before/after the maintained patch."""
import math
from pathlib import Path
import subprocess
import tempfile
from types import SimpleNamespace
import unittest

ROOT=Path(__file__).resolve().parents[2]
class BatteryTests(unittest.TestCase):
    def test_real_source_marks_unknown_without_changing_pack_fields(self):
        original=ROOT/'artifacts/hardware/20261012-puma-repair-candidate/upstream/clearpath_robot-2.9.4/clearpath_hardware_interfaces/clearpath_hardware_interfaces/battery_state/battery.py'
        if not original.exists():self.skipTest('matching official source archive required')
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);p=root/'clearpath_hardware_interfaces/clearpath_hardware_interfaces/battery_state/battery.py';p.parent.mkdir(parents=True);p.write_bytes(original.read_bytes())
            subprocess.run(['patch','--batch','-p1','--input',str(ROOT/'patches/clearpath_battery_unknown_cells.patch')],cwd=root,check=True,capture_output=True)
            for series,count in [(1,2),(2,4),(1,1)]:
                source=p.read_text();begin=source.index('        def update_cells(self):');end=source.index('\n    # Battery Types',begin)
                body='\n'.join(line[8:] for line in source[begin:end].splitlines());scope={'nan':float('nan')};exec(body,scope)
                msg=SimpleNamespace(voltage=26.5,current=2.,percentage=.8,temperature=float('nan'))
                obj=SimpleNamespace(_msg=msg,series=series,cell_count=count);scope['update_cells'](obj)
                self.assertEqual(len(msg.cell_voltage),count);self.assertTrue(all(math.isnan(v) for v in msg.cell_voltage));self.assertTrue(all(math.isnan(v) for v in msg.cell_temperature))
                self.assertEqual((msg.voltage,msg.current,msg.percentage),(26.5,2.,.8))
