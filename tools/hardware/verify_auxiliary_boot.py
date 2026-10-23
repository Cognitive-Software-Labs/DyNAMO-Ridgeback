#!/usr/bin/env python3
"""Read-only post-reboot verification of installed auxiliary repair and clock-gate ordering."""
import argparse
import hashlib
import json
import os
import re
from pathlib import Path
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools/intel_thor'))
from desktop_ready import listener_identity
from install_commissioning_caps import identity,run_probe
from install_auxiliary_repairs import probe_auxiliary_data


def command(*args):return subprocess.run(args,capture_output=True,text=True,check=True,timeout=15).stdout.strip()


def verify(receipt,output):
    output=output.resolve()
    if not output.is_relative_to(ROOT/'artifacts/hardware') or output==ROOT/'artifacts/hardware':raise ValueError('fresh artifacts/hardware output required')
    output.mkdir(parents=True,exist_ok=False);record=json.loads((receipt/'result.json').read_text())
    if record['status']!='installed-verified-stationary':raise RuntimeError('installed verified receipt required')
    report={'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),'checks':{},'service_states':{}}
    before=receipt/'boot-baseline.json'
    report['checks']['different_boot']=before.exists() and json.loads(before.read_text())['boot_id']!=report['boot_id']
    for name,entry in record['files'].items():
        p=Path(name);report['checks']['file:'+name]=not p.is_symlink() and p.exists() and p.read_text()==entry['content']
    units=['clearpath-platform.service','dynamo-commissioning-speed-guard.service','clearpath-sensors.service','clearpath-scan-merger.service','dynamo-camera.service','ridgeback-vnc.service','ridgeback-vnc-websockify.service','ridgeback-remote-screen.service','controller-autoconnect.service']
    for unit in units:
        values=dict(row.split('=',1) for row in command('systemctl','show',unit,'-p','ActiveState','-p','NRestarts','-p','MainPID','-p','ActiveEnterTimestampMonotonic').splitlines() if '=' in row)
        report['service_states'][unit]=values;report['checks']['active:'+unit]=values['ActiveState']=='active';report['checks']['no_restarts:'+unit]=values['NRestarts']=='0'
    gate=command('systemctl','show','clearpath-platform.service','-p','ExecStartPre','--value')
    (output/'platform-prestart.txt').write_text(gate)
    # An active platform plus the named non-ignored prestart's successful exit
    # establishes systemd ran the gate before ExecStart. It is not a clock-loss test.
    gate_records=re.findall(r'\{[^}]*check_clock_ready\.py[^}]*\}',gate)
    report['checks']['gate_executed_successfully']=len(gate_records)==1 and 'ignore_errors=no' in gate_records[0] and re.search(r'\bstatus=0\b',gate_records[0]) is not None
    journal=command('journalctl','-b','-u','clearpath-platform.service','-u','dynamo-commissioning-speed-guard.service','--no-pager');(output/'platform-boot.log').write_text(journal)
    clock=json.loads(command('/usr/bin/python3',str(Path(__file__).with_name('check_clock_ready.py')),'--timeout','10'));report['clock']=clock;report['checks']['clock_ready_now']=clock['ready']
    report['listeners']={}
    for port,unit in [(5901,'ridgeback-vnc.service'),(9005,'ridgeback-vnc-websockify.service'),(9100,'ridgeback-remote-screen.service')]:
        try:report['listeners'][str(port)]=listener_identity(port,unit);report['checks']['listener:'+str(port)]=True
        except (OSError,RuntimeError,subprocess.SubprocessError) as error:report['checks']['listener:'+str(port)]=False;report['listeners'][str(port)]=str(error)
    report['stationary']=run_probe(identity(),output,'boot-stationary')
    report['data']=probe_auxiliary_data(identity());report['checks']['stationary']=report['stationary']['ok'];report['checks']['auxiliary_data']=report['data']['ok']
    report['status']='post-reboot-check-passed' if all(report['checks'].values()) else 'post-reboot-check-failed'
    report['limits']='One observed reboot, not repeated cold power cycles, full IMU calibration, battery sensing installation or external-source-loss qualification.'
    (output/'result.json').write_text(json.dumps(report,indent=2)+'\n');return report

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--receipt',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);a=parser.parse_args();r=verify(a.receipt,a.output);print(json.dumps({'status':r['status'],'failed':[k for k,v in r['checks'].items() if not v]},indent=2));raise SystemExit(0 if all(r['checks'].values()) else 1)
