#!/usr/bin/env python3
"""Qualify complete sensor/TF coverage, rerecording invalid corpus cases."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from ridgeback_leader_follow.benchmarking.dataset import SensorBag
from ridgeback_leader_follow.crowd_qualify import validate_run
from ridgeback_leader_follow.crowd_runner import stop_process


def validate(run):
    bag=SensorBag(run)
    for st,sim in bag.schedule:
        bag.camera(sim)
        for sensor in ('lidar2d_0','lidar2d_1'):
            at,scan=bag.latest(f'/{sensor}/scan',sim)
            bag.transform('odom',scan.header.frame_id,at)
    return {'status':'passed','frames':len(bag.schedule),'scope':'RGB-D registration, freshness and complete sensor-to-odom TF at every evaluation tick'}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True);p.add_argument('--ros-domain',type=int,default=211)
    a=p.parse_args();path=a.root/'corpus.json'
    while not path.exists() or json.loads(path.read_text())['status'] == 'running':time.sleep(2)
    old=path.read_text();(a.root/'corpus-before-sensor-qualification.json').write_text(old)
    corpus=json.loads(old);corpus['status']='validating';path.write_text(json.dumps(corpus,indent=2)+'\n')
    checks=[]
    for i,case in enumerate(corpus['results']):
        run=Path(case['run'])
        try:result=validate(run)
        except Exception as error:
            print('Rerecord',run.name,repr(error),flush=True)
            for attempt in range(3):
                target=a.root/(run.name+'-sensor-retry'+str(attempt+1))
                python=sys.executable if case['backend'] == 'gz' else str(Path.cwd()/'isaac_venv/bin/python3')
                cmd=[python,'-m','ridgeback_leader_follow.crowd_runner','--backend',case['backend'],'--scenario',case['scenario'],
                     '--crowd-size',str(case['crowd_size']),'--ros-domain',str(a.ros_domain+i),'--record','--output-dir',str(target)]
                with target.with_suffix('.log').open('w') as log:
                    proc=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                    try:code=proc.wait(timeout=420)
                    finally:stop_process(proc)
                if code:continue
                result=validate(target);case=validate_run(target);corpus['results'][i]=case;break
            else:raise RuntimeError('Sensor rerecord failed')
        checks.append({'run':case['run'],**result})
        print('Qualified',case['run'],flush=True)
    corpus.update(status='complete',sensor_status='passed',sensor_checks=checks)
    temporary=path.with_suffix('.new');temporary.write_text(json.dumps(corpus,indent=2)+'\n');temporary.replace(path)


if __name__ == '__main__':main()
