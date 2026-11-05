#!/usr/bin/env python3
"""Record/resume the sixteen F2 simulation cases without overwriting evidence."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
from ridgeback_leader_follow.crowd_qualify import validate_run
from ridgeback_leader_follow.crowd_scenarios import SCENARIOS,CROWD_SIZES
from ridgeback_leader_follow.crowd_runner import stop_process


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--ros-domain',type=int,default=185)
    args=parser.parse_args(); args.root.mkdir(parents=True,exist_ok=True)
    results=[]
    cases=[(b,n,s) for b in ('gz','isaac') for n in SCENARIOS for s in (CROWD_SIZES if n == 'crowd' else (4,))]
    for index,(backend,name,size) in enumerate(cases):
        complete=None
        for attempt in range(3):
            output=args.root/f'{backend}-{name}-{size}{"" if attempt == 0 else "-retry"+str(attempt)}'
            if (output/'manifest.json').exists():
                try: complete=validate_run(output); break
                except (ValueError,KeyError): continue
            python=sys.executable if backend == 'gz' else os.environ.get('ISAAC_PYTHON',str(Path.cwd()/'isaac_venv/bin/python3'))
            command=[python,'-m','ridgeback_leader_follow.crowd_runner','--backend',backend,'--scenario',name,
                     '--crowd-size',str(size),'--ros-domain',str(args.ros_domain+index+16*attempt),'--record','--output-dir',str(output)]
            print('Recording',output,flush=True)
            with (args.root/f'{output.name}.log').open('w') as log:
                process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                try: code=process.wait(timeout=420)
                finally: stop_process(process)
            if code: continue
            complete=validate_run(output); break
        if complete is None: raise RuntimeError(f'Case failed after retries: {backend}/{name}/{size}')
        results.append(complete)
        (args.root/'corpus.json').write_text(json.dumps({'status':'running','results':results},indent=2)+'\n')
    (args.root/'corpus.json').write_text(json.dumps({'status':'complete','results':results},indent=2)+'\n')


if __name__ == '__main__': main()
