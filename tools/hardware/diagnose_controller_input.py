#!/usr/bin/env python3
"""Snapshot controller driver state. No device reads, signals, restarts or commands."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

ROOT=Path(__file__).resolve().parents[2]


def separate_release_evidence(rows):
    """Find L1 release while the same forward stick remains displaced."""
    result = {}
    for kind in ('physical', 'virtual'):
        samples = [row for row in rows if row['kind'] == kind]
        held = [row for row in samples if 310 in row['buttons']
                and row['axes'].get('1', 128) <= 100]
        released = [row for row in samples if 310 not in row['buttons']
                    and row['axes'].get('1', 128) <= 100
                    and held and row['elapsed_s'] > held[0]['elapsed_s']]
        result[kind] = {
            'held_l1_with_forward': bool(held),
            'released_l1_while_forward_held': bool(released),
            'first_held_s': held[0]['elapsed_s'] if held else None,
            'first_separate_release_s': released[0]['elapsed_s'] if released else None,
            'release_samples': len(released),
        }
    result['physical_and_virtual_release_observed'] = all(
        result[kind]['released_l1_while_forward_held'] for kind in ('physical', 'virtual'))
    return result


def read(path,limit=16000):
    try:return Path(path).read_text(errors='replace')[:limit]
    except OSError as error:return {'error':str(error)}


def query(argv):
    try:
        result=subprocess.run(argv,stdin=subprocess.DEVNULL,capture_output=True,text=True,timeout=5)
        return {'returncode':result.returncode,'stdout':result.stdout[-20000:],'stderr':result.stderr[-2000:]}
    except (OSError,subprocess.TimeoutExpired) as error:return {'error':str(error)}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir',type=Path,required=True)
    parser.add_argument('--sample-states',action='store_true',help='compare cached physical/virtual input state; never consume or grab events')
    parser.add_argument('--separate-release',action='store_true',
                        help='stationary L1 release while forward stick remains held; requires --sample-states')
    parser.add_argument('--operator-confirmed',action='store_true')
    args=parser.parse_args()
    if args.separate_release and not args.sample_states:
        parser.error('--separate-release requires --sample-states')
    if args.sample_states and not args.operator_confirmed:
        parser.error('state sampling requires operator confirmation; keep physical e-stop pressed')
    root=args.run_dir.resolve()
    if not root.is_relative_to(ROOT/'artifacts/hardware') or root==ROOT/'artifacts/hardware':
        parser.error('use a fresh directory under artifacts/hardware')
    root.mkdir(parents=True,exist_ok=False)
    result={'read_only':True,'uid':os.geteuid(),
            'service':query(['systemctl','show','ds4drv.service','-p','MainPID','-p','ActiveState','-p','ExecStart']),
            'journal':query(['journalctl','-u','ds4drv.service','--since','30 minutes ago','--no-pager','-n','100']),
            'input_devices':read('/proc/bus/input/devices'),'processes':[]}
    for item in Path('/proc').iterdir():
        if not item.name.isdigit():continue
        try:cmd=(item/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
        except OSError:continue
        if not (cmd.startswith('/usr/bin/python3 /usr/sbin/ds4drv ') or
                cmd.startswith('/opt/ros/jazzy/lib/joy_linux/joy_linux_node ')):continue
        process={'pid':int(item.name),'command':cmd[:2000],'status':read(item/'status'),'threads':[],'device_fds':[]}
        try:
            for thread in (item/'task').iterdir():
                process['threads'].append({'tid':int(thread.name),'wchan':read(thread/'wchan'),
                                          'stack':read(thread/'stack'),'syscall':read(thread/'syscall')})
            for fd in (item/'fd').iterdir():
                link=os.readlink(fd)
                if link.startswith(('/dev/input/','/dev/hidraw')):
                    process['device_fds'].append({'fd':fd.name,'target':link,'fdinfo':read(item/'fdinfo'/fd.name)})
        except OSError as error:process['inspection_error']=str(error)
        result['processes'].append(process)
    result['source_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    (root/'diagnostic.json').write_text(json.dumps(result,indent=2)+'\n')
    if args.sample_states:
        from install_commissioning_caps import identity,desired_files,run_probe
        ns=identity()
        for path,data in desired_files(ns,True).items():
            if path.read_bytes()!=data:raise RuntimeError('verified temporary interlock required: '+str(path))
        run_probe(ns,root,'before')
        import evdev
        devices=[]
        try:
            for item in Path('/sys/class/input').glob('event*'):
                name=read(item/'device/name')
                uniq=read(item/'device/uniq')
                if (isinstance(name,str) and name.strip()=='DS4DRV Managed Wireless Controller'):
                    devices.append(('virtual',evdev.InputDevice('/dev/input/'+item.name)))
                elif (isinstance(name,str) and name.strip()=='Wireless Controller'
                      and isinstance(uniq,str) and uniq.strip().lower()=='ac:36:1b:89:a1:0a'):
                    devices.append(('physical',evdev.InputDevice('/dev/input/'+item.name)))
            if sorted(kind for kind,_ in devices)!=['physical','virtual']:
                raise RuntimeError('one physical and one managed virtual controller required')
            if args.separate_release:
                print('SEPARATE RELEASE READY: KEEP ROBOT PHYSICAL E-STOP PRESSED.\n'
                      'Hold top-left L1 and push the LEFT thumbstick forward for one second.\n'
                      'Release ONLY L1. Keep the thumbstick forward for two more seconds.\n'
                      'Then centre the thumbstick. Do not release the robot e-stop.\n'
                      'Recording waits up to 60 seconds for held input, then up to 20 seconds for release; no commands or permits.',flush=True)
            else:
                print('STATE SAMPLING READY: KEEP ROBOT PHYSICAL E-STOP PRESSED.\n'
                      'Hold driving-enable and move the forward stick for one second, then release and centre.\n'
                      'Reading cached input state for 20 seconds; no event reads, grabs, permits or commands.',flush=True)
            start=time.monotonic()
            sampled=[]
            first_held=None
            separate_seen=None
            centre_prompted=False
            with (root/'input-states.jsonl').open('w') as log:
                while True:
                    for kind,device in devices:
                        axes=device.capabilities(absinfo=False).get(evdev.ecodes.EV_ABS,[])
                        row={'elapsed_s':time.monotonic()-start,'kind':kind,'path':device.path,
                             'axes':{str(axis):device.absinfo(axis).value for axis in axes},
                             'buttons':device.active_keys()}
                        log.write(json.dumps(row)+'\n')
                        if args.separate_release:
                            sampled.append(row)
                            if kind == 'physical':
                                held = 310 in row['buttons'] and row['axes'].get('1',128) <= 100
                                released_forward = 310 not in row['buttons'] and row['axes'].get('1',128) <= 100
                                if held and first_held is None:
                                    first_held=time.monotonic()
                                    print('L1 + FORWARD RECORDED. Release ONLY L1 now; keep the stick forward.',flush=True)
                                if first_held is not None and released_forward and separate_seen is None:
                                    separate_seen=time.monotonic()
                                    print('L1 RELEASE WITH FORWARD RECORDED. Keep the stick forward for two seconds.',flush=True)
                    log.flush();time.sleep(.05)
                    now=time.monotonic()
                    if not args.separate_release and now-start>=20:break
                    if args.separate_release:
                        if first_held is None and now-start>=60:
                            print('No L1 + forward input was seen within 60 seconds.',flush=True)
                            break
                        if separate_seen is not None and now-separate_seen>=2 and not centre_prompted:
                            print('Two-second separation captured. Centre the stick now.',flush=True)
                            centre_prompted=True
                        if centre_prompted and now-separate_seen>=4:break
                        if first_held is not None and now-first_held>=20:
                            print('No separate release was captured within 20 seconds of held input.',flush=True)
                            break
            run_probe(ns,root,'after')
            if args.separate_release:
                result['separate_release']=separate_release_evidence(sampled)
                (root/'separate-release.json').write_text(json.dumps(result['separate_release'],indent=2)+'\n')
        finally:
            for _,device in devices:device.close()
    print(json.dumps({'status':'read-only-snapshot-saved','path':str(root/'diagnostic.json'),
                      'processes':len(result['processes']),
                      'separate_release':result.get('separate_release')},indent=2))


if __name__=='__main__':main()
