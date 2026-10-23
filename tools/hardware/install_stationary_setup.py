#!/usr/bin/env python3
"""Stage/install isolated desktop and Bluetooth setup; never restart robot hardware services."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools/intel_thor'))
from repair_hardware_workflow import desired_files as desktop_files, snapshot_file, write_file
from desktop_ready import listener_identity
from install_commissioning_caps import identity, run_probe

SYSTEM=Path('/etc/systemd/system');LIB=Path('/usr/local/lib/dynamo-hardware')
VNC='ridgeback-vnc.service';PROXY='ridgeback-vnc-websockify.service';REMOTE='ridgeback-remote-screen.service';AUTO='controller-autoconnect.service'
MANAGED=(AUTO,PROXY,REMOTE,VNC)
INDEPENDENT=('clearpath-platform.service','dynamo-commissioning-speed-guard.service','clearpath-sensors.service','clearpath-scan-merger.service','dynamo-camera.service')


def desired_files():
    source=desktop_files()
    selected=[SYSTEM/VNC,SYSTEM/PROXY,SYSTEM/(REMOTE+'.d/20-independent-vnc.conf'),LIB/'vnc-xstartup']
    files={str(p):source[p] for p in selected}
    files[str(LIB/'controller_autoconnect.py')]={'content':Path(__file__).with_name('controller_autoconnect.py').read_text(),'mode':0o755}
    files[str(SYSTEM/(AUTO+'.d/30-dynamo-no-platform-restart.conf'))]={'content':'[Service]\nExecStart=\nExecStart=/usr/bin/python3 /usr/local/lib/dynamo-hardware/controller_autoconnect.py\n','mode':0o644}
    return files


def command(*args):
    return subprocess.run(args,check=True,capture_output=True,text=True,timeout=60).stdout.strip()


def state(unit):
    text=command('systemctl','show',unit,'-p','MainPID','-p','ActiveState','-p','UnitFileState')
    return dict(row.split('=',1) for row in text.splitlines() if '=' in row)


def save(root,value):
    temporary=root/'result.json.tmp';temporary.write_text(json.dumps(value,indent=2)+'\n');temporary.replace(root/'result.json')


def stage(root):
    root.mkdir(parents=True,exist_ok=False)
    files=desired_files()
    before={path:snapshot_file(Path(path)) for path in files}
    for unit in (VNC,PROXY,REMOTE,AUTO):
        (root/(unit+'.before.txt')).write_text(command('systemctl','cat',unit))
    for path,entry in files.items():
        target=root/'staged'/path.lstrip('/');target.parent.mkdir(parents=True,exist_ok=True);target.write_text(entry['content']);target.chmod(entry['mode'])
    units=[]
    for unit in (VNC,PROXY):
        target=root/unit;target.write_text(files[str(SYSTEM/unit)]['content']);units.append(str(target))
    command('systemd-analyze','verify',*units)
    record={'status':'staged-not-installed','files':files,'before':before,'activity':{u:state(u) for u in MANAGED},'namespace':identity(),'independent':{u:state(u) for u in INDEPENDENT},'installed':[],'limits':'Desktop restart will disconnect remote access. No hardware services are changed.'}
    save(root,record);return record


def load(root):
    record=json.loads((root/'result.json').read_text())
    if record['files']!=desired_files() or set(record['before'])!=set(record['files']):raise RuntimeError('staged inputs changed; stage again')
    if set(record['activity'])!=set(MANAGED) or set(record['independent'])!=set(INDEPENDENT):raise RuntimeError('invalid service scope')
    for path,entry in record['files'].items():
        if (root/'staged'/path.lstrip('/')).read_text()!=entry['content']:raise RuntimeError('staged file changed')
    return record


def restore_files(record):
    for name,before in record['before'].items():
        path=Path(name)
        if before is None:path.unlink(missing_ok=True)
        else:write_file(path,bytes.fromhex(before['content_hex']),before['mode'],before['uid'],before['gid'])


def restart_previous(record):
    command('systemctl','daemon-reload')
    for unit in (VNC,REMOTE,PROXY,AUTO):
        if record['activity'][unit]['ActiveState']=='active':command('systemctl','start',unit)


def apply(root,confirmed):
    if not confirmed or os.geteuid()!=0:raise RuntimeError('sudo and operator confirmation required; physical e-stop must remain pressed')
    record=load(root)
    if record['status']!='staged-not-installed':raise RuntimeError('stage is not fresh')
    for path,before in record['before'].items():
        if snapshot_file(Path(path))!=before:raise RuntimeError('host file changed: '+path)
    for unit in MANAGED:
        if state(unit)!=record['activity'][unit]:raise RuntimeError('service activity changed: '+unit)
    for unit in INDEPENDENT:
        if state(unit)!=record['independent'][unit]:raise RuntimeError('hardware activity changed since staging: '+unit)
    run_probe(record['namespace'],root,'stationary-before')
    record['status']='applying';save(root,record)
    try:
        for unit in MANAGED:command('systemctl','stop',unit)
        for name,entry in record['files'].items():
            # Persist intent before mutation so a crash has recoverable ownership.
            record['installed'].append(name);save(root,record)
            write_file(Path(name),entry['content'].encode(),entry['mode'])
        command('systemctl','daemon-reload')
        for unit in (VNC,PROXY,REMOTE,AUTO):command('systemctl','start',unit)
        deadline=time.monotonic()+60
        while True:
            try:
                listeners={str(port):listener_identity(port,unit) for port,unit in [(5901,VNC),(9005,PROXY),(9100,REMOTE)]};break
            except (OSError,RuntimeError,subprocess.SubprocessError):
                if time.monotonic()>=deadline:raise RuntimeError('desktop ownership verification failed')
                time.sleep(.25)
        for unit in INDEPENDENT:
            if state(unit)!=record['independent'][unit]:raise RuntimeError('hardware service changed: '+unit)
        run_probe(record['namespace'],root,'stationary-after')
        record.update(status='installed-verified-stationary',listeners=listeners);save(root,record)
    except Exception as error:
        record['error']=str(error);save(root,record)
        for unit in MANAGED:
            subprocess.run(['systemctl','stop',unit],capture_output=True,timeout=60)
        restore_files(record);restart_previous(record)
        record['status']='failed-restored-original';save(root,record);raise
    return record


def restore(root,confirmed):
    if not confirmed or os.geteuid()!=0:raise RuntimeError('sudo and operator confirmation required')
    record=load(root)
    if record['status'] not in ('installed-verified-stationary','applying'):raise RuntimeError('nothing installed to restore')
    for name in record['installed']:
        snapshot=snapshot_file(Path(name));before=record['before'][name];entry=record['files'][name]
        expected=entry['content'].encode().hex()
        if snapshot!=before and (snapshot is None or snapshot['content_hex']!=expected or snapshot['mode']!=entry['mode'] or snapshot['uid']!=0 or snapshot['gid']!=0):raise RuntimeError('refusing restore over changed host file: '+name)
    run_probe(record['namespace'],root,'restore-before')
    for unit in MANAGED:command('systemctl','stop',unit)
    restore_files(record);restart_previous(record)
    record['status']='restored-original';save(root,record);return record


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    mode=parser.add_mutually_exclusive_group(required=True)
    for name in ['stage','apply','restore']:mode.add_argument('--'+name,action='store_true')
    parser.add_argument('--run-dir',type=Path,required=True);parser.add_argument('--operator-confirmed',action='store_true');parser.add_argument('--detach',action='store_true')
    args=parser.parse_args();root=args.run_dir.resolve()
    if not root.is_relative_to(ROOT/'artifacts/hardware') or root==ROOT/'artifacts/hardware':parser.error('fresh artifacts/hardware directory required')
    if args.detach:
        if args.stage or not args.operator_confirmed or os.geteuid()!=0:parser.error('detachment requires sudo, apply/restore and operator confirmation')
        load(root)
        unit='dynamo-stationary-setup-'+str(int(time.time()))
        command('systemd-run','--collect','--unit='+unit,'--property=Type=exec','/usr/bin/python3',str(Path(__file__).resolve()),'--restore' if args.restore else '--apply','--operator-confirmed','--run-dir',str(root))
        print(json.dumps({'status':'detached','unit':unit,'result':str(root/'result.json')}));return 0
    if args.stage:
        record=stage(root)
    else:
        if os.geteuid()!=0:parser.error('sudo is required for apply/restore')
        fd=os.open('/run/lock/dynamo-stationary-setup.lock',os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW,0o600)
        try:
            if os.fstat(fd).st_uid!=0:raise RuntimeError('unexpected lock owner')
            fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
            record=restore(root,args.operator_confirmed) if args.restore else apply(root,args.operator_confirmed)
        finally:os.close(fd)
    print(json.dumps({'status':record['status'],'result':str(root/'result.json')}));return 0

if __name__=='__main__':raise SystemExit(main())
