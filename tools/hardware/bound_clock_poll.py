#!/usr/bin/env python3
"""Bound Intel chrony source polls to 128 s without restarting or stepping clocks."""
import argparse
import csv
import fcntl
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
PROFILE = Path('/etc/chrony/conf.d/dynamo-time.conf')
OLD = 'pool ntp.ubuntu.com iburst maxsources 4'
NEW = OLD + ' maxpoll 7'


def command(*args):
    p = subprocess.run(args, check=True, capture_output=True, text=True, timeout=15)
    return p.stdout


def candidate(text):
    if text.splitlines().count(OLD) != 1 or 'maxslewrate 500' not in text.splitlines():
        raise RuntimeError('unexpected installed Intel chrony profile')
    return text.replace(OLD + '\n', NEW + '\n', 1)


def addresses():
    rows = list(csv.reader(command('chronyc', '-c', '-n', 'sources').splitlines()))
    result = [row[2] for row in rows if len(row) > 4 and row[2] and row[2] != '0.0.0.0']
    if len(result) < 2 or len(set(result)) != len(result) or any(not re.fullmatch(r'[0-9.]+', a) for a in result):
        raise RuntimeError('expected multiple distinct numeric IPv4 sources')
    return result


def write_profile(text, stat):
    fd, name = tempfile.mkstemp(dir=PROFILE.parent, prefix='.dynamo-poll-')
    try:
        with os.fdopen(fd, 'w') as f:
            f.write(text);f.flush();os.fsync(f.fileno())
        os.chown(name, stat['uid'], stat['gid']);os.chmod(name, stat['mode'])
        os.replace(name, PROFILE)
    finally:
        if os.path.exists(name):os.unlink(name)


def save(root, record):
    tmp=root/'result.tmp';tmp.write_text(json.dumps(record,indent=2)+'\n');tmp.replace(root/'result.json')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    action=parser.add_mutually_exclusive_group(required=True)
    action.add_argument('--stage',action='store_true')
    action.add_argument('--apply',action='store_true')
    action.add_argument('--restore',action='store_true')
    parser.add_argument('--operator-confirmed',action='store_true')
    parser.add_argument('--run-dir',type=Path,required=True)
    args=parser.parse_args()
    root=args.run_dir.resolve()
    if not root.is_relative_to(ROOT/'artifacts/hardware') or root==ROOT/'artifacts/hardware':
        parser.error('run directory must be under artifacts/hardware')
    if args.stage:
        root.mkdir(parents=True,exist_ok=False)
        before=PROFILE.read_text();after=candidate(before);stat=PROFILE.stat()
        (root/'candidate.conf').write_text(after)
        record={'status':'staged','before':before,'after':after,
                'metadata':{'uid':stat.st_uid,'gid':stat.st_gid,'mode':stat.st_mode&0o777},
                'sources_before':command('chronyc','-n','sources'),'addresses':addresses(),
                'intent':[],'limits':'Keeps 180-second gate; 128-second source poll cap. No clock step or restart.'}
        save(root,record)
    else:
        if os.geteuid()!=0 or not args.operator_confirmed:parser.error('sudo and operator confirmation required')
        record=json.loads((root/'result.json').read_text())
        fd=os.open('/run/lock/dynamo-clock-poll.lock',os.O_RDWR|os.O_CREAT|os.O_NOFOLLOW,0o600)
        try:
            fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
            if args.apply:
                if record['status']!='staged' or PROFILE.read_text()!=record['before'] or addresses()!=record['addresses']:
                    raise RuntimeError('staged inputs changed; no clock setting changed')
                # Chronyd on this host reads configs only from its confined /etc/chrony path.
                validation=PROFILE.parent/'.dynamo-poll-validation.conf'
                if validation.exists() or validation.is_symlink():raise RuntimeError('validation path exists')
                try:
                    validation.write_text(record['after']);validation.chmod(0o644)
                    command('chronyd','-p','-f',str(validation))
                finally:validation.unlink(missing_ok=True)
                record['status']='applying';save(root,record)
                try:
                    for address in record['addresses']:
                        record['intent'].append(address);save(root,record)
                        command('chronyc','maxpoll',address,'7')
                    write_profile(record['after'],record['metadata'])
                    record['status']='installed-poll-bound';record['sources_after']=command('chronyc','-n','sources')
                except Exception as error:
                    for address in record['intent']:
                        try:command('chronyc','maxpoll',address,'10')
                        except Exception:pass
                    if PROFILE.read_text()==record['after']:write_profile(record['before'],record['metadata'])
                    record.update(status='failed-rollback-attempted',error=str(error));save(root,record)
                    raise
            else:
                if record['status'] not in ('installed-poll-bound','applying') or PROFILE.read_text() not in (record['before'],record['after']):
                    raise RuntimeError('unexpected profile or receipt; refusing rollback')
                for address in record['intent']:
                    command('chronyc','maxpoll',address,'10')
                write_profile(record['before'],record['metadata'])
                record['status']='restored-original-poll';record['sources_restored']=command('chronyc','-n','sources')
            save(root,record)
        finally:os.close(fd)
    print(json.dumps({'status':record['status'],'result':str(root/'result.json')}))

if __name__=='__main__':main()
