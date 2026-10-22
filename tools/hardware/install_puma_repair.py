#!/usr/bin/env python3
"""Stage a pinned Puma repair and DDS network-readiness gate beside existing caps.

Apply/restore require a stationary robot, physical e-stop pressed and an
operator present. Only platform and guard are stopped/started. No command,
reset/enable publication, network change or camera/sensor/desktop restart.
"""
import argparse
import base64
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import install_commissioning_caps as caps
from build_puma_repair import versions, PATCHES, verify_applied_patches

ROOT=Path(__file__).resolve().parents[2]
PREFIX_ROOT=Path('/opt/dynamo-puma')
WAIT=Path('/usr/local/lib/dynamo-hardware/wait_dds_interfaces.py')
GUARD_DROP=Path('/etc/systemd/system/dynamo-commissioning-speed-guard.service.d/20-dynamo-dds-readiness.conf')
PLATFORM_DROP=Path('/etc/systemd/system/clearpath-platform.service.d/80-dynamo-puma-repair.conf')
OWNED={WAIT,GUARD_DROP,PLATFORM_DROP}
INDEPENDENT=('ridgeback-vnc.service','realsense-camera.service','clearpath-sensors.service')


def file_manifest(root):
    result={}
    if root.is_symlink():
        raise ValueError('refuse symlink package root')
    for path in sorted(root.rglob('*')):
        if path.is_symlink():
            raise ValueError('refuse symlink in pinned package: '+str(path))
        if path.is_file():
            if path.stat().st_mode & 0o7777 not in (0o644,0o755):
                raise ValueError('unexpected pinned-file permissions: '+str(path))
            result[str(path.relative_to(root))]={'sha256':caps.digest(path.read_bytes()),
                                               'mode':path.stat().st_mode & 0o777}
        elif not path.is_dir():
            raise ValueError('refuse special file in pinned package')
    if not result:
        raise ValueError('empty Puma package')
    return result


def owned_files(prefix):
    return {
        WAIT:WAIT_SOURCE.read_bytes(),
        GUARD_DROP:('''[Service]
ExecStartPre=/usr/bin/python3 /usr/local/lib/dynamo-hardware/wait_dds_interfaces.py --profile /etc/clearpath/cyclonedds.xml --timeout 60
TimeoutStartSec=75
''').encode(),
        PLATFORM_DROP:('''[Service]
ExecStart=
ExecStart=/bin/bash -c 'source /etc/clearpath/setup.bash && source '''+
                      str(prefix)+'''/share/puma_motor_driver/local_setup.bash && exec ros2 launch /usr/local/lib/dynamo-hardware/commissioning_platform.launch.py'
''').encode(),
    }


WAIT_SOURCE=Path(__file__).with_name('wait_dds_interfaces.py')


def check_caps(ns):
    for interlock in (False,True):
        files=caps.desired_files(ns,interlock)
        if all(not path.is_symlink() and path.is_file() and path.read_bytes()==data for path,data in files.items()):
            return {str(path):caps.digest(data) for path,data in files.items()}
    raise RuntimeError('verified normal or test-interlocked capped installation required')


def run_directory(path):
    path=path.resolve()
    if not path.is_relative_to(ROOT/'artifacts/hardware') or path==ROOT/'artifacts/hardware':
        raise ValueError('run directory must be below repository artifacts/hardware')
    return path


def stage(candidate,run):
    run=run_directory(run)
    if run.exists():raise ValueError('choose a fresh staging directory')
    candidate=candidate.resolve()
    if not candidate.is_relative_to(ROOT/'artifacts/hardware'):
        raise ValueError('candidate must be below repository artifacts/hardware')
    provenance=json.loads((candidate/'SOURCE.json').read_text())
    if provenance.get('patch_application_verified') is not True:
        raise ValueError('candidate lacks verified applied patches; rebuild with corrected builder')
    if provenance.get('build_status')!='completed-not-installed' or provenance.get('upstream_tag')!='2.9.4':
        raise ValueError('candidate lacks completed pinned build provenance')
    if provenance.get('archive_sha256')!='49d0ef5bcc6421856d6a4644aa4bd0dd117b1052d569d5b3a3238385184206df':
        raise ValueError('wrong official source archive')
    patch_hashes={name:caps.digest((ROOT/'patches'/name).read_bytes()) for name in PATCHES}
    if provenance.get('patch_hashes')!=patch_hashes:
        raise ValueError('candidate was not built with current maintained patches')
    dependency_versions=versions()
    if provenance.get('dependency_versions')!=dependency_versions:
        raise ValueError('runtime dependencies differ from candidate build')
    cache=json.loads((candidate/'cache-sanitizers/result.json').read_text())
    fault=json.loads((candidate/'fault-verification.json').read_text())
    if (cache.get('status')!='sanitizer-verified-not-installed' or
            cache.get('patch_sha256')!=patch_hashes['clearpath_puma_cache_safety.patch'] or
            fault.get('status')!='verified-patch-only-not-installed' or
            fault.get('patch_sha256')!=patch_hashes['clearpath_puma_fault_flags.patch']):
        raise ValueError('candidate lacks matching cache/fault verification')
    for name,digest in provenance['candidate_source_hashes'].items():
        source=candidate/'src'/name
        if not source.resolve().is_relative_to(candidate/'src') or caps.digest(source.read_bytes())!=digest:
            raise ValueError('candidate source changed since build')
    verify_applied_patches(candidate/'src/puma_motor_driver')
    package=candidate/'install/puma_motor_driver'
    binary=package/'lib/puma_motor_driver/multi_puma_node'
    if binary.read_bytes()[:4]!=b'\x7fELF' or caps.digest(binary.read_bytes())!=provenance['binary_sha256']:
        raise ValueError('built driver differs from recorded candidate')
    manifest=file_manifest(package)
    package_hash=caps.digest(json.dumps(manifest,sort_keys=True).encode())
    prefix=PREFIX_ROOT/package_hash
    ns=caps.identity()
    baseline=check_caps(ns)
    run.mkdir(parents=True)
    shutil.copytree(package,run/'package')
    shutil.copy2(candidate/'SOURCE.json',run/'SOURCE.json')
    files=owned_files(prefix)
    for path,data in files.items():
        target=run/'staged'/str(path).lstrip('/');target.parent.mkdir(parents=True,exist_ok=True)
        target.write_bytes(data)
    record={'status':'staged','namespace':ns,'prefix':str(prefix),'package_manifest':manifest,
            'package_hash':package_hash,'baseline_caps':baseline,'dependency_versions':dependency_versions,
            'files':{str(path):caps.digest(data) for path,data in files.items()},
            'source':'tools/hardware/install_puma_repair.py','candidate':str(candidate)}
    (run/'result.json').write_text(json.dumps(record,indent=2)+'\n')
    return record


def read_record(run):
    run=run_directory(run)
    record=json.loads((run/'result.json').read_text())
    if json.loads((run/'SOURCE.json').read_text()).get('patch_application_verified') is not True:
        raise ValueError('staged package lacks verified applied patches; old candidate must not be installed')
    if set(map(Path,record['files']))!=OWNED:
        raise ValueError('record has unexpected owned paths')
    prefix=Path(record['prefix'])
    if prefix.parent!=PREFIX_ROOT or prefix.name!=record['package_hash']:
        raise ValueError('invalid pinned prefix')
    if caps.digest(json.dumps(record['package_manifest'],sort_keys=True).encode())!=prefix.name:
        raise ValueError('package manifest changed')
    return run,record,prefix


def deployed_driver(prefix):
    expected=str(prefix/'lib/puma_motor_driver/multi_puma_node')
    drivers=[]
    for directory in Path('/proc').iterdir():
        if not directory.name.isdecimal():continue
        try:args=(directory/'cmdline').read_bytes().split(b'\0')
        except (FileNotFoundError,PermissionError,ProcessLookupError):continue
        if args and args[0].decode(errors='replace').endswith('/multi_puma_node'):
            drivers.append({'pid':int(directory.name),'executable':args[0].decode()})
    if len(drivers)!=1 or drivers[0]['executable']!=expected:
        raise RuntimeError('unexpected Puma executable ownership: '+json.dumps(drivers))
    return drivers[0]


def secure_prefix(prefix):
    """The service install must not follow a robot-owned workspace parent."""
    for path in [prefix.parent,*prefix.parent.parents]:
        info=path.lstat()
        if path.is_symlink() or info.st_uid!=0 or info.st_mode & 0o022:
            raise RuntimeError('pinned prefix parent is not root-controlled: '+str(path))


def root_owned_package(prefix):
    if any(path.lstat().st_uid!=0 for path in [prefix]+list(prefix.rglob('*'))):
        raise RuntimeError('pinned package is not wholly root-owned')


def apply(run,operator):
    if os.geteuid()!=0 or not operator:
        raise RuntimeError('apply requires sudo and --operator-confirmed with physical stop pressed')
    run,record,prefix=read_record(run)
    if record['status']!='staged':raise ValueError('only a staged record can be applied')
    if check_caps(record['namespace'])!=record['baseline_caps']:
        raise RuntimeError('capped installation changed after staging')
    if file_manifest(run/'package')!=record['package_manifest']:
        raise RuntimeError('staged package changed')
    if versions()!=record['dependency_versions']:
        raise RuntimeError('runtime dependencies changed after build/staging')
    staged={path:(run/'staged'/str(path).lstrip('/')).read_bytes() for path in OWNED}
    if {str(p):caps.digest(data) for p,data in staged.items()}!=record['files']:
        raise RuntimeError('staged host files changed')
    if any(path.exists() or path.is_symlink() for path in OWNED):
        raise RuntimeError('repair-owned host files already exist; inspect or restore prior repair')
    caps.run_probe(record['namespace'],run,'before')
    units=(caps.PLATFORM,caps.GUARD)+INDEPENDENT
    before={u:caps.state(u) for u in units}
    if any(before[u].get('ActiveState')!='active' for u in (caps.PLATFORM,caps.GUARD)):
        raise RuntimeError('verified capped platform and guard must be active')
    (run/'backup.json').write_text(json.dumps({'services':before,
        'baseline_caps':record['baseline_caps'],'owned_paths_existed':False},indent=2)+'\n')
    prefix.parent.mkdir(parents=True,exist_ok=True)
    secure_prefix(prefix)
    if prefix.exists():
        if file_manifest(prefix)!=record['package_manifest']:
            raise RuntimeError('existing pinned package differs')
    else:
        shutil.copytree(run/'package',prefix)
        for path in [prefix]+list(prefix.rglob('*')):
            os.chown(path,0,0)
            if path.is_dir():path.chmod(0o755)
    root_owned_package(prefix)
    stopped=False
    try:
        caps.systemctl('stop',caps.PLATFORM);stopped=True
        caps.systemctl('stop',caps.GUARD)
        for path,data in staged.items():caps.write_file(path,data)
        caps.systemctl('daemon-reload')
        caps.systemctl('start',caps.PLATFORM)
        caps.run_probe(record['namespace'],run,'after')
        record['driver_process']=deployed_driver(prefix)
        if file_manifest(prefix)!=record['package_manifest']:
            raise RuntimeError('installed package differs from staged package')
        after={u:caps.state(u) for u in units}
        record['service_states']=after
        record['independent_service_changes']=[u for u in INDEPENDENT if before[u]!=after[u]]
        record['status']='installed-verified-stationary'
    except Exception as error:
        if stopped:caps.systemctl('stop',caps.PLATFORM,check=False)
        record['status']='failed-platform-left-stopped';record['error']=str(error)
        raise
    finally:(run/'result.json').write_text(json.dumps(record,indent=2)+'\n')
    return record


def restore(run,operator):
    if os.geteuid()!=0 or not operator:
        raise RuntimeError('restore requires sudo and physical stop pressed with operator present')
    run,record,prefix=read_record(run)
    backup=json.loads((run/'backup.json').read_text())
    if backup.get('owned_paths_existed') is not False:
        raise RuntimeError('unsupported previous-file ownership')
    if check_caps(record['namespace'])!=backup['baseline_caps']:
        raise RuntimeError('original capped installation changed')
    installed=[]
    for path in OWNED:
        if not path.exists() and not path.is_symlink() and record['status']=='failed-platform-left-stopped':
            continue  # An interrupted install may have written only some owned files.
        if path.is_symlink() or not path.is_file() or caps.digest(path.read_bytes())!=record['files'][str(path)]:
            raise RuntimeError('installed repair file changed; refuse rollback: '+str(path))
        installed.append(path)
    if caps.state(caps.PLATFORM).get('ActiveState')=='active':
        caps.run_probe(record['namespace'],run,'restore-before')
    caps.systemctl('stop',caps.PLATFORM)
    caps.systemctl('stop',caps.GUARD)
    for path in installed:path.unlink()
    caps.systemctl('daemon-reload')
    try:
        caps.systemctl('start',caps.PLATFORM)
        caps.run_probe(record['namespace'],run,'after')
    except Exception:
        caps.systemctl('stop',caps.PLATFORM,check=False)
        raise
    record['status']='restored-original-capped-driver'
    record['unused_pinned_prefix_retained']=str(prefix)
    (run/'restore-result.json').write_text(json.dumps(record,indent=2)+'\n')
    return record


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate',type=Path)
    parser.add_argument('--run-dir',type=Path,required=True)
    parser.add_argument('--apply',action='store_true')
    parser.add_argument('--restore',action='store_true')
    parser.add_argument('--operator-confirmed',action='store_true')
    args=parser.parse_args()
    if args.apply and args.restore:parser.error('apply and restore are exclusive')
    if (args.apply or args.restore) and args.candidate:
        parser.error('apply/restore use the frozen staged record; omit --candidate')
    if not (args.apply or args.restore) and not args.candidate:
        parser.error('staging requires --candidate')
    try:
        record=(apply(args.run_dir,args.operator_confirmed) if args.apply else
                restore(args.run_dir,args.operator_confirmed) if args.restore else
                stage(args.candidate,args.run_dir))
        print(json.dumps(record,indent=2));return 0
    except Exception as error:
        print('Puma repair stopped: '+str(error),file=sys.stderr);return 1


if __name__=='__main__':raise SystemExit(main())
