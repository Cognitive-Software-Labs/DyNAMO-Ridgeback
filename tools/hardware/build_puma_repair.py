#!/usr/bin/env python3
"""Build and sanitizer-test the pinned Puma repair into a fresh evidence directory.

Consumes a downloaded official source archive; never downloads, installs,
participates in ROS, opens CAN, or changes a live source checkout.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile

from verify_puma_cache_patch import verify as verify_cache
from verify_puma_fault_patch import verify as verify_fault

ROOT=Path(__file__).resolve().parents[2]
ARCHIVE_SHA='49d0ef5bcc6421856d6a4644aa4bd0dd117b1052d569d5b3a3238385184206df'
PATCHES=('clearpath_puma_fault_flags.patch','clearpath_puma_cache_safety.patch')
DEPENDENCIES=('ros-jazzy-puma-motor-driver','ros-jazzy-can-hardware',
              'ros-jazzy-clearpath-motor-msgs','ros-jazzy-rclcpp','ros-jazzy-diagnostic-updater')


def versions():
    return subprocess.run(['dpkg-query','-W','-f=${Package}\t${Version}\n',*DEPENDENCIES],
                          check=True,capture_output=True,text=True).stdout


def apply_patch_checked(checkout, patch):
    # git apply can silently skip paths when this tree lies below an enclosing
    # Git repository's ignored artifacts directory. patch uses this cwd directly.
    base=['patch','--batch','--forward','-p1','--input',str(patch)]
    subprocess.run(base+['--dry-run'],cwd=checkout,check=True,capture_output=True,text=True)
    subprocess.run(base,cwd=checkout,check=True,capture_output=True,text=True)
    subprocess.run(['patch','--batch','--reverse','--dry-run','-p1','--input',str(patch)],
                   cwd=checkout,check=True,capture_output=True,text=True)


def verify_applied_patches(package):
    with tempfile.TemporaryDirectory(prefix='verify-puma-patches-') as directory:
        tree=Path(directory)
        shutil.copytree(package,tree/'clearpath_motor_drivers/puma_motor_driver')
        for name in PATCHES:
            subprocess.run(['patch','--batch','--reverse','--dry-run','-p1','--input',str(ROOT/'patches'/name)],
                           cwd=tree,check=True,capture_output=True,text=True)


def build(archive,output):
    output=output.resolve()
    if (not output.is_relative_to(ROOT/'artifacts/hardware') and
            not output.is_relative_to(Path('/tmp'))) or output in (ROOT/'artifacts/hardware',Path('/tmp')):
        raise ValueError('build into a fresh subdirectory of artifacts/hardware or /tmp')
    if hashlib.sha256(archive.read_bytes()).hexdigest()!=ARCHIVE_SHA:
        raise ValueError('archive differs from inspected official Clearpath 2.9.4')
    dependency_versions=versions()
    output.mkdir(parents=True,exist_ok=False)
    upstream=output/'upstream';upstream.mkdir()
    with tarfile.open(archive) as source:
        source.extractall(upstream,filter='data')
    roots=list(upstream.iterdir())
    if len(roots)!=1 or roots[0].name!='clearpath_robot-2.9.4':
        raise ValueError('unexpected source archive layout')
    checkout=roots[0];package=checkout/'clearpath_motor_drivers/puma_motor_driver'
    fault=verify_fault(package/'src/multi_puma_node.cpp')
    (output/'fault-verification.json').write_text(json.dumps(fault,indent=2)+'\n')
    verify_cache(package,output/'cache-sanitizers')
    for name in PATCHES:
        patch=ROOT/'patches'/name
        apply_patch_checked(checkout,patch)
    verify_applied_patches(package)
    src=output/'src/puma_motor_driver';src.parent.mkdir()
    shutil.copytree(package,src)
    command=['/bin/bash','-c',
             'unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH; '
             'source /opt/ros/jazzy/setup.bash && exec nice -n 10 colcon "$@"','puma-build',
             '--log-base',str(output/'log'),'build','--base-paths',str(output/'src'),
             '--build-base',str(output/'build'),'--install-base',str(output/'install'),
             '--executor','sequential','--packages-select','puma_motor_driver',
             '--allow-overriding','puma_motor_driver','--cmake-args',
             '-DBUILD_TESTING=OFF','-DCMAKE_BUILD_TYPE=Release']
    with (output/'build.log').open('w') as log:
        subprocess.run(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,check=True)
    if versions()!=dependency_versions:
        raise RuntimeError('runtime dependencies changed during candidate build')
    binary=output/'install/puma_motor_driver/lib/puma_motor_driver/multi_puma_node'
    source_hashes={str(f.relative_to(output/'src')):hashlib.sha256(f.read_bytes()).hexdigest()
                   for f in sorted(src.rglob('*')) if f.is_file()}
    record={'archive_sha256':ARCHIVE_SHA,'upstream_tag':'2.9.4',
            'archive_url':'https://codeload.github.com/clearpathrobotics/clearpath_robot/tar.gz/refs/tags/2.9.4',
            'build_status':'completed-not-installed','candidate_only_not_installed':True,
            'patch_application_verified':True,
            'candidate_source_hashes':source_hashes,
            'binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),
            'patch_hashes':{name:hashlib.sha256((ROOT/'patches'/name).read_bytes()).hexdigest()
                            for name in PATCHES},'dependency_versions':dependency_versions,
            'source':'tools/hardware/build_puma_repair.py'}
    (output/'SOURCE.json').write_text(json.dumps(record,indent=2)+'\n')
    return record


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    print(json.dumps(build(args.archive,args.output),indent=2))
