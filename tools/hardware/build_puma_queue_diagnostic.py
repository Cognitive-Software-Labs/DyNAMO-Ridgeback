#!/usr/bin/env python3
"""Build a pinned, diagnostic-only Puma queue-timing candidate offline.

This tool never installs the node, opens CAN, or changes the running platform.
It retains the installed feedback policy and adds five-second queue-age logs.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tarfile

from build_puma_repair import ARCHIVE_SHA, PATCHES, ROOT, apply_patch_checked, versions

DIAGNOSTIC_PATCH = 'clearpath_puma_queue_diagnostic.patch'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def build(archive, output):
    archive, output = archive.resolve(), output.resolve()
    evidence = ROOT / 'artifacts/hardware'
    if output == evidence or not output.is_relative_to(evidence) or output.exists():
        raise ValueError('use a fresh output below artifacts/hardware')
    if digest(archive) != ARCHIVE_SHA:
        raise ValueError('source archive differs from pinned Clearpath 2.9.4')
    patch_names = (*PATCHES, DIAGNOSTIC_PATCH)
    patches = [ROOT / 'patches' / name for name in patch_names]
    inputs = {str(path.relative_to(ROOT)): digest(path) for path in patches}
    dependency_versions = versions()
    output.mkdir(parents=True)
    upstream = output / 'upstream'
    upstream.mkdir()
    with tarfile.open(archive) as source:
        source.extractall(upstream, filter='data')
    checkout = upstream / 'clearpath_robot-2.9.4'
    if not checkout.is_dir() or len(list(upstream.iterdir())) != 1:
        raise ValueError('unexpected pinned source layout')
    for patch in patches:
        apply_patch_checked(checkout, patch)
    package = output / 'src/puma_motor_driver'
    package.parent.mkdir()
    shutil.copytree(checkout / 'clearpath_motor_drivers/puma_motor_driver', package)
    snapshots = output / 'inputs'
    snapshots.mkdir()
    for patch in patches:
        shutil.copy2(patch, snapshots / patch.name)
    receipt = {'status': 'offline-build-in-progress', 'not_deployable': True,
               'archive_sha256': ARCHIVE_SHA, 'input_sha256': inputs,
               'dependency_versions': dependency_versions,
               'policy_change': False, 'motor_node_started': False,
               'system_files_changed': False}
    (output / 'candidate.json').write_text(json.dumps(receipt, indent=2) + '\n')
    command = ['/bin/bash', '-c',
               'unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH; '
               'source /opt/ros/jazzy/setup.bash && exec nice -n 10 colcon "$@"', 'queue-build',
               '--log-base', str(output / 'log'), 'build', '--base-paths', str(output / 'src'),
               '--build-base', str(output / 'build'), '--install-base', str(output / 'install'),
               '--executor', 'sequential', '--packages-select', 'puma_motor_driver',
               '--allow-overriding', 'puma_motor_driver', '--cmake-args',
               '-DBUILD_TESTING=OFF', '-DCMAKE_BUILD_TYPE=Release']
    with (output / 'build.log').open('w') as log:
        subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=True)
    if versions() != dependency_versions or any(digest(ROOT / name) != sha for name, sha in inputs.items()):
        raise RuntimeError('inputs or dependencies changed during the build')
    binary = output / 'install/puma_motor_driver/lib/puma_motor_driver/multi_puma_node'
    receipt.update(status='offline-queue-diagnostic-built-not-installed',
                   binary_sha256=digest(binary),
                   limits='Compiled diagnostic only. It has not run on live CAN or under load. '
                          'Callback-to-dequeue age omits kernel/network delay, and no production '
                          'field-age threshold is selected.')
    (output / 'candidate.json').write_text(json.dumps(receipt, indent=2) + '\n')
    return receipt


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(build(args.archive, args.output), indent=2))
