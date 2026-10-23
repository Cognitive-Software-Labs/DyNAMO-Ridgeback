#!/usr/bin/env python3
"""Reproduce the offline feedback-consumer candidate; never install or access live ROS/CAN."""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import shutil
import subprocess
import tarfile
from build_puma_repair import ARCHIVE_SHA, apply_patch_checked

ROOT = Path(__file__).resolve().parents[2]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def build(archive, output):
    output = output.resolve()
    if not output.is_relative_to(ROOT / 'artifacts/hardware') or output == ROOT / 'artifacts/hardware':
        raise ValueError('output must be a fresh directory below artifacts/hardware')
    if digest(archive) != ARCHIVE_SHA:
        raise ValueError('requires the pinned official clearpath_robot 2.9.4 archive')
    output.mkdir(parents=True, exist_ok=False)
    upstream = output / 'upstream'
    upstream.mkdir()
    with tarfile.open(archive) as source:
        # Accept only ordinary files/directories beneath this verified source root.
        members = [m for m in source.getmembers() if m.name.startswith('clearpath_robot-2.9.4/clearpath_hardware_interfaces/')]
        if not members or any(not (m.isfile() or m.isdir()) or '..' in Path(m.name).parts for m in members):
            raise ValueError('unexpected source archive layout')
        source.extractall(upstream, members=members, filter='data')
    tree = upstream / 'clearpath_robot-2.9.4'
    patch = ROOT / 'patches/clearpath_puma_feedback_consumer.patch'
    patch_snapshot = output / patch.name
    shutil.copyfile(patch, patch_snapshot)
    apply_patch_checked(tree, patch_snapshot)
    package = output / 'src/clearpath_hardware_interfaces'
    shutil.copytree(tree / 'clearpath_hardware_interfaces', package)
    header = ROOT / 'tools/hardware/puma_feedback_safety.hpp'
    header_snapshot = package / 'include/clearpath_hardware_interfaces/puma/feedback_safety.hpp'
    shutil.copyfile(header, header_snapshot)
    for script, args in [
        ('check_puma_feedback_safety.py', ['--output', str(output / 'policy-check')]),
        ('check_puma_feedback_adapter.py', ['--package', str(package), '--output', str(output / 'adapter-check')]),
    ]:
        result = subprocess.run(['python3', str(ROOT / 'tools/hardware' / script), *args], check=True, capture_output=True, text=True)
        (output / (script + '.log')).write_text(result.stdout + result.stderr)
    command = 'source /opt/ros/jazzy/setup.bash && colcon --log-base ' + shlex.quote(str(output / 'colcon-log')) + ' build --base-paths ' + shlex.quote(str(output / 'src')) + ' --build-base ' + shlex.quote(str(output / 'build')) + ' --install-base ' + shlex.quote(str(output / 'install')) + ' --packages-select clearpath_hardware_interfaces'
    with (output / 'build.log').open('w') as log:
        subprocess.run(['bash', '-c', command], cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=True)
    if digest(header) != digest(header_snapshot) or digest(patch) != digest(patch_snapshot):
        raise RuntimeError('maintained inputs changed during build; rerun in a fresh directory')
    sources = {str(p.relative_to(package)): digest(p) for p in package.rglob('*') if p.is_file()}
    versions = subprocess.run(['dpkg-query', '-W', '-f=${Package}\t${Version}\n', 'ros-jazzy-clearpath-hardware-interfaces', 'ros-jazzy-hardware-interface', 'ros-jazzy-clearpath-motor-msgs', 'ros-jazzy-rclcpp'], check=True, capture_output=True, text=True).stdout
    (output / 'package-versions.txt').write_text(versions)
    report = {'status': 'built-offline-not-installed', 'archive_sha256': ARCHIVE_SHA,
              'patch_sha256': digest(patch_snapshot), 'policy_sha256': digest(header_snapshot), 'sources': sources,
              'libraries': {str(p.relative_to(output)): digest(p) for p in (output / 'install').rglob('libpuma_hardware.so')},
              'limits': 'Runtime bodies tested with inert ROS. Real ROS lifecycle/controller-manager behavior and physical stopping remain unqualified. No deployment approval implied.'}
    (output / 'SOURCE.json').write_text(json.dumps(report, indent=2) + '\n')
    return {'status': report['status'], 'path': str(output / 'SOURCE.json')}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-archive', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(build(args.source_archive, args.output), indent=2))
