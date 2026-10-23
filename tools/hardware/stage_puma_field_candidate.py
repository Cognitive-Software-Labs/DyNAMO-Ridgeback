#!/usr/bin/env python3
"""Build the experimental per-field age driver offline; never install or run it.

The only executed candidate is an integration checker with null ROS/CAN handles.
The motor node is compiled but never started. No SOURCE.json deployment receipt
is emitted, so this output is not an input to the normal Puma installer.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tarfile

from build_puma_repair import ARCHIVE_SHA, PATCHES, ROOT, apply_patch_checked, versions
from verify_puma_field_node import verify as verify_aggregate


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if digest(args.archive) != ARCHIVE_SHA:
        parser.error('archive differs from the pinned Clearpath 2.9.4 source')
    root = args.output.resolve()
    evidence = ROOT/'artifacts/hardware'
    if root == evidence or not root.is_relative_to(evidence):
        parser.error('use a fresh directory under artifacts/hardware')
    patches = [ROOT/'patches'/name for name in (*PATCHES, 'clearpath_puma_freshness.patch',
                                              'clearpath_puma_field_age.patch',
                                              'clearpath_puma_queue_timing.patch')]
    local = [Path(__file__), Path(__file__).with_name('puma_field_freshness.hpp'),
             Path(__file__).with_name('puma_field_driver_check.cpp'),
             Path(__file__).with_name('verify_puma_field_node.py'),
             Path(__file__).with_name('verify_puma_freshness_patch.py')]
    inputs = {str(path.relative_to(ROOT)): digest(path) for path in patches+local}
    dependency_versions = versions()
    root.mkdir(parents=True, exist_ok=False)
    upstream = root/'upstream'; upstream.mkdir()
    with tarfile.open(args.archive) as archive:
        archive.extractall(upstream, filter='data')
    checkout = upstream/'clearpath_robot-2.9.4'
    if not checkout.is_dir() or len(list(upstream.iterdir())) != 1:
        raise ValueError('unexpected source archive layout')
    for path in patches:
        apply_patch_checked(checkout, path)
    package = root/'src/puma_motor_driver'
    package.parent.mkdir()
    shutil.copytree(checkout/'clearpath_motor_drivers/puma_motor_driver', package)
    shutil.copy2(local[1], package/'include/puma_motor_driver/puma_field_freshness.hpp')
    shutil.copy2(local[2], package/'src/puma_field_driver_check.cpp')
    cmake = package/'CMakeLists.txt'
    cmake.write_text(cmake.read_text()+'''
# Offline verification target only. No ROS node or CAN transport is created.
add_executable(puma_field_driver_check src/puma_field_driver_check.cpp src/driver.cpp)
target_include_directories(puma_field_driver_check PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(puma_field_driver_check PRIVATE cxx_std_17)
ament_target_dependencies(puma_field_driver_check ${DEPENDENCIES})
target_compile_options(puma_field_driver_check PRIVATE -fsanitize=undefined -fno-sanitize-recover=all)
target_link_options(puma_field_driver_check PRIVATE -fsanitize=undefined)
''')
    snapshots = root/'inputs'; snapshots.mkdir()
    for path in patches+local:
        shutil.copy2(path, snapshots/path.name)
    record = {'status': 'offline-candidate-build-in-progress', 'not_deployable': True,
              'archive_sha256': ARCHIVE_SHA, 'input_sha256': inputs,
              'dependency_versions': dependency_versions, 'speed_mode_only': True,
              'candidate_age_bounds_ms': {'motion': 150, 'auxiliary': 500},
              'production_age_bound_selected': False,
              'motor_node_started': False, 'system_files_changed': False,
              'queue_timing_diagnostic': 'five-second callback-to-dequeue mean/max and count above 100 ms'}
    (root/'candidate.json').write_text(json.dumps(record, indent=2)+'\n')
    command = ['/bin/bash', '-c',
               'unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH; '
               'source /opt/ros/jazzy/setup.bash && exec nice -n 10 colcon "$@"', 'field-build',
               '--log-base', str(root/'log'), 'build', '--base-paths', str(root/'src'),
               '--build-base', str(root/'build'), '--install-base', str(root/'install'),
               '--executor', 'sequential', '--packages-select', 'puma_motor_driver',
               '--allow-overriding', 'puma_motor_driver', '--cmake-args',
               '-DBUILD_TESTING=OFF', '-DCMAKE_BUILD_TYPE=Release']
    with (root/'build.log').open('w') as log:
        subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=True)
    check = root/'build/puma_motor_driver/puma_field_driver_check'
    result = subprocess.run(['/bin/bash', '-c', 'source /opt/ros/jazzy/setup.bash && exec "$1"',
                             'field-check', str(check)], capture_output=True, text=True)
    (root/'integration.log').write_text(result.stdout+result.stderr)
    result.check_returncode()
    aggregate = verify_aggregate(package, root/'aggregate-check')
    if versions() != dependency_versions or any(digest(ROOT/name) != sha for name, sha in inputs.items()):
        raise RuntimeError('candidate inputs or dependencies changed during build')
    record.update(status='offline-field-age-driver-built-not-installed',
                  integration_stdout=result.stdout.strip(),
                  aggregate_check=aggregate,
                  motor_binary_sha256=digest(root/'install/puma_motor_driver/lib/puma_motor_driver/multi_puma_node'),
                  limits='Real driver cache/decoder routines with null transports and synthetic timestamps. Queue timing instrumentation is compiled but not run against live CAN; ROS publications, loaded/moving behavior and production age-bound qualification remain open.')
    (root/'candidate.json').write_text(json.dumps(record, indent=2)+'\n')
    print(json.dumps(record, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
