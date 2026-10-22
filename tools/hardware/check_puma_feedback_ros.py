#!/usr/bin/env python3
"""Exercise the compiled candidate with fake wheels on loopback ROS domain 214."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[2]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check(candidate, output):
    candidate = candidate.resolve()
    output = output.resolve()
    if not output.is_relative_to(ROOT / 'artifacts/hardware') or output == ROOT / 'artifacts/hardware':
        raise ValueError('save in a fresh directory below artifacts/hardware')
    record = json.loads((candidate / 'SOURCE.json').read_text())
    if record['status'] != 'built-offline-not-installed' or (candidate / 'DO_NOT_INSTALL.json').exists():
        raise ValueError('requires a verified offline candidate')
    library = candidate / 'install/clearpath_hardware_interfaces/lib/libpuma_hardware.so'
    if record['libraries'][str(library.relative_to(candidate))] != digest(library):
        raise ValueError('candidate library changed')
    output.mkdir(parents=True, exist_ok=False)
    source = ROOT / 'tools/hardware/puma_feedback_ros_check.cpp'
    # Absolute quoted CMake inputs do not assume the caller's checkout path.
    cmake = output / 'CMakeLists.txt'
    prefix = library.parent.parent
    cmake.write_text('''cmake_minimum_required(VERSION 3.16)
project(feedback_ros_check)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(clearpath_motor_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
''' + f'add_executable(feedback_ros_check "{source}")\ntarget_include_directories(feedback_ros_check PRIVATE "{prefix / "include"}")\n' +
                    'ament_target_dependencies(feedback_ros_check rclcpp hardware_interface clearpath_motor_msgs sensor_msgs)\n' +
                    f'target_link_libraries(feedback_ros_check "{library}")\n')
    with (output / 'build.log').open('w') as log:
        subprocess.run(['bash', '-c', 'source /opt/ros/jazzy/setup.bash && cmake -S ' + shlex.quote(str(output)) + ' -B ' + shlex.quote(str(output / 'build')) + ' && cmake --build ' + shlex.quote(str(output / 'build'))], check=True, stdout=log, stderr=subprocess.STDOUT)
    cyclone = output / 'cyclone.xml'
    cyclone.write_text('<CycloneDDS><Domain id="any"><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>')
    environment = os.environ.copy()
    environment.update(ROS_DOMAIN_ID='214', RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
                       CYCLONEDDS_URI=cyclone.as_uri(), ROS_LOG_DIR=str(output / 'ros-log'),
                       DYNAMO_FEEDBACK_CANDIDATE_LIBRARY=str(library))
    with (output / 'run.log').open('w') as log:
        # Source base ROS, then prioritize exactly the candidate library. Never source
        # a live robot/workspace environment or launch the motor driver.
        command = 'source /opt/ros/jazzy/setup.bash && export LD_LIBRARY_PATH=' + shlex.quote(str(library.parent)) + ':"$LD_LIBRARY_PATH" && exec ' + shlex.quote(str(output / 'build/feedback_ros_check')) + ' --ros-args -r __ns:=/isolated_feedback_' + str(os.getpid())
        subprocess.run(['bash', '-c', command], env=environment, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=20)
    report = {'status': 'compiled-plugin-real-ros-check-passed-not-installed', 'library_sha256': digest(library),
              'harness_sha256': digest(source), 'domain': 214, 'interface': 'lo',
              'limits': 'Direct compiled plugin fixture with real ROS messaging; controller-manager error/lifecycle integration and physical stopping untested.'}
    (output / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(check(args.candidate, args.output), indent=2))
