#!/usr/bin/env python3
"""Test candidate controller-manager fault handling with fake wheels, loopback domain 215."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

ROOT = Path(__file__).resolve().parents[2]


def check(candidate, output, plugin_prefix=None):
    candidate = candidate.resolve(); output = output.resolve()
    if not output.is_relative_to(ROOT / 'artifacts/hardware') or output == ROOT / 'artifacts/hardware':
        raise ValueError('use a fresh artifacts/hardware directory')
    record = json.loads((candidate / 'SOURCE.json').read_text())
    library = candidate / 'install/clearpath_hardware_interfaces/lib/libpuma_hardware.so'
    if record['status'] != 'built-offline-not-installed' or (candidate / 'DO_NOT_INSTALL.json').exists() or hashlib.sha256(library.read_bytes()).hexdigest() != record['libraries'][str(library.relative_to(candidate))]:
        raise ValueError('candidate provenance or library mismatch')
    output.mkdir(parents=True, exist_ok=False)
    cyclone = output / 'cyclone.xml'
    cyclone.write_text('<CycloneDDS><Domain id="any"><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>')
    prefix = library.parent.parent
    if plugin_prefix is not None:
        prefix = plugin_prefix.resolve()
        staged = prefix / 'lib/libpuma_hardware.so'
        if hashlib.sha256(staged.read_bytes()).hexdigest() != hashlib.sha256(library.read_bytes()).hexdigest():
            raise ValueError('staged plugin differs from tested candidate')
        library = staged
    os.environ.update(ROS_DOMAIN_ID='215', RMW_IMPLEMENTATION='rmw_cyclonedds_cpp', CYCLONEDDS_URI=cyclone.as_uri(), ROS_LOG_DIR=str(output / 'ros-log'))
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import String, Float64MultiArray
    from sensor_msgs.msg import JointState
    from clearpath_motor_msgs.msg import PumaFeedback, PumaMultiFeedback
    from controller_manager_msgs.srv import LoadController, ConfigureController, SwitchController, ListControllers, ListHardwareComponents, SetHardwareComponentState
    from lifecycle_msgs.msg import State
    import yaml

    ns = '/feedback_manager_fake_' + str(os.getpid())
    joints = ['fl', 'fr', 'rl', 'rr']
    urdf = '<robot name="fake"><link name="base"/>'
    for name in joints:
        urdf += f'<link name="{name}"/><joint name="{name}" type="continuous"><parent link="base"/><child link="{name}"/><axis xyz="0 1 0"/></joint>'
    urdf += '<ros2_control name="FakePuma" type="system"><hardware><plugin>clearpath_hardware_interfaces/PumaHardware</plugin></hardware>'
    for name in joints:
        urdf += f'<joint name="{name}"><command_interface name="velocity"/><state_interface name="position"/><state_interface name="velocity"/></joint>'
    urdf += '</ros2_control></robot>'
    (output / 'robot.urdf').write_text(urdf)
    config = output / 'controller.yaml'
    config.write_text(yaml.safe_dump({ns + '/controller_manager': {'ros__parameters': {'update_rate': 20, 'hardware_components_initial_state': {'inactive': ['FakePuma']}, 'fake_drive': {'type': 'forward_command_controller/ForwardCommandController'}}}, ns + '/fake_drive': {'ros__parameters': {'joints': joints, 'interface_name': 'velocity'}}}))
    environment = os.environ.copy()
    environment['AMENT_PREFIX_PATH'] = str(prefix) + ':' + environment.get('AMENT_PREFIX_PATH', '')
    environment['LD_LIBRARY_PATH'] = str(library.parent) + ':' + environment.get('LD_LIBRARY_PATH', '')
    rclpy.init(); node = rclpy.create_node('fake_wheels', namespace=ns)
    feed = node.create_publisher(PumaMultiFeedback, 'platform/motors/feedback', QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
    requested = node.create_publisher(Float64MultiArray, 'fake_drive/commands', 10)
    description = node.create_publisher(String, 'robot_description', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    commands = []
    node.create_subscription(JointState, 'platform/motors/cmd', lambda m: commands.append((time.monotonic(), list(m.velocity))), QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))
    state = {'feedback': True, 'command': False, 'last': 0.0, 'last_feedback_ros': None, 'last_feedback_monotonic': None}
    checks = []; child = None
    def require(name, value):
        checks.append({'name': name, 'passed': bool(value)})
        if not value: raise RuntimeError(name)
    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if child is not None and child.poll() is not None: raise RuntimeError('isolated controller manager exited')
            now = time.monotonic()
            if now - state['last'] >= .04:
                state['last'] = now
                if state['feedback']:
                    msg = PumaMultiFeedback(); msg.header.stamp = node.get_clock().now().to_msg()
                    for name, device in zip(joints, [5, 4, 2, 3]):
                        wheel = PumaFeedback(); wheel.device_name = name; wheel.device_number = device; wheel.speed = 0.0; wheel.travel = 1.0
                        msg.drivers_feedback.append(wheel)
                    feed.publish(msg); state['last_feedback_ros'] = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9; state['last_feedback_monotonic'] = now
                if state['command']: requested.publish(Float64MultiArray(data=[.4] * 4))
            rclpy.spin_once(node, timeout_sec=.005)
    def call(kind, name, request):
        client = node.create_client(kind, 'controller_manager/' + name)
        end = time.monotonic() + 8
        while not client.service_is_ready() and time.monotonic() < end: pump(.02)
        if not client.service_is_ready(): raise RuntimeError(name + ' unavailable')
        future = client.call_async(request)
        while not future.done() and time.monotonic() < end: pump(.02)
        if not future.done(): raise RuntimeError(name + ' response timeout')
        response = future.result(); node.destroy_client(client); return response
    def hardware(target):
        request = SetHardwareComponentState.Request(name='FakePuma', target_state=State(id=target))
        return call(SetHardwareComponentState, 'set_hardware_component_state', request)
    def controller_switch():
        request = SwitchController.Request(); request.activate_controllers = ['fake_drive']; request.strictness = 2; request.activate_asap = True; request.timeout.sec = 3
        return call(SwitchController, 'switch_controller', request)
    def controller_states():
        return {c.name: c.state for c in call(ListControllers, 'list_controllers', ListControllers.Request()).controller}
    report = {'domain': 215, 'interface': 'lo', 'namespace': ns, 'candidate_library_sha256': hashlib.sha256(library.read_bytes()).hexdigest(), 'checks': checks, 'harness_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), 'limits': 'Fake wheel feedback and command sink only. Physical braking, CAN completeness and deployment remain unqualified.'}
    try:
        with (output / 'controller.log').open('w') as log:
            child = subprocess.Popen(['/opt/ros/jazzy/lib/controller_manager/ros2_control_node', '--ros-args', '-r', '__ns:=' + ns, '--params-file', str(config)], env=environment, stdout=log, stderr=log, start_new_session=True)
            description.publish(String(data=urdf)); pump(1.)
            require('hardware activates with fresh stationary feedback', hardware(State.PRIMARY_STATE_ACTIVE).ok)
            mapped = [line for line in Path(f'/proc/{child.pid}/maps').read_text().splitlines() if 'libpuma_hardware.so' in line]
            (output / 'loaded-puma-library.txt').write_text('\n'.join(mapped) + '\n')
            require('candidate library is actually loaded', bool(mapped) and all(str(library) in line for line in mapped))
            require('controller loads', call(LoadController, 'load_controller', LoadController.Request(name='fake_drive')).ok)
            require('controller configures', call(ConfigureController, 'configure_controller', ConfigureController.Request(name='fake_drive')).ok)
            require('controller activates', controller_switch().ok)
            state['command'] = True; mark = len(commands); pump(2.)
            require('controller remains active under continuous feedback', controller_states().get('fake_drive') == 'active')
            require('healthy controller sends nonzero to fake motor sink', any(any(v != 0 for v in values) for _, values in commands[mark:]))
            state['feedback'] = False; lost = time.monotonic(); report['feedback_disabled_monotonic'] = lost; report['feedback_disabled_ros'] = node.get_clock().now().nanoseconds / 1e9; pump(.8)
            report['last_feedback_ros'] = state['last_feedback_ros']; report['last_feedback_monotonic'] = state['last_feedback_monotonic']
            report['first_zero_after_loss_s'] = next((t-lost for t, values in commands if t>=lost and values and all(v==0 for v in values)), None)
            report['last_nonzero_after_loss_s'] = max((t-lost for t, values in commands if t>=lost and any(v!=0 for v in values)), default=None)
            report['controller_after_fault'] = controller_states()
            report['hardware_after_fault'] = [{'name': c.name, 'state': c.state.label} for c in call(ListHardwareComponents, 'list_hardware_components', ListHardwareComponents.Request()).component]
            require('stale feedback sends zero', any(t >= lost and values and all(v == 0 for v in values) for t, values in commands))
            require('no nonzero command after stale gate', all(all(v == 0 for v in values) for t, values in commands if t >= lost + .4))
            report['controller_after_fault'] = controller_states()
            report['hardware_after_fault'] = [{'name': c.name, 'state': c.state.label} for c in call(ListHardwareComponents, 'list_hardware_components', ListHardwareComponents.Request()).component]
            require('controller is deactivated by hardware error', report['controller_after_fault'].get('fake_drive') != 'active')
            state['feedback'] = True; recovered = time.monotonic(); pump(.5)
            require('good feedback does not automatically reactivate controller', controller_states().get('fake_drive') != 'active')
            require('good feedback does not replay held commands', all(all(v == 0 for v in values) for t, values in commands if t >= recovered))
            state['command'] = False
            requested.publish(Float64MultiArray(data=[0.] * 4)); pump(.1)
            require('explicit hardware configuration succeeds', hardware(State.PRIMARY_STATE_INACTIVE).ok)
            require('explicit stationary hardware activation succeeds', hardware(State.PRIMARY_STATE_ACTIVE).ok)
            rearmed = time.monotonic()
            require('explicit controller activation succeeds', controller_switch().ok); pump(.3)
            require('explicit reactivation does not replay old command', bool([v for t,v in commands if t>=rearmed]) and all(all(value==0 for value in values) for t,values in commands if t>=rearmed))
            state['command'] = True; mark = len(commands); pump(.3)
            require('new command works after explicit recovery', any(any(v!=0 for v in values) for _,values in commands[mark:]))
            state['command'] = False; requested.publish(Float64MultiArray(data=[0.] * 4)); pump(.15)
            request = SwitchController.Request(); request.deactivate_controllers = ['fake_drive']; request.strictness = 2; request.timeout.sec = 3
            require('controller deactivates cleanly', call(SwitchController, 'switch_controller', request).ok)
            require('hardware deactivates cleanly', hardware(State.PRIMARY_STATE_INACTIVE).ok)
            report['status'] = 'controller-manager-stale-feedback-check-passed-not-installed'
    except Exception as error:
        report['status'] = 'failed'; report['error'] = str(error)
        raise
    finally:
        if child is not None and child.poll() is None:
            os.killpg(child.pid, signal.SIGTERM)
            try: child.wait(timeout=3)
            except subprocess.TimeoutExpired: os.killpg(child.pid, signal.SIGKILL); child.wait()
        (output / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
        (output / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
        node.destroy_node(); rclpy.shutdown()
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate', type=Path, required=True); parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--plugin-prefix', type=Path)
    args = parser.parse_args(); print(json.dumps(check(args.candidate, args.output, args.plugin_prefix), indent=2))
