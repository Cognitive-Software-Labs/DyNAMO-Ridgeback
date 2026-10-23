"""Offline safety checks; every deployment path/service operation is isolated."""

import importlib.util
import json
import os
from pathlib import Path
import subprocess

import pytest


SPEC = importlib.util.spec_from_file_location(
    'dds_controller', Path(__file__).with_name('run_local_dds_test.py'))
runner = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(runner)


@pytest.mark.parametrize('cgroup', [
    '0::/system.slice/mbs-webserver.service',
    '0::/system.slice/mbs-webserver.service/nested.scope',
    '9:memory:/system.slice/clearpath-sensors.service\n0::/',
])
def test_controller_refuses_its_own_service_restart(cgroup):
    with pytest.raises(RuntimeError, match='kills the desktop and experiment controller'):
        runner.ensure_independent_controller(cgroup)


@pytest.mark.parametrize('cgroup', [
    '0::/system.slice/dynamo-dds-run.service',
    '0::/user.slice/user-1000.slice/session-42.scope',
    '0::/system.slice/not-mbs-webserver.service',
])
def test_controller_accepts_independent_service_or_ssh(cgroup):
    runner.ensure_independent_controller(cgroup)


@pytest.mark.parametrize('recovery', [False, True])
def test_detached_controller_has_independent_unit_and_preserves_arguments(tmp_path, recovery):
    directory = tmp_path / 'evidence with spaces'
    unit, command = runner.detached_command(directory, recovery)
    assert unit.startswith('dynamo-dds-') and unit.endswith('.service')
    assert ('-recovery.service' in unit) is recovery
    assert command[0] == 'systemd-run'
    assert '--property=KillMode=mixed' in command
    assert '--property=TimeoutStopSec=600' in command
    assert command[command.index('--') + 1] == '/usr/bin/python3'
    assert command[command.index('--run-dir') + 1] == str(directory)
    assert '--operator-confirmed' in command
    assert '--detach' not in command
    assert ('--recover' in command) is recovery


def test_detached_launcher_preserves_screening_mode(tmp_path):
    _, command = runner.detached_command(tmp_path / 'run', screening=True)
    assert '--screening' in command


def test_detached_launcher_preserves_focused_mode(tmp_path):
    _, command = runner.detached_command(tmp_path / 'run', focused_native=True)
    assert '--focused-native' in command
    assert '--screening' not in command


def test_detached_launcher_preserves_explicit_retention(tmp_path):
    _, command = runner.detached_command(tmp_path / 'run', focused_native=True, retain_passing_native=True)
    assert '--focused-native' in command
    assert '--retain-passing-native' in command


def test_detached_launcher_preserves_current_profile_verification(tmp_path):
    _, command = runner.detached_command(tmp_path / 'run', verify_current=True)
    assert '--verify-current' in command
    assert not {'--focused-native', '--screening', '--recover', '--retain-passing-native'} & set(command)


@pytest.mark.parametrize('conflict', [
    '--screening', '--focused-native', '--recover', '--retain-passing-native',
])
def test_current_profile_mode_rejects_mutating_or_comparison_modes(monkeypatch, tmp_path, conflict):
    monkeypatch.setattr(runner.sys, 'argv', ['runner', '--operator-confirmed', '--run-dir', str(tmp_path),
                                          '--verify-current', conflict])
    with pytest.raises(SystemExit) as error:
        runner.parse_args()
    assert error.value.code == 2


def test_current_profile_mode_is_accepted(monkeypatch, tmp_path):
    monkeypatch.setattr(runner.sys, 'argv', ['runner', '--operator-confirmed', '--run-dir', str(tmp_path),
                                          '--verify-current'])
    args = runner.parse_args()
    assert args.verify_current and not args.recover and not args.retain_passing_native


@pytest.mark.parametrize('baseline_passed,capture_passed', [(True, True), (True, False), (False, False)])
def test_current_profile_capture_skips_installation_and_comparison(tmp_path, monkeypatch, baseline_passed, capture_passed):
    controller = runner.Controller(tmp_path / 'run')
    controller.verify_current = True
    calls = []
    for method in ('validate', 'snapshot', 'state', 'ready', 'graph', 'verify_profile', 'verify_current_files'):
        monkeypatch.setattr(controller, method, lambda *args: None)
    for method in ('install_B', 'restart_active', 'install_aligned', 'recover_sensors', 'native_stage', 'aligned_and_soak'):
        monkeypatch.setattr(controller, method, lambda *args: pytest.fail('Verification must not install or reload'))
    monkeypatch.setattr(controller, 'baseline', lambda stage: (
        {'passed': baseline_passed, 'failure': None if baseline_passed else 'quiet baseline failed'}, tmp_path / 'baseline.json'))
    def trial(label, topics, **kwargs):
        calls.append((label, topics, kwargs))
        return {'passed': capture_passed, 'failure': None if capture_passed else 'camera gap'}, None
    monkeypatch.setattr(controller, 'trial', trial)
    controller.execute()
    assert not controller.retain and not controller.dds_changed and not controller.camera_changed
    assert controller.results['current-native']['passed'] is (baseline_passed and capture_passed)
    assert len(calls) == int(baseline_passed)
    if calls:
        assert calls[0][1:] == ('color,depth', {'readers': 2, 'warmup': 10, 'duration': 60,
                                             'baseline': tmp_path / 'baseline.json'})
    report = json.loads((controller.directory / 'report.json').read_text())
    state = (controller.directory / 'report.md').read_text()
    assert report['mode'] == 'verify-current' and report['current_profile_only']
    assert not report['comparison_performed'] and not report['retained_B']
    assert report['not_reproduced'] == []
    assert set(['A', 'B', 'aligned', 'soak']).issubset(report['still_untested'])
    assert 'no new B retention' in state and 'no DDS installation, service reload' in state


@pytest.mark.parametrize('method', ['install_B', 'install_aligned'])
def test_current_profile_blocks_installers_before_touching_deployment(tmp_path, method):
    controller = runner.Controller(tmp_path / 'run')
    controller.verify_current = True
    with pytest.raises(RuntimeError, match='Current-profile verification cannot'):
        getattr(controller, method)()
    assert not controller.dds_changed and not controller.camera_changed


@pytest.mark.parametrize('action', ['restart', 'reload', 'daemon-reload', 'start', 'stop', 'enable'])
def test_current_profile_blocks_service_mutations(tmp_path, monkeypatch, action):
    controller = runner.Controller(tmp_path / 'run')
    controller.verify_current = True
    monkeypatch.setattr(runner.subprocess, 'run', lambda *args, **kwargs: pytest.fail('Service mutation must never run'))
    with pytest.raises(RuntimeError, match='prohibits service mutations'):
        controller.command(['systemctl', action, 'realsense-camera.service'])


def test_validation_accepts_existing_spdp_only_for_current_mode(tmp_path, monkeypatch):
    controller = runner.Controller(tmp_path / 'run')
    robot, dds, merger, net = tmp_path / 'robot.yaml', tmp_path / 'dds.xml', tmp_path / 'merger', tmp_path / 'net'
    robot.write_text('serial_number: r100-0160\nsystem:\n  ros2:\n    namespace: r100_0160\n'
                     '    middleware:\n      implementation: rmw_cyclonedds_cpp\n')
    original = b'<CycloneDDS><Domain><General><AllowMulticast>spdp</AllowMulticast></General></Domain></CycloneDDS>'
    dds.write_bytes(original)
    (merger / 'launch').mkdir(parents=True)
    (merger / 'launch/merge_ridgeback.launch.py').write_text('"rmw_fastrtps_cpp"')
    for port in runner.PORTS:
        (net / port).mkdir(parents=True)
    monkeypatch.setattr(runner, 'ROBOT', robot)
    monkeypatch.setattr(runner, 'DDS', dds)
    monkeypatch.setattr(runner, 'MERGER', merger)
    original_path = runner.Path
    monkeypatch.setattr(runner, 'Path', lambda value: net if value == '/sys/class/net' else original_path(value))
    monkeypatch.setattr(runner.os, 'geteuid', lambda: 0)
    monkeypatch.setattr(runner.socket, 'gethostname', lambda: 'cpr-r100-0160')
    monkeypatch.setattr(runner.shutil, 'which', lambda name: '/usr/bin/' + name)
    monkeypatch.setattr(controller, 'command', lambda *args: json.dumps([{'addr_info': [{'local': '192.168.131.1'}]}]))
    with pytest.raises(ValueError, match='AllowMulticast already exists'):
        controller.validate()
    controller.verify_current = True
    controller.validate()
    assert dds.read_bytes() == original
    dds.write_bytes(b'<malformed')
    with pytest.raises(runner.ET.ParseError):
        controller.validate()


@pytest.fixture
def current_controller(tmp_path, monkeypatch):
    controller = runner.Controller(tmp_path / 'run')
    controller.verify_current = True
    controller.results['current-native'] = {'passed': True, 'complete': True}
    dds, merger = tmp_path / 'dds.xml', tmp_path / 'merger'
    original = b'<CycloneDDS>\r\n<!-- retained before verification -->\r\n<Domain><General><AllowMulticast>spdp</AllowMulticast></General></Domain></CycloneDDS>\r\n'
    dds.write_bytes(original)
    dds.chmod(0o640)
    monkeypatch.setattr(runner, 'DDS', dds)
    monkeypatch.setattr(runner, 'ROBOT', tmp_path / 'robot.yaml')
    monkeypatch.setattr(runner, 'START', tmp_path / 'camera-start')
    monkeypatch.setattr(runner, 'MERGER', merger)
    monkeypatch.setattr(runner, 'OVERRIDE', tmp_path / 'absent-override')
    monkeypatch.setattr(runner, 'LEGACY', tmp_path / 'absent-legacy')
    controller.original_dds = original
    meta = dds.stat()
    controller.dds_meta = (0o640, meta.st_uid, meta.st_gid)
    controller.profile = controller.directory / 'reader-current.xml'
    controller.profile.write_bytes(original)
    controller.original_active = {'clearpath-sensors.service': 'active', 'robot-slam.service': 'inactive'}
    controller.active = ['clearpath-sensors.service']
    before = controller.directory / 'before'
    manifest = {}
    for source in (runner.ROBOT, runner.START, merger / 'launch/merge_ridgeback.launch.py',
                   merger / 'config/params_ridgeback.yaml', tmp_path / 'setup.bash'):
        source.parent.mkdir(parents=True, exist_ok=True)
        source.write_text('preserved ' + source.name)
        name = str(source.relative_to('/'))
        saved = before / 'files' / name
        saved.parent.mkdir(parents=True, exist_ok=True)
        saved.write_bytes(source.read_bytes())
        meta = source.stat()
        manifest[name] = {'metadata': [meta.st_mode & 0o777, meta.st_uid, meta.st_gid], 'symlink_target': None}
    runner.dump(before / 'preserved-file-metadata.json', manifest)
    runner.dump(before / 'legacy-backup-hashes.json', {})
    monkeypatch.setattr(runner, 'atomic_restore', lambda *args: pytest.fail('Current verification must not rewrite installed files'))
    monkeypatch.setattr(controller, 'ready', lambda: pytest.fail('Cleanup must not reset the service identity guard'))
    monkeypatch.setattr(controller, 'guard_services', lambda: None)
    monkeypatch.setattr(controller, 'verify_profile', lambda *args: None)
    monkeypatch.setattr(controller, 'unit_info', lambda unit: {'ActiveState': controller.original_active[unit]})
    monkeypatch.setattr(controller, 'command', lambda *args, **kwargs: pytest.fail('Cleanup must not invoke a service action'))
    monkeypatch.setattr(controller, 'evidence', lambda *args: None)
    return controller, dds, original


@pytest.mark.parametrize('final_passed', [True, False])
def test_current_cleanup_preserves_retained_spdp_and_final_scan_result(current_controller, monkeypatch, final_passed):
    controller, dds, original = current_controller
    calls = []
    monkeypatch.setattr(controller, 'trial', lambda label, **kwargs:
                        (calls.append((label, kwargs)) or {'passed': final_passed, 'failure': None if final_passed else 'scan gap'}, None))
    assert controller.restore() is final_passed
    assert dds.read_bytes() == original and dds.stat().st_mode & 0o777 == 0o640
    assert calls == [('restore-verification', {'warmup': 3, 'duration': 10})]
    assert controller.results['restoration']['passed'] is final_passed
    assert not controller.retain and not controller.dds_changed
    assert controller.stage == ('current-profile-verified' if final_passed else 'verification-failed')
    state = json.loads((controller.directory / 'CURRENT_CONFIG.json').read_text())
    assert state['current_profile_only'] and not state['comparison_performed'] and not state['retain_B']
    assert state['reader_profile'].endswith('reader-current.xml')


@pytest.mark.parametrize('change', ['dds', 'dds-mode', 'reader', 'source', 'activity', 'worker'])
def test_current_cleanup_detects_external_change_without_overwriting(current_controller, monkeypatch, change):
    controller, dds, original = current_controller
    monkeypatch.setattr(controller, 'trial', lambda *args, **kwargs: pytest.fail('Unsafe final scans must not start'))
    if change == 'dds':
        dds.write_bytes(b'external DDS edit')
    elif change == 'dds-mode':
        dds.chmod(0o600)
    elif change == 'reader':
        controller.profile.write_bytes(b'different reader XML')
    elif change == 'source':
        runner.START.write_text('external startup edit')
    elif change == 'activity':
        monkeypatch.setattr(controller, 'unit_info', lambda unit: {'ActiveState': 'inactive'})
    else:
        monkeypatch.setattr(controller, 'guard_services', lambda: 'Unexpected ROS worker exit/restart')
    assert not controller.restore()
    assert dds.read_bytes() == (b'external DDS edit' if change == 'dds' else original)
    assert dds.stat().st_mode & 0o777 == (0o600 if change == 'dds-mode' else 0o640)
    if change == 'source':
        assert runner.START.read_text() == 'external startup edit'
    assert controller.stage == 'verification-failed'
    assert not controller.retain and not controller.dds_changed


def test_current_cleanup_catches_changes_during_final_scans(current_controller, monkeypatch):
    controller, dds, original = current_controller
    def trial(*args, **kwargs):
        runner.START.write_text('changed during final scans')
        return {'passed': True, 'failure': None}, None
    monkeypatch.setattr(controller, 'trial', trial)
    assert not controller.restore()
    assert dds.read_bytes() == original
    assert runner.START.read_text() == 'changed during final scans'
    assert any('Preserved deployment file changed' in failure for failure in controller.failures)


@pytest.mark.parametrize('mutation_flag', [None, 'dds_changed', 'camera_changed', 'retain_B'])
def test_current_recovery_never_undoes_preexisting_spdp(current_controller, monkeypatch, mutation_flag):
    controller, dds, original = current_controller
    before = controller.directory / 'before'
    (before / 'cyclonedds.xml').write_bytes(original)
    runner.dump(before / 'dds-metadata.json', controller.dds_meta)
    runner.dump(before / 'active-services.json', controller.original_active)
    controller.state()
    controller.report()
    saved = json.loads((controller.directory / 'CURRENT_CONFIG.json').read_text())
    if mutation_flag:
        saved[mutation_flag] = True
        runner.dump(controller.directory / 'CURRENT_CONFIG.json', saved)
    monkeypatch.setattr(runner.socket, 'gethostname', lambda: 'cpr-r100-0160')
    calls = []
    monkeypatch.setattr(controller, 'ready', lambda: calls.append('ready'))
    monkeypatch.setattr(controller, 'trial', lambda *args, **kwargs: ({'passed': True, 'failure': None}, None))
    if mutation_flag:
        with pytest.raises(RuntimeError, match='unexpected mutation flags'):
            runner.recover(controller)
        assert calls == []
    else:
        assert runner.recover(controller)
        assert calls == ['ready']
        assert not controller.retain and not controller.dds_changed and not controller.camera_changed
    assert dds.read_bytes() == original and dds.stat().st_mode & 0o777 == 0o640


def test_current_watchdog_stops_both_readers_before_preserving_evidence(tmp_path, monkeypatch):
    controller = runner.Controller(tmp_path / 'run')
    controller.verify_current = True
    controller.profile = tmp_path / 'reader.xml'
    events = []
    class Process:
        def __init__(self, name):
            self.name, self.stopped = name, False
        def poll(self):
            return 0 if self.stopped else None
    def launch(argv, log):
        process = Process(Path(log).name)
        controller.processes.append(process)
        return process
    def stop(process):
        process.stopped = True
        events.append(('stop', process.name))
    monkeypatch.setattr(controller, 'launch', launch)
    monkeypatch.setattr(controller, 'ros', lambda args: args)
    monkeypatch.setattr(runner, 'stop_group', stop)
    monkeypatch.setattr(runner.os, 'chown', lambda *args: None)
    monkeypatch.setattr(runner.pwd, 'getpwnam', lambda name: type('Account', (), {'pw_uid': os.getuid(), 'pw_gid': os.getgid()})())
    clock = iter(number * .5 for number in range(100))
    monkeypatch.setattr(runner.time, 'monotonic', lambda: next(clock))
    monkeypatch.setattr(runner.time, 'sleep', lambda *args: None)
    monkeypatch.setattr(controller, 'network', lambda: {p: {'tx': 0, 'rx': 0} for p in runner.PORTS})
    monkeypatch.setattr(controller, 'guard_services', lambda: 'Unexpected camera worker restart' if controller.cameras else None)
    monkeypatch.setattr(controller, 'evidence', lambda *args: events.append(('evidence', len(controller.cameras))))
    monkeypatch.setattr(controller, 'packet_evidence', lambda *args: None)
    summary, _ = controller.trial('current/capture', 'color,depth', readers=2, warmup=10, duration=60)
    assert not summary['passed'] and summary['failure'] == 'Unexpected camera worker restart'
    assert events[:3] == [('stop', 'reader_1.log'), ('stop', 'reader_2.log'), ('evidence', 0)]
    assert all(process.stopped for process in controller.processes)
    assert not controller.cameras and controller.scan is None


@pytest.mark.parametrize('arguments', [[], ['--screening'], ['--focused-native', '--recover']])
def test_retention_requires_focused_test_and_cannot_override_recovery(monkeypatch, tmp_path, arguments):
    monkeypatch.setattr(runner.sys, 'argv', ['runner', '--operator-confirmed', '--run-dir', str(tmp_path),
                                          '--retain-passing-native', *arguments])
    with pytest.raises(SystemExit) as error:
        runner.parse_args()
    assert error.value.code == 2


@pytest.mark.parametrize('capture_passed', [True, False])
def test_explicit_retention_requires_passing_capture(tmp_path, monkeypatch, capture_passed):
    controller = runner.Controller(tmp_path / 'run')
    controller.focused_native = controller.retain_passing_native = True
    for method in ('validate', 'snapshot', 'state', 'graph', 'verify_profile', 'install_B'):
        monkeypatch.setattr(controller, method, lambda *args: None)
    monkeypatch.setattr(controller, 'baseline', lambda stage: ({'passed': True}, tmp_path / 'baseline'))
    monkeypatch.setattr(controller, 'trial', lambda *args, **kwargs:
                        ({'passed': capture_passed, 'failure': None if capture_passed else 'camera gap'}, None))
    controller.execute()
    assert controller.retain is capture_passed
    report = json.loads((controller.directory / 'report.json').read_text())
    assert report['retained_B'] is capture_passed
    assert set(['A', 'B', 'aligned', 'soak']).issubset(report['still_untested'])


@pytest.mark.parametrize('final_passed', [True, False])
def test_final_verification_controls_candidate_retention(tmp_path, monkeypatch, final_passed):
    controller = runner.Controller(tmp_path / 'run')
    dds = tmp_path / 'dds.xml'
    original = b'<CycloneDDS><Domain><General></General></Domain></CycloneDDS>'
    dds.write_bytes(runner.spdp_only(original))
    monkeypatch.setattr(runner, 'DDS', dds)
    monkeypatch.setattr(runner.os, 'chown', lambda *args: None)
    controller.original_dds = original
    controller.dds_meta = (0o644, os.getuid(), os.getgid())
    controller.original_active = {'clearpath-sensors.service': 'active'}
    controller.active = list(controller.original_active)
    controller.dds_changed = controller.retain = True
    calls = []
    monkeypatch.setattr(controller, 'command', lambda argv, **kwargs: calls.append(argv) or '')
    monkeypatch.setattr(controller, 'unit_info', lambda unit: {'ActiveState': 'active'})
    monkeypatch.setattr(controller, 'ready', lambda: None)
    monkeypatch.setattr(controller, 'verify_profile', lambda *args: None)
    monkeypatch.setattr(controller, 'trial', lambda *args, **kwargs:
                        ({'passed': final_passed, 'failure': None if final_passed else 'scan gap'}, None))
    monkeypatch.setattr(runner, 'digest', lambda *args: 'unchanged')
    monkeypatch.setattr(runner, 'LEGACY', tmp_path / 'absent-legacy')
    runner.dump(controller.directory / 'before/legacy-backup-hashes.json', {})
    assert controller.restore() is final_passed
    assert controller.retain is final_passed
    assert dds.read_bytes() == (runner.spdp_only(original) if final_passed else original)
    assert (['systemctl', 'restart', 'clearpath-sensors.service'] in calls) is (not final_passed)
    assert controller.stage == ('candidate-retained' if final_passed else 'recovery-required')


@pytest.mark.parametrize('baseline_passed', [True, False])
def test_focused_capture_skips_other_matrices_and_always_restores(tmp_path, monkeypatch, baseline_passed):
    controller = runner.Controller(tmp_path / 'run')
    controller.focused_native = True
    calls = []
    for method in ('validate', 'snapshot', 'state', 'graph', 'verify_profile'):
        monkeypatch.setattr(controller, method, lambda *args: None)
    monkeypatch.setattr(controller, 'install_B', lambda: calls.append('B'))
    monkeypatch.setattr(controller, 'baseline', lambda stage: (
        {'passed': baseline_passed, 'failure': None if baseline_passed else 'scan_gap'}, tmp_path / 'baseline.json'))
    monkeypatch.setattr(controller, 'native_stage', lambda *args: pytest.fail('Focused capture must skip full native matrices'))
    monkeypatch.setattr(controller, 'aligned_and_soak', lambda: pytest.fail('Focused capture must skip alignment and soak'))
    def trial(label, topics, **kwargs):
        calls.append((label, topics, kwargs))
        return {'passed': True, 'failure': None}, None
    monkeypatch.setattr(controller, 'trial', trial)
    controller.execute()
    assert not controller.retain
    result = controller.results['B-focused']
    assert result['passed'] is baseline_passed
    assert result['complete'] is baseline_passed
    if baseline_passed:
        assert calls[1][1] == 'color,depth'
        assert calls[1][2]['warmup'] == 10
        assert calls[1][2]['duration'] == 60
        assert calls[1][2]['readers'] == 2
    else:
        assert calls == ['B']
        assert controller.failures == ['B-focused baseline: scan_gap']
    report = json.loads((controller.directory / 'report.json').read_text())
    assert report['mode'] == 'focused-native'
    assert report['not_reproduced'] == []
    assert set(['A', 'B', 'aligned', 'soak']).issubset(report['still_untested'])


def test_screening_runs_only_short_native_matrix_and_does_not_claim_qualification(tmp_path, monkeypatch):
    controller = runner.Controller(tmp_path / 'run')
    controller.screening = True
    calls = []
    for method in ('state', 'ready', 'graph', 'verify_profile'):
        monkeypatch.setattr(controller, method, lambda *args: None)
    monkeypatch.setattr(controller, 'baseline', lambda stage: ({'passed': True}, tmp_path / 'baseline.json'))
    def trial(label, topics, readers, **kwargs):
        calls.append((topics, readers, kwargs))
        return {'passed': True, 'short_safety_trial': True}, None
    monkeypatch.setattr(controller, 'trial', trial)
    assert controller.native_stage('A')
    assert [(topics, readers) for topics, readers, _ in calls] == [
        ('color', 1), ('depth', 1), ('color,depth', 1), ('color,depth', 2)]
    assert all(kwargs['duration'] == 5 and kwargs['warmup'] == 0 and kwargs['short'] for _, _, kwargs in calls)
    report = json.loads((controller.directory / 'report.json').read_text())
    assert report['mode'] == 'screening'
    assert report['results']['A']['screening_passed']
    assert not report['results']['A']['passed']
    assert not report['results']['A']['complete']
    assert report['not_reproduced'] == []
    assert report['still_untested'] == ['A', 'B', 'aligned', 'soak']


def test_screening_never_installs_alignment_or_retains_candidate(tmp_path, monkeypatch):
    controller = runner.Controller(tmp_path / 'run')
    controller.screening = True
    visited = []
    monkeypatch.setattr(controller, 'validate', lambda: None)
    monkeypatch.setattr(controller, 'snapshot', lambda: None)
    monkeypatch.setattr(controller, 'install_B', lambda: visited.append('install_B'))
    def native(stage):
        visited.append(stage)
        controller.results[stage] = {'screening_passed': True, 'screening_complete': True}
        return True
    monkeypatch.setattr(controller, 'native_stage', native)
    monkeypatch.setattr(controller, 'aligned_and_soak', lambda: pytest.fail('Screening must skip alignment and soak'))
    controller.execute()
    assert visited == ['A', 'install_B', 'B']
    assert not controller.retain


@pytest.mark.parametrize('newline', [b'\n', b'\r\n'])
def test_candidate_changes_only_multicast_and_preserves_comment(newline):
    original = newline.join([
        b'<CycloneDDS xmlns="https://cdds.io/config">',
        b'<!-- <General> is explanatory text -->',
        b'<Domain id="any"><General>',
        b'<Interfaces><NetworkInterface name="br0"/><NetworkInterface name="wlp3s0"/></Interfaces>',
        b'</General><Discovery><MaxAutoParticipantIndex>250</MaxAutoParticipantIndex></Discovery>',
        b'<Internal><SocketReceiveBufferSize max="16MB"/></Internal></Domain></CycloneDDS>',
    ])
    candidate = runner.spdp_only(original)
    addition = newline + b'      <AllowMulticast>spdp</AllowMulticast>'
    assert candidate.count(addition) == 1
    assert candidate.replace(addition, b'', 1) == original


@pytest.mark.parametrize('xml', [
    b'<CycloneDDS><General><AllowMulticast>spdp</AllowMulticast></General></CycloneDDS>',
    b'<CycloneDDS><General/><General/></CycloneDDS>',
    b'<CycloneDDS><!-- <General> --></CycloneDDS>',
])
def test_candidate_refuses_ambiguous_or_already_changed_configuration(xml):
    with pytest.raises(ValueError):
        runner.spdp_only(xml)


def test_cold_start_arguments_survive_shell_and_yaml(tmp_path):
    # Fake ros2 only prints its argument vector; no ROS or camera is touched.
    executable = tmp_path / 'ros2'
    executable.write_text('#!/usr/bin/python3\nimport json,sys\nprint(json.dumps(sys.argv[1:]))\n')
    executable.chmod(0o755)
    original = '''#!/bin/bash
exec ros2 launch realsense2_camera rs_launch.py \\
 enable_infra1:=true enable_infra2:=true pointcloud.enable:=true \\
 rgb_camera.color_profile:=640x480x30 depth_module.depth_profile:=640x480x30
'''
    result = runner.aligned_script(original, '346543063318')
    import yaml
    output = subprocess.check_output(['bash', '-c', result], text=True,
                                     env={**os.environ, 'PATH': str(tmp_path) + ':/usr/bin:/bin'})
    args = json.loads(output)
    assert '+' not in args
    assert args[:3] == ['launch', 'realsense2_camera', 'rs_launch.py']
    values = dict(item.split(':=', 1) for item in args[3:])
    assert yaml.safe_load(values['serial_no']) == '346543063318'
    assert yaml.safe_load(values['align_depth.enable']) is True
    assert yaml.safe_load(values['enable_infra2']) is False
    assert yaml.safe_load(values['enable_infra1']) is True
    assert yaml.safe_load(values['pointcloud.enable']) is True


def test_xml_restore_preserves_exact_bytes_and_metadata(tmp_path, monkeypatch):
    path = tmp_path / 'installed.xml'
    path.write_bytes(b'candidate')
    ownership = []
    monkeypatch.setattr(runner.os, 'chown', lambda p, uid, gid: ownership.append((uid, gid)))
    runner.atomic_restore(path, b'original\r\n<!-- exact -->', 0o640, 123, 456)
    assert path.read_bytes() == b'original\r\n<!-- exact -->'
    assert path.stat().st_mode & 0o777 == 0o640
    assert ownership == [(123, 456)]
    assert not path.with_name(path.name + '.dds-experiment-tmp').exists()


def test_camera_failure_does_not_restart_lidar():
    assert not runner.needs_sensor_recovery({'trials': [{'failure': 'Camera rate acceptance failed'}]})
    assert runner.needs_sensor_recovery({'trials': [{'scan_guard': {'failures': [{'reason': 'scan_gap'}]}}]})
    assert runner.needs_sensor_recovery({'baseline': {'failure': 'Driver journal error in clearpath-sensors.service: reconnecting'}})


def test_traffic_guard_requires_same_port_two_consecutive_samples():
    assert not runner.tx_violation({'eno1': 21, 'enp2s0': 0}, {'eno1': 0, 'enp2s0': 21})
    assert not runner.tx_violation({'eno1': 20}, {'eno1': 100})
    assert runner.tx_violation({'eno1': 21}, {'eno1': 21})


def scan_events(start=10, end=11):
    yield {'event': 'startup', 'monotonic_s': 0}
    # Warm-up traffic must not dilute the high traffic in the capture window.
    yield {'event': 'host_sample', 'monotonic_s': start, 'interfaces': {
        p: {'tx_mbps': 0, 'rx_mbps': 0} for p in runner.PORTS}}
    for index in range(40):
        for name in ('front', 'rear', 'merged'):
            at = start + .01 + index * .025
            yield {'event': 'scan', 'monotonic_s': at, 'receive_monotonic_s': at,
                   'stream': name, 'stamp_ns': int(at * 1e9), 'rays': 1081}
    yield {'event': 'host_sample', 'monotonic_s': end, 'interfaces': {
        p: {'tx_mbps': 8, 'rx_mbps': 1} for p in runner.PORTS}}


def test_scan_capture_windows_do_not_dilute_traffic_with_warmup(tmp_path):
    events = tmp_path / 'events.jsonl'
    events.write_text(''.join(json.dumps(e) + '\n' for e in scan_events()))
    baseline = {'interfaces': {p: {'mean_tx_mbps': 1} for p in runner.PORTS},
                'streams': {'merged': {'rate_hz': 40}}}
    summary = runner.scan_window(events, 10, 11, baseline)
    assert not summary['passed']
    assert summary['streams']['front']['rate_hz'] == 40
    assert summary['interfaces']['eno1']['incremental_tx_mbps'] == 7
    assert summary['interfaces']['eno1']['duration_s'] == 1


def test_scan_capture_missing_stream_or_empty_window_fails(tmp_path):
    path = tmp_path / 'events.jsonl'
    path.write_text(''.join(json.dumps(e) + '\n' for e in scan_events()
                            if e.get('stream') != 'rear'))
    assert not runner.scan_window(path, 10, 11)['passed']
    assert not runner.scan_window(path, 11, 11)['passed']


def test_merger_parent_generic_library_is_not_data_plane(tmp_path, monkeypatch):
    controller = runner.Controller(tmp_path / 'run')
    controller.active = ['clearpath-scan-merger.service']
    parent = {'pid': 1, 'environment': {'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp'},
              'middleware_libraries': ['librmw_implementation.so', 'librmw_dds_common.so']}
    child = {'pid': 2, 'environment': {'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp'},
             'middleware_libraries': ['/opt/ros/jazzy/lib/librmw_fastrtps_cpp.so']}
    monkeypatch.setattr(controller, 'process_snapshot', lambda label: {
        controller.active[0]: {'systemd': {'MainPID': '1'}, 'processes': [parent, child]}})
    controller.verify_profile('mock')
    # The real ros2 launch parent also loads its chosen CycloneDDS library.
    parent['middleware_libraries'].append('/opt/ros/jazzy/lib/librmw_cyclonedds_cpp.so')
    parent['environment']['CYCLONEDDS_URI'] = runner.DDS.as_uri()
    controller.verify_profile('real-launch-parent')
    child['environment']['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
    with pytest.raises(RuntimeError, match='Unexpected RMW'):
        controller.verify_profile('mock')


def test_failed_camera_restart_does_not_skip_xml_restoration(tmp_path, monkeypatch):
    controller = runner.Controller(tmp_path / 'run')
    dds, override = tmp_path / 'dds.xml', tmp_path / 'override.conf'
    dds.write_bytes(b'candidate')
    override.write_bytes(b'camera override')
    monkeypatch.setattr(runner, 'DDS', dds)
    monkeypatch.setattr(runner, 'OVERRIDE', override)
    monkeypatch.setattr(runner.os, 'chown', lambda *args: None)
    controller.original_dds = b'original'
    controller.dds_meta = (0o644, os.getuid(), os.getgid())
    controller.original_active = {'realsense-camera.service': 'active', 'clearpath-sensors.service': 'active'}
    controller.active = list(controller.original_active)
    controller.dds_changed = controller.camera_changed = True
    calls = []
    def command(argv, **kwargs):
        calls.append(argv)
        if argv == ['systemctl', 'restart', 'realsense-camera.service']:
            raise RuntimeError('simulated camera restart failed')
        return ''
    monkeypatch.setattr(controller, 'command', command)
    monkeypatch.setattr(controller, 'unit_info', lambda unit: {'ActiveState': 'active'})
    assert not controller.restore()
    assert dds.read_bytes() == b'original'
    assert not override.exists()
    assert ['systemctl', 'restart', 'clearpath-sensors.service'] in calls
    assert controller.stage == 'recovery-required'


def test_install_B_refuses_external_configuration_change(tmp_path, monkeypatch):
    controller = runner.Controller(tmp_path / 'run')
    dds = tmp_path / 'installed.xml'
    dds.write_bytes(b'external edit')
    monkeypatch.setattr(runner, 'DDS', dds)
    controller.original_dds = b'original'
    with pytest.raises(RuntimeError, match='changed during A'):
        controller.install_B()
    assert dds.read_bytes() == b'external edit'
    assert not controller.dds_changed


def test_restart_guard_tracks_ros_workers_but_not_temporary_helpers(tmp_path, monkeypatch):
    proc = tmp_path / 'proc'
    for pid, arguments in {
        '1': ['/bin/bash', '/usr/sbin/clearpath-platform-start'],
        '2': ['/usr/bin/python3', '/opt/ros/jazzy/bin/ros2', 'launch', '/tmp/platform.launch.py'],
        '3': ['/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher', '--ros-args'],
        '4': ['/usr/bin/iw', 'dev', 'wlp3s0', 'link'],
        '5': ['/usr/bin/python3', '/opt/ros/jazzy/bin/ros2', 'node', 'list'],
    }.items():
        directory = proc / pid
        directory.mkdir(parents=True)
        (directory / 'cmdline').write_bytes(('\0'.join(arguments) + '\0').encode())
    original_path = runner.Path
    monkeypatch.setattr(runner, 'Path', lambda value: proc if value == '/proc' else original_path(value))
    monkeypatch.setattr(runner, 'process_tree', lambda pid: {str(i): i * 100 for i in range(1, 6)})
    assert runner.persistent_ros_tree(1) == {'1': 100, '2': 200, '3': 300}


def test_refuses_before_anything_when_the_camera_service_owns_the_camera(monkeypatch, tmp_path, capsys):
    monkeypatch.setattr(runner, 'vendor_camera_retired', lambda: True)
    monkeypatch.setattr(runner.sys, 'argv', ['run_local_dds_test.py', '--operator-confirmed',
                                              '--verify-current', '--run-dir', str(tmp_path / 'run')])
    assert runner.main() == 2
    assert 'dynamo-camera.service owns the camera' in capsys.readouterr().err
    assert not (tmp_path / 'run').exists()
