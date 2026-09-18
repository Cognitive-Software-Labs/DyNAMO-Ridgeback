"""Exercise deployment scripts with isolated files and fake host commands."""
import json
import os
from pathlib import Path
import subprocess

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[3]
ENV_SCRIPT = ROOT / 'src/ridgeback_autonomy_hardware/config/intel_thor/dds_env.sh'
SERVICE_SCRIPT = ROOT / 'tools/intel_thor/intel_services_rmw'


@pytest.mark.parametrize('role', ['invalid', ''])
def test_invalid_role_stops_chained_launch(tmp_path, role):
    ip = tmp_path / 'ip'
    ip.write_text('#!/bin/sh\nexit 0\n')
    ip.chmod(0o755)
    env = dict(os.environ, PATH=f'{tmp_path}:/usr/bin:/bin',
               RMW_IMPLEMENTATION='rmw_fastrtps_cpp', CYCLONEDDS_URI='old-config')
    result = subprocess.run(['bash', '-c', '''
source "$1" "$2" && touch "$3"
rc=$?
printf '%s|%s' "$RMW_IMPLEMENTATION" "$CYCLONEDDS_URI"
exit "$rc"
''', 'test', str(ENV_SCRIPT), role, str(tmp_path / 'launched')],
        env=env, capture_output=True, text=True)
    assert result.returncode != 0
    assert not (tmp_path / 'launched').exists()
    assert result.stdout == 'rmw_fastrtps_cpp|old-config'


@pytest.mark.parametrize('role', ['intel', 'thor'])
def test_explicit_role_selects_profile_and_keeps_domain(role):
    result = subprocess.run(['bash', '-eu', '-c', '''
export ROS_DOMAIN_ID=193 FASTRTPS_DEFAULT_PROFILES_FILE=old-profile
source "$1" "$2"
printf '%s|%s|%s|%s' "$RMW_IMPLEMENTATION" "$CYCLONEDDS_URI" "$ROS_DOMAIN_ID" "${FASTRTPS_DEFAULT_PROFILES_FILE-unset}"
''', 'test', str(ENV_SCRIPT), role], capture_output=True, text=True, check=True)
    assert result.stdout == (
        f'rmw_cyclonedds_cpp|file://{ENV_SCRIPT.parent}/cyclonedds_{role}.xml|193|unset')


@pytest.fixture
def host(tmp_path):
    """Redirect every privileged path; fake commands never touch host services."""
    etc = tmp_path / 'etc'
    clearpath = etc / 'clearpath'
    clearpath.mkdir(parents=True)
    (etc / 'sysctl.d').mkdir()
    bin_dir = tmp_path / 'bin'
    bin_dir.mkdir()
    library = tmp_path / 'librmw_cyclonedds_cpp.so'
    library.touch()
    script = tmp_path / 'intel_services_rmw'
    source = SERVICE_SCRIPT.read_text().replace('/etc/', f'{etc}/')
    source = source.replace('/opt/ros/jazzy/lib/librmw_cyclonedds_cpp.so', str(library))
    script.write_text(source)
    mock = bin_dir / 'host-command'
    mock.write_text('''#!/usr/bin/python3
import json, os, pathlib, sys, yaml
name = pathlib.Path(sys.argv[0]).name
args = sys.argv[1:]
root = pathlib.Path(os.environ['TEST_HOST'])
cp = root / 'etc/clearpath'
if name == 'git':
    print(os.environ['TEST_REPO'])
elif name == 'id':
    print('0')
elif name == 'sysctl':
    print('16777216')
elif name == 'systemctl':
    if args[0] == 'is-active':
        active = args[-1] == 'clearpath-platform.service' and not (root / 'service-failed').exists()
        if '--quiet' not in args:
            print('active' if active else 'inactive')
        sys.exit(0 if active else 3)
    elif args[0] == 'show':
        print('0')
    elif args[0] == 'restart':
        robot = yaml.safe_load((cp / 'robot.yaml').read_text())
        rmw = robot.get('system', {}).get('ros2', {}).get('middleware', {}).get('implementation', 'rmw_fastrtps_cpp')
        config = cp / 'cyclonedds.xml'
        with (root / 'restarts.jsonl').open('a') as log:
            log.write(json.dumps(dict(unit=args[1], rmw=rmw,
                config=config.read_text() if config.exists() else None)) + '\\n')
        if args[1] == 'clearpath-robot.service':
            (cp / 'setup.bash').write_text(f'export RMW_IMPLEMENTATION="{rmw}"\\n')
        if args[1] == 'clearpath-platform.service' and (root / 'fail-restart').exists():
            (root / 'service-failed').touch()
            sys.exit(1)
        if args[1] == 'clearpath-platform.service':
            (root / 'service-failed').unlink(missing_ok=True)
''')
    mock.chmod(0o755)
    for name in ('git', 'id', 'sysctl', 'systemctl', 'sleep'):
        (bin_dir / name).symlink_to(mock)
    env = dict(os.environ, PATH=f'{bin_dir}:/usr/bin:/bin', TEST_HOST=str(tmp_path),
               TEST_REPO=str(ROOT))

    def run(command):
        return subprocess.run(['bash', str(script), command, '--yes'], env=env,
                              capture_output=True, text=True, timeout=10)

    def setup(rmw):
        (clearpath / 'robot.yaml').write_text(yaml.safe_dump({
            'system': {'ros2': {'middleware': {'implementation': rmw}}}}))
        (clearpath / 'setup.bash').write_text(f'export RMW_IMPLEMENTATION="{rmw}"\n')

    return tmp_path, clearpath, setup, run


@pytest.mark.parametrize('rmw', ['rmw_fastrtps_cpp', 'rmw_cyclonedds_cpp'])
@pytest.mark.parametrize('config_kind', ['absent', 'file', 'symlink'])
def test_repeated_apply_and_rollback_restore_original_files(host, rmw, config_kind):
    root, cp, setup, run = host
    setup(rmw)
    original_yaml = (cp / 'robot.yaml').read_bytes()
    config = cp / 'cyclonedds.xml'
    old_content = '<CycloneDDS><!-- original host configuration --></CycloneDDS>'
    if config_kind == 'file':
        config.write_text(old_content)
        config.chmod(0o640)
    elif config_kind == 'symlink':
        (cp / 'original.xml').write_text(old_content)
        config.symlink_to('original.xml')
    dropin = root / 'etc/systemd/system/clearpath-scan-merger.service.d/60-dynamo-cyclonedds.conf'
    dropin.parent.mkdir(parents=True)
    old_dropin = '[Service]\nEnvironment=ORIGINAL=1\n'
    if config_kind == 'symlink':
        (dropin.parent / 'original.conf').write_text(old_dropin)
        dropin.symlink_to('original.conf')
    else:
        dropin.write_text(old_dropin)
    for _ in range(2):
        result = run('apply')
        assert result.returncode == 0, result.stderr + result.stdout
    if config_kind == 'symlink':
        assert (cp / 'original.xml').read_text() == old_content
        assert (dropin.parent / 'original.conf').read_text() == old_dropin
    (root / 'restarts.jsonl').unlink(missing_ok=True)
    result = run('rollback')
    assert result.returncode == 0, result.stderr + result.stdout
    assert (cp / 'robot.yaml').read_bytes() == original_yaml
    assert dropin.read_text() == old_dropin
    assert dropin.is_symlink() == (config_kind == 'symlink')
    if config_kind == 'absent':
        assert not config.exists()
    else:
        assert config.read_text() == old_content
        if config_kind == 'symlink':
            assert config.is_symlink() and config.readlink() == Path('original.xml')
        else:
            assert config.stat().st_mode & 0o777 == 0o640
    assert not (cp / 'dynamo-rmw-backup').exists()
    assert not (root / 'etc/systemd/system/realsense-camera.service.d/60-dynamo-cyclonedds.conf').exists()
    restarts = [json.loads(line) for line in (root / 'restarts.jsonl').read_text().splitlines()]
    assert [item['unit'] for item in restarts] == ['clearpath-robot.service', 'clearpath-platform.service']
    assert all(item['rmw'] == rmw for item in restarts)
    assert all(item['config'] == (None if config_kind == 'absent' else old_content) for item in restarts)


@pytest.mark.parametrize('command', ['apply', 'rollback'])
def test_legacy_incomplete_backup_is_not_silently_used(host, command):
    root, cp, setup, run = host
    setup('rmw_cyclonedds_cpp')
    config = cp / 'cyclonedds.xml'
    config.write_text('current config')
    backup = cp / 'dynamo-rmw-backup'
    backup.mkdir()
    (backup / 'robot.yaml').write_bytes((cp / 'robot.yaml').read_bytes())
    result = run(command)
    assert result.returncode != 0
    assert 'backup' in result.stderr.lower()
    assert config.read_text() == 'current config'
    assert (backup / 'robot.yaml').exists()
    assert not (root / 'restarts.jsonl').exists()


def test_failed_rollback_retains_backup_for_retry(host):
    root, cp, setup, run = host
    setup('rmw_fastrtps_cpp')
    original = (cp / 'robot.yaml').read_bytes()
    assert run('apply').returncode == 0
    (root / 'fail-restart').touch()
    assert run('rollback').returncode != 0
    assert (root / 'service-failed').exists()
    assert (cp / 'dynamo-rmw-backup/robot.yaml').read_bytes() == original
    (root / 'fail-restart').unlink()
    result = run('rollback')
    assert result.returncode == 0, result.stderr + result.stdout
    assert not (cp / 'dynamo-rmw-backup').exists()
    assert not (root / 'service-failed').exists()
