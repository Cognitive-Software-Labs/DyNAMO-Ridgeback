"""Benchmark backend selection: registry, settings, catalogs and provenance."""

from __future__ import annotations

from pathlib import Path
import subprocess
import sys

import pytest

from ridgeback_autonomy.benchmarking.backends import (
    BACKEND_SETTING_NAMES,
    BENCHMARK_BACKENDS,
    DEFAULT_BACKEND,
    benchmark_backend,
    decode_backend_settings,
    encode_backend_settings,
    normalize_backend_settings,
    resolve_benchmark_environment,
    simulator_provenance,
)


SOURCE_ROOT = Path(__file__).resolve().parents[2]


def _source_share(package: str) -> str:
    return str(SOURCE_ROOT / package)


def test_registry_selects_each_simulators_adapter_and_scene_control_namespace() -> None:
    assert DEFAULT_BACKEND == 'gz'
    assert sorted(BENCHMARK_BACKENDS) == ['gz', 'isaac']
    gz, isaac = BENCHMARK_BACKENDS['gz'], BENCHMARK_BACKENDS['isaac']
    assert (gz.adapter_package, gz.scene_control_service_namespace) == (
        'ridgeback_autonomy_gz', '/gzserver')
    assert (isaac.adapter_package, isaac.scene_control_service_namespace) == (
        'ridgeback_autonomy_isaac', '/isaac_sim')
    assert set(gz.settings) == {'gz_gui', 'headless_rendering'}
    assert set(isaac.settings) == {'headless', 'rtf', 'sim_mode', 'depth_fidelity'}
    assert BACKEND_SETTING_NAMES == set(gz.settings) | set(isaac.settings)
    assert not set(gz.settings) & set(isaac.settings)


def test_hardware_is_refused_with_the_reason() -> None:
    with pytest.raises(ValueError) as raised:
        benchmark_backend('hardware')

    message = str(raised.value)
    assert 'backend "hardware" cannot run the target-distance benchmark' in message
    assert 'scene-control' in message
    assert 'Use one of: gz, isaac' in message


@pytest.mark.parametrize('name', ['', 'gazebo', 'Isaac', None])
def test_unknown_backends_are_refused(name) -> None:
    with pytest.raises(ValueError, match='Unknown benchmark backend'):
        benchmark_backend(name)


def test_settings_are_completed_with_defaults_and_normalized() -> None:
    isaac = BENCHMARK_BACKENDS['isaac']

    assert normalize_backend_settings(isaac, {}) == {
        'headless': 'true', 'rtf': '1.0', 'sim_mode': 'realtime', 'depth_fidelity': 'ideal'}
    assert normalize_backend_settings(
        isaac, {'headless': 'False', 'rtf': '0', 'depth_fidelity': 'd455'}) == {
        'headless': 'false', 'rtf': '0.0', 'sim_mode': 'realtime', 'depth_fidelity': 'd455'}
    assert normalize_backend_settings(
        BENCHMARK_BACKENDS['gz'], {'headless_rendering': 'TRUE'}) == {
        'gz_gui': 'true', 'headless_rendering': 'true'}


@pytest.mark.parametrize(
    ('backend', 'name', 'owner'),
    [('gz', 'depth_fidelity', 'isaac'), ('gz', 'sim_mode', 'isaac'),
     ('isaac', 'gz_gui', 'gz'), ('isaac', 'headless_rendering', 'gz')],
)
def test_a_setting_for_the_other_simulator_is_refused(backend, name, owner) -> None:
    with pytest.raises(ValueError, match=rf'"{name}" applies only to backend {owner}'):
        normalize_backend_settings(BENCHMARK_BACKENDS[backend], {name: 'true'})


@pytest.mark.parametrize(
    ('name', 'value'),
    [('rtf', '-0.5'), ('rtf', 'nan'), ('rtf', 'inf'), ('rtf', ''),
     ('sim_mode', 'Realtime'), ('depth_fidelity', 'stereo'), ('headless', 'yes')],
)
def test_invalid_setting_values_are_refused(name, value) -> None:
    with pytest.raises(ValueError, match=rf'"{name}" must be'):
        normalize_backend_settings(BENCHMARK_BACKENDS['isaac'], {name: value})


@pytest.mark.parametrize(
    ('backend', 'native_world', 'resource'),
    [('gz', 'target_distance_calibration', 'target_distance_calibration.sdf'),
     ('isaac', 'g1_distance_calibration', 'g1_distance_calibration.usda')],
)
def test_the_public_world_resolves_through_only_the_selected_catalog(
    backend, native_world, resource,
) -> None:
    looked_up = []

    def share(package):
        looked_up.append(package)
        return _source_share(package)

    environment = resolve_benchmark_environment(
        backend, 'target_distance_calibration', package_share=share)

    assert looked_up == [BENCHMARK_BACKENDS[backend].adapter_package]
    assert environment.public_world == 'target_distance_calibration'
    assert environment.native_world == native_world
    assert environment.world.resource.uri.endswith(resource)
    signature = environment.signature()
    assert signature == {
        'backend': backend,
        'public_world': 'target_distance_calibration',
        'native_world': native_world,
        'asset_catalog_schema': 'ridgeback_benchmark_assets/1',
        'asset_catalog_sha256': environment.catalog.sha256,
        'settings': BENCHMARK_BACKENDS[backend].default_settings(),
    }
    provenance = environment.provenance()
    assert provenance['asset_catalog_path'] == str(
        SOURCE_ROOT / BENCHMARK_BACKENDS[backend].adapter_package / 'sim'
        / 'benchmark_assets.yaml')
    assert provenance['scene_control_service_namespace'] == (
        BENCHMARK_BACKENDS[backend].scene_control_service_namespace)


def test_a_native_world_name_is_not_a_public_benchmark_world() -> None:
    with pytest.raises(ValueError, match='no benchmark world "g1_distance_calibration"'):
        resolve_benchmark_environment(
            'isaac', 'g1_distance_calibration', package_share=_source_share)


def test_hardware_is_refused_before_any_package_lookup() -> None:
    with pytest.raises(ValueError, match='cannot run the target-distance benchmark'):
        resolve_benchmark_environment(
            'hardware', 'target_distance_calibration',
            package_share=lambda package: pytest.fail(f'looked up {package}'))


def test_settings_round_trip_through_one_stable_parameter_value() -> None:
    settings = normalize_backend_settings(BENCHMARK_BACKENDS['isaac'], {'rtf': '2'})

    encoded = encode_backend_settings(settings)

    assert encoded == (
        '{"depth_fidelity":"ideal","headless":"true","rtf":"2.0","sim_mode":"realtime"}')
    assert decode_backend_settings(encoded) == settings
    assert decode_backend_settings('') == {}
    with pytest.raises(ValueError):
        decode_backend_settings('["not", "a", "mapping"]')


def test_gazebo_provenance_names_the_vendored_release(tmp_path) -> None:
    shares = {
        'ridgeback_autonomy_gz': tmp_path / 'adapter',
        'gz_sim_vendor': tmp_path / 'vendor',
    }
    for share in shares.values():
        share.mkdir()
    (shares['ridgeback_autonomy_gz'] / 'package.xml').write_text(
        '<package><name>ridgeback_autonomy_gz</name><version>0.1.0</version></package>')
    (shares['gz_sim_vendor'] / 'package.xml').write_text(
        '<package><version>0.0.13</version><description>\n'
        '    Vendor package for: gz-sim8 8.15.0\n\n    Gazebo Sim\n'
        '</description></package>')

    provenance = simulator_provenance(
        BENCHMARK_BACKENDS['gz'], package_share=lambda package: str(shares[package]))

    assert provenance == {
        'adapter_package': 'ridgeback_autonomy_gz',
        'adapter_version': '0.1.0',
        'simulator': {'name': 'gz-sim8', 'version': '8.15.0'},
    }


def test_isaac_provenance_reads_the_interpreter_the_adapter_launches(
    tmp_path, monkeypatch,
) -> None:
    venv = tmp_path / 'isaac_venv'
    (venv / 'bin').mkdir(parents=True)
    (venv / 'lib' / 'python3.11' / 'site-packages' / 'isaacsim-6.1.0.0.dist-info').mkdir(
        parents=True)
    monkeypatch.setenv('ISAAC_PYTHON', str(venv / 'bin' / 'python3'))

    def share(package):
        if package != 'ridgeback_autonomy_isaac':
            pytest.fail(f'looked up {package}')
        raise LookupError(package)

    provenance = simulator_provenance(BENCHMARK_BACKENDS['isaac'], package_share=share)

    assert provenance['adapter_version'] is None
    assert provenance['simulator'] == {
        'name': 'isaacsim', 'version': '6.1.0.0',
        'interpreter': str(venv / 'bin' / 'python3')}


def test_registry_imports_without_ament_or_any_simulator() -> None:
    subprocess.run([sys.executable, '-c', '''
import importlib.abc, sys
BLOCKED = {'ament_index_python', 'ridgeback_autonomy_gz', 'ridgeback_autonomy_isaac',
           'ros_gz_sim', 'gz', 'omni', 'isaacsim', 'pxr', 'carb', 'launch', 'rclpy'}
class Block(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in BLOCKED:
            raise ImportError('blocked: ' + fullname)
sys.meta_path.insert(0, Block())
import ridgeback_autonomy.benchmarking.backends
import ridgeback_autonomy.benchmarking.sweep
loaded = sorted(name for name in sys.modules if name.split('.')[0] in BLOCKED)
assert not loaded, loaded
'''], check=True)
