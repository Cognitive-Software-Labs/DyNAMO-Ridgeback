"""Benchmark scene control against a real Gazebo server.

The Gazebo provider of the standard ``simulation_interfaces`` services is the
upstream ``ros_gz_sim`` ``gzserver`` node, so these tests run that node on a
small physics-only world and drive it through the common scene-control
client. They pin the provider behavior the client relies on, including the
places where Gazebo answers differently from the letter of the standard.

The server runs under a unique ROS namespace and Gazebo partition, so the
tests cannot reach a simulation that happens to be running on the same host.
"""

from __future__ import annotations

import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

from ament_index_python.packages import get_package_prefix
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from simulation_interfaces.msg import Result, SimulatorFeatures
from simulation_interfaces.srv import DeleteEntity, GetEntities, SpawnEntity

from ridgeback_autonomy.benchmarking.asset_catalog import load_asset_catalog
from ridgeback_autonomy.benchmarking.scene_control import (
    BENCHMARK_ENTITY_PREFIX,
    SceneControlError,
    SceneControlFailure,
    SpawnSpec,
    create_scene_control_client,
)


CATALOG = Path(__file__).resolve().parents[1] / 'sim' / 'benchmark_assets.yaml'
ROBOT = 'r100_0001/robot'
AUTHORED = 'authored_floor'
SERVER_START_TIMEOUT_SEC = 30.0
SERVER_STOP_TIMEOUT_SEC = 15.0
POSITION_TOLERANCE_M = 1e-3
YAW_TOLERANCE_RAD = 1e-3

WORLD = f'''<?xml version="1.0"?>
<sdf version="1.10">
  <world name="scene_control_contract">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <model name="{AUTHORED}">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>10 10 0.1</size></box></geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
'''
# A stand-in for the Clearpath robot: an entity the benchmark does not own.
ROBOT_SDF = '''<?xml version="1.0"?>
<sdf version="1.10">
  <model name="robot">
    <static>true</static>
    <link name="base_link">
      <collision name="collision">
        <geometry><box><size>0.9 0.7 0.3</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>
'''


class GazeboProvider:
    """One ``gzserver`` node plus a client node in an isolated ROS context."""

    def __init__(self, tmp_dir: Path) -> None:
        token = uuid.uuid4().hex[:8]
        self.namespace = f'/scene_control_test_{token}'
        self.service_namespace = f'{self.namespace}/gzserver'
        world = tmp_dir / 'world.sdf'
        world.write_text(WORLD)
        env = dict(os.environ)
        env['GZ_PARTITION'] = f'scene_control_test_{token}'
        env['GZ_IP'] = '127.0.0.1'
        env.setdefault('ROS_LOG_DIR', str(tmp_dir / 'ros_log'))
        self.context = rclpy.Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node(
            f'scene_control_provider_test_{token}', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        # A bare server publishes no ROS /clock; these cases test the
        # provider's services, not the benchmark's readiness gate.
        self.client = create_scene_control_client(
            self.node, self.service_namespace, spin_once=self.spin_once,
            observe_clock=False)
        self.raw_spawn = self.node.create_client(
            SpawnEntity, f'{self.service_namespace}/spawn_entity')
        self.raw_delete = self.node.create_client(
            DeleteEntity, f'{self.service_namespace}/delete_entity')
        self.raw_entities = self.node.create_client(
            GetEntities, f'{self.service_namespace}/get_entities')

        # Started last, so a failure above cannot leave a server running.
        self.log_path = tmp_dir / 'gzserver.log'
        executable = Path(get_package_prefix('ros_gz_sim')) / 'lib' / 'ros_gz_sim' / 'gzserver'
        with open(self.log_path, 'w') as log:
            self.process = subprocess.Popen(
                [str(executable), '--ros-args',
                 '-r', f'__ns:={self.namespace}',
                 '-p', f'world_sdf_file:={world}',
                 '-p', 'verbosity_level:=2'],
                stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)

    def spin_once(self, timeout_sec: float) -> None:
        self.executor.spin_once(timeout_sec=timeout_sec)

    def call(self, client, request, timeout_sec: float = 10.0):
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_sec
        while not future.done():
            assert time.monotonic() < deadline, f'{client.srv_name} did not answer'
            self.spin_once(0.1)
        return future.result()

    def names(self, name_filter: str = '') -> list[str]:
        request = GetEntities.Request()
        request.filters.filter = name_filter
        response = self.call(self.raw_entities, request)
        assert response.result.result == Result.RESULT_OK, response.result.error_message
        return list(response.entities)

    def benchmark_names(self) -> list[str]:
        """Owned entities from an unfiltered query, selected client-side."""

        return [name for name in self.names() if name.startswith(BENCHMARK_ENTITY_PREFIX)]

    def spawn_raw(self, name: str, *, uri: str = '', resource_string: str = ''):
        request = SpawnEntity.Request()
        request.name = name
        request.allow_renaming = False
        request.uri = uri
        request.resource_string = resource_string
        request.initial_pose.header.frame_id = 'world'
        request.initial_pose.pose.orientation.w = 1.0
        return self.call(self.raw_spawn, request)

    def delete_raw(self, name: str):
        request = DeleteEntity.Request()
        request.entity = name
        return self.call(self.raw_delete, request)

    def close(self) -> None:
        try:
            self.executor.shutdown()
            self.node.destroy_node()
            rclpy.shutdown(context=self.context)
        finally:
            if self.process.poll() is None:
                os.killpg(self.process.pid, signal.SIGINT)
                try:
                    self.process.wait(timeout=SERVER_STOP_TIMEOUT_SEC)
                except subprocess.TimeoutExpired:
                    os.killpg(self.process.pid, signal.SIGKILL)
                    self.process.wait()


@pytest.fixture(scope='module')
def provider(tmp_path_factory):
    gazebo = GazeboProvider(tmp_path_factory.mktemp('gzserver'))
    try:
        try:
            gazebo.client.wait_for_services(SERVER_START_TIMEOUT_SEC)
        except SceneControlError as exc:
            pytest.fail(f'{exc}; server log: {gazebo.log_path}')
        response = gazebo.spawn_raw(ROBOT, resource_string=ROBOT_SDF)
        assert response.result.result == Result.RESULT_OK, response.result.error_message
        gazebo.client.wait_until_present([ROBOT], timeout_sec=10.0)
        yield gazebo
    finally:
        gazebo.close()
    assert gazebo.process.returncode == 0, f'server log: {gazebo.log_path}'


@pytest.fixture(autouse=True)
def no_benchmark_entities_remain(provider):
    yield
    # Remove anything a failed test left behind by its exact name, then prove
    # absence from an unfiltered query rather than from a name filter.
    for name in provider.benchmark_names():
        provider.client.delete_entity(name)
    assert provider.benchmark_names() == []
    assert {AUTHORED, ROBOT} <= set(provider.names())


@pytest.fixture(scope='module')
def catalog():
    return load_asset_catalog(str(CATALOG))


def _yaw(pose) -> float:
    q = pose['orientation']
    return math.atan2(2.0 * (q['w'] * q['z'] + q['x'] * q['y']),
                      1.0 - 2.0 * (q['y'] ** 2 + q['z'] ** 2))


def test_provider_advertises_what_the_benchmark_requires(provider, catalog) -> None:
    selection = catalog.select('target_distance_calibration', 'g1', catalog.objects)

    capabilities = provider.client.require_capabilities(selection.spawn_formats)

    assert {SimulatorFeatures.SPAWNING, SimulatorFeatures.DELETING,
            SimulatorFeatures.ENTITY_STATE_GETTING} <= capabilities.features
    with pytest.raises(SceneControlError) as error:
        provider.client.require_capabilities(['usd'])
    assert error.value.failure is SceneControlFailure.MISSING_CAPABILITY


def test_catalog_resources_spawn_by_uri_at_the_requested_static_pose(provider, catalog) -> None:
    provider.client.require_capabilities(['sdf'])
    specs = [
        SpawnSpec.for_resource(catalog.targets['g1'], 'bench_test_target_0', 2.345, -0.678, 0.7),
        SpawnSpec.for_resource(catalog.objects['table'], 'bench_test_obj_0', -1.5, 0.25, -2.1),
    ]

    with provider.client.scene(specs) as names:
        snapshot = provider.client.get_scene_snapshot([ROBOT, *names])

    assert names == ('bench_test_target_0', 'bench_test_obj_0')
    # One response: the robot and both entities share a frame and a stamp.
    assert snapshot.frame_id == 'world'
    assert set(snapshot.poses) == {ROBOT, *names}
    for spec in specs:
        pose = snapshot.poses[spec.name]
        assert pose['position']['x'] == pytest.approx(spec.x, abs=POSITION_TOLERANCE_M)
        assert pose['position']['y'] == pytest.approx(spec.y, abs=POSITION_TOLERANCE_M)
        assert pose['position']['z'] == pytest.approx(spec.z, abs=POSITION_TOLERANCE_M)
        yaw_error = math.remainder(_yaw(pose) - spec.yaw, math.tau)
        assert abs(yaw_error) <= YAW_TOLERANCE_RAD


def test_a_duplicate_name_is_refused_without_a_renamed_copy(provider, catalog) -> None:
    provider.client.require_capabilities(['sdf'])
    spec = SpawnSpec.for_resource(catalog.objects['cabinet'], 'bench_test_dup', 1.0, 1.0, 0.0)
    provider.client.spawn_entities([spec], [])

    with pytest.raises(SceneControlError) as error:
        provider.client.spawn_entity(spec)

    # Gazebo reports a generic failure, not NAME_NOT_UNIQUE; either way the
    # response proves nothing was created, so nothing is queued for rollback.
    assert error.value.result_code == Result.RESULT_OPERATION_FAILED
    assert not error.value.may_have_succeeded
    assert provider.benchmark_names() == ['bench_test_dup']


@pytest.mark.parametrize('resource', ['missing', 'malformed'])
def test_an_invalid_resource_fails_without_creating_an_entity(
        provider, tmp_path, resource) -> None:
    path = tmp_path / 'model.sdf'
    if resource == 'malformed':
        path.write_text('<sdf version="1.10"><model name="x"><link name="l"></sdf>')
    spec = SpawnSpec(name='bench_test_invalid', uri=str(path), resource_format='sdf',
                     x=0.0, y=0.0, z=0.0, yaw=0.0)

    with pytest.raises(SceneControlError) as error:
        provider.client.spawn_entity(spec)

    assert error.value.result_code == Result.RESULT_OPERATION_FAILED
    assert provider.benchmark_names() == []


def test_name_filters_match_whole_names(provider, catalog) -> None:
    for index in range(2):
        spec = SpawnSpec.for_resource(
            catalog.objects['iv_pole'], f'bench_test_filter_{index}', index, 2.0, 0.0)
        provider.client.spawn_entities([spec], [])

    assert provider.names('^bench_') == []
    assert provider.names('test_filter') == []
    assert provider.names('^bench_test_filter_1$') == ['bench_test_filter_1']
    assert sorted(provider.names('^bench_.*$')) == ['bench_test_filter_0', 'bench_test_filter_1']
    assert provider.client.find_benchmark_entities('bench_test_filter_') == (
        'bench_test_filter_0', 'bench_test_filter_1')
    assert provider.client.entity_exists('bench_test_filter_0')
    assert not provider.client.entity_exists('bench_test_filter_')


def test_delete_confirms_absence_and_a_repeated_delete_is_accepted(provider, catalog) -> None:
    spec = SpawnSpec.for_resource(catalog.objects['cabinet'], 'bench_test_delete', 0.0, -2.0, 0.0)
    provider.client.spawn_entities([spec], [])

    provider.client.delete_entity(spec.name)

    assert not provider.client.entity_exists(spec.name)
    # Gazebo answers a delete of an absent entity with a generic failure rather
    # than NOT_FOUND, so the client treats confirmed absence as success.
    assert provider.delete_raw(spec.name).result.result == Result.RESULT_OPERATION_FAILED
    provider.client.delete_entity(spec.name)
    assert provider.benchmark_names() == []


def test_a_failed_scene_rolls_back_every_spawn(provider, catalog, tmp_path) -> None:
    provider.client.require_capabilities(['sdf'])
    specs = [
        SpawnSpec.for_resource(catalog.targets['g1'], 'bench_test_rollback_target', 3.0, 0.0, 0.0),
        SpawnSpec.for_resource(catalog.objects['table'], 'bench_test_rollback_obj', 3.0, 1.0, 0.0),
        SpawnSpec(name='bench_test_rollback_missing', uri=str(tmp_path / 'missing.sdf'),
                  resource_format='sdf', x=0.0, y=0.0, z=0.0, yaw=0.0),
    ]

    with pytest.raises(SceneControlError):
        with provider.client.scene(specs):
            pass

    assert provider.benchmark_names() == []


def test_entities_the_benchmark_does_not_own_are_out_of_reach(provider) -> None:
    for name in (ROBOT, AUTHORED):
        with pytest.raises(SceneControlError) as error:
            provider.client.delete_entity(name)
        assert error.value.failure is SceneControlFailure.UNOWNED_ENTITY
    assert ROBOT not in provider.client.find_benchmark_entities()
    assert {AUTHORED, ROBOT} <= set(provider.names())
