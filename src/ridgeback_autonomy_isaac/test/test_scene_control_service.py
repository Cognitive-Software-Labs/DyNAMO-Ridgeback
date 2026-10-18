"""Isaac scene-control services driven through the common benchmark client.

The services, their executor thread, and the command queue are the ones the
runner uses; a thread stands in for the SimulationApp main loop and a
recording fake stands in for the stage, so no Isaac Kit is needed. Each test
serves under a unique namespace, so it cannot reach a running simulator.
"""

from __future__ import annotations

import math
from pathlib import Path
import threading
import time
import uuid

import pytest
import rclpy
from simulation_interfaces.msg import Result, SimulatorFeatures
from simulation_interfaces.srv import DeleteEntity, GetEntities, SpawnEntity

from ridgeback_autonomy.benchmarking.asset_catalog import load_asset_catalog
from ridgeback_autonomy.benchmarking.scene_control import (
    SceneControlError,
    SceneControlFailure,
    SpawnSpec,
    create_scene_control_client,
)
from scene_provider import (
    RobotState,
    SceneCommandQueue,
    SceneControlService,
    SceneEntities,
    SimFrame,
)
from test_scene_provider import FakeNativeScene


CATALOG = Path(__file__).resolve().parents[1] / 'sim' / 'benchmark_assets.yaml'
ROBOT = 'r100_0001/robot'
ROBOT_STATE = RobotState(1.5, -0.5, 0.07617, 0.3, 0.2, 0.1, 0.05)
SIM_DT = 0.025
WAIT_SEC = 10.0


class MainLoop:
    """Drains the scene queue once per simulated update, like the runner."""

    def __init__(self, service: SceneControlService) -> None:
        self._service = service
        self.sim_time = 100.0
        self.running = threading.Event()
        self.running.set()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            if self.running.is_set():
                self.sim_time += SIM_DT
                self._service.process_pending(SimFrame(self.sim_time, ROBOT_STATE))
            time.sleep(0.002)

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(WAIT_SEC)


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def native():
    return FakeNativeScene()


@pytest.fixture
def queue():
    return SceneCommandQueue(timeout_sec=1.0)


@pytest.fixture
def provider(ros, native, queue):
    service = SceneControlService(
        SceneEntities(native, ROBOT),
        namespace=f'/scene_control_test_{uuid.uuid4().hex[:8]}',
        queue=queue,
    )
    loop = MainLoop(service)
    yield service, loop
    loop.stop()
    if service.executor_thread.is_alive():
        service.shutdown()


@pytest.fixture
def node(ros):
    node = rclpy.create_node(f'scene_control_client_{uuid.uuid4().hex[:8]}')
    yield node
    node.destroy_node()


@pytest.fixture
def client(provider, node):
    service, _loop = provider
    # The provider runs without a simulation loop publishing /clock; these
    # cases test its services, not the benchmark's readiness gate.
    client = create_scene_control_client(
        node, service.service_namespace, observe_clock=False,
        ready_timeout_sec=3.0, delete_timeout_sec=3.0, retry_sleep_sec=0.05)
    client.wait_for_services(WAIT_SEC)
    client.require_capabilities(['usd'])
    return client


@pytest.fixture
def resource(tmp_path):
    path = tmp_path / 'thing.usda'
    path.write_text('#usda 1.0\n')
    return str(path)


def raw_call(node, service, suffix, srv_type, request):
    raw = node.create_client(srv_type, f'{service.service_namespace}/{suffix}')
    try:
        assert raw.wait_for_service(timeout_sec=WAIT_SEC)
        future = raw.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=WAIT_SEC)
        assert future.done()
        return future.result()
    finally:
        node.destroy_client(raw)


def spec(name, resource, x=2.0, y=1.0, yaw=0.7) -> SpawnSpec:
    return SpawnSpec(name, resource, 'usd', x, y, 0.0, yaw)


def yaw_of(pose) -> float:
    q = pose['orientation']
    return math.atan2(2 * (q['w'] * q['z'] + q['x'] * q['y']),
                      1 - 2 * (q['y'] ** 2 + q['z'] ** 2))


def test_service_names_and_advertised_capabilities(provider, node) -> None:
    service, _loop = provider
    client = create_scene_control_client(
        node, service.service_namespace, observe_clock=False)
    client.wait_for_services(WAIT_SEC)

    capabilities = client.require_capabilities(['usd'])

    assert service.service_namespace.endswith('/isaac_sim')
    assert capabilities.features == frozenset({
        SimulatorFeatures.SPAWNING, SimulatorFeatures.DELETING,
        SimulatorFeatures.ENTITY_STATE_GETTING})
    assert capabilities.spawn_formats == ('usd',)
    with pytest.raises(SceneControlError) as excinfo:
        client.require_capabilities(['sdf'])
    assert excinfo.value.failure is SceneControlFailure.MISSING_CAPABILITY


def test_catalog_g1_spawns_and_reads_back_with_the_robot_in_one_update(
        provider, client, native) -> None:
    g1 = load_asset_catalog(str(CATALOG)).targets['g1']
    target = SpawnSpec.for_resource(g1, 'bench_g1', 3.0, -1.0, 2.5)
    spawned: list[str] = []

    client.spawn_entities([target], spawned)
    snapshot = client.get_scene_snapshot([ROBOT, 'bench_g1'])

    assert spawned == ['bench_g1']
    assert native.prims['/scene_entities/bench_g1'][0] == g1.uri
    assert snapshot.frame_id == 'world'
    robot, pose = snapshot.poses[ROBOT], snapshot.poses['bench_g1']
    assert (robot['position']['x'], robot['position']['y']) == (1.5, -0.5)
    assert yaw_of(robot) == pytest.approx(0.3)
    assert (pose['position']['x'], pose['position']['y'], pose['position']['z']) == (
        3.0, -1.0, 0.0)
    assert yaw_of(pose) == pytest.approx(2.5)


def test_duplicate_name_is_a_name_conflict(client, resource) -> None:
    client.spawn_entity(spec('bench_g1', resource))

    with pytest.raises(SceneControlError) as excinfo:
        client.spawn_entity(spec('bench_g1', resource))

    assert excinfo.value.failure is SceneControlFailure.NAME_CONFLICT
    assert not excinfo.value.may_have_succeeded


def test_unusable_resources_are_explicit_failures(client, native, resource, tmp_path) -> None:
    sdf = tmp_path / 'model.sdf'
    sdf.write_text('<sdf/>')
    native.invalid.add(resource)
    cases = [
        (str(sdf), SceneControlFailure.UNSUPPORTED_FORMAT),
        (str(tmp_path / 'missing.usda'), SceneControlFailure.INVALID_RESOURCE),
        (resource, SceneControlFailure.INVALID_RESOURCE),
    ]
    for uri, failure in cases:
        with pytest.raises(SceneControlError) as excinfo:
            client.spawn_entity(spec('bench_g1', uri))
        assert excinfo.value.failure is failure, uri

    assert native.prims == {}
    assert client.find_benchmark_entities() == ()


def test_failed_scene_rolls_back_in_reverse_order(client, native, resource, tmp_path) -> None:
    missing = str(tmp_path / 'missing.usda')
    specs = [spec('bench_g1', resource), spec('bench_table', resource, x=4.0),
             spec('bench_bad', missing)]

    with pytest.raises(SceneControlError) as excinfo:
        with client.scene(specs):
            pass

    assert excinfo.value.failure is SceneControlFailure.INVALID_RESOURCE
    assert native.deleted == ['/scene_entities/bench_table', '/scene_entities/bench_g1']
    assert native.prims == {}
    assert client.find_benchmark_entities() == ()


def test_discovery_and_delete_touch_only_benchmark_entities(
        provider, client, node, native, resource) -> None:
    service, _loop = provider
    for name in ('bench_a', 'bench_b'):
        client.spawn_entity(spec(name, resource))

    assert client.find_benchmark_entities() == ('bench_a', 'bench_b')
    everything = raw_call(node, service, 'get_entities', GetEntities, GetEntities.Request())
    assert list(everything.entities) == [ROBOT, 'bench_a', 'bench_b']

    for name, code in ((ROBOT, Result.RESULT_OPERATION_FAILED),
                       ('/ridgeback', Result.RESULT_NOT_FOUND),
                       ('/scene_entities/bench_a', Result.RESULT_NOT_FOUND)):
        response = raw_call(node, service, 'delete_entity', DeleteEntity,
                            DeleteEntity.Request(entity=name))
        assert response.result.result == code, name
    assert native.deleted == []

    client.delete_entity('bench_a')
    client.delete_entity('bench_a')  # absent already: verified, not an error
    assert client.find_benchmark_entities() == ('bench_b',)
    assert client.entity_exists(ROBOT)


def test_request_the_main_loop_never_takes_is_withdrawn(provider, node, native, resource) -> None:
    service, loop = provider
    loop.running.clear()
    request = SpawnEntity.Request(name='bench_g1', uri=resource)
    request.initial_pose.pose.orientation.w = 1.0

    response = raw_call(node, service, 'spawn_entity', SpawnEntity, request)

    assert response.result.result == Result.RESULT_INCORRECT_STATE
    assert 'withdrawn' in response.result.error_message
    loop.running.set()
    listing = raw_call(node, service, 'get_entities', GetEntities, GetEntities.Request())
    assert list(listing.entities) == [ROBOT]
    assert native.prims == {}


def test_shutdown_cancels_a_queued_request_and_joins_the_executor(
        provider, node, native, queue, resource) -> None:
    service, loop = provider
    loop.running.clear()
    raw = node.create_client(SpawnEntity, f'{service.service_namespace}/spawn_entity')
    assert raw.wait_for_service(timeout_sec=WAIT_SEC)
    request = SpawnEntity.Request(name='bench_g1', uri=resource)
    request.initial_pose.pose.orientation.w = 1.0
    future = raw.call_async(request)
    deadline = time.monotonic() + WAIT_SEC
    while len(queue) == 0:
        assert time.monotonic() < deadline
        rclpy.spin_once(node, timeout_sec=0.01)

    service.shutdown()

    rclpy.spin_until_future_complete(node, future, timeout_sec=WAIT_SEC)
    assert future.result().result.result == Result.RESULT_INCORRECT_STATE
    assert 'shutting down' in future.result().result.error_message
    assert not service.executor_thread.is_alive()
    loop.running.set()
    assert native.prims == {}
