"""Scene-control client against an in-memory standard-services provider."""

from __future__ import annotations

import ast
import math
from pathlib import Path
import re
import subprocess
import sys

import pytest
from simulation_interfaces.msg import EntityState, Result, SimulatorFeatures
from simulation_interfaces.srv import (
    DeleteEntity,
    GetEntities,
    GetEntitiesStates,
    GetSimulatorFeatures,
    SpawnEntity,
)

from ridgeback_autonomy.benchmarking.scene_control import (
    BENCHMARK_ENTITY_PREFIX,
    SERVICE_TYPES,
    SceneControlClient,
    SceneControlError,
    SceneControlFailure,
    SpawnSpec,
    posix_ere_literal,
    scene_control_service_names,
)
from ridgeback_autonomy.benchmarking.simulation import (
    compute_ground_truth_instances,
    pose_snapshot_from_payload,
)


ROBOT = 'r100_0001/robot'
WORLD_ENTITIES = ('calibration_floor', 'wall_north', 'wall_south', ROBOT)
REQUIRED_FEATURES = (
    SimulatorFeatures.SPAWNING,
    SimulatorFeatures.DELETING,
    SimulatorFeatures.ENTITY_STATE_GETTING,
)
PENDING = object()
COMMON_MODULES = ('asset_catalog.py', 'scene_control.py')
BENCHMARK_ROOT = Path(__file__).parents[1] / 'ridgeback_autonomy' / 'benchmarking'
SIMULATOR_PACKAGES = (
    'ridgeback_autonomy_gz',
    'ridgeback_autonomy_isaac',
    'ros_gz_sim',
    'ros_gz_bridge',
    'ros_gz_interfaces',
    'gz',
    'omni',
    'isaacsim',
    'pxr',
    'carb',
)


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


class FakeFuture:
    def __init__(self, response=None, error=None, pending=False) -> None:
        self._response = response
        self._error = error
        self._pending = pending
        self.cancelled = False

    def done(self) -> bool:
        return not self._pending or self.cancelled

    def result(self):
        return self._response

    def exception(self):
        return self._error

    def cancel(self) -> None:
        self.cancelled = True


class FakeServiceClient:
    def __init__(self, simulator: 'FakeSimulator', suffix: str) -> None:
        self.simulator = simulator
        self.suffix = suffix
        self.removed = []

    def wait_for_service(self, timeout_sec=None) -> bool:
        return self.suffix in self.simulator.available

    def call_async(self, request) -> FakeFuture:
        outcome = self.simulator.handle(self.suffix, request)
        if outcome is PENDING:
            return FakeFuture(pending=True)
        if isinstance(outcome, BaseException):
            return FakeFuture(error=outcome)
        return FakeFuture(response=outcome)

    def remove_pending_request(self, future) -> None:
        self.removed.append(future)


def _ok(response):
    response.result.result = Result.RESULT_OK
    return response


def _failed(response, code: int, message: str):
    response.result.result = code
    response.result.error_message = message
    return response


class FakeSimulator:
    """Serves the five standard services from a name -> pose dictionary.

    Behaves like the qualified Gazebo server where the standard leaves room:
    deleting an absent entity is a generic failure, not NOT_FOUND.
    """

    def __init__(self, clock: FakeClock) -> None:
        self.clock = clock
        self.entities = {name: (0.0, 0.0, 0.0, 0.0) for name in WORLD_ENTITIES}
        self.features = list(REQUIRED_FEATURES)
        self.formats = ['sdf']
        self.available = set(SERVICE_TYPES)
        self.overrides = {}
        self.calls = []
        self.stamp = (72, 462_000_000)
        self.frame_id = 'world'
        self.hidden_states = {}
        self.lingering = {}

    def handle(self, suffix: str, request):
        self.calls.append((suffix, request))
        if suffix in self.overrides:
            return self.overrides[suffix](request)
        return getattr(self, suffix)(request)

    def calls_to(self, suffix: str) -> list:
        return [request for name, request in self.calls if name == suffix]

    def get_simulator_features(self, request):
        response = GetSimulatorFeatures.Response()
        response.features.features = self.features
        response.features.spawn_formats = self.formats
        return response

    def spawn_entity(self, request):
        response = SpawnEntity.Response()
        if request.name in self.entities:
            return _failed(response, SpawnEntity.Response.NAME_NOT_UNIQUE, 'name taken')
        self.commit_spawn(request)
        response.entity_name = request.name
        return _ok(response)

    def commit_spawn(self, request) -> None:
        pose = request.initial_pose.pose
        yaw = 2.0 * math.atan2(pose.orientation.z, pose.orientation.w)
        self.entities[request.name] = (pose.position.x, pose.position.y, pose.position.z, yaw)

    def delete_entity(self, request):
        response = DeleteEntity.Response()
        if request.entity not in self.entities:
            return _failed(response, Result.RESULT_OPERATION_FAILED,
                           'Error while trying to remove entity')
        del self.entities[request.entity]
        return _ok(response)

    def get_entities(self, request):
        response = GetEntities.Response()
        pattern = re.compile(request.filters.filter)
        names = [*self.entities, *(name for name, polls in self.lingering.items() if polls)]
        for name in self.lingering:
            self.lingering[name] = max(0, self.lingering[name] - 1)
        response.entities = [name for name in names if pattern.search(name)]
        return _ok(response)

    def get_entities_states(self, request):
        response = GetEntitiesStates.Response()
        for name, (x, y, z, yaw) in self.entities.items():
            if self.hidden_states.get(name, 0) > 0:
                self.hidden_states[name] -= 1
                continue
            state = EntityState()
            state.header.frame_id = self.frame_id
            state.header.stamp.sec, state.header.stamp.nanosec = self.stamp
            state.pose.position.x, state.pose.position.y, state.pose.position.z = x, y, z
            state.pose.orientation.z = math.sin(0.5 * yaw)
            state.pose.orientation.w = math.cos(0.5 * yaw)
            response.entities.append(name)
            response.states.append(state)
        return _ok(response)


@pytest.fixture
def simulator() -> FakeSimulator:
    return FakeSimulator(FakeClock())


def make_client(simulator: FakeSimulator, **options) -> tuple[SceneControlClient, dict]:
    clients = {suffix: FakeServiceClient(simulator, suffix) for suffix in SERVICE_TYPES}
    client = SceneControlClient(
        clients,
        simulator.clock.advance,
        service_names=scene_control_service_names('/sim'),
        clock=simulator.clock,
        **options,
    )
    return client, clients


def spec(name: str, x: float = 2.0, y: float = 0.5, yaw: float = math.pi) -> SpawnSpec:
    return SpawnSpec(
        name=name, uri='/assets/model.sdf', resource_format='sdf', x=x, y=y, z=0.0, yaw=yaw)


def test_service_names_compose_the_standard_suffixes_under_the_provider_namespace() -> None:
    assert scene_control_service_names('/gzserver/') == {
        'get_simulator_features': '/gzserver/get_simulator_features',
        'spawn_entity': '/gzserver/spawn_entity',
        'delete_entity': '/gzserver/delete_entity',
        'get_entities': '/gzserver/get_entities',
        'get_entities_states': '/gzserver/get_entities_states',
    }
    assert scene_control_service_names('/')['spawn_entity'] == '/spawn_entity'
    # A relative name would resolve under the runner's robot namespace.
    with pytest.raises(ValueError, match='absolute'):
        scene_control_service_names('gzserver')


def test_missing_service_fails_discovery_naming_it(simulator) -> None:
    simulator.available.discard('spawn_entity')
    client, _clients = make_client(simulator)

    with pytest.raises(SceneControlError) as error:
        client.wait_for_services(timeout_sec=5.0)

    assert error.value.failure is SceneControlFailure.UNAVAILABLE
    assert '/sim/spawn_entity' in str(error.value)
    assert '/sim/delete_entity' not in str(error.value)


def test_capability_gate_accepts_required_features_and_formats(simulator) -> None:
    simulator.features = [*REQUIRED_FEATURES, SimulatorFeatures.SPAWNING_RESOURCE_STRING]
    simulator.formats = ['SDF', 'urdf']
    client, _clients = make_client(simulator)

    capabilities = client.require_capabilities({'sdf'})

    assert capabilities.spawn_formats == ('sdf', 'urdf')
    assert SimulatorFeatures.SPAWNING_RESOURCE_STRING in capabilities.features
    assert client.capabilities == capabilities


@pytest.mark.parametrize('missing_feature, label', [
    (SimulatorFeatures.SPAWNING, 'SPAWNING'),
    (SimulatorFeatures.DELETING, 'DELETING'),
    (SimulatorFeatures.ENTITY_STATE_GETTING, 'ENTITY_STATE_GETTING'),
])
def test_capability_gate_rejects_a_missing_required_feature(
        simulator, missing_feature, label) -> None:
    simulator.features = [f for f in REQUIRED_FEATURES if f != missing_feature]
    client, _clients = make_client(simulator)

    with pytest.raises(SceneControlError) as error:
        client.require_capabilities()

    assert error.value.failure is SceneControlFailure.MISSING_CAPABILITY
    assert label in str(error.value)
    assert client.capabilities is None


def test_capability_gate_rejects_an_unadvertised_resource_format(simulator) -> None:
    client, _clients = make_client(simulator)

    with pytest.raises(SceneControlError, match='spawn format "usd"') as error:
        client.require_capabilities({'sdf', 'usd'})

    assert error.value.failure is SceneControlFailure.MISSING_CAPABILITY


def test_spawn_sends_an_exact_world_frame_request_and_returns_the_canonical_name(
        simulator) -> None:
    client, _clients = make_client(simulator)

    name = client.spawn_entity(spec('bench_run_t0_target_0', x=3.0, y=-0.4, yaw=math.pi / 2))

    assert name == 'bench_run_t0_target_0'
    request = simulator.calls_to('spawn_entity')[0]
    assert request.allow_renaming is False
    assert request.uri == '/assets/model.sdf'
    assert request.initial_pose.header.frame_id == 'world'
    assert (request.initial_pose.pose.position.x, request.initial_pose.pose.position.y,
            request.initial_pose.pose.position.z) == (3.0, -0.4, 0.0)
    orientation = request.initial_pose.pose.orientation
    assert (orientation.x, orientation.y) == (0.0, 0.0)
    assert orientation.z == pytest.approx(math.sin(math.pi / 4))
    assert orientation.w == pytest.approx(math.cos(math.pi / 4))


def test_the_returned_canonical_name_is_used_for_later_calls(simulator) -> None:
    def canonicalizing_spawn(request):
        response = FakeSimulator.spawn_entity(simulator, request)
        simulator.entities[f'{request.name}_canonical'] = simulator.entities.pop(request.name)
        response.entity_name = f'{request.name}_canonical'
        return response

    simulator.overrides['spawn_entity'] = canonicalizing_spawn
    client, _clients = make_client(simulator)

    with client.scene([spec('bench_run_t0_target_0')]) as names:
        assert names == ('bench_run_t0_target_0_canonical',)

    assert [r.entity for r in simulator.calls_to('delete_entity')] == [
        'bench_run_t0_target_0_canonical']
    assert 'bench_run_t0_target_0_canonical' not in simulator.entities


@pytest.mark.parametrize('code, failure', [
    (SpawnEntity.Response.NAME_NOT_UNIQUE, SceneControlFailure.NAME_CONFLICT),
    (SpawnEntity.Response.NAME_INVALID, SceneControlFailure.INVALID_NAME),
    (SpawnEntity.Response.UNSUPPORTED_FORMAT, SceneControlFailure.UNSUPPORTED_FORMAT),
    (SpawnEntity.Response.RESOURCE_PARSE_ERROR, SceneControlFailure.INVALID_RESOURCE),
    (SpawnEntity.Response.MISSING_ASSETS, SceneControlFailure.INVALID_RESOURCE),
    (SpawnEntity.Response.INVALID_POSE, SceneControlFailure.INVALID_POSE),
    (Result.RESULT_FEATURE_UNSUPPORTED, SceneControlFailure.UNSUPPORTED_FEATURE),
    (Result.RESULT_INCORRECT_STATE, SceneControlFailure.INCORRECT_STATE),
    (Result.RESULT_OPERATION_FAILED, SceneControlFailure.OPERATION_FAILED),
    (77, SceneControlFailure.OPERATION_FAILED),
])
def test_spawn_result_codes_normalize_and_keep_the_provider_message(
        simulator, code, failure) -> None:
    simulator.overrides['spawn_entity'] = lambda request: _failed(
        SpawnEntity.Response(), code, f'provider says {code}')
    client, _clients = make_client(simulator)

    with pytest.raises(SceneControlError) as error:
        client.spawn_entity(spec('bench_run_t0_obj_0'))

    assert error.value.failure is failure
    assert error.value.result_code == code
    assert error.value.entity == 'bench_run_t0_obj_0'
    assert f'provider says {code}' in str(error.value)


def test_duplicate_name_is_rejected_instead_of_renamed(simulator) -> None:
    client, _clients = make_client(simulator)
    client.spawn_entity(spec('bench_run_t0_obj_0'))

    with pytest.raises(SceneControlError) as error:
        client.spawn_entity(spec('bench_run_t0_obj_0'))

    assert error.value.failure is SceneControlFailure.NAME_CONFLICT
    assert all(request.allow_renaming is False for request in simulator.calls_to('spawn_entity'))


def test_a_known_unsupported_format_fails_before_the_provider_is_asked(simulator) -> None:
    client, _clients = make_client(simulator)
    client.require_capabilities()
    usd = SpawnSpec('bench_run_t0_obj_0', '/assets/model.usda', 'usd', 0.0, 0.0, 0.0, 0.0)

    with pytest.raises(SceneControlError) as error:
        client.spawn_entity(usd)

    assert error.value.failure is SceneControlFailure.UNSUPPORTED_FORMAT
    assert simulator.calls_to('spawn_entity') == []


def test_an_unanswered_call_times_out_inside_its_budget_and_is_withdrawn(simulator) -> None:
    simulator.overrides['spawn_entity'] = lambda request: PENDING
    client, clients = make_client(simulator, call_timeout_sec=10.0)

    with pytest.raises(SceneControlError) as error:
        client.spawn_entity(spec('bench_run_t0_target_0'))

    assert error.value.failure is SceneControlFailure.TIMEOUT
    assert '/sim/spawn_entity' in str(error.value)
    assert 10.0 <= simulator.clock.now <= 10.0 + 0.11
    withdrawn = clients['spawn_entity'].removed
    assert len(withdrawn) == 1 and withdrawn[0].cancelled


def test_a_transport_exception_is_an_operation_failure(simulator) -> None:
    simulator.overrides['get_entities_states'] = lambda request: RuntimeError('middleware')
    client, _clients = make_client(simulator)

    with pytest.raises(SceneControlError, match='middleware') as error:
        client.get_scene_snapshot([ROBOT])

    assert error.value.failure is SceneControlFailure.OPERATION_FAILED


def test_partial_scene_spawn_rolls_back_in_reverse_order(simulator) -> None:
    def fail_third(request):
        if request.name.endswith('obj_1'):
            return _failed(SpawnEntity.Response(), SpawnEntity.Response.RESOURCE_PARSE_ERROR,
                           'bad model')
        return FakeSimulator.spawn_entity(simulator, request)

    simulator.overrides['spawn_entity'] = fail_third
    client, _clients = make_client(simulator)
    specs = [spec('bench_run_t0_target_0'), spec('bench_run_t0_obj_0'), spec('bench_run_t0_obj_1')]

    with pytest.raises(SceneControlError) as error:
        with client.scene(specs):
            pytest.fail('a failed spawn must not enter the scene body')

    assert error.value.failure is SceneControlFailure.INVALID_RESOURCE
    assert [r.entity for r in simulator.calls_to('delete_entity')] == [
        'bench_run_t0_obj_0', 'bench_run_t0_target_0']
    assert not [name for name in simulator.entities if name.startswith(BENCHMARK_ENTITY_PREFIX)]


@pytest.mark.parametrize('interruption', [RuntimeError('capture failed'), KeyboardInterrupt()])
def test_a_failure_inside_the_scene_rolls_back_and_propagates(simulator, interruption) -> None:
    client, _clients = make_client(simulator)
    specs = [spec('bench_run_t0_target_0'), spec('bench_run_t0_obj_0')]

    with pytest.raises(type(interruption)):
        with client.scene(specs):
            raise interruption

    assert [r.entity for r in simulator.calls_to('delete_entity')] == [
        'bench_run_t0_obj_0', 'bench_run_t0_target_0']
    assert set(simulator.entities) == set(WORLD_ENTITIES)


def test_rollback_failures_are_attached_to_the_original_error(simulator) -> None:
    simulator.overrides['delete_entity'] = lambda request: _failed(
        DeleteEntity.Response(), Result.RESULT_INCORRECT_STATE, 'paused')
    client, _clients = make_client(simulator, delete_timeout_sec=1.0)

    with pytest.raises(RuntimeError, match='capture failed') as error:
        with client.scene([spec('bench_run_t0_target_0')]):
            raise RuntimeError('capture failed')

    assert any('bench_run_t0_target_0' in note for note in error.value.__notes__)


def test_a_clean_scene_with_a_stuck_entity_raises_after_trying_every_removal(
        simulator) -> None:
    def refuse_target(request):
        if request.entity.endswith('target_0'):
            return _failed(DeleteEntity.Response(), Result.RESULT_INCORRECT_STATE, 'paused')
        return FakeSimulator.delete_entity(simulator, request)

    simulator.overrides['delete_entity'] = refuse_target
    client, _clients = make_client(simulator, delete_timeout_sec=1.0)

    with pytest.raises(SceneControlError) as error:
        with client.scene([spec('bench_run_t0_target_0'), spec('bench_run_t0_obj_0')]):
            pass

    assert error.value.failure is SceneControlFailure.CLEANUP_INCOMPLETE
    assert 'bench_run_t0_target_0' in str(error.value)
    assert 'bench_run_t0_obj_0' not in simulator.entities


def test_a_spawn_with_no_response_is_still_rolled_back(simulator) -> None:
    def commit_without_answering(request):
        simulator.commit_spawn(request)
        return PENDING

    simulator.overrides['spawn_entity'] = commit_without_answering
    client, _clients = make_client(simulator, call_timeout_sec=1.0)
    spawned = []

    with pytest.raises(SceneControlError) as error:
        client.spawn_entities([spec('bench_run_t0_target_0')], spawned)

    assert error.value.failure is SceneControlFailure.TIMEOUT
    assert spawned == ['bench_run_t0_target_0']
    assert client.rollback(spawned) == []
    assert 'bench_run_t0_target_0' not in simulator.entities


def test_each_spawn_waits_for_state_readiness_before_the_next(simulator) -> None:
    client, _clients = make_client(simulator, retry_sleep_sec=0.5)
    original_spawn = simulator.spawn_entity

    def spawn_hidden_for_two_snapshots(request):
        simulator.hidden_states[request.name] = 2
        return original_spawn(request)

    simulator.overrides['spawn_entity'] = spawn_hidden_for_two_snapshots
    spawned = []

    client.spawn_entities([spec('bench_run_t0_target_0'), spec('bench_run_t0_obj_0')], spawned)

    assert spawned == ['bench_run_t0_target_0', 'bench_run_t0_obj_0']
    order = [name for name, _request in simulator.calls]
    assert order == ['spawn_entity', *['get_entities_states'] * 3] * 2


def test_readiness_times_out_as_missing_state(simulator) -> None:
    simulator.overrides['spawn_entity'] = lambda request: _ok(
        SpawnEntity.Response(entity_name=request.name))
    client, _clients = make_client(simulator, ready_timeout_sec=2.0)
    spawned = []

    with pytest.raises(SceneControlError) as error:
        client.spawn_entities([spec('bench_run_t0_target_0')], spawned)

    assert error.value.failure is SceneControlFailure.MISSING_STATE
    assert spawned == ['bench_run_t0_target_0']
    assert 2.0 <= simulator.clock.now <= 2.0 + 0.6


def test_one_snapshot_converts_into_the_ground_truth_pose_contract(simulator) -> None:
    simulator.entities[ROBOT] = (0.5, -0.25, 0.0759, 0.0)
    simulator.entities['bench_run_t0_target_0'] = (3.75, -0.25, 0.0, math.pi)
    simulator.entities['bench_run_t0_target_1'] = (2.0, 1.0, 0.0, math.pi)
    simulator.frame_id = ''
    client, _clients = make_client(simulator)

    snapshot = client.get_scene_snapshot(
        [ROBOT, 'bench_run_t0_target_0', 'bench_run_t0_target_1'])

    assert snapshot.frame_id == 'world'
    assert snapshot.stamp_ns == 72_462_000_000
    assert set(snapshot.poses) == {ROBOT, 'bench_run_t0_target_0', 'bench_run_t0_target_1'}
    assert len(simulator.calls_to('get_entities_states')) == 1
    targets = [(0, 'bench_run_t0_target_0', None), (1, 'bench_run_t0_target_1', None)]
    truth = compute_ground_truth_instances(snapshot.poses, ROBOT, targets)
    legacy = compute_ground_truth_instances(
        pose_snapshot_from_payload({'pose': list(snapshot.poses.values())}), ROBOT, targets)
    assert truth == legacy
    assert truth[0].world_x == pytest.approx(3.75)
    assert truth[0].lateral_m == pytest.approx(0.0)
    assert truth[1].lateral_m == pytest.approx(1.25)


def test_missing_robot_or_target_state_is_reported_by_name(simulator) -> None:
    client, _clients = make_client(simulator)
    del simulator.entities[ROBOT]

    with pytest.raises(SceneControlError) as error:
        client.get_scene_snapshot([ROBOT, 'bench_run_t0_target_0'])

    assert error.value.failure is SceneControlFailure.MISSING_STATE
    assert ROBOT in str(error.value) and 'bench_run_t0_target_0' in str(error.value)


def test_states_from_different_simulation_stamps_are_rejected(simulator) -> None:
    simulator.entities['bench_run_t0_target_0'] = (2.0, 0.0, 0.0, 0.0)

    def split_stamps(request):
        response = FakeSimulator.get_entities_states(simulator, request)
        response.states[-1].header.stamp.nanosec += 1_000_000
        return response

    simulator.overrides['get_entities_states'] = split_stamps
    client, _clients = make_client(simulator)

    with pytest.raises(SceneControlError) as error:
        client.get_scene_snapshot([ROBOT, 'bench_run_t0_target_0'])

    assert error.value.failure is SceneControlFailure.INCOHERENT_STATE


def test_states_outside_the_world_frame_are_rejected(simulator) -> None:
    simulator.frame_id = 'odom'
    client, _clients = make_client(simulator)

    with pytest.raises(SceneControlError, match='frame "world"') as error:
        client.get_scene_snapshot([ROBOT])

    assert error.value.failure is SceneControlFailure.INCOHERENT_STATE


def test_mismatched_entity_and_state_lists_are_malformed(simulator) -> None:
    def drop_a_state(request):
        response = FakeSimulator.get_entities_states(simulator, request)
        response.states.pop()
        return response

    simulator.overrides['get_entities_states'] = drop_a_state
    client, _clients = make_client(simulator)

    with pytest.raises(SceneControlError) as error:
        client.get_scene_snapshot([ROBOT])

    assert error.value.failure is SceneControlFailure.MALFORMED_RESPONSE


def test_delete_returns_only_after_absence_is_observed(simulator) -> None:
    simulator.entities['bench_run_t0_obj_0'] = (1.0, 0.0, 0.0, 0.0)
    simulator.lingering['bench_run_t0_obj_0'] = 2
    client, _clients = make_client(simulator)

    client.delete_entity('bench_run_t0_obj_0')

    assert len(simulator.calls_to('delete_entity')) == 1
    assert len(simulator.calls_to('get_entities')) == 3
    assert simulator.calls_to('get_entities')[0].filters.filter == '^bench_run_t0_obj_0$'


def test_delete_that_never_takes_effect_is_incomplete_cleanup(simulator) -> None:
    simulator.entities['bench_run_t0_obj_0'] = (1.0, 0.0, 0.0, 0.0)
    simulator.overrides['delete_entity'] = lambda request: _ok(DeleteEntity.Response())
    client, _clients = make_client(simulator, delete_timeout_sec=3.0)

    with pytest.raises(SceneControlError) as error:
        client.delete_entity('bench_run_t0_obj_0')

    assert error.value.failure is SceneControlFailure.CLEANUP_INCOMPLETE
    assert len(simulator.calls_to('delete_entity')) == 1
    assert 3.0 <= simulator.clock.now <= 3.0 + 0.6


def test_a_rejected_delete_is_retried_while_the_entity_remains(simulator) -> None:
    simulator.entities['bench_run_t0_obj_0'] = (1.0, 0.0, 0.0, 0.0)
    attempts = []

    def busy_once(request):
        attempts.append(request.entity)
        if len(attempts) == 1:
            return _failed(DeleteEntity.Response(), Result.RESULT_INCORRECT_STATE, 'busy')
        return FakeSimulator.delete_entity(simulator, request)

    simulator.overrides['delete_entity'] = busy_once
    client, _clients = make_client(simulator)

    client.delete_entity('bench_run_t0_obj_0')

    assert attempts == ['bench_run_t0_obj_0', 'bench_run_t0_obj_0']
    assert 'bench_run_t0_obj_0' not in simulator.entities


@pytest.mark.parametrize('code, message', [
    (Result.RESULT_NOT_FOUND, 'no such entity'),
    # The qualified Gazebo server answers a repeated delete this way.
    (Result.RESULT_OPERATION_FAILED, 'Error while trying to remove entity'),
])
def test_cleanup_of_an_absent_entity_is_idempotent(simulator, code, message) -> None:
    simulator.overrides['delete_entity'] = lambda request: _failed(
        DeleteEntity.Response(), code, message)
    client, _clients = make_client(simulator)

    client.delete_entity('bench_run_t0_obj_0')
    client.delete_entity('bench_run_t0_obj_0')

    assert len(simulator.calls_to('delete_entity')) == 2


def test_prefix_discovery_cannot_return_unowned_names(simulator) -> None:
    for name in ('bench_run1_t0_target_0', 'bench_run1_t0_obj_0', 'bench_run2_t0_target_0',
                 'xbench_run1_t0', 'r100_0001/bench_run1_t0'):
        simulator.entities[name] = (0.0, 0.0, 0.0, 0.0)
    client, _clients = make_client(simulator)

    assert client.find_benchmark_entities('bench_run1_') == (
        'bench_run1_t0_obj_0', 'bench_run1_t0_target_0')
    assert simulator.calls_to('get_entities')[-1].filters.filter == '^bench_run1_'

    # A provider that ignores the filter still cannot leak another name.
    simulator.overrides['get_entities'] = lambda request: _ok(
        GetEntities.Response(entities=list(simulator.entities)))
    assert client.find_benchmark_entities('bench_run1_') == (
        'bench_run1_t0_obj_0', 'bench_run1_t0_target_0')
    assert client.find_benchmark_entities() == (
        'bench_run1_t0_obj_0', 'bench_run1_t0_target_0', 'bench_run2_t0_target_0')


def test_entity_filters_escape_regular_expression_metacharacters() -> None:
    assert posix_ere_literal('bench_a.b+c(1)/[x]{2}|y^$?*\\') == (
        'bench_a\\.b\\+c\\(1\\)/\\[x\\]\\{2\\}\\|y\\^\\$\\?\\*\\\\')
    pattern = re.compile(f'^{posix_ere_literal("bench_a.b")}$')
    assert pattern.search('bench_a.b') and not pattern.search('bench_aXb')


@pytest.mark.parametrize('name', ['', 'bench_', ROBOT, 'calibration_floor', 'xbench_run'])
def test_mutations_refuse_names_outside_the_benchmark_namespace(simulator, name) -> None:
    client, _clients = make_client(simulator)

    for operation in (lambda: client.spawn_entity(spec(name)),
                      lambda: client.delete_entity(name)):
        with pytest.raises(SceneControlError) as error:
            operation()
        assert error.value.failure is SceneControlFailure.UNOWNED_ENTITY
    assert simulator.calls == []
    assert set(simulator.entities) == set(WORLD_ENTITIES)


@pytest.mark.parametrize('prefix', ['', 'r100_0001', 'calibration', 'xbench_'])
def test_discovery_refuses_prefixes_outside_the_benchmark_namespace(simulator, prefix) -> None:
    client, _clients = make_client(simulator)

    with pytest.raises(SceneControlError) as error:
        client.find_benchmark_entities(prefix)

    assert error.value.failure is SceneControlFailure.UNOWNED_ENTITY
    assert simulator.calls == []


def test_a_canonical_name_outside_the_benchmark_namespace_is_rejected(simulator) -> None:
    simulator.overrides['spawn_entity'] = lambda request: _ok(
        SpawnEntity.Response(entity_name='robot_copy'))
    client, _clients = make_client(simulator)

    with pytest.raises(SceneControlError) as error:
        client.spawn_entity(spec('bench_run_t0_obj_0'))

    assert error.value.failure is SceneControlFailure.UNOWNED_ENTITY


def test_spawn_spec_takes_its_height_from_the_catalog_resource() -> None:
    from ridgeback_autonomy.benchmarking.asset_catalog import CatalogResource

    resource = CatalogResource(
        logical_name='g1', declared='models/g1.usda', uri='/abs/models/g1.usda',
        resource_format='usd', spawn_z_m=0.012)

    assert SpawnSpec.for_resource(resource, 'bench_run_t0_target_0', 2, -1, 3.0) == SpawnSpec(
        name='bench_run_t0_target_0', uri='/abs/models/g1.usda', resource_format='usd',
        x=2.0, y=-1.0, z=0.012, yaw=3.0)


def test_common_modules_import_with_simulator_packages_unavailable() -> None:
    blocked = repr(SIMULATOR_PACKAGES)
    subprocess.run([sys.executable, '-c', f'''
import importlib.abc, sys
BLOCKED = set({blocked})
class BlockSimulators(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in BLOCKED:
            raise ImportError('simulator package requested: ' + fullname)
sys.meta_path.insert(0, BlockSimulators())
import ridgeback_autonomy.benchmarking.asset_catalog
import ridgeback_autonomy.benchmarking.scene_control
import ridgeback_autonomy.benchmarking.simulation
loaded = sorted(name for name in sys.modules if name.split('.')[0] in BLOCKED)
assert not loaded, loaded
'''], check=True)


def test_common_modules_never_reach_a_simulator_or_its_command_line() -> None:
    forbidden = ('subprocess', *SIMULATOR_PACKAGES)
    violations = []
    for module in COMMON_MODULES:
        tree = ast.parse((BENCHMARK_ROOT / module).read_text(encoding='utf-8'))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                names = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom):
                names = [node.module or '']
            else:
                continue
            violations.extend(
                f'{module}:{node.lineno} imports {name}'
                for name in names if name.split('.')[0] in forbidden)

    assert not violations, violations
