"""Isaac scene-control request validation, entity registry, and poses."""

from __future__ import annotations

import json
import math
import re

import pytest
from simulation_interfaces.msg import EntityFilters, EntityCategory, Result
from simulation_interfaces.srv import SetEntityState, SpawnEntity

from scene_provider import (
    FABRIC_AUDIT_UPDATES,
    FABRIC_GRACE_UPDATES,
    FABRIC_REPAIR_ATTEMPTS,
    InvalidResource,
    RobotPlacement,
    RobotState,
    SceneControlService,
    SceneEntities,
    SceneRequestError,
    SimFrame,
    SpawnPlan,
    compile_entity_filter,
    robot_entity_name,
    stamp_parts,
    validate_spawn_request,
    validate_state_request,
    world_twist,
    yaw_quaternion,
)


ROBOT = 'r100_0001/robot'
IDENTITY = (0.0, 0.0, 0.0, 1.0)


class FakeNativeScene:
    """Records stage operations; resources listed in ``invalid`` fail to load."""

    def __init__(self) -> None:
        self.prims: dict[str, tuple[str, tuple, tuple]] = {}
        self.invalid: set[str] = set()
        self.stuck: set[str] = set()
        self.deleted: list[str] = []
        self.fabric: set[str] = set()
        self.fabric_stuck = False
        self.fabric_queries: list[str] = []
        self.fabric_removals: list[list[str]] = []

    def spawn(self, prim_path, resource_path, position, orientation):
        assert prim_path not in self.prims
        if resource_path in self.invalid:
            raise InvalidResource(f'"{resource_path}" has no default prim')
        self.prims[prim_path] = (resource_path, tuple(position), tuple(orientation))

    def delete(self, prim_path):
        self.deleted.append(prim_path)
        if prim_path not in self.stuck:
            self.prims.pop(prim_path, None)
        return prim_path not in self.prims

    def pose(self, prim_path):
        if prim_path not in self.prims:
            raise RuntimeError(f'missing prim {prim_path}')
        _resource, position, orientation = self.prims[prim_path]
        return position, orientation


    def fabric_prims_under(self, prefix):
        self.fabric_queries.append(prefix)
        return [path for path in self.fabric if path == prefix or path.startswith(prefix + '/')]

    def remove_fabric_prims(self, paths):
        self.fabric_removals.append(list(paths))
        if self.fabric_stuck:
            return []
        self.fabric.difference_update(paths)
        return list(paths)


@pytest.fixture
def resource(tmp_path):
    path = tmp_path / 'thing.usda'
    path.write_text('#usda 1.0\n')
    return str(path)


def spawn_request(resource, name='bench_g1', **overrides) -> SpawnEntity.Request:
    request = SpawnEntity.Request()
    request.name = name
    request.uri = resource
    request.initial_pose.header.frame_id = 'world'
    request.initial_pose.pose.position.x = 2.0
    request.initial_pose.pose.position.y = -1.0
    request.initial_pose.pose.orientation.z = math.sin(0.4)
    request.initial_pose.pose.orientation.w = math.cos(0.4)
    for field, value in overrides.items():
        setattr(request, field, value)
    return request


def plan(name, resource, allow_renaming=False) -> SpawnPlan:
    return SpawnPlan(name, allow_renaming, resource, (1.0, 2.0, 0.0), IDENTITY)


def frame(sim_time=12.5) -> SimFrame:
    return SimFrame(sim_time, RobotState(1.5, -0.5, 0.07617, 0.3, 0.2, 0.1, 0.05))


def code_of(excinfo) -> int:
    return excinfo.value.code


# ---- request validation -----------------------------------------------------

def test_valid_request_becomes_a_normalized_plan(resource) -> None:
    request = spawn_request(resource)
    request.initial_pose.pose.orientation.z *= 2.0
    request.initial_pose.pose.orientation.w *= 2.0

    result = validate_spawn_request(request)

    assert result.name == 'bench_g1'
    assert result.resource_path == resource
    assert result.position == (2.0, -1.0, 0.0)
    assert result.orientation == pytest.approx((0.0, 0.0, math.sin(0.4), math.cos(0.4)))


def test_file_uri_and_empty_frame_are_accepted(resource) -> None:
    request = spawn_request('file://' + resource)
    request.initial_pose.header.frame_id = ''

    assert validate_spawn_request(request).resource_path == resource


@pytest.mark.parametrize('name', ['bench g1', '1bench', ROBOT, 'bench-g1', '/bench_g1'])
def test_names_must_be_usd_identifiers(resource, name) -> None:
    with pytest.raises(SceneRequestError) as excinfo:
        validate_spawn_request(spawn_request(resource, name=name))
    assert code_of(excinfo) == SpawnEntity.Response.NAME_INVALID


def test_empty_name_needs_renaming(resource) -> None:
    with pytest.raises(SceneRequestError) as excinfo:
        validate_spawn_request(spawn_request(resource, name=''))
    assert code_of(excinfo) == SpawnEntity.Response.NAME_INVALID
    assert validate_spawn_request(
        spawn_request(resource, name='', allow_renaming=True)).name == ''


@pytest.mark.parametrize(('overrides', 'code'), [
    ({'entity_namespace': 'r100_0001'}, SpawnEntity.Response.NAMESPACE_INVALID),
    ({'resource_string': '#usda 1.0'}, Result.RESULT_FEATURE_UNSUPPORTED),
    ({'uri': ''}, SpawnEntity.Response.NO_RESOURCE),
    ({'uri': 'https://example.com/g1.usda'}, SpawnEntity.Response.UNSUPPORTED_FORMAT),
    ({'uri': 'file://remote-host/g1.usda'}, SpawnEntity.Response.UNSUPPORTED_FORMAT),
    ({'uri': 'models/g1/g1.usda'}, SpawnEntity.Response.RESOURCE_PARSE_ERROR),
    ({'uri': '/nonexistent/g1.usda'}, SpawnEntity.Response.RESOURCE_PARSE_ERROR),
])
def test_unusable_resources_are_refused(resource, overrides, code) -> None:
    with pytest.raises(SceneRequestError) as excinfo:
        validate_spawn_request(spawn_request(resource, **overrides))
    assert code_of(excinfo) == code


def test_non_usd_files_are_an_unsupported_format(tmp_path) -> None:
    sdf = tmp_path / 'model.sdf'
    sdf.write_text('<sdf/>')
    with pytest.raises(SceneRequestError) as excinfo:
        validate_spawn_request(spawn_request(str(sdf)))
    assert code_of(excinfo) == SpawnEntity.Response.UNSUPPORTED_FORMAT


@pytest.mark.parametrize('mutate', [
    lambda r: setattr(r.initial_pose.header, 'frame_id', 'map'),
    lambda r: setattr(r.initial_pose.pose.position, 'x', math.nan),
    lambda r: setattr(r.initial_pose.pose.orientation, 'w', math.inf),
    lambda r: (setattr(r.initial_pose.pose.orientation, 'z', 0.0),
               setattr(r.initial_pose.pose.orientation, 'w', 0.0)),
])
def test_invalid_poses_are_refused(resource, mutate) -> None:
    request = spawn_request(resource)
    mutate(request)
    with pytest.raises(SceneRequestError) as excinfo:
        validate_spawn_request(request)
    assert code_of(excinfo) == SpawnEntity.Response.INVALID_POSE


def test_only_name_filters_are_supported() -> None:
    assert compile_entity_filter(EntityFilters()) is None
    assert compile_entity_filter(EntityFilters(filter='^bench_.*$')).pattern == '^bench_.*$'

    filters = EntityFilters()
    filters.categories = [EntityCategory(category=EntityCategory.CATEGORY_ROBOT)]
    with pytest.raises(SceneRequestError) as excinfo:
        compile_entity_filter(filters)
    assert code_of(excinfo) == Result.RESULT_FEATURE_UNSUPPORTED

    filters = EntityFilters()
    filters.tags.tags = ['target']
    with pytest.raises(SceneRequestError) as excinfo:
        compile_entity_filter(filters)
    assert code_of(excinfo) == Result.RESULT_FEATURE_UNSUPPORTED

    filters = EntityFilters()
    filters.bounds.type = filters.bounds.TYPE_SPHERE
    with pytest.raises(SceneRequestError) as excinfo:
        compile_entity_filter(filters)
    assert code_of(excinfo) == Result.RESULT_FEATURE_UNSUPPORTED

    with pytest.raises(SceneRequestError) as excinfo:
        compile_entity_filter(EntityFilters(filter='bench_('))
    assert code_of(excinfo) == Result.RESULT_OPERATION_FAILED


# ---- registry and ownership ---------------------------------------------------

def test_spawned_entities_live_only_under_the_scene_scope(resource) -> None:
    native = FakeNativeScene()
    entities = SceneEntities(native, ROBOT)

    assert entities.spawn(plan('bench_g1', resource)) == 'bench_g1'

    assert list(native.prims) == ['/scene_entities/bench_g1']
    assert entities.names() == [ROBOT, 'bench_g1']


def test_duplicate_names_are_refused_without_touching_the_stage(resource) -> None:
    native = FakeNativeScene()
    entities = SceneEntities(native, ROBOT)
    entities.spawn(plan('bench_g1', resource))

    with pytest.raises(SceneRequestError) as excinfo:
        entities.spawn(plan('bench_g1', resource))

    assert code_of(excinfo) == SpawnEntity.Response.NAME_NOT_UNIQUE
    assert list(native.prims) == ['/scene_entities/bench_g1']


def test_renaming_returns_the_name_actually_created(resource) -> None:
    native = FakeNativeScene()
    entities = SceneEntities(native, ROBOT)
    entities.spawn(plan('bench_g1', resource))

    assert entities.spawn(plan('bench_g1', resource, allow_renaming=True)) == 'bench_g1_0'
    assert entities.spawn(plan('bench_g1', resource, allow_renaming=True)) == 'bench_g1_1'
    assert entities.spawn(plan('', resource, allow_renaming=True)) == 'entity_0'
    assert set(native.prims) == {f'/scene_entities/{name}' for name in (
        'bench_g1', 'bench_g1_0', 'bench_g1_1', 'entity_0')}


def test_unloadable_resource_registers_nothing(resource) -> None:
    native = FakeNativeScene()
    native.invalid.add(resource)
    entities = SceneEntities(native, ROBOT)

    with pytest.raises(SceneRequestError) as excinfo:
        entities.spawn(plan('bench_g1', resource))

    assert code_of(excinfo) == SpawnEntity.Response.RESOURCE_PARSE_ERROR
    assert entities.names() == [ROBOT]
    native.invalid.clear()
    assert entities.spawn(plan('bench_g1', resource)) == 'bench_g1'


def test_robot_cannot_be_deleted(resource) -> None:
    native = FakeNativeScene()
    entities = SceneEntities(native, ROBOT)

    with pytest.raises(SceneRequestError) as excinfo:
        entities.delete(ROBOT)

    assert code_of(excinfo) == Result.RESULT_OPERATION_FAILED
    assert entities.names() == [ROBOT]
    assert native.deleted == []


@pytest.mark.parametrize('name', [
    'bench_never_spawned', '/ridgeback', 'g1_distance_calibration',
    '/scene_entities', '/scene_entities/bench_g1/model',
])
def test_unregistered_names_never_reach_the_stage(resource, name) -> None:
    native = FakeNativeScene()
    entities = SceneEntities(native, ROBOT)
    entities.spawn(plan('bench_g1', resource))

    with pytest.raises(SceneRequestError) as excinfo:
        entities.delete(name)

    assert code_of(excinfo) == Result.RESULT_NOT_FOUND
    assert native.deleted == []
    assert '/scene_entities/bench_g1' in native.prims


def test_delete_unregisters_only_after_the_prim_is_gone(resource) -> None:
    native = FakeNativeScene()
    entities = SceneEntities(native, ROBOT)
    entities.spawn(plan('bench_g1', resource))
    native.stuck.add('/scene_entities/bench_g1')

    with pytest.raises(SceneRequestError) as excinfo:
        entities.delete('bench_g1')
    assert code_of(excinfo) == Result.RESULT_OPERATION_FAILED
    assert 'bench_g1' in entities.names()

    native.stuck.clear()
    entities.delete('bench_g1')
    assert entities.names() == [ROBOT]
    with pytest.raises(SceneRequestError) as excinfo:
        entities.delete('bench_g1')
    assert code_of(excinfo) == Result.RESULT_NOT_FOUND


def test_filters_select_by_search_so_anchors_select_whole_names(resource) -> None:
    entities = SceneEntities(FakeNativeScene(), ROBOT)
    for name in ('bench_a', 'bench_b', 'other_bench_c'):
        entities.spawn(plan(name, resource))

    assert entities.select(None) == [ROBOT, 'bench_a', 'bench_b', 'other_bench_c']
    assert entities.select(re.compile('^bench_.*$')) == ['bench_a', 'bench_b']
    assert entities.select(re.compile('^bench_a$')) == ['bench_a']
    assert entities.select(re.compile('bench_')) == ['bench_a', 'bench_b', 'other_bench_c']


# ---- states and poses -------------------------------------------------------------

def test_states_come_from_one_update(resource) -> None:
    native = FakeNativeScene()
    entities = SceneEntities(native, ROBOT)
    entities.spawn(SpawnPlan('bench_g1', False, resource, (3.0, 1.0, 0.0),
                             yaw_quaternion(1.2)))

    poses = {pose.name: pose for pose in entities.states(None, frame())}

    robot = poses[ROBOT]
    assert robot.position == (1.5, -0.5, 0.07617)
    assert robot.orientation == pytest.approx(yaw_quaternion(0.3))
    assert robot.linear == (0.2, 0.1, 0.0)
    assert robot.angular == (0.0, 0.0, 0.05)
    target = poses['bench_g1']
    assert target.position == (3.0, 1.0, 0.0)
    assert target.orientation == pytest.approx(yaw_quaternion(1.2))
    assert target.linear == target.angular == (0.0, 0.0, 0.0)


def test_world_twist_rotates_the_body_twist_by_yaw() -> None:
    vx, vy, wz = world_twist((1.0, 0.5, 0.2), math.pi / 2)
    assert (vx, vy, wz) == pytest.approx((-0.5, 1.0, 0.2))
    assert world_twist((0.3, -0.1, 0.0), 0.0) == pytest.approx((0.3, -0.1, 0.0))


def test_stamps_split_sim_time_like_the_runner_clock() -> None:
    for sim_time in (0.0, 1.5, 12.025, 3601.999999):
        sec, nanosec = stamp_parts(sim_time)
        assert sec == int(sim_time)
        assert nanosec == int((sim_time - int(sim_time)) * 1e9)


def test_robot_entity_name_matches_the_gazebo_convention() -> None:
    assert robot_entity_name('r100_0001') == ROBOT
    assert robot_entity_name('/r100_0001/') == ROBOT


# ---- placing the robot ------------------------------------------------------

def state_request(name=ROBOT, x=2.35, y=0.0, z=0.07617, yaw=0.0) -> SetEntityState.Request:
    request = SetEntityState.Request()
    request.entity = name
    request.state.header.frame_id = 'world'
    request.state.pose.position.x, request.state.pose.position.y = x, y
    request.state.pose.position.z = z
    request.state.pose.orientation.z = math.sin(0.5 * yaw)
    request.state.pose.orientation.w = math.cos(0.5 * yaw)
    return request


def test_a_level_pose_at_rest_becomes_a_placement() -> None:
    placement = validate_state_request(state_request(yaw=0.4))

    assert placement == RobotPlacement(ROBOT, 2.35, 0.0, 0.07617, pytest.approx(0.4))


@pytest.mark.parametrize('mutate, code', [
    (lambda r: setattr(r.state.twist.linear, 'x', 0.1), Result.RESULT_OPERATION_FAILED),
    (lambda r: setattr(r.state.twist.angular, 'z', 0.1), Result.RESULT_OPERATION_FAILED),
    (lambda r: setattr(r.state.acceleration.linear, 'y', 0.1), Result.RESULT_OPERATION_FAILED),
    (lambda r: setattr(r.state.pose.orientation, 'x', 0.05), SetEntityState.Response.INVALID_POSE),
    (lambda r: setattr(r.state.header, 'frame_id', 'odom'), SetEntityState.Response.INVALID_POSE),
    (lambda r: setattr(r.state.pose.orientation, 'w', 0.0), SetEntityState.Response.INVALID_POSE),
    (lambda r: setattr(r.state.pose.position, 'x', math.nan), SetEntityState.Response.INVALID_POSE),
])
def test_motion_tilt_and_foreign_frames_are_refused(mutate, code) -> None:
    request = state_request()
    mutate(request)

    with pytest.raises(SceneRequestError) as excinfo:
        validate_state_request(request)

    assert code_of(excinfo) == code


def test_only_the_robot_is_placed_and_only_at_its_own_height(resource) -> None:
    moves = []
    entities = SceneEntities(FakeNativeScene(), ROBOT,
                             move_robot=lambda *pose: moves.append(pose))
    entities.spawn(plan('bench_g1', resource))

    entities.place_robot(RobotPlacement(ROBOT, 2.35, 0.0, 0.07617, 0.0), frame())
    assert moves == [(2.35, 0.0, 0.0)]
    for placement, code in (
            (RobotPlacement('bench_g1', 1.0, 0.0, 0.0, 0.0), Result.RESULT_OPERATION_FAILED),
            (RobotPlacement('wall_north', 1.0, 0.0, 0.0, 0.0), Result.RESULT_NOT_FOUND),
            (RobotPlacement(ROBOT, 1.0, 0.0, 0.2, 0.0), SetEntityState.Response.INVALID_POSE)):
        with pytest.raises(SceneRequestError) as excinfo:
            entities.place_robot(placement, frame())
        assert code_of(excinfo) == code
    assert moves == [(2.35, 0.0, 0.0)]


def test_a_runner_without_a_mover_cannot_place_the_robot() -> None:
    entities = SceneEntities(FakeNativeScene(), ROBOT)

    assert not entities.moves_robot
    with pytest.raises(SceneRequestError) as excinfo:
        entities.place_robot(RobotPlacement(ROBOT, 1.0, 0.0, 0.07617, 0.0), frame())
    assert code_of(excinfo) == Result.RESULT_FEATURE_UNSUPPORTED


# ---- Fabric audit ------------------------------------------------------------

def fabric_events(capfd):
    return [json.loads(line.split(' ', 1)[1])
            for line in capfd.readouterr().out.splitlines()
            if line.startswith('SCENE_FABRIC_RESIDUE ')]


def test_clean_delete_has_a_bounded_audit_window(resource, capfd):
    native = FakeNativeScene()
    entities = SceneEntities(native, ROBOT)
    entities.spawn(plan('bench_a', resource))
    entities.delete('bench_a')
    assert entities.names() == [ROBOT]
    for _ in range(FABRIC_GRACE_UPDATES + FABRIC_AUDIT_UPDATES + 1):
        entities.audit(12.5)

    # One batch sweep on the following update, then one scan per audit update.
    assert len(native.fabric_queries) == 1 + FABRIC_AUDIT_UPDATES + 1
    assert native.fabric_removals == []
    assert entities.names() == [ROBOT]
    assert fabric_events(capfd) == []


def test_late_residue_is_repaired_and_verified_on_the_next_update(resource, capfd):
    native = FakeNativeScene()
    entities = SceneEntities(native, ROBOT)
    entities.spawn(plan('bench_a', resource))
    entities.delete('bench_a')
    for _ in range(FABRIC_GRACE_UPDATES + 1):
        entities.audit(12.5)
    leaf = '/scene_entities/bench_a/model/collision'
    native.fabric.add(leaf)
    entities.audit(12.5)
    entities.audit(12.6)

    events = fabric_events(capfd)
    assert [event['event'] for event in events] == ['found', 'repaired']
    assert events[0]['update'] == FABRIC_GRACE_UPDATES + 2
    assert events[1]['sim_time'] == 12.6
    assert events[0]['entity'] == 'bench_a'
    assert events[0]['prims'] == [leaf]
    assert native.fabric_removals == [[leaf]]
    assert entities.names() == [ROBOT]


def test_persistent_residue_is_listed_with_last_pose_and_can_be_retried(resource, capfd):
    native = FakeNativeScene()
    entities = SceneEntities(native, ROBOT)
    entities.spawn(plan('bench_a', resource))
    expected = entities.states(re.compile('^bench_a$'), frame())
    entities.delete('bench_a')
    native.fabric = {'/scene_entities/bench_a/model'}
    native.fabric_stuck = True
    for _ in range(FABRIC_GRACE_UPDATES + FABRIC_REPAIR_ATTEMPTS):
        entities.audit(12.5)

    assert len(native.fabric_removals) == FABRIC_REPAIR_ATTEMPTS
    assert entities.names() == [ROBOT, 'bench_a']
    assert entities.states(re.compile('^bench_a$'), frame()) == expected
    events = fabric_events(capfd)
    assert [event['event'] for event in events] == ['found', 'persistent']
    assert events[-1]['attempt'] == FABRIC_REPAIR_ATTEMPTS

    native.fabric_stuck = False
    entities.delete('bench_a')
    assert entities.names() == [ROBOT, 'bench_a']
    entities.audit(12.6)
    assert entities.names() == [ROBOT]
    assert [event['event'] for event in fabric_events(capfd)] == ['found', 'repaired']
    assert native.deleted == ['/scene_entities/bench_a']


def test_sweep_detects_an_orphan_with_no_registry_entry(capfd):
    native = FakeNativeScene()
    entities = SceneEntities(native, ROBOT)
    native.fabric = {'/scene_entities', '/scene_entities/bench_x/model/leaf',
                     '/scene_entities_other/bench_y/model'}
    native.fabric_stuck = True
    entities.sweep()
    assert entities.select(re.compile('^bench_')) == ['bench_x']
    entities.sweep()
    assert len(native.fabric_removals) == 1
    for _ in range(FABRIC_REPAIR_ATTEMPTS):
        entities.audit(12.5)

    assert entities.names() == [ROBOT, 'bench_x']
    events = fabric_events(capfd)
    assert [event['event'] for event in events] == ['found', 'persistent']
    assert events[0]['source'] == 'sweep'


def test_sweep_ignores_live_paths_and_removal_during_grace(resource, capfd):
    native = FakeNativeScene()
    entities = SceneEntities(native, ROBOT)
    entities.spawn(plan('bench_a', resource))
    entities.spawn(plan('bench_ab', resource))
    entities.delete('bench_a')
    native.fabric = {'/scene_entities/bench_a/model', '/scene_entities/bench_ab/model'}
    entities.audit(12.5)
    entities.sweep()

    # A query reports the pending subtree until Fabric clears. It is
    # still within normal propagation time, so no repair or finding runs.
    assert entities.names() == [ROBOT, 'bench_ab', 'bench_a']
    assert native.fabric_removals == []
    native.fabric.remove('/scene_entities/bench_a/model')
    entities.audit(12.6)
    assert entities.names() == [ROBOT, 'bench_ab']
    assert fabric_events(capfd) == []


def test_respawn_during_audit_never_repairs_a_live_entity(resource):
    native = FakeNativeScene()
    entities = SceneEntities(native, ROBOT)
    entities.spawn(plan('bench_a', resource))
    entities.delete('bench_a')
    entities.spawn(plan('bench_a', resource))
    native.fabric.add('/scene_entities/bench_a/model')
    for _ in range(FABRIC_GRACE_UPDATES + 2):
        entities.audit(12.5)
    entities.sweep()
    assert native.fabric_removals == []


def test_sweep_keeps_last_pose_after_audit_window(resource):
    native = FakeNativeScene()
    entities = SceneEntities(native, ROBOT)
    entities.spawn(plan('bench_a', resource))
    expected = entities.states(re.compile('^bench_a$'), frame())
    entities.delete('bench_a')
    for _ in range(FABRIC_GRACE_UPDATES + FABRIC_AUDIT_UPDATES + 1):
        entities.audit(12.5)
    native.fabric.add('/scene_entities/bench_a/model')
    entities.sweep()
    assert entities.states(re.compile('^bench_a$'), frame()) == expected


def test_process_pending_audits_before_draining_once(monkeypatch):
    order = []
    service = object.__new__(SceneControlService)
    service._entities = SceneEntities(FakeNativeScene(), ROBOT)
    monkeypatch.setattr(service._entities, 'audit', lambda sim_time: order.append(sim_time))

    class Queue:
        def drain(self, update):
            order.append(update)
            return 0

    service._queue = Queue()
    update = frame()
    assert service.process_pending(update) == 0
    assert order == [update.sim_time, update]


def test_already_absent_usd_prim_can_be_deleted_and_audited(resource):
    native = FakeNativeScene()
    entities = SceneEntities(native, ROBOT)
    entities.spawn(plan('bench_a', resource))
    path = entities.prim_path('bench_a')
    del native.prims[path]
    leaf = f'{path}/model/leaf'
    native.fabric.add(leaf)

    entities.delete('bench_a')
    for _ in range(FABRIC_GRACE_UPDATES):
        entities.audit(12.5)
    assert native.fabric_removals == [[leaf]]
