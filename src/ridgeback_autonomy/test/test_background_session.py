"""Run-scoped background props against the in-memory standard-services provider."""

from __future__ import annotations

import contextlib
from pathlib import Path
import random
from types import SimpleNamespace

import pytest
import yaml

from ridgeback_autonomy.benchmarking.background import (
    WALL,
    Anchor,
    BackgroundLayout,
    BackgroundProp,
    Placement,
    Room,
    SceneBackground,
    ScenePlacement,
    resolve_scene_background,
)
from ridgeback_autonomy.benchmarking.background_session import (
    background_entity_prefix,
    background_scene,
)
from ridgeback_autonomy.benchmarking.scenarios import ObjectSpec, RobotSpec, Scene
from ridgeback_autonomy.benchmarking.scene_control import (
    ROLLBACK_INTERRUPT_BUDGET_SEC,
    SpawnSpec,
)
from ridgeback_autonomy.benchmarking.scene_state import SceneStateMismatch
from ridgeback_autonomy.benchmarking.target_distance_benchmark_runner_node import (
    TargetDistanceBenchmarkRunner,
)
from test_scene_control import (
    ROBOT,
    WORLD_ENTITIES,
    FakeClock,
    FakeSimulator,
    make_client,
)


CONFIG_DIR = Path(__file__).resolve().parents[1] / 'config'
PROPS = ('desk', 'chair', 'cart', 'plant')
PREFIX = background_entity_prefix('run')


def _background(scene_drops, dx=None) -> SceneBackground:
    layout = BackgroundLayout(
        path='lab.yaml', sha256='l', world='target_distance_calibration',
        room=Room(-5.0, 5.0, -5.0, 5.0),
        placement=Placement(backdrop_gap_m=1.0, dx_step_m=0.05),
        props=tuple(BackgroundProp(prop_id, 'thing', float(i), 3.0, 0.0)
                    for i, prop_id in enumerate(PROPS)),
    )
    return SceneBackground(
        name='lab', layout=layout,
        drops=SimpleNamespace(path='drops.yaml', sha256='d', rules={}),
        placements={
            scene_id: ScenePlacement(
                (dx or {}).get(scene_id, 0.0), Anchor(0, WALL, 1.0), tuple(dropped))
            for scene_id, dropped in scene_drops.items()},
    )


RESOURCES = {'thing': SimpleNamespace(uri='/m.sdf', resource_format='sdf', spawn_z_m=0.05)}


@pytest.fixture
def simulator() -> FakeSimulator:
    return FakeSimulator(FakeClock())


def _props(simulator: FakeSimulator) -> set[str]:
    return {name.removeprefix(PREFIX) for name in simulator.entities if name.startswith(PREFIX)}


def test_the_layout_spawns_once_in_order_on_the_floor_and_leaves_with_the_run(simulator) -> None:
    client, _ = make_client(simulator)

    with background_scene(client, _background({}), RESOURCES, 'run',
                          clock=simulator.clock) as session:
        assert _props(simulator) == set(PROPS)
        assert session.spawn_sec is not None
        spawned = [request.name for request in simulator.calls_to('spawn_entity')]
        assert spawned == [f'{PREFIX}{prop_id}' for prop_id in PROPS]
        assert {simulator.entities[name][2] for name in spawned} == {0.05}

    assert set(simulator.entities) == set(WORLD_ENTITIES)
    deleted = [request.entity for request in simulator.calls_to('delete_entity')]
    assert deleted == [f'{PREFIX}{prop_id}' for prop_id in reversed(PROPS)]


def test_each_scene_sees_the_layout_minus_its_own_drops_whatever_ran_before(simulator) -> None:
    client, _ = make_client(simulator)
    drops = {'a': ('chair',), 'b': ('cart', 'plant'), 'c': ()}

    with background_scene(client, _background(drops), RESOURCES, 'run',
                          clock=simulator.clock) as session:
        first = session.adjust('a')
        assert _props(simulator) == {'desk', 'cart', 'plant'}
        assert (first.removed, first.restored) == (('chair',), ())

        second = session.adjust('b')
        assert _props(simulator) == {'desk', 'chair'}
        assert (second.removed, second.restored) == (('cart', 'plant'), ('chair',))

        repeat = session.adjust('b')
        assert (repeat.removed, repeat.restored) == ((), ())

        last = session.adjust('c')
        assert _props(simulator) == set(PROPS)
        assert last.restored == ('cart', 'plant')
        assert last.as_record()['dropped'] == []


def test_an_adjustment_starts_from_what_is_present_not_what_was_intended(simulator) -> None:
    client, _ = make_client(simulator)
    drops = {'a': ('chair',), 'b': ()}

    with background_scene(client, _background(drops), RESOURCES, 'run',
                          clock=simulator.clock) as session:
        # A prop that vanished behind the session's back -- a failed earlier
        # adjustment, say -- is simply restored by the next one.
        del simulator.entities[f'{PREFIX}desk']
        adjustment = session.adjust('b')

        assert adjustment.restored == ('desk',)
        assert _props(simulator) == set(PROPS)


def test_any_scene_order_shows_each_scene_the_layout_minus_only_its_own_drops(
        simulator) -> None:
    # The packaged lab background over the full scenario file, visited in
    # random orders with repeats: nothing an earlier scene dropped may stay
    # missing from a scene that keeps it.
    scenario = CONFIG_DIR / 'benchmark_scenarios_full.yaml'
    scene_ids = [scene['id'] for scene in yaml.safe_load(scenario.read_text())['scenes']]
    background = resolve_scene_background('lab', str(CONFIG_DIR), str(scenario), scene_ids)
    resources = {model: RESOURCES['thing'] for model in background.layout.models}
    every_prop = {prop.id for prop in background.layout.props}
    present = lambda: {name.removeprefix(PREFIX) for name in simulator.entities  # noqa: E731
                       if name.startswith(PREFIX)}
    order = random.Random(20261101)

    client, _ = make_client(simulator)
    with background_scene(client, background, resources, 'run',
                          clock=simulator.clock) as session:
        for _ in range(6):
            sequence = order.choices(scene_ids, k=len(scene_ids))
            for scene_id in sequence:
                session.adjust(scene_id)
                assert present() == every_prop - set(background.scene_drops[scene_id]), scene_id


def test_a_failed_layout_spawn_removes_what_it_spawned_and_propagates(simulator) -> None:
    client, _ = make_client(simulator)
    simulator.entities[f'{PREFIX}cart'] = (0.0, 0.0, 0.0, 0.0)

    with pytest.raises(Exception, match='not unique'):
        with background_scene(client, _background({}), RESOURCES, 'run',
                              clock=simulator.clock):
            pytest.fail('the body must not run')

    assert _props(simulator) == {'cart'}


def test_an_interrupt_removes_the_background_inside_the_rollback_budget(simulator) -> None:
    client, _ = make_client(simulator)

    with pytest.raises(KeyboardInterrupt):
        with background_scene(client, _background({}), RESOURCES, 'run',
                              clock=simulator.clock):
            raise KeyboardInterrupt

    assert _props(simulator) == set()
    assert simulator.clock.now < ROLLBACK_INTERRUPT_BUDGET_SEC + 1.0


# --- the runner around the session ------------------------------------------

def test_a_trial_adjusts_the_background_before_spawning_its_scene() -> None:
    events = []

    class SceneControl:
        def scene(self, specs):
            events.append('scene')
            raise RuntimeError('stop after the order is known')

    logger = SimpleNamespace(error=lambda message: events.append('error'), info=print)
    runner = SimpleNamespace(
        run_label='run',
        scene_control=SceneControl(),
        scene_spawn_specs=lambda _scene, _prefix: ([], []),
        adjust_background=lambda trial: events.append(('adjust', trial['scene'].id)),
        get_logger=lambda: logger,
    )

    result = TargetDistanceBenchmarkRunner.run_trial(
        runner, {'trial_id': 's1', 'scene': SimpleNamespace(id='s1')})

    assert result is None
    assert events == [('adjust', 's1'), 'scene', 'error']


def test_a_failed_adjustment_skips_the_trial_without_spawning_it() -> None:
    events = []
    logger = SimpleNamespace(error=lambda message: events.append(message))
    runner = SimpleNamespace(
        run_label='run',
        scene_control=SimpleNamespace(scene=lambda specs: events.append('scene')),
        scene_spawn_specs=lambda _scene, _prefix: ([], []),
        adjust_background=lambda trial: (_ for _ in ()).throw(RuntimeError('delete stuck')),
        get_logger=lambda: logger,
    )

    result = TargetDistanceBenchmarkRunner.run_trial(
        runner, {'trial_id': 's1', 'scene': SimpleNamespace(id='s1')})

    assert result is None
    assert events == ['s1 failed: delete stuck']


def test_the_runner_records_each_adjustment_against_its_trial(simulator) -> None:
    client, _ = make_client(simulator, robot_entity=ROBOT)
    background = _background({'a': ('chair',)}, dx={'a': 1.5})
    logged = []
    with background_scene(client, background, RESOURCES, 'run',
                          clock=simulator.clock) as session:
        runner = SimpleNamespace(
            scene_background=background,
            background_session=session,
            background_adjustments=[],
            scene_control=client,
            get_logger=lambda: SimpleNamespace(info=logged.append),
        )
        TargetDistanceBenchmarkRunner.adjust_background(
            runner, {'trial_id': 'a_rep1', 'scene': SimpleNamespace(id='a')})

    record = runner.background_adjustments[0]
    assert (record['trial_id'], record['dx_m'], record['removed']) == ('a_rep1', 1.5, ['chair'])
    assert record['anchor'] == {'target_index': 0, 'backdrop': WALL, 'gap_m': 1.0}
    assert record['robot']['commanded'] == {'x': 1.5, 'y': 0.0, 'yaw': 0.0}
    assert record['robot']['measured']['x'] == pytest.approx(1.5)
    assert {'remove_sec', 'move_sec', 'restore_sec'} <= set(record)
    assert logged and 'dx 1.50' in logged[0] and 'removed chair' in logged[0]


def test_leftover_benchmark_entities_refuse_the_run() -> None:
    runner = SimpleNamespace(scene_control=SimpleNamespace(
        find_benchmark_entities=lambda: ('bench_old_bg_desk', 'bench_old_t_target_0')))

    with pytest.raises(RuntimeError, match='2 benchmark entities are already'):
        TargetDistanceBenchmarkRunner.refuse_leftover_entities(runner)


# --- the trial order and the placed robot -----------------------------------------

def _mutations(simulator) -> list[str]:
    """The scene-changing calls in order: spawn and delete by name, moves by robot x."""

    calls = []
    for suffix, request in simulator.calls:
        if suffix == 'spawn_entity':
            calls.append(f'spawn {request.name.removeprefix(PREFIX)}')
        elif suffix == 'delete_entity':
            calls.append(f'delete {request.entity.removeprefix(PREFIX)}')
        elif suffix == 'set_entity_state':
            calls.append(f'move {request.state.pose.position.x:g}')
    return calls


def test_a_trial_removes_moves_restores_then_spawns_objects_and_targets(simulator) -> None:
    client, _ = make_client(simulator, robot_entity=ROBOT)
    background = _background({'a': ('cart',), 'b': ('chair',)}, dx={'a': 0.0, 'b': 2.0})
    # Where the robot stood whenever a prop was spawned.
    robot_at_spawn = {}

    def spawn(request):
        robot_at_spawn[request.name.removeprefix(PREFIX)] = simulator.entities[ROBOT][0]
        return FakeSimulator.spawn_entity(simulator, request)
    simulator.overrides['spawn_entity'] = spawn

    with background_scene(client, background, RESOURCES, 'run',
                          clock=simulator.clock) as session:
        session.adjust('a', lambda: client.move_robot(0.0, 0.0, 0.0))
        simulator.calls.clear()
        session.adjust('b', lambda: client.move_robot(2.0, 0.0, 0.0))
        with client.scene([
                SpawnSpec('bench_run_b_obj_0', '/m.sdf', 'sdf', 5.0, 1.0, 0.0, 0.0),
                SpawnSpec('bench_run_b_target_0', '/m.sdf', 'sdf', 6.0, 0.0, 0.0, 3.1)]):
            order = _mutations(simulator)

    assert order == [
        'delete chair', 'move 2', 'spawn cart',
        'spawn bench_run_b_obj_0', 'spawn bench_run_b_target_0']
    # The prop came back only once the robot had left for its new placement.
    assert robot_at_spawn['cart'] == pytest.approx(2.0)


def test_the_control_arm_moves_the_robot_and_spawns_no_prop(simulator) -> None:
    client, _ = make_client(simulator, robot_entity=ROBOT)
    background = _background({'a': ('chair',)}, dx={'a': 3.0})
    homes = []

    with background_scene(client, background, {}, 'run', props=False,
                          after_removal=homes.append, clock=simulator.clock) as session:
        adjustment = session.adjust('a', lambda: client.move_robot(3.0, 0.0, 0.0))
        assert simulator.entities[ROBOT][0] == pytest.approx(3.0)

    assert _props(simulator) == set()
    assert simulator.calls_to('spawn_entity') == []
    assert simulator.calls_to('delete_entity') == []
    assert (adjustment.dropped, adjustment.removed, adjustment.restored) == ((), (), ())
    assert homes == [None]


def test_the_robot_returns_home_after_the_props_and_within_an_interrupt_budget(
        simulator) -> None:
    client, _ = make_client(simulator, robot_entity=ROBOT)
    calls = []

    def home(deadline):
        calls.append((len(_props(simulator)), deadline))
        client.move_robot(0.0, 0.0, 0.0)

    with background_scene(client, _background({}), RESOURCES, 'run',
                          after_removal=home, clock=simulator.clock):
        client.move_robot(2.0, 0.0, 0.0)
    assert calls == [(0, None)]
    assert simulator.entities[ROBOT][0] == pytest.approx(0.0)

    calls.clear()
    with pytest.raises(KeyboardInterrupt):
        with background_scene(client, _background({}), RESOURCES, 'run',
                              after_removal=home, clock=simulator.clock):
            raise KeyboardInterrupt
    [(props_left, deadline)] = calls
    assert props_left == 0 and deadline is not None


def test_a_spent_interrupt_budget_leaves_the_robot_where_it_is(simulator) -> None:
    client, _ = make_client(simulator, robot_entity=ROBOT)
    calls = []

    def slow_delete(request):
        simulator.clock.advance(ROLLBACK_INTERRUPT_BUDGET_SEC)
        return FakeSimulator.delete_entity(simulator, request)
    simulator.overrides['delete_entity'] = slow_delete

    with pytest.raises(KeyboardInterrupt):
        with background_scene(client, _background({}), RESOURCES, 'run',
                              after_removal=calls.append, clock=simulator.clock):
            raise KeyboardInterrupt

    assert calls == []


def _scene() -> Scene:
    return Scene(id='a', robots=(RobotSpec(x=4.0, y=0.5, yaw=3.1),),
                 objects=(ObjectSpec(model='table', x=3.0, y=-1.0, yaw=0.2),),
                 repeats_override=None)


def test_spawn_specs_shift_every_scene_entity_by_the_placement() -> None:
    runner = SimpleNamespace(
        scene_background=_background({'a': ()}, dx={'a': 2.5}),
        asset_selection=SimpleNamespace(
            objects={'table': RESOURCES['thing']}, target=RESOURCES['thing']),
    )

    specs, _targets = TargetDistanceBenchmarkRunner.scene_spawn_specs(runner, _scene(), 'p')

    assert [(spec.x, spec.y, spec.yaw) for spec in specs] == [(5.5, -1.0, 0.2), (6.5, 0.5, 3.1)]


def _verifying_runner(client, background, session) -> SimpleNamespace:
    return SimpleNamespace(
        scene_background=background, background_session=session, background_props=True,
        run_label='run', robot_model_name=ROBOT, scene_control=client,
        background_adjustments=[{'trial_id': 'a'}],
    )


def test_the_state_check_passes_the_commanded_room_and_fingerprints_it(simulator) -> None:
    client, _ = make_client(simulator, robot_entity=ROBOT)
    background = _background({'a': ('chair',)}, dx={'a': 1.0})
    specs = [SpawnSpec('bench_run_a_obj_0', '/m.sdf', 'sdf', 4.0, -1.0, 0.0, 0.2),
             SpawnSpec('bench_run_a_target_0', '/m.sdf', 'sdf', 5.0, 0.5, 0.0, 3.1)]
    with background_scene(client, background, RESOURCES, 'run',
                          clock=simulator.clock) as session:
        session.adjust('a', lambda: client.move_robot(1.0, 0.0, 0.0))
        with client.scene(specs):
            runner = _verifying_runner(client, background, session)
            TargetDistanceBenchmarkRunner.verify_scene_state(
                runner, {'trial_id': 'a', 'scene': SimpleNamespace(id='a')}, specs)
            check = runner.background_adjustments[0]['state_check']

            assert check['ok'] and check['differences'] == []
            assert len(check['state_hash']) == 64

            # A target off its pose, and an entity nobody commanded.
            simulator.entities['bench_run_a_target_0'] = (5.02, 0.5, 0.0, 3.1)
            simulator.entities['bench_old_bg_desk'] = (0.0, 3.0, 0.0, 0.0)
            with pytest.raises(SceneStateMismatch, match='state_mismatch') as error:
                TargetDistanceBenchmarkRunner.verify_scene_state(
                    runner, {'trial_id': 'a', 'scene': SimpleNamespace(id='a')}, specs)
            del simulator.entities['bench_old_bg_desk']

    differences = error.value.check.differences
    assert any(difference.startswith('unexpected bench_old_bg_desk') for difference in differences)
    assert any(difference.startswith('target_0 at (5.0200') for difference in differences)
    assert runner.background_adjustments[0]['state_check']['reason'] == 'state_mismatch'


def test_a_state_mismatch_fails_the_trial_before_any_capture() -> None:
    events = []
    logger = SimpleNamespace(error=events.append, info=print)

    class SceneControl:
        @contextlib.contextmanager
        def scene(self, specs):
            events.append('scene')
            yield ()

    def mismatch(_trial, _specs):
        from ridgeback_autonomy.benchmarking.scene_state import StateCheck
        raise SceneStateMismatch(StateCheck(False, ('missing bg_chair',), 'h', 0.0, 0.0))

    runner = SimpleNamespace(
        run_label='run', scene_control=SceneControl(),
        scene_spawn_specs=lambda _scene, _prefix: ([], []),
        adjust_background=lambda trial: events.append('adjust'),
        spin_for=lambda _seconds: events.append('settle'), settle_sec=0.0,
        verify_scene_state=mismatch,
        capture_measurement_window=lambda _seconds: events.append('capture'),
        get_logger=lambda: logger,
    )

    result = TargetDistanceBenchmarkRunner.run_trial(
        runner, {'trial_id': 's1', 'scene': SimpleNamespace(id='s1')})

    assert result is None
    assert events == ['adjust', 'scene', 'settle', 's1 failed: state_mismatch: missing bg_chair']


def test_run_json_carries_the_placement_and_the_props_flag() -> None:
    background = _background({'a': ('chair',)}, dx={'a': 1.0})
    background = SceneBackground(
        name='lab', layout=background.layout,
        drops=SimpleNamespace(path='d.yaml', sha256='d', rules={'margin_m': 0.3}),
        placements=background.placements)
    runner = SimpleNamespace(
        scene_background=background, background_props=False, background_spawn_sec=0.0,
        robot_home_record={'start': {'moved': False}}, background_adjustments=[{'trial_id': 'a'}])

    record = TargetDistanceBenchmarkRunner.background_provenance(runner)

    assert record['name'] == 'lab' and record['props_spawned'] is False
    assert record['backdrop_gap_m'] == 1.0 and record['rules'] == {'margin_m': 0.3}
    assert record['layout']['sha256'] == 'l' and record['drops']['sha256'] == 'd'
    assert record['scenes']['a']['dx_m'] == 1.0
    assert record['robot_home'] == {'start': {'moved': False}}
    assert record['trials'] == [{'trial_id': 'a'}]
