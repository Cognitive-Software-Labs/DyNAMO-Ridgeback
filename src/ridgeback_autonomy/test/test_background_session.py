"""Run-scoped background props against the in-memory standard-services provider."""

from __future__ import annotations

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
from ridgeback_autonomy.benchmarking.scene_control import ROLLBACK_INTERRUPT_BUDGET_SEC
from ridgeback_autonomy.benchmarking.target_distance_benchmark_runner_node import (
    TargetDistanceBenchmarkRunner,
)
from test_scene_control import (
    WORLD_ENTITIES,
    FakeClock,
    FakeSimulator,
    make_client,
)


CONFIG_DIR = Path(__file__).resolve().parents[1] / 'config'
PROPS = ('desk', 'chair', 'cart', 'plant')
PREFIX = background_entity_prefix('run')


def _background(scene_drops) -> SceneBackground:
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
            scene_id: ScenePlacement(0.0, Anchor(0, WALL, 1.0), tuple(dropped))
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
    client, _ = make_client(simulator)
    background = _background({'a': ('chair',)})
    logged = []
    with background_scene(client, background, RESOURCES, 'run',
                          clock=simulator.clock) as session:
        runner = SimpleNamespace(
            background_session=session,
            background_adjustments=[],
            get_logger=lambda: SimpleNamespace(info=logged.append),
        )
        TargetDistanceBenchmarkRunner.adjust_background(
            runner, {'trial_id': 'a_rep1', 'scene': SimpleNamespace(id='a')})

    assert runner.background_adjustments[0]['trial_id'] == 'a_rep1'
    assert runner.background_adjustments[0]['removed'] == ['chair']
    assert logged and 'removed chair' in logged[0]


def test_leftover_benchmark_entities_refuse_the_run() -> None:
    runner = SimpleNamespace(scene_control=SimpleNamespace(
        find_benchmark_entities=lambda: ('bench_old_bg_desk', 'bench_old_t_target_0')))

    with pytest.raises(RuntimeError, match='2 benchmark entities are already'):
        TargetDistanceBenchmarkRunner.refuse_leftover_entities(runner)
