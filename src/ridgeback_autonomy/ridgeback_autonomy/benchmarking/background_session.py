"""Run-scoped background props over the simulator-neutral scene-control client.

A background is spawned once, before the first trial, and removed once, after
the last. Between trials only the difference changes: before each scene the
session removes the props that scene drops, has the robot moved to the scene's
placement, and only then brings back any prop an earlier scene removed that
this one keeps, so a prop never reappears where the robot stood. The room at
capture time therefore depends only on the scene, never on which scenes ran
before it.

A session without props -- the control arm -- runs the same steps with nothing
to remove or restore, so the robot's moves are the only thing it does.

Each adjustment starts from the props actually present in the simulation, not
from what the session believes, so a trial that failed half-way through an
adjustment needs no repair: the next adjustment simply finishes the job.
Every prop entity carries the benchmark prefix, so the rollback rules, the
sweep's orphan reaper and the runner's start-up check cover them unchanged.
"""

from __future__ import annotations

from collections.abc import Iterator, Mapping, Sequence
import contextlib
from dataclasses import dataclass
import time
from typing import Any, Callable

from ridgeback_autonomy.benchmarking.background import BackgroundProp, SceneBackground
from ridgeback_autonomy.benchmarking.scene_control import (
    ROLLBACK_INTERRUPT_BUDGET_SEC,
    SceneControlClient,
    SceneControlError,
    SceneControlFailure,
    SpawnSpec,
)
from ridgeback_common.ros_signals import shielded


def background_entity_prefix(run_label: str) -> str:
    return f'bench_{run_label}_bg_'


@dataclass(frozen=True)
class BackgroundAdjustment:
    """What one scene's adjustment did, by prop id, and how long each step took."""

    scene_id: str
    dropped: tuple[str, ...]
    removed: tuple[str, ...]
    restored: tuple[str, ...]
    duration_sec: float
    remove_sec: float = 0.0
    move_sec: float = 0.0
    restore_sec: float = 0.0
    # What the robot move returned, when there was one.
    move: Any = None

    def as_record(self) -> dict[str, Any]:
        return {
            'scene_id': self.scene_id,
            'dropped': list(self.dropped),
            'removed': list(self.removed),
            'restored': list(self.restored),
            'duration_sec': self.duration_sec,
            'remove_sec': self.remove_sec,
            'move_sec': self.move_sec,
            'restore_sec': self.restore_sec,
            **({'robot': self.move.as_record()} if hasattr(self.move, 'as_record') else {}),
        }


class BackgroundSession:
    """The background props of one run and the scene they are adjusted to."""

    def __init__(
        self,
        client: SceneControlClient,
        background: SceneBackground,
        resources: Mapping[str, Any],
        run_label: str,
        *,
        props: bool = True,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._client = client
        self._background = background
        self._props_enabled = props
        self._clock = clock
        self.prefix = background_entity_prefix(run_label)
        self._specs = {
            prop.id: SpawnSpec.for_resource(
                resources[prop.model], f'{self.prefix}{prop.id}', prop.x, prop.y, prop.yaw)
            for prop in self.props
        }
        self._prop_by_entity = {spec.name: prop_id for prop_id, spec in self._specs.items()}
        # Every prop that may exist, in layout order: spawned, or sent with an
        # unknown outcome. Removal verifies absence, so a stale entry is safe.
        self._tracked: dict[str, str] = {}
        self.spawn_sec: float | None = None

    @property
    def props(self) -> tuple[BackgroundProp, ...]:
        return self._background.layout.props if self._props_enabled else ()

    @property
    def props_enabled(self) -> bool:
        return self._props_enabled

    def spawn_all(self) -> None:
        """Spawn the whole layout in order, each confirmed before the next."""

        started = self._clock()
        self._spawn([prop.id for prop in self.props])
        self.spawn_sec = self._clock() - started

    def present(self) -> set[str]:
        """Prop ids whose entities exist in the simulation right now."""

        if not self._props_enabled:
            return set()
        return {
            self._prop_by_entity[name]
            for name in self._client.find_benchmark_entities(self.prefix)
            if name in self._prop_by_entity
        }

    def adjust(
        self,
        scene_id: str,
        move_robot: Callable[[], Any] | None = None,
    ) -> BackgroundAdjustment:
        """Make the room the full layout minus ``scene_id``'s drops.

        The order is fixed: remove what the scene drops, then ``move_robot``,
        then restore what it keeps, so the robot has left the place a returning
        prop stands before that prop appears.
        """

        started = self._clock()
        dropped = self._background.scene_drops[scene_id] if self._props_enabled else ()
        drop_set = set(dropped)
        present = self.present()
        order = [prop.id for prop in self.props]
        remove = [prop_id for prop_id in order if prop_id in drop_set and prop_id in present]
        for prop_id in reversed(remove):
            self._client.delete_entity(self._specs[prop_id].name)
            self._tracked.pop(prop_id, None)
        removed_at = self._clock()
        move = move_robot() if move_robot is not None else None
        moved_at = self._clock()
        restore = [prop_id for prop_id in order
                   if prop_id not in drop_set and prop_id not in present]
        self._spawn(restore)
        finished = self._clock()
        return BackgroundAdjustment(
            scene_id=scene_id,
            dropped=tuple(dropped),
            removed=tuple(remove),
            restored=tuple(restore),
            duration_sec=finished - started,
            remove_sec=removed_at - started,
            move_sec=moved_at - removed_at,
            restore_sec=finished - moved_at,
            move=move,
        )

    def remove_all(
        self,
        *,
        deadline: float | None = None,
        signals: list[int] | None = None,
    ) -> list[Exception]:
        """Delete every prop that may exist, in reverse layout order."""

        names = [self._tracked[prop.id] for prop in self.props if prop.id in self._tracked]
        failures = self._client.rollback(names, deadline=deadline, signals=signals)
        failed = {failure.entity for failure in failures
                  if isinstance(failure, SceneControlError)}
        self._tracked = {
            prop_id: name for prop_id, name in self._tracked.items() if name in failed}
        return failures

    def _spawn(self, prop_ids: Sequence[str]) -> None:
        spawned: list[str] = []
        try:
            self._client.spawn_entities([self._specs[prop_id] for prop_id in prop_ids], spawned)
        finally:
            for name in spawned:
                self._tracked[self._prop_by_entity[name]] = name
            self._tracked = {
                prop.id: self._tracked[prop.id] for prop in self.props
                if prop.id in self._tracked}


@contextlib.contextmanager
def background_scene(
    client: SceneControlClient,
    background: SceneBackground,
    resources: Mapping[str, Any],
    run_label: str,
    *,
    props: bool = True,
    after_removal: Callable[[float | None], None] | None = None,
    clock: Callable[[], float] = time.monotonic,
) -> Iterator[BackgroundSession]:
    """Spawn the background for the body, then remove it.

    Mirrors :meth:`SceneControlClient.scene`: a failure while spawning or in
    the body -- an interrupt included -- removes every prop before the error
    propagates, with removal failures attached as notes; an interrupt bounds
    that removal by ``ROLLBACK_INTERRUPT_BUDGET_SEC``. What the budget cannot
    reach keeps its benchmark name for the sweep's reaper and the next run's
    start-up check.

    ``after_removal`` runs once the props are gone -- returning the robot
    home, say -- with the interrupt deadline, or None after a clean body.
    Under an interrupt it runs only if that budget is not already spent, and
    its failure is attached as a note rather than replacing the interrupt.
    """

    session = BackgroundSession(
        client, background, resources, run_label, props=props, clock=clock)
    try:
        session.spawn_all()
        yield session
    except BaseException as exc:
        deadline = None
        if isinstance(exc, KeyboardInterrupt):
            deadline = clock() + ROLLBACK_INTERRUPT_BUDGET_SEC
        with shielded() as received:
            failures = session.remove_all(deadline=deadline, signals=received)
            if after_removal is not None and (deadline is None or clock() < deadline):
                try:
                    after_removal(deadline)
                except Exception as failure:
                    failures.append(failure)
        for failure in failures:
            exc.add_note(f'background removal: {failure}')
        raise
    with shielded() as received:
        failures = session.remove_all(signals=received)
        if after_removal is not None:
            deadline = clock() + ROLLBACK_INTERRUPT_BUDGET_SEC if received else None
            try:
                after_removal(deadline)
            except Exception as failure:
                failures.append(failure)
    if received:
        interrupt = KeyboardInterrupt()
        for failure in failures:
            interrupt.add_note(f'background removal: {failure}')
        raise interrupt
    if failures:
        raise SceneControlError(
            SceneControlFailure.CLEANUP_INCOMPLETE,
            'remove background',
            '; '.join(str(failure) for failure in failures),
        )
