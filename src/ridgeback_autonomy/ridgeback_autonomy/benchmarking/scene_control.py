"""Simulator-neutral benchmark scene control over ROS ``simulation_interfaces``.

The benchmark spawns, observes, and removes its targets and occluders only
through the standard services, so it never learns which simulator answers
them. Every simulator-specific concern -- resource formats, native world
operations, threading -- stays behind the provider that serves the services.

The client owns the parts of the contract the standard leaves to the caller:
bounded calls, normalized result codes, one coherent state snapshot per truth
calculation, readiness after spawn, verified absence after delete, reverse
rollback, and the ownership rule that confines every mutation to
benchmark-named entities.

Services appearing is not readiness. A provider may answer its services while
the simulation has not yet stepped -- a renderer compiling shaders, say -- and
one that runs scene operations on its simulation loop then queues them and
withdraws them unanswered. The client therefore refuses every scene operation
until ``/clock`` has been seen advancing, so nothing is ever sent to a stalled
simulation, and a readiness timeout leaves no request behind to run later.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable, Iterator, Mapping, Sequence
import contextlib
from dataclasses import dataclass
import enum
import math
import time
from typing import Any, Protocol

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from simulation_interfaces.msg import Result, SimulatorFeatures
from simulation_interfaces.srv import (
    DeleteEntity,
    GetEntities,
    GetEntitiesStates,
    GetSimulatorFeatures,
    SpawnEntity,
)

from ridgeback_autonomy.benchmarking.asset_catalog import CatalogResource
from ridgeback_autonomy.benchmarking.simulation import normalize_pose


# Every entity the benchmark creates carries this prefix, and the client
# refuses to spawn, delete, or reap anything without it: the robot, the
# world-authored entities, and anything else in the simulation stay out of
# reach by construction.
BENCHMARK_ENTITY_PREFIX = 'bench_'
WORLD_FRAME_ID = 'world'
# Global by ROS convention; every simulator publishes its time here.
CLOCK_TOPIC = '/clock'

# Default budgets are the benchmark's established command, pose-wait, and
# delete windows.
CALL_TIMEOUT_SEC = 10.0
READY_TIMEOUT_SEC = 120.0
DELETE_TIMEOUT_SEC = 15.0
RETRY_SLEEP_SEC = 0.5
# Upper bound on one executor slice while waiting, so a deadline is overshot by
# at most this much.
SPIN_SLICE_SEC = 0.1

# Service name suffixes under the provider's namespace, as the standard names
# them.
SERVICE_TYPES = {
    'get_simulator_features': GetSimulatorFeatures,
    'spawn_entity': SpawnEntity,
    'delete_entity': DeleteEntity,
    'get_entities': GetEntities,
    'get_entities_states': GetEntitiesStates,
}

# Every operation on the simulated scene. Only feature discovery, which reads
# no simulation state, may be answered before the simulation clock advances.
SCENE_OPERATION_SERVICES = frozenset(SERVICE_TYPES) - {'get_simulator_features'}

REQUIRED_FEATURES = {
    SimulatorFeatures.SPAWNING: 'SPAWNING',
    SimulatorFeatures.DELETING: 'DELETING',
    SimulatorFeatures.ENTITY_STATE_GETTING: 'ENTITY_STATE_GETTING',
}

# POSIX extended regular expression metacharacters; the standard specifies ERE
# for entity filters, where escaping any other character is undefined.
_ERE_SPECIAL_CHARACTERS = frozenset('\\.[](){}*+?|^$')


class SceneControlFailure(enum.Enum):
    """Normalized failure categories, independent of the providing simulator."""

    UNAVAILABLE = 'service unavailable'
    TIMEOUT = 'no response before the deadline'
    MISSING_CAPABILITY = 'required capability not advertised'
    UNSUPPORTED_FEATURE = 'feature unsupported'
    NOT_FOUND = 'entity not found'
    INCORRECT_STATE = 'simulator in an incorrect state'
    OPERATION_FAILED = 'operation failed'
    NAME_CONFLICT = 'entity name not unique'
    INVALID_NAME = 'entity name invalid'
    UNSUPPORTED_FORMAT = 'resource format unsupported'
    INVALID_RESOURCE = 'resource invalid'
    INVALID_POSE = 'pose invalid'
    MISSING_STATE = 'entity state missing'
    INCOHERENT_STATE = 'state snapshot incoherent'
    MALFORMED_RESPONSE = 'malformed response'
    UNOWNED_ENTITY = 'entity outside the benchmark namespace'
    CLEANUP_INCOMPLETE = 'entity still present after delete'


_COMMON_RESULT_FAILURES = {
    Result.RESULT_FEATURE_UNSUPPORTED: SceneControlFailure.UNSUPPORTED_FEATURE,
    Result.RESULT_NOT_FOUND: SceneControlFailure.NOT_FOUND,
    Result.RESULT_INCORRECT_STATE: SceneControlFailure.INCORRECT_STATE,
    Result.RESULT_OPERATION_FAILED: SceneControlFailure.OPERATION_FAILED,
}

_SPAWN_RESULT_FAILURES = {
    SpawnEntity.Response.NAME_NOT_UNIQUE: SceneControlFailure.NAME_CONFLICT,
    SpawnEntity.Response.NAME_INVALID: SceneControlFailure.INVALID_NAME,
    SpawnEntity.Response.NAMESPACE_INVALID: SceneControlFailure.INVALID_NAME,
    SpawnEntity.Response.UNSUPPORTED_FORMAT: SceneControlFailure.UNSUPPORTED_FORMAT,
    SpawnEntity.Response.NO_RESOURCE: SceneControlFailure.INVALID_RESOURCE,
    SpawnEntity.Response.RESOURCE_PARSE_ERROR: SceneControlFailure.INVALID_RESOURCE,
    SpawnEntity.Response.MISSING_ASSETS: SceneControlFailure.INVALID_RESOURCE,
    SpawnEntity.Response.UNSUPPORTED_ASSETS: SceneControlFailure.INVALID_RESOURCE,
    SpawnEntity.Response.INVALID_POSE: SceneControlFailure.INVALID_POSE,
}


class SceneControlError(RuntimeError):
    """A scene-control operation that did not reach its contract outcome."""

    def __init__(
        self,
        failure: SceneControlFailure,
        operation: str,
        detail: str,
        *,
        entity: str | None = None,
        result_code: int | None = None,
        may_have_succeeded: bool = False,
    ) -> None:
        self.failure = failure
        self.operation = operation
        self.detail = detail
        self.entity = entity
        self.result_code = result_code
        self.may_have_succeeded = may_have_succeeded
        target = f' "{entity}"' if entity else ''
        code = f' (result {result_code})' if result_code is not None else ''
        super().__init__(f'{operation}{target}: {failure.value}{code}: {detail}')


class ServiceFuture(Protocol):
    def done(self) -> bool: ...

    def result(self) -> Any: ...

    def exception(self) -> BaseException | None: ...

    def cancel(self) -> None: ...


class ServiceClient(Protocol):
    """The subset of ``rclpy.client.Client`` the scene-control client uses."""

    def wait_for_service(self, timeout_sec: float | None = None) -> bool: ...

    def call_async(self, request: Any) -> ServiceFuture: ...

    def remove_pending_request(self, future: ServiceFuture) -> None: ...


class SimulationClockObserver:
    """Simulation time as published on ``/clock``.

    The clock counts as advancing once a stamp later than the first one seen
    arrives; a simulation that publishes one stamp and then stalls never
    passes. Wall times are recorded so a run can show how long the simulator
    took to start stepping after its services appeared.
    """

    def __init__(self, wall_clock: Callable[[], float] = time.time) -> None:
        self._wall_clock = wall_clock
        self.messages = 0
        self.first_stamp_ns: int | None = None
        self.first_message_wall: float | None = None
        self.latest_stamp_ns: int | None = None
        self.advanced_stamp_ns: int | None = None
        self.advanced_wall: float | None = None

    @property
    def advanced(self) -> bool:
        return self.advanced_stamp_ns is not None

    def observe(self, stamp_ns: int) -> None:
        stamp_ns = int(stamp_ns)
        self.messages += 1
        if self.first_stamp_ns is None:
            self.first_stamp_ns = stamp_ns
            self.first_message_wall = self._wall_clock()
        elif self.advanced_stamp_ns is None and stamp_ns > self.first_stamp_ns:
            self.advanced_stamp_ns = stamp_ns
            self.advanced_wall = self._wall_clock()
        self.latest_stamp_ns = stamp_ns

    def on_clock_message(self, message: Clock) -> None:
        self.observe(int(message.clock.sec) * 1_000_000_000 + int(message.clock.nanosec))


def _seconds(stamp_ns: int | None) -> float | None:
    return None if stamp_ns is None else stamp_ns / 1_000_000_000.0


@dataclass(frozen=True)
class SimulatorCapabilities:
    """What the provider advertised through ``GetSimulatorFeatures``."""

    features: frozenset[int]
    spawn_formats: tuple[str, ...]
    custom_info: str


@dataclass(frozen=True)
class SpawnSpec:
    """One static entity to create at a world-frame pose."""

    name: str
    uri: str
    resource_format: str
    x: float
    y: float
    z: float
    yaw: float

    @classmethod
    def for_resource(
        cls,
        resource: CatalogResource,
        name: str,
        x: float,
        y: float,
        yaw: float,
    ) -> 'SpawnSpec':
        """Place a catalog resource; its ground-origin metadata sets the height."""

        return cls(
            name=name,
            uri=resource.uri,
            resource_format=resource.resource_format,
            x=float(x),
            y=float(y),
            z=float(resource.spawn_z_m),
            yaw=float(yaw),
        )


@dataclass(frozen=True)
class SceneSnapshot:
    """Poses of selected entities taken from one simulator update.

    ``poses`` uses the normalized pose mapping that the ground-truth math
    consumes, so truth derived from a snapshot cannot mix robot and target
    poses from different simulation steps.
    """

    frame_id: str
    stamp_ns: int
    poses: Mapping[str, dict[str, Any]]


def scene_control_service_names(service_namespace: str) -> dict[str, str]:
    """Absolute standard service names under one provider namespace."""

    namespace = service_namespace.strip()
    if not namespace.startswith('/'):
        raise ValueError(
            f'Scene-control service namespace must be absolute, got {service_namespace!r}.')
    namespace = namespace.rstrip('/')
    return {suffix: f'{namespace}/{suffix}' for suffix in SERVICE_TYPES}


def posix_ere_literal(text: str) -> str:
    """Escape ``text`` so a POSIX ERE filter matches it literally."""

    return ''.join(
        f'\\{character}' if character in _ERE_SPECIAL_CHARACTERS else character
        for character in text
    )


def is_benchmark_entity(name: str) -> bool:
    return name.startswith(BENCHMARK_ENTITY_PREFIX) and len(name) > len(BENCHMARK_ENTITY_PREFIX)


def _require_benchmark_entity(name: str, operation: str) -> None:
    if not is_benchmark_entity(name):
        raise SceneControlError(
            SceneControlFailure.UNOWNED_ENTITY,
            operation,
            f'names must start with "{BENCHMARK_ENTITY_PREFIX}"',
            entity=name,
        )


def _quaternion_from_yaw(yaw_rad: float) -> tuple[float, float, float, float]:
    half = 0.5 * yaw_rad
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _check_result(
    result: Any,
    operation: str,
    entity: str | None = None,
    extended: Mapping[int, SceneControlFailure] | None = None,
) -> None:
    code = int(result.result)
    if code == Result.RESULT_OK:
        return
    failure = (extended or {}).get(code) or _COMMON_RESULT_FAILURES.get(
        code, SceneControlFailure.OPERATION_FAILED)
    raise SceneControlError(
        failure,
        operation,
        result.error_message or 'no error message',
        entity=entity,
        result_code=code,
    )


def _pose_from_state(name: str, state: Any) -> dict[str, Any]:
    position = state.pose.position
    orientation = state.pose.orientation
    return normalize_pose({
        'name': name,
        'position': {'x': position.x, 'y': position.y, 'z': position.z},
        'orientation': {
            'x': orientation.x,
            'y': orientation.y,
            'z': orientation.z,
            'w': orientation.w,
        },
    })


class SceneControlClient:
    """Benchmark scene control through the standard simulation services.

    ``clients`` maps each standard service suffix to its client and
    ``spin_once`` drives their responses; :func:`create_scene_control_client`
    supplies both from a ROS node. Waiting always goes through ``spin_once`` so
    the owning node keeps servicing its callbacks.
    """

    def __init__(
        self,
        clients: Mapping[str, ServiceClient],
        spin_once: Callable[[float], None],
        *,
        service_names: Mapping[str, str] | None = None,
        call_timeout_sec: float = CALL_TIMEOUT_SEC,
        ready_timeout_sec: float = READY_TIMEOUT_SEC,
        delete_timeout_sec: float = DELETE_TIMEOUT_SEC,
        retry_sleep_sec: float = RETRY_SLEEP_SEC,
        world_frame: str = WORLD_FRAME_ID,
        clock: Callable[[], float] = time.monotonic,
        simulation_clock: SimulationClockObserver | None = None,
        wall_clock: Callable[[], float] = time.time,
    ) -> None:
        missing = sorted(set(SERVICE_TYPES) - set(clients))
        if missing:
            raise ValueError(f'Scene-control clients missing for: {", ".join(missing)}')
        self._clients = dict(clients)
        self._spin_once = spin_once
        self._service_names = dict(service_names or {suffix: suffix for suffix in clients})
        self._call_timeout_sec = call_timeout_sec
        self._ready_timeout_sec = ready_timeout_sec
        self._delete_timeout_sec = delete_timeout_sec
        self._retry_sleep_sec = retry_sleep_sec
        self._world_frame = world_frame
        self._clock = clock
        self._capabilities: SimulatorCapabilities | None = None
        # Without an observed clock (in-memory providers) nothing is gated.
        self._simulation_clock = simulation_clock
        self._clock_ready = simulation_clock is None
        self._wall_clock = wall_clock
        self._services_available_wall: float | None = None
        self._first_spawn_requested_wall: float | None = None

    @property
    def capabilities(self) -> SimulatorCapabilities | None:
        return self._capabilities

    def readiness_timeline(self) -> dict[str, Any]:
        """When the services appeared, the clock advanced, and the first spawn left.

        Recorded with every run so a long simulator start (a cold shader cache,
        say) reads as a late clock rather than as a slow or failed provider.
        """

        observer = self._simulation_clock
        services = self._services_available_wall
        advanced = observer.advanced_wall if observer is not None else None
        spawn = self._first_spawn_requested_wall

        def delay(start: float | None, end: float | None) -> float | None:
            return None if start is None or end is None else end - start

        return {
            'services_available_wall': services,
            'clock_observed': observer is not None,
            'clock_first_message_wall': (
                observer.first_message_wall if observer is not None else None),
            'clock_first_message_sim_sec': _seconds(
                observer.first_stamp_ns if observer is not None else None),
            'clock_advanced_wall': advanced,
            'clock_advanced_sim_sec': _seconds(
                observer.advanced_stamp_ns if observer is not None else None),
            'first_spawn_requested_wall': spawn,
            'clock_advance_after_services_sec': delay(services, advanced),
            'first_spawn_after_clock_advance_sec': delay(advanced, spawn),
        }

    def wait_for_services(self, timeout_sec: float) -> None:
        """Fail unless every standard service appears within one shared budget."""

        deadline = self._clock() + timeout_sec
        missing = []
        for suffix, client in self._clients.items():
            remaining = max(0.0, deadline - self._clock())
            if not client.wait_for_service(timeout_sec=remaining):
                missing.append(self._service_names[suffix])
        if missing:
            raise SceneControlError(
                SceneControlFailure.UNAVAILABLE,
                'discover scene-control services',
                f'not available within {timeout_sec:.1f} s: {", ".join(missing)}',
            )
        if self._services_available_wall is None:
            self._services_available_wall = self._wall_clock()

    def wait_for_advancing_clock(self, timeout_sec: float) -> SimulationClockObserver:
        """Block until ``/clock`` advances; until then no scene operation is sent.

        Waiting dispatches nothing, so a timeout leaves no queued request that
        a simulator could still run once it starts stepping; later scene
        operations stay refused.
        """

        observer = self._simulation_clock
        if observer is None:
            raise ValueError('This scene-control client does not observe a simulation clock.')
        deadline = self._clock() + timeout_sec
        while not observer.advanced:
            remaining = deadline - self._clock()
            if remaining <= 0.0:
                if observer.first_stamp_ns is None:
                    detail = f'no {CLOCK_TOPIC} message within {timeout_sec:.1f} s'
                else:
                    detail = (
                        f'{CLOCK_TOPIC} stayed at {_seconds(observer.latest_stamp_ns):.3f} s '
                        f'across {observer.messages} message(s) for {timeout_sec:.1f} s')
                raise SceneControlError(
                    SceneControlFailure.TIMEOUT,
                    'wait for the simulation clock to advance',
                    f'{detail}; no scene operation was sent',
                )
            self._spin_once(min(remaining, SPIN_SLICE_SEC))
        self._clock_ready = True
        return observer

    def require_capabilities(
        self,
        resource_formats: Iterable[str] = (),
    ) -> SimulatorCapabilities:
        """Gate a run on spawning, deletion, state support, and resource formats."""

        response = self._call(
            'get_simulator_features',
            GetSimulatorFeatures.Request(),
            'read simulator features',
        )
        features = response.features
        capabilities = SimulatorCapabilities(
            features=frozenset(int(feature) for feature in features.features),
            spawn_formats=tuple(str(fmt).strip().lower() for fmt in features.spawn_formats),
            custom_info=str(features.custom_info),
        )
        missing = [
            label for feature, label in REQUIRED_FEATURES.items()
            if feature not in capabilities.features
        ]
        missing.extend(
            f'spawn format "{fmt}"'
            for fmt in sorted({str(fmt).strip().lower() for fmt in resource_formats})
            if fmt not in capabilities.spawn_formats
        )
        if missing:
            raise SceneControlError(
                SceneControlFailure.MISSING_CAPABILITY,
                'read simulator features',
                f'provider does not advertise {", ".join(missing)}',
            )
        self._capabilities = capabilities
        return capabilities

    def spawn_entity(self, spec: SpawnSpec) -> str:
        """Create one entity under its exact name; return the canonical name."""

        operation = 'spawn entity'
        _require_benchmark_entity(spec.name, operation)
        self._require_advancing_clock(operation, spec.name)
        resource_format = spec.resource_format.strip().lower()
        if (self._capabilities is not None
                and resource_format not in self._capabilities.spawn_formats):
            raise SceneControlError(
                SceneControlFailure.UNSUPPORTED_FORMAT,
                operation,
                f'format "{resource_format}" is not among advertised formats '
                f'{list(self._capabilities.spawn_formats)}',
                entity=spec.name,
            )

        request = SpawnEntity.Request()
        request.name = spec.name
        request.allow_renaming = False
        request.uri = spec.uri
        request.initial_pose.header.frame_id = self._world_frame
        request.initial_pose.pose.position.x = float(spec.x)
        request.initial_pose.pose.position.y = float(spec.y)
        request.initial_pose.pose.position.z = float(spec.z)
        (
            request.initial_pose.pose.orientation.x,
            request.initial_pose.pose.orientation.y,
            request.initial_pose.pose.orientation.z,
            request.initial_pose.pose.orientation.w,
        ) = _quaternion_from_yaw(float(spec.yaw))

        if self._first_spawn_requested_wall is None:
            self._first_spawn_requested_wall = self._wall_clock()
        try:
            response = self._call('spawn_entity', request, operation, spec.name)
        except SceneControlError as exc:
            # The request was handed to the transport, so a missing or unusable
            # response cannot prove that the provider did not create the entity.
            exc.may_have_succeeded = True
            raise
        _check_result(response.result, operation, spec.name, _SPAWN_RESULT_FAILURES)
        canonical = str(response.entity_name).strip()
        if not canonical:
            raise SceneControlError(
                SceneControlFailure.MALFORMED_RESPONSE,
                operation,
                'provider accepted the spawn but returned no entity name',
                entity=spec.name,
                may_have_succeeded=True,
            )
        try:
            _require_benchmark_entity(canonical, operation)
        except SceneControlError as exc:
            exc.may_have_succeeded = True
            raise
        return canonical

    def spawn_entities(self, specs: Sequence[SpawnSpec], spawned: list[str]) -> None:
        """Spawn sequentially, each visible in a state snapshot before the next.

        Each canonical name is appended to ``spawned`` before its readiness
        wait, so the caller can roll back an entity that never settles. A spawn
        whose response never arrived has an unknown outcome; its requested name
        is recorded too, and rollback verifies absence rather than assuming it.
        """

        for spec in specs:
            try:
                name = self.spawn_entity(spec)
            except SceneControlError as exc:
                if exc.may_have_succeeded:
                    spawned.append(spec.name)
                raise
            spawned.append(name)
            self.wait_until_present([name])

    def rollback(self, names: Sequence[str]) -> list[SceneControlError]:
        """Delete ``names`` in reverse spawn order; return failures, never raise.

        Every name is attempted even after one fails, so a single stuck entity
        cannot strand the rest.
        """

        failures = []
        for name in reversed(names):
            try:
                self.delete_entity(name)
            except SceneControlError as exc:
                failures.append(exc)
        return failures

    @contextlib.contextmanager
    def scene(self, specs: Sequence[SpawnSpec]) -> Iterator[tuple[str, ...]]:
        """Spawn ``specs`` for the body, then remove them in reverse order.

        A failure while spawning or inside the body -- including an interrupt
        -- rolls back every recorded spawn before the original error
        propagates; rollback failures are attached to it as notes. After a
        clean body, a failed removal raises.
        """

        spawned: list[str] = []
        try:
            self.spawn_entities(specs, spawned)
            yield tuple(spawned)
        except BaseException as exc:
            for failure in self.rollback(spawned):
                exc.add_note(f'rollback: {failure}')
            raise
        failures = self.rollback(spawned)
        if failures:
            raise SceneControlError(
                SceneControlFailure.CLEANUP_INCOMPLETE,
                'remove scene',
                '; '.join(str(failure) for failure in failures),
            )

    def get_scene_snapshot(self, names: Iterable[str]) -> SceneSnapshot:
        """Poses of ``names`` from a single ``GetEntitiesStates`` response."""

        operation = 'read entity states'
        wanted = list(dict.fromkeys(names))
        # Unfiltered, so the selection never depends on a provider's
        # regular-expression dialect; ``wanted`` is picked out locally.
        request = GetEntitiesStates.Request()
        response = self._call('get_entities_states', request, operation)
        _check_result(response.result, operation)
        entities = [str(name) for name in response.entities]
        if len(entities) != len(response.states):
            raise SceneControlError(
                SceneControlFailure.MALFORMED_RESPONSE,
                operation,
                f'{len(entities)} entity names for {len(response.states)} states',
            )
        if len(set(entities)) != len(entities):
            raise SceneControlError(
                SceneControlFailure.MALFORMED_RESPONSE,
                operation,
                'entity names are not unique',
            )

        states = dict(zip(entities, response.states))
        missing = [name for name in wanted if name not in states]
        if missing:
            raise SceneControlError(
                SceneControlFailure.MISSING_STATE,
                operation,
                f'no state for {", ".join(missing)}',
            )

        headers = {
            name: (
                states[name].header.frame_id or self._world_frame,
                int(states[name].header.stamp.sec) * 1_000_000_000
                + int(states[name].header.stamp.nanosec),
            )
            for name in wanted
        }
        wrong_frame = sorted(
            name for name, (frame_id, _stamp) in headers.items()
            if frame_id != self._world_frame
        )
        if wrong_frame:
            raise SceneControlError(
                SceneControlFailure.INCOHERENT_STATE,
                operation,
                f'states not in frame "{self._world_frame}": {", ".join(wrong_frame)}',
            )
        stamps = {stamp for _frame_id, stamp in headers.values()}
        if len(stamps) > 1:
            raise SceneControlError(
                SceneControlFailure.INCOHERENT_STATE,
                operation,
                f'states span {len(stamps)} simulation stamps',
            )

        return SceneSnapshot(
            frame_id=self._world_frame,
            stamp_ns=stamps.pop() if stamps else 0,
            poses={name: _pose_from_state(name, states[name]) for name in wanted},
        )

    def wait_until_present(
        self,
        names: Iterable[str],
        timeout_sec: float | None = None,
    ) -> SceneSnapshot:
        """Poll state snapshots until every name appears in one of them."""

        wanted = list(names)
        budget = self._ready_timeout_sec if timeout_sec is None else timeout_sec
        deadline = self._clock() + budget
        while True:
            try:
                return self.get_scene_snapshot(wanted)
            except SceneControlError as exc:
                if exc.failure not in (
                        SceneControlFailure.MISSING_STATE, SceneControlFailure.TIMEOUT):
                    raise
                if self._clock() >= deadline:
                    raise SceneControlError(
                        exc.failure,
                        'wait for entity state',
                        f'not ready within {budget:.1f} s: {exc.detail}',
                        entity=', '.join(wanted),
                    ) from exc
            self._pause(self._retry_sleep_sec)

    def delete_entity(self, name: str) -> None:
        """Remove ``name`` and return only once it is verifiably absent.

        A delete result is not trusted as the absence contract: providers may
        answer an already-removed name with a generic failure instead of
        NOT_FOUND. Absence is therefore always confirmed by query, and a failed
        delete is retried only while the entity is still present.
        """

        operation = 'delete entity'
        _require_benchmark_entity(name, operation)
        self._require_advancing_clock(operation, name)
        deadline = self._clock() + self._delete_timeout_sec
        deleted = False
        last_error: SceneControlError | None = None
        while True:
            if not deleted:
                request = DeleteEntity.Request()
                request.entity = name
                try:
                    response = self._call('delete_entity', request, operation, name)
                    _check_result(response.result, operation, name)
                    deleted = True
                except SceneControlError as exc:
                    last_error = exc
            try:
                if not self.entity_exists(name):
                    return
            except SceneControlError as exc:
                last_error = exc
            if self._clock() >= deadline:
                detail = f'still present after {self._delete_timeout_sec:.1f} s'
                if last_error is not None:
                    detail = f'{detail}; last error: {last_error}'
                raise SceneControlError(
                    SceneControlFailure.CLEANUP_INCOMPLETE,
                    operation,
                    detail,
                    entity=name,
                )
            self._pause(self._retry_sleep_sec)

    def entity_exists(self, name: str) -> bool:
        return name in self._query_entities(f'^{posix_ere_literal(name)}$')

    def find_benchmark_entities(self, prefix: str = BENCHMARK_ENTITY_PREFIX) -> tuple[str, ...]:
        """Entities whose names start with ``prefix``, a benchmark-owned prefix.

        The standard does not say whether a filter must match a whole name or
        only part of it, and Gazebo requires the whole name, so the filter
        spells out the rest of the name. The prefix is re-checked on the
        response, so a provider with looser regular-expression semantics still
        cannot hand back an unowned name for deletion.
        """

        if not prefix.startswith(BENCHMARK_ENTITY_PREFIX):
            raise SceneControlError(
                SceneControlFailure.UNOWNED_ENTITY,
                'find benchmark entities',
                f'prefix must start with "{BENCHMARK_ENTITY_PREFIX}"',
                entity=prefix,
            )
        names = self._query_entities(f'^{posix_ere_literal(prefix)}.*$')
        return tuple(sorted({name for name in names if name.startswith(prefix)}))

    def _query_entities(self, name_filter: str) -> list[str]:
        operation = 'list entities'
        request = GetEntities.Request()
        request.filters.filter = name_filter
        response = self._call('get_entities', request, operation)
        _check_result(response.result, operation)
        return [str(name) for name in response.entities]

    def _require_advancing_clock(self, operation: str, entity: str | None = None) -> None:
        if self._clock_ready:
            return
        raise SceneControlError(
            SceneControlFailure.INCORRECT_STATE,
            operation,
            f'the simulation clock has not been seen advancing on {CLOCK_TOPIC}; '
            'scene operations wait for it',
            entity=entity,
        )

    def _call(
        self,
        suffix: str,
        request: Any,
        operation: str,
        entity: str | None = None,
    ) -> Any:
        if suffix in SCENE_OPERATION_SERVICES:
            self._require_advancing_clock(operation, entity)
        client = self._clients[suffix]
        future = client.call_async(request)
        deadline = self._clock() + self._call_timeout_sec
        while not future.done():
            remaining = deadline - self._clock()
            if remaining <= 0.0:
                future.cancel()
                client.remove_pending_request(future)
                raise SceneControlError(
                    SceneControlFailure.TIMEOUT,
                    operation,
                    f'{self._service_names[suffix]} did not answer within '
                    f'{self._call_timeout_sec:.1f} s',
                    entity=entity,
                )
            self._spin_once(min(remaining, SPIN_SLICE_SEC))

        error = future.exception()
        if error is not None:
            raise SceneControlError(
                SceneControlFailure.OPERATION_FAILED,
                operation,
                f'{self._service_names[suffix]} call raised {error!r}',
                entity=entity,
            ) from error
        response = future.result()
        if response is None:
            raise SceneControlError(
                SceneControlFailure.MALFORMED_RESPONSE,
                operation,
                f'{self._service_names[suffix]} returned no response',
                entity=entity,
            )
        return response

    def _pause(self, duration_sec: float) -> None:
        deadline = self._clock() + duration_sec
        while True:
            remaining = deadline - self._clock()
            if remaining <= 0.0:
                return
            self._spin_once(min(remaining, SPIN_SLICE_SEC))


def create_scene_control_client(
    node: Any,
    service_namespace: str,
    *,
    spin_once: Callable[[float], None] | None = None,
    observe_clock: bool = True,
    **options: Any,
) -> SceneControlClient:
    """Build a client for the provider serving ``service_namespace``.

    The namespace is supplied by whoever selected the simulator; this module
    never chooses or derives it. ``spin_once`` defaults to spinning ``node``
    alone. The client subscribes to ``/clock`` on ``node`` and refuses scene
    operations until :meth:`SceneControlClient.wait_for_advancing_clock` has
    seen it advance. ``observe_clock=False`` is only for exercising a provider
    on its own, where nothing publishes simulation time.
    """

    names = scene_control_service_names(service_namespace)
    clients = {
        suffix: node.create_client(SERVICE_TYPES[suffix], name)
        for suffix, name in names.items()
    }
    observer = None
    if observe_clock:
        observer = SimulationClockObserver()
        # Best effort accepts both reliable and best-effort publishers.
        node.create_subscription(
            Clock,
            CLOCK_TOPIC,
            observer.on_clock_message,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
    if spin_once is None:
        def spin_once(timeout_sec: float) -> None:
            rclpy.spin_once(node, timeout_sec=timeout_sec)
    return SceneControlClient(
        clients, spin_once, service_names=names, simulation_clock=observer, **options)
