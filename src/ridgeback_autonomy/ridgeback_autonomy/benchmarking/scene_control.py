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

    @property
    def capabilities(self) -> SimulatorCapabilities | None:
        return self._capabilities

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

        The anchored filter narrows the query, and the prefix is re-checked on
        the response, so a provider with looser regular-expression semantics
        still cannot hand back an unowned name for deletion.
        """

        if not prefix.startswith(BENCHMARK_ENTITY_PREFIX):
            raise SceneControlError(
                SceneControlFailure.UNOWNED_ENTITY,
                'find benchmark entities',
                f'prefix must start with "{BENCHMARK_ENTITY_PREFIX}"',
                entity=prefix,
            )
        names = self._query_entities(f'^{posix_ere_literal(prefix)}')
        return tuple(sorted({name for name in names if name.startswith(prefix)}))

    def _query_entities(self, name_filter: str) -> list[str]:
        operation = 'list entities'
        request = GetEntities.Request()
        request.filters.filter = name_filter
        response = self._call('get_entities', request, operation)
        _check_result(response.result, operation)
        return [str(name) for name in response.entities]

    def _call(
        self,
        suffix: str,
        request: Any,
        operation: str,
        entity: str | None = None,
    ) -> Any:
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
    **options: Any,
) -> SceneControlClient:
    """Build a client for the provider serving ``service_namespace``.

    The namespace is supplied by whoever selected the simulator; this module
    never chooses or derives it. ``spin_once`` defaults to spinning ``node``
    alone.
    """

    names = scene_control_service_names(service_namespace)
    clients = {
        suffix: node.create_client(SERVICE_TYPES[suffix], name)
        for suffix, name in names.items()
    }
    if spin_once is None:
        def spin_once(timeout_sec: float) -> None:
            rclpy.spin_once(node, timeout_sec=timeout_sec)
    return SceneControlClient(clients, spin_once, service_names=names, **options)
