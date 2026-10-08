"""Standard ``simulation_interfaces`` scene control for the Isaac runner.

Serves the benchmark's spawn, delete, discovery, and entity-state services
without letting a ROS callback touch USD or PhysX:

1. a dedicated executor thread receives each request, validates everything
   that needs no simulation state, and enqueues it;
2. the SimulationApp main loop drains the queue once per update, right after
   it has published that update's clock and ground truth;
3. the main loop records the outcome stamped with that update's sim time;
4. the waiting callback answers, or reports that the simulation did not take
   the request in time -- in which case the request was withdrawn and will
   never run;
5. shutdown cancels whatever is still queued and joins the executor thread.

Entities are addressed by public names held in a registry. Only entities this
provider spawned can be deleted, and each maps to its own prim under one scope,
so a request can never name an arbitrary prim. The robot is a read-only entity
whose state is the rig's exact ground truth.

Nothing here imports Kit or pxr: the USD operations live in ``usd_scene.py``
and arrive through the :class:`NativeScene` protocol.
"""
from __future__ import annotations

import collections
from collections.abc import Callable, Iterable
from dataclasses import dataclass
import enum
import math
import os
import re
import threading
from typing import Any, Protocol
from urllib.parse import unquote, urlparse

from simulation_interfaces.msg import EntityState, Result, SimulatorFeatures
from simulation_interfaces.srv import (
    DeleteEntity,
    GetEntities,
    GetEntitiesStates,
    GetSimulatorFeatures,
    SpawnEntity,
)


# The provider node's name; its private services form the standard service
# namespace clients are configured with.
SERVICE_NODE_NAME = 'isaac_sim'
SERVICE_NAMESPACE = f'/{SERVICE_NODE_NAME}'

ADVERTISED_FEATURES = (
    SimulatorFeatures.SPAWNING,
    SimulatorFeatures.DELETING,
    SimulatorFeatures.ENTITY_STATE_GETTING,
)
SPAWN_FORMATS = ('usd',)
USD_SUFFIXES = frozenset({'.usd', '.usda', '.usdc', '.usdz'})

WORLD_FRAME_ID = 'world'
# Every spawned entity is the prim <scope>/<name>; nothing else is deletable.
SPAWNED_ENTITY_SCOPE = '/scene_entities'

# Kept below the common client's call budget, so the client always receives
# an explicit answer instead of timing out on an unknown outcome.
REQUEST_TIMEOUT_SEC = 5.0
QUEUE_CAPACITY = 8

# Public names double as prim names, so they follow USD identifier rules.
_ENTITY_NAME = re.compile(r'[A-Za-z_][A-Za-z0-9_]*')


class SceneRequestError(Exception):
    """A request refused with a standard result code."""

    def __init__(self, code: int, message: str) -> None:
        super().__init__(message)
        self.code = code
        self.message = message


class InvalidResource(Exception):
    """A spawn resource the native scene could not load."""


class NativeScene(Protocol):
    """Static-entity operations on the live stage, called on the main loop."""

    def spawn(self, prim_path: str, resource_path: str,
              position: tuple[float, float, float],
              orientation: tuple[float, float, float, float]) -> None: ...

    def delete(self, prim_path: str) -> bool: ...

    def pose(self, prim_path: str) -> tuple[tuple[float, float, float],
                                            tuple[float, float, float, float]]: ...


# --------------------------------------------------------------------------
# Main-loop command queue
# --------------------------------------------------------------------------

class _CommandState(enum.Enum):
    PENDING = 'pending'
    RUNNING = 'running'
    DONE = 'done'
    CANCELLED = 'cancelled'


class _Command:
    def __init__(self, operation: Callable[[Any], Any]) -> None:
        self.operation = operation
        self.state = _CommandState.PENDING
        self.result: Any = None
        self.error: BaseException | None = None
        self.finished = threading.Event()


class SceneCommandQueue:
    """A bounded hand-off from service callbacks to the simulation main loop.

    :meth:`call` blocks its (executor) thread until the main loop has run the
    operation in :meth:`drain`. A request the main loop has not started within
    the timeout is withdrawn, so a refused request is guaranteed never to run;
    one the main loop has started is always waited for, so its outcome is
    never lost.
    """

    def __init__(self, capacity: int = QUEUE_CAPACITY,
                 timeout_sec: float = REQUEST_TIMEOUT_SEC) -> None:
        if capacity < 1:
            raise ValueError('queue capacity must be positive')
        self._capacity = capacity
        self._timeout_sec = timeout_sec
        self._lock = threading.Lock()
        self._pending: collections.deque[_Command] = collections.deque()
        self._closed = False

    def __len__(self) -> int:
        """Requests waiting for the main loop."""
        with self._lock:
            return len(self._pending)

    def call(self, operation: Callable[[Any], Any]) -> Any:
        command = _Command(operation)
        with self._lock:
            if self._closed:
                raise SceneRequestError(
                    Result.RESULT_INCORRECT_STATE, 'the simulation is shutting down')
            if len(self._pending) >= self._capacity:
                raise SceneRequestError(
                    Result.RESULT_INCORRECT_STATE,
                    f'{self._capacity} scene requests are already waiting')
            self._pending.append(command)

        if not command.finished.wait(self._timeout_sec):
            with self._lock:
                if command.state is _CommandState.PENDING:
                    command.state = _CommandState.CANCELLED
                    self._pending.remove(command)
                    raise SceneRequestError(
                        Result.RESULT_INCORRECT_STATE,
                        'the simulation did not take the request within '
                        f'{self._timeout_sec:.1f} s; it was withdrawn')
            command.finished.wait()

        if command.state is _CommandState.CANCELLED:
            raise SceneRequestError(
                Result.RESULT_INCORRECT_STATE, 'the simulation is shutting down')
        if command.error is not None:
            raise command.error
        return command.result

    def drain(self, frame: Any) -> int:
        """Run the requests queued before this call, in arrival order."""

        with self._lock:
            batch = list(self._pending)
            self._pending.clear()
            for command in batch:
                command.state = _CommandState.RUNNING
        for command in batch:
            try:
                command.result = command.operation(frame)
            except Exception as exc:  # reported to the caller, never fatal here
                command.error = exc
            finally:
                command.state = _CommandState.DONE
                command.finished.set()
        return len(batch)

    def close(self) -> None:
        """Refuse new requests and cancel every queued one."""

        with self._lock:
            self._closed = True
            cancelled = list(self._pending)
            self._pending.clear()
            for command in cancelled:
                command.state = _CommandState.CANCELLED
        for command in cancelled:
            command.finished.set()


# --------------------------------------------------------------------------
# Poses and frames
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class RobotState:
    """The robot's exact world-frame pose and twist in one update."""

    x: float
    y: float
    z: float
    yaw: float
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


@dataclass(frozen=True)
class SimFrame:
    """What one main-loop update offers the queued requests."""

    sim_time: float
    robot: RobotState


@dataclass(frozen=True)
class EntityPose:
    name: str
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    linear: tuple[float, float, float] = (0.0, 0.0, 0.0)
    angular: tuple[float, float, float] = (0.0, 0.0, 0.0)


def yaw_quaternion(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def world_twist(body_twist: tuple[float, float, float],
                yaw: float) -> tuple[float, float, float]:
    """Rotate a planar body twist (vx, vy, wz) into the world frame."""

    vx, vy, wz = body_twist
    c, s = math.cos(yaw), math.sin(yaw)
    return c * vx - s * vy, s * vx + c * vy, wz


def stamp_parts(sim_time: float) -> tuple[int, int]:
    """Split sim time exactly as the runner stamps /clock and ground truth."""

    return int(sim_time), int((sim_time - int(sim_time)) * 1e9)


def robot_entity_name(namespace: str) -> str:
    """The Ridgeback's public entity name, identical to Gazebo's."""

    return f'{namespace.strip("/")}/robot'


# --------------------------------------------------------------------------
# Stateless request validation (executor thread)
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class SpawnPlan:
    name: str
    allow_renaming: bool
    resource_path: str
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]


def resolve_resource_path(uri: str) -> str:
    """Absolute local path of a USD resource given as a path or file URI."""

    parsed = urlparse(uri)
    if parsed.scheme == 'file':
        if parsed.netloc not in ('', 'localhost'):
            raise SceneRequestError(
                SpawnEntity.Response.UNSUPPORTED_FORMAT,
                f'file URI host "{parsed.netloc}" is not local')
        path = unquote(parsed.path)
    elif parsed.scheme:
        raise SceneRequestError(
            SpawnEntity.Response.UNSUPPORTED_FORMAT,
            f'only local USD files are supported, got a "{parsed.scheme}" URI')
    else:
        path = uri
    if not os.path.isabs(path):
        raise SceneRequestError(
            SpawnEntity.Response.RESOURCE_PARSE_ERROR,
            f'resource path "{path}" must be absolute')
    if os.path.splitext(path)[1].lower() not in USD_SUFFIXES:
        raise SceneRequestError(
            SpawnEntity.Response.UNSUPPORTED_FORMAT,
            f'resource "{path}" is not a USD file')
    if not os.path.isfile(path):
        raise SceneRequestError(
            SpawnEntity.Response.RESOURCE_PARSE_ERROR,
            f'resource "{path}" not found')
    return os.path.normpath(path)


def validate_spawn_request(request: Any) -> SpawnPlan:
    name = request.name
    if name and not _ENTITY_NAME.fullmatch(name):
        raise SceneRequestError(
            SpawnEntity.Response.NAME_INVALID,
            f'entity name "{name}" must be a USD identifier '
            '([A-Za-z_][A-Za-z0-9_]*)')
    if not name and not request.allow_renaming:
        raise SceneRequestError(
            SpawnEntity.Response.NAME_INVALID,
            'an empty entity name requires allow_renaming')
    if request.entity_namespace:
        raise SceneRequestError(
            SpawnEntity.Response.NAMESPACE_INVALID,
            'entity namespaces are not supported')
    if request.resource_string:
        raise SceneRequestError(
            Result.RESULT_FEATURE_UNSUPPORTED,
            'resource strings are not supported; pass a USD file uri')
    if not request.uri:
        raise SceneRequestError(SpawnEntity.Response.NO_RESOURCE, 'no resource uri given')
    resource_path = resolve_resource_path(request.uri)

    header_frame = request.initial_pose.header.frame_id
    if header_frame not in ('', WORLD_FRAME_ID):
        raise SceneRequestError(
            SpawnEntity.Response.INVALID_POSE,
            f'initial pose must be in frame "{WORLD_FRAME_ID}", got "{header_frame}"')
    pose = request.initial_pose.pose
    position = (pose.position.x, pose.position.y, pose.position.z)
    quaternion = (pose.orientation.x, pose.orientation.y,
                  pose.orientation.z, pose.orientation.w)
    if not all(math.isfinite(v) for v in (*position, *quaternion)):
        raise SceneRequestError(
            SpawnEntity.Response.INVALID_POSE, 'initial pose is not finite')
    norm = math.sqrt(sum(v * v for v in quaternion))
    if norm < 1e-9:
        raise SceneRequestError(
            SpawnEntity.Response.INVALID_POSE, 'initial orientation is a zero quaternion')
    return SpawnPlan(
        name=name,
        allow_renaming=bool(request.allow_renaming),
        resource_path=resource_path,
        position=tuple(float(v) for v in position),
        orientation=tuple(float(v) / norm for v in quaternion),
    )


def compile_entity_filter(filters: Any) -> re.Pattern[str] | None:
    """The name filter, or None for all entities; other filters are unsupported."""

    if filters.categories:
        raise SceneRequestError(
            Result.RESULT_FEATURE_UNSUPPORTED, 'entity categories are not supported')
    if filters.tags.tags:
        raise SceneRequestError(
            Result.RESULT_FEATURE_UNSUPPORTED, 'entity tags are not supported')
    if filters.bounds.type != filters.bounds.TYPE_EMPTY or filters.bounds.points:
        raise SceneRequestError(
            Result.RESULT_FEATURE_UNSUPPORTED, 'entity bounds are not supported')
    if not filters.filter:
        return None
    try:
        return re.compile(filters.filter)
    except re.error as exc:
        raise SceneRequestError(
            Result.RESULT_OPERATION_FAILED,
            f'invalid entity filter "{filters.filter}": {exc}') from exc


# --------------------------------------------------------------------------
# Entity registry and main-loop operations
# --------------------------------------------------------------------------

class SceneEntities:
    """The entity registry and the operations the main loop runs on it.

    Every method here runs on the main loop only, so the registry needs no
    lock. Matching uses search semantics, as POSIX regexec does; anchored
    filters therefore select whole names.
    """

    def __init__(self, native: NativeScene, robot_name: str,
                 scope: str = SPAWNED_ENTITY_SCOPE) -> None:
        self._native = native
        self._robot_name = robot_name
        self._scope = scope.rstrip('/')
        self._spawned: dict[str, str] = {}

    @property
    def robot_name(self) -> str:
        return self._robot_name

    def names(self) -> list[str]:
        return [self._robot_name, *self._spawned]

    def prim_path(self, name: str) -> str:
        return f'{self._scope}/{name}'

    def _claim_name(self, requested: str, allow_renaming: bool) -> str:
        taken = set(self.names())
        if requested and requested not in taken:
            return requested
        if not allow_renaming:
            raise SceneRequestError(
                SpawnEntity.Response.NAME_NOT_UNIQUE,
                f'entity "{requested}" already exists')
        stem = requested or 'entity'
        index = 0
        while f'{stem}_{index}' in taken:
            index += 1
        return f'{stem}_{index}'

    def spawn(self, plan: SpawnPlan) -> str:
        name = self._claim_name(plan.name, plan.allow_renaming)
        try:
            self._native.spawn(self.prim_path(name), plan.resource_path,
                               plan.position, plan.orientation)
        except InvalidResource as exc:
            raise SceneRequestError(
                SpawnEntity.Response.RESOURCE_PARSE_ERROR, str(exc)) from exc
        self._spawned[name] = self.prim_path(name)
        return name

    def delete(self, name: str) -> None:
        if name == self._robot_name:
            raise SceneRequestError(
                Result.RESULT_OPERATION_FAILED,
                f'entity "{name}" was not spawned through scene control and '
                'cannot be deleted')
        if name not in self._spawned:
            raise SceneRequestError(
                Result.RESULT_NOT_FOUND, f'entity "{name}" does not exist')
        if not self._native.delete(self._spawned[name]):
            raise SceneRequestError(
                Result.RESULT_OPERATION_FAILED,
                f'entity "{name}" is still present after removal')
        del self._spawned[name]

    def select(self, name_filter: re.Pattern[str] | None) -> list[str]:
        return [name for name in self.names()
                if name_filter is None or name_filter.search(name)]

    def states(self, name_filter: re.Pattern[str] | None,
               frame: SimFrame) -> list[EntityPose]:
        poses = []
        for name in self.select(name_filter):
            if name == self._robot_name:
                robot = frame.robot
                poses.append(EntityPose(
                    name=name,
                    position=(robot.x, robot.y, robot.z),
                    orientation=yaw_quaternion(robot.yaw),
                    linear=(robot.vx, robot.vy, 0.0),
                    angular=(0.0, 0.0, robot.wz),
                ))
            else:
                position, orientation = self._native.pose(self._spawned[name])
                poses.append(EntityPose(name, tuple(position), tuple(orientation)))
        return poses


# --------------------------------------------------------------------------
# ROS service layer
# --------------------------------------------------------------------------

def _set_result(response: Any, code: int, message: str = '') -> Any:
    response.result.result = code
    response.result.error_message = message
    return response


def _entity_state(pose: EntityPose, sim_time: float) -> EntityState:
    state = EntityState()
    state.header.stamp.sec, state.header.stamp.nanosec = stamp_parts(sim_time)
    state.header.frame_id = WORLD_FRAME_ID
    p = state.pose.position
    p.x, p.y, p.z = pose.position
    q = state.pose.orientation
    q.x, q.y, q.z, q.w = pose.orientation
    t = state.twist
    t.linear.x, t.linear.y, t.linear.z = pose.linear
    t.angular.x, t.angular.y, t.angular.z = pose.angular
    return state


class SceneControlService:
    """Serves the standard services from a dedicated executor thread.

    The runner calls :meth:`process_pending` once per update and
    :meth:`shutdown` before ROS itself shuts down.
    """

    def __init__(self, entities: SceneEntities, *,
                 node_name: str = SERVICE_NODE_NAME,
                 namespace: str = '/',
                 queue: SceneCommandQueue | None = None) -> None:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor

        self._entities = entities
        self._queue = queue if queue is not None else SceneCommandQueue()
        self.node = rclpy.create_node(node_name, namespace=namespace)
        for srv_type, suffix, callback in (
            (GetSimulatorFeatures, 'get_simulator_features', self._on_features),
            (SpawnEntity, 'spawn_entity', self._on_spawn),
            (DeleteEntity, 'delete_entity', self._on_delete),
            (GetEntities, 'get_entities', self._on_entities),
            (GetEntitiesStates, 'get_entities_states', self._on_states),
        ):
            self.node.create_service(srv_type, f'~/{suffix}', callback)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._thread = threading.Thread(
            target=self._spin, name='scene-control-executor', daemon=True)
        self._thread.start()

    @property
    def service_namespace(self) -> str:
        return self.node.get_fully_qualified_name()

    @property
    def executor_thread(self) -> threading.Thread:
        return self._thread

    def _spin(self) -> None:
        from rclpy.executors import ExternalShutdownException

        try:
            self._executor.spin()
        except ExternalShutdownException:
            pass

    def process_pending(self, frame: SimFrame) -> int:
        """Main loop: run the queued requests against this update."""

        return self._queue.drain(frame)

    def shutdown(self, timeout_sec: float = 5.0) -> None:
        self._queue.close()
        self._executor.shutdown(timeout_sec=timeout_sec)
        self._thread.join(timeout=timeout_sec)
        self.node.destroy_node()

    # ---- callbacks (executor thread) ---------------------------------------

    def _on_features(self, request, response):
        response.features.features = list(ADVERTISED_FEATURES)
        response.features.spawn_formats = list(SPAWN_FORMATS)
        response.features.custom_info = (
            'Static USD entities on the Isaac stage; names are USD identifiers '
            'and only spawned entities can be deleted.')
        return response

    def _answer(self, response, work: Callable[[], Any],
                apply: Callable[[Any], None]) -> Any:
        try:
            outcome = work()
        except SceneRequestError as exc:
            return _set_result(response, exc.code, exc.message)
        except Exception as exc:
            return _set_result(response, Result.RESULT_OPERATION_FAILED,
                               f'{type(exc).__name__}: {exc}')
        apply(outcome)
        return _set_result(response, Result.RESULT_OK)

    def _on_spawn(self, request, response):
        def work():
            plan = validate_spawn_request(request)
            return self._queue.call(lambda _frame: self._entities.spawn(plan))

        def apply(name):
            response.entity_name = name
        return self._answer(response, work, apply)

    def _on_delete(self, request, response):
        name = request.entity
        return self._answer(
            response,
            lambda: self._queue.call(lambda _frame: self._entities.delete(name)),
            lambda _outcome: None)

    def _on_entities(self, request, response):
        def work():
            name_filter = compile_entity_filter(request.filters)
            return self._queue.call(lambda _frame: self._entities.select(name_filter))

        def apply(names: Iterable[str]):
            response.entities = list(names)
        return self._answer(response, work, apply)

    def _on_states(self, request, response):
        def work():
            name_filter = compile_entity_filter(request.filters)
            return self._queue.call(
                lambda frame: (frame.sim_time, self._entities.states(name_filter, frame)))

        def apply(outcome):
            sim_time, poses = outcome
            response.entities = [pose.name for pose in poses]
            response.states = [_entity_state(pose, sim_time) for pose in poses]
        return self._answer(response, work, apply)
