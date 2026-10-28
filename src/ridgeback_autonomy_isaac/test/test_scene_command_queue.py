"""The hand-off from scene-control callbacks to the simulation main loop."""

from __future__ import annotations

import threading
import time

import pytest
from simulation_interfaces.msg import Result

from scene_provider import SceneCommandQueue, SceneRequestError


WAIT_SEC = 5.0


class Caller:
    """Calls the queue from its own thread, as the executor thread does."""

    def __init__(self, queue: SceneCommandQueue, operation) -> None:
        self.result = None
        self.error: BaseException | None = None
        self._thread = threading.Thread(target=self._run, args=(queue, operation))
        self._thread.start()

    def _run(self, queue, operation) -> None:
        try:
            self.result = queue.call(operation)
        except BaseException as exc:
            self.error = exc

    def join(self) -> 'Caller':
        self._thread.join(WAIT_SEC)
        assert not self._thread.is_alive()
        return self


def wait_for_pending(queue: SceneCommandQueue, count: int) -> None:
    deadline = time.monotonic() + WAIT_SEC
    while len(queue) < count:
        assert time.monotonic() < deadline, f'{len(queue)} of {count} requests queued'
        time.sleep(0.001)


def test_request_runs_on_the_draining_thread_with_its_frame() -> None:
    queue = SceneCommandQueue(timeout_sec=WAIT_SEC)
    seen = []
    caller = Caller(queue, lambda frame: seen.append(
        (frame, threading.current_thread().name)) or 'done')
    wait_for_pending(queue, 1)

    assert queue.drain('frame-7') == 1

    assert caller.join().result == 'done'
    assert seen == [('frame-7', threading.current_thread().name)]


def test_requests_run_in_arrival_order() -> None:
    queue = SceneCommandQueue(timeout_sec=WAIT_SEC)
    order = []
    callers = []
    for index in range(3):
        callers.append(Caller(queue, lambda _frame, index=index: order.append(index)))
        wait_for_pending(queue, index + 1)

    assert queue.drain(None) == 3

    for caller in callers:
        caller.join()
    assert order == [0, 1, 2]


def test_request_not_taken_in_time_is_withdrawn_and_never_runs() -> None:
    queue = SceneCommandQueue(timeout_sec=0.05)
    ran = []

    with pytest.raises(SceneRequestError) as excinfo:
        queue.call(lambda _frame: ran.append(True))

    assert excinfo.value.code == Result.RESULT_INCORRECT_STATE
    assert 'withdrawn' in excinfo.value.message
    assert len(queue) == 0
    assert queue.drain(None) == 0
    assert ran == []


def test_request_already_running_is_waited_for_past_the_timeout() -> None:
    queue = SceneCommandQueue(timeout_sec=0.05)
    started = threading.Event()

    def slow(_frame):
        started.set()
        time.sleep(0.3)
        return 'finished'

    caller = Caller(queue, slow)
    wait_for_pending(queue, 1)
    drainer = threading.Thread(target=queue.drain, args=(None,))
    drainer.start()
    assert started.wait(WAIT_SEC)
    drainer.join(WAIT_SEC)

    assert caller.join().result == 'finished'
    assert caller.error is None


def test_full_queue_refuses_immediately() -> None:
    queue = SceneCommandQueue(capacity=1, timeout_sec=WAIT_SEC)
    first = Caller(queue, lambda _frame: 'first')
    wait_for_pending(queue, 1)

    started = time.monotonic()
    with pytest.raises(SceneRequestError) as excinfo:
        queue.call(lambda _frame: 'second')

    assert time.monotonic() - started < 1.0
    assert excinfo.value.code == Result.RESULT_INCORRECT_STATE
    queue.drain(None)
    assert first.join().result == 'first'


def test_close_cancels_waiting_requests_and_refuses_new_ones() -> None:
    queue = SceneCommandQueue(timeout_sec=WAIT_SEC)
    ran = []
    caller = Caller(queue, lambda _frame: ran.append(True))
    wait_for_pending(queue, 1)

    queue.close()

    caller.join()
    assert isinstance(caller.error, SceneRequestError)
    assert caller.error.code == Result.RESULT_INCORRECT_STATE
    assert 'shutting down' in caller.error.message
    with pytest.raises(SceneRequestError, match='shutting down'):
        queue.call(lambda _frame: ran.append(True))
    assert queue.drain(None) == 0
    assert ran == []


def test_operation_error_reaches_the_caller_not_the_main_loop() -> None:
    queue = SceneCommandQueue(timeout_sec=WAIT_SEC)

    def broken(_frame):
        raise ValueError('stage refused')

    caller = Caller(queue, broken)
    wait_for_pending(queue, 1)

    assert queue.drain(None) == 1

    assert isinstance(caller.join().error, ValueError)


def test_capacity_must_be_positive() -> None:
    with pytest.raises(ValueError):
        SceneCommandQueue(capacity=0)
