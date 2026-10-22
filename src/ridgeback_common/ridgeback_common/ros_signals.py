"""SIGINT and SIGTERM handling owned by the process instead of by rclpy.

rclpy's default handlers shut every context down from rclpy's own signal
thread the moment SIGINT arrives, before the interrupted code has unwound.
Cleanup that still needs ROS -- deleting the entities a benchmark spawned,
publishing a final state -- then fails with "context is not valid", and a
second signal can land inside that cleanup.

``init_owned()`` keeps the context valid and turns both signals into a plain
``KeyboardInterrupt`` in the main thread, so cleanup runs while ROS still
works and the caller shuts the context down afterwards. ``shielded()`` defers
further signals while that cleanup runs.

A KeyboardInterrupt must never surface inside an executor step: raised
between taking a message and finishing its callback, it leaves the callback
group marked busy and the entity out of the wait set, so that node never runs
another callback of that group. ``spin_step()`` therefore holds a signal until
the step returns and raises it there. Spin with it, or with
``spin_until_interrupted()``, never with ``rclpy.spin_once()``; under these
handlers ``rclpy.spin()`` does not even wake on a signal. Wrap other rclpy
calls that must not be torn halfway, such as ``Client.call_async()``, in
``signals_held()``.

Only ``init_owned()`` and the spin helpers import rclpy, so code that merely
shields its cleanup stays importable without ROS.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
from contextlib import contextmanager
import signal
import sys
import threading


OWNED_SIGNALS = (signal.SIGINT, signal.SIGTERM)

_held = 0
_pending: list[int] = []


def _interrupt(signum, _frame) -> None:
    if _held:
        _pending.append(signum)
        return
    raise KeyboardInterrupt


@contextmanager
def signals_held() -> Iterator[None]:
    """Hold owned signals for the block, then raise one KeyboardInterrupt for them."""
    global _held
    _held += 1
    try:
        yield
    finally:
        _held -= 1
        if not _held and _pending:
            _pending.clear()
            raise KeyboardInterrupt


def init_owned(args=None) -> None:
    """Initialise rclpy without its signal handlers; both signals raise KeyboardInterrupt."""
    import rclpy
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    for signum in OWNED_SIGNALS:
        signal.signal(signum, _interrupt)


def spin_step(node, *, executor=None, timeout_sec: float = 0.1) -> None:
    """Run one executor step; a signal received meanwhile is raised when it returns.

    Keep ``timeout_sec`` bounded: the middleware wait does not return when a
    Python-level handler runs, so the wait is the latency of an interrupt.
    """
    import rclpy

    with signals_held():
        rclpy.spin_once(node, executor=executor, timeout_sec=timeout_sec)


def spin_until_interrupted(node, *, executor=None, period_sec: float = 0.1) -> None:
    """Spin until the context shuts down or an owned signal raises KeyboardInterrupt."""
    from rclpy.executors import SingleThreadedExecutor

    owned = executor is None
    if owned:
        executor = SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    try:
        while node.context.ok():
            with signals_held():
                executor.spin_once(timeout_sec=period_sec)
    finally:
        if owned:
            executor.remove_node(node)
            executor.shutdown()


def _stderr(message: str) -> None:
    print(message, file=sys.stderr, flush=True)


def _rclpy_handles_signals() -> bool:
    """Whether rclpy's own handlers are installed, without importing rclpy.

    ``signal.getsignal()`` cannot tell: rclpy installs below Python, which
    still reports its own previous handler.
    """
    signals_module = sys.modules.get('rclpy.signals')
    if signals_module is None:
        return False
    options = signals_module.get_current_signal_handlers_options()
    return options != signals_module.SignalHandlerOptions.NO


@contextmanager
def shielded(
    log: Callable[[str], None] | None = None,
    *,
    final: bool = False,
) -> Iterator[list[int]]:
    """Record SIGINT and SIGTERM instead of raising them until the block exits.

    Yields the list of signal numbers received meanwhile. The first one is
    logged; none is re-raised afterwards, because the caller is already
    shutting down. ``final`` marks the cleanup the process exits after: both
    signals then stay ignored instead of being restored, because one arriving
    later -- the second half of a group signal plus a launch relay -- would
    otherwise raise outside any handler, or land during interpreter
    finalization, which resets the handlers and turns a clean exit into a
    death by signal. Under rclpy's own handlers it shields nothing and leaves
    them installed: replacing them from Python would drop them for good, and
    they have shut the context down already. Off the main thread it is a
    no-op too.
    """
    received: list[int] = []
    if (threading.current_thread() is not threading.main_thread()
            or _rclpy_handles_signals()):
        yield received
        return

    report = log or _stderr

    def record(signum, _frame) -> None:
        if not received:
            report(
                f'Received {signal.Signals(signum).name} during cleanup; '
                'finishing cleanup before exiting.')
        received.append(signum)

    # None marks a handler installed outside Python, which cannot be restored.
    previous = {
        signum: handler for signum in OWNED_SIGNALS
        if (handler := signal.getsignal(signum)) is not None
    }
    for signum in previous:
        signal.signal(signum, record)
    try:
        yield received
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, signal.SIG_IGN if final else handler)
