"""The runner releases threads Kit pins to CPU 0, so a busy CPU 0 cannot stall its exit."""

from __future__ import annotations

import os
import threading

import pytest

from isaac_runner import release_cpu0_pins


pytestmark = pytest.mark.skipif(
    len(os.sched_getaffinity(0)) < 2 or 0 not in os.sched_getaffinity(0),
    reason='needs CPU 0 and at least one other CPU')


def _pinned_thread(cpus):
    """A live thread pinned to ``cpus``, its id, and the event that ends it."""

    ready, done = threading.Event(), threading.Event()
    tid = []

    def body():
        tid.append(threading.get_native_id())
        os.sched_setaffinity(0, cpus)
        ready.set()
        done.wait(10.0)

    thread = threading.Thread(target=body, daemon=True)
    thread.start()
    ready.wait(10.0)
    return thread, tid[0], done


def test_a_thread_pinned_to_cpu0_gets_the_process_cpus_back() -> None:
    allowed = os.sched_getaffinity(0)
    thread, tid, done = _pinned_thread({0})
    try:
        assert tid in release_cpu0_pins()
        assert os.sched_getaffinity(tid) == allowed
    finally:
        done.set()
        thread.join(10.0)


def test_threads_pinned_elsewhere_are_left_alone() -> None:
    other = max(os.sched_getaffinity(0))
    thread, tid, done = _pinned_thread({other})
    try:
        assert tid not in release_cpu0_pins()
        assert os.sched_getaffinity(tid) == {other}
    finally:
        done.set()
        thread.join(10.0)
