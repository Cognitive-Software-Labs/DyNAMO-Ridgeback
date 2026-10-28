# BENCHMARKING — Shutdown lifecycle and crash qualification

Status: **in progress**. The user approved this investigation and implementation
on 2026-10-23. The [benchmarking backlog](../target_distance_benchmarking/BACKLOG.md#benchmark-shutdown-qualification)
owns the remaining gap; the [technical contract](../target_distance_benchmarking/overview.md#shutdown-lifecycle)
owns implemented behavior.

## Established boundary

The single-run compatibility wrapper originally placed configuration and
environment in one foreground process group. An interrupt asked the client to
roll back while stopping its provider. Physics-only real Gazebo probes reproduced
unconfirmed cleanup and a native `Broken promise` abort. Configuration-first
ordering and the new product supervisor both confirmed cleanup before provider
stop. The owned Gazebo core confirms `BlockingServiceHandler<RemoveCommand>`
rethrowing the broken promise from `future.get()` during ROS service execution.
Detailed populations and limits belong in the dated evidence record.

The maintained RViz Image subscription patch addresses a separately reproduced
abort. Full active Isaac cancellation reproduced the late DDS unload crash with
that patch loaded: a DDS thread executed an address whose library mapping had
already disappeared. The core also retained an unparented RViz view manager and
its current controller. Their reset-time service provides a concrete path for
retaining the ROS node. A focused ownership regression fails before correction
and passes after it, including release of the observed `NodeBase`. The first
traced full-workload cancellation exits RViz and its application owner cleanly,
but the strict whole-process matrix stopped on two SDK startup outcomes: a
driver-feature updater exited 1, and the SDK killed a forked child before the
operator interrupt. Neither is established as an expected status. This is not
full SDK qualification. Standalone synthetic traffic, simulated-time controls,
and environment-only tests cover different populations.

The separate real sweep cancellation confirms clean configuration-before-
environment teardown and no leftovers. Its stricter full-workload audit remains
unqualified: the same SDK startup outcomes recur, and capture-truth freshness at
the interrupt misses the declared threshold. Preserve that distinction when
extending the matrix; ordered cleanup evidence does not establish every workload
gate.

## Remaining execution

- Preserve exact source patches, probe inputs, dependency pins, loaded objects,
  versions, commands, child statuses, escalation, and owned-session checks for
  final qualification runs. Keep failed startup and debugging attempts separate.
- Exercise the single-run owner and sweep on both backends: normal completion,
  inspection, active cancellation, pending spawn/delete, repeated signals,
  startup failure, and unexpected child death. Confirm scene absence while the
  provider is still alive, not from a failed query during teardown.
- Extend the demonstrated view-manager correction across the remaining full
  workload and entrypoint populations. Repeat active Isaac cancellation
  with the actual node-scoped simulated-time parameter, namespace, and TF remaps.
  Preserve loaded-library paths and complete process outcomes. A clean ownership
  test alone cannot qualify the full workload.
- Establish callback quiescence before direct Gazebo provider destruction.
  The native stack identifies the pending remove promise, while source shows raw
  callback targets can outlive deregistration. Catching `future_error` alone does
  not establish safe lifetime. Retaining server/interfaces until the standalone
  ROS executor returns is a source-backed candidate, not a validated correction;
  component unload needs its own proof. Ordered application shutdown does not
  establish safe direct interruption.
- Give the recorded rare image-bridge residual its own reproducer and failure
  classification. A clean mini sweep cannot close that signature.
- Verify SDK helpers in separate process groups remain in their launch's owned
  session and are accounted for. Escalation or an unexplained helper exit must
  remain visible rather than being equated with clean launch-child status.
- Identify the pre-interrupt SDK child termination and
  `nvidia-ngx-updater -api update ... -feature dlssg` exit 1. Preserve startup and
  shutdown verdicts separately; do not waive these outcomes to make the matrix
  pass. No driver change is part of the implemented fixes.

## Completion condition

Each remaining signature has a preserved failing schedule and a mechanism-backed
correction, or a precise independently actionable blocker. Repeat counts and
conditions are declared before comparisons; retain every attempt. Both public
entrypoints report failed lifecycle outcomes even when valid measurements exist.
No forced escalation, unexplained child crash, or living owned session member
counts as a clean ordinary stop. Full matched benchmark accuracy acceptance stays
separate from this lifecycle investigation.
