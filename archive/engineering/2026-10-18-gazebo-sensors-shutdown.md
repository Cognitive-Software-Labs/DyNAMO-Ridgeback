# Gazebo Sensors waiter and concurrent-stop shutdown races

Recorded dates: 2026-10-18
Tested revisions: `e33b7bd4e1e81ef2e50b205c32a29e220272dfd4`, `446a44335a45b704b4d36dabcc5508ee34eeb3d8`
Provenance: complete

## Configuration and evidence boundary

The root revision above was tested with the uncommitted dependency patch
`gz_sim_sensors_shutdown.patch`, SHA-256
`20560bb638393dbac0352140931b61e25175b550b9d03a3b4866580be76024d8`.
The dependency remains gz-sim 8.15.0, matching `gz_sim_vendor` 0.0.13 and
ROS `ros_gz_sim` 1.0.24. The host used NVIDIA 595.99.02, an RTX PRO 6000,
display `:12.0`, CycloneDDS, isolated domains 83/84, and per-run Gazebo
partitions. No gz-common overlay or self-pipe patch was present.

The preserved evidence root is
`artifacts/benchmarks/gz_sensors_shutdown_20261018_e33b7bd4/`.
Its request, scripts, source inputs, dependency checks, binary hashes, process
maps, child exits, and lifecycle summaries distinguish the source populations.
`candidate_1_maintained.patch` and `candidate_1_objects/` preserve the first
candidate. `source_inputs.tar` and `root_review.diff` preserve the corrected
dirty-tree inputs; the source pin and maintained nested patches identify the
remaining dependency state. Routine logs stay in that evidence root.

## Two synchronization defects

The [pinned Sensors implementation](https://github.com/gazebosim/gz-sim/blob/446a44335a45b704b4d36dabcc5508ee34eeb3d8/src/systems/sensors/Sensors.cc)
waited on `updateTimeCv` without a shutdown predicate; `Stop` notified only
`renderCv`. Adding the shutdown predicate, coordinating its transition with
`renderUtilMutex`, notifying both waits, and returning before update work
corrected the focused waiter failures. A protocol model also distinguished
this synchronized change from predicate-plus-notify without the wait mutex.
The model did not construct a Sensors plugin or rendering engine.

The first maintained candidate was proven loaded from the workspace, including
the dynamically loaded Sensors plugin, but still needed SIGKILL. Thus passing
the protocol and focused tests did not qualify live teardown.

GDB was then used as the server's parent, without changing `ptrace_scope=1`.
Two diagnostic runs identified the **same** Sensors object in threads 1 and 14:
the signal callback's `ServerPrivate::OnSignal -> Stop` path and the server
destruction path both entered `SensorsPrivate::Stop`. In the second diagnostic,
object `0x7fffc807fdd0` logged render-thread exit, while the destruction thread
remained blocked in `std::thread::join` on that render thread. See
`diagnostic_02/launch.log`, especially the object/caller traces and the captured
thread-14 stack. The signal thread had already returned to the ROS executor.
Concurrent joins on one `std::thread` are undefined; a new same-instance
regression reproduced the remaining hang against the first candidate.

The corrected patch therefore adds a per-instance mutex around the complete
`Stop` operation, including connection teardown and join. It retains the
waiter correction and releases both render-related mutexes before joining.
This is a Sensors lifecycle correction justified by observed concurrent calls;
it does not alter benchmark stopping, launch timeouts, signal handlers, or
system libraries. It does not establish that every earlier historical hang had
the same cause.

The debugger runs are diagnostic evidence only. The first debugger process was
terminated by launch escalation before the planned full stack capture. The
second deliberately stopped its child for stacks and then killed only that
inferior. Neither is an ordinary terminal timing or acceptance result.

## Terminal qualification populations

All ordinary cycles attached stdin/stdout/stderr to a foreground PTY and sent
one byte `0x03` after active camera/lidar data and advancing simulation time.
Each cycle independently queried filtered and unfiltered standard entity names,
recorded every launch child exit, and checked owned processes and GUI configs.
The corrected cycles additionally checked camera TF and the loaded objects'
workspace paths and hashes before Ctrl-C.

| Population | Observed result |
|---|---|
| Three fresh unpatched cycles | One SIGKILL stop in 10.072 s; two normal stops in 1.901 s and 1.898 s |
| First maintained candidate | SIGKILL in 10.060 s despite loaded patched Sensors plugin |
| Corrected patch, ten varied active-sensor cycles | Ten normal gzserver exits, 1.797–2.037 s, no escalation or owned leaks |

Both failing ordinary runs had launcher exit 0. This is why qualification uses
the server's exit and escalation logs rather than the launcher's status alone.
`lifecycle_table.md` and `qualification_summary.json` retain every attempt;
the GDB attempts remain separate from the acceptance population.

## Installed workflow qualification

The all-estimator examples benchmark completed 5 scenes, 8 instances, 5 included
trials and no skipped trials. Outcomes, miss reasons, output schemas, and
accounting matched the previous Gazebo examples population. Independently
recomputed CSV statistics agreed with `run.json`; maximum truth difference was
`5.10e-12 m`. Scenario bytes retained SHA-256
`6200a4d4ea2a94a121978f5e028dce74dfcd8c786852db8f63a6bc81f1f943c6`.
The benchmark's ordinary terminal shutdown took 1.952 s with normal gzserver
exit and no escalation.

The two-configuration miniature sweep completed both estimator selections,
with 5 included trials each. Its actual process-group SIGINT stop completed
normally: the server exited approximately 1.21 s after the supervisor's stop
message, and the sweep CLI exited after 2.335 s. These are log-derived timings,
not PTY Ctrl-C timings. The last successful filtered **and** unfiltered entity
queries before the stop showed no benchmark entities and preserved the world
and robot. A later partial query during teardown failed and was not counted as
absence evidence. Preflight cleanup was explicitly skipped on the shared host.

An installed exploration launch on `initial_test_world`, using headless EGL,
completed a frontier navigation goal. Active sensor streams and camera TF were
verified, then terminal Ctrl-C produced normal gzserver exit in 2.097 s without
escalation. No owned process or generated Gazebo GUI config remained after
either installed workflow. Direct maps and object hashes proved that the
examples, sweep, and exploration used the corrected workspace Sensors stack.

Exact result paths relative to the evidence root:

- `gazebo_examples/run.json`
- `sweeps_gz/20261018_175025_gz_sensors_shutdown_mini/sweep.json`
- `sweeps_gz/20261018_175025_gz_sensors_shutdown_mini/pointcloud_polar/run.json`
- `sweeps_gz/20261018_175025_gz_sensors_shutdown_mini/depth_masks/run.json`
- `exploration_lifecycle/lifecycle.json`

## Limits

The six focused tests invoke the real Sensors methods with controlled internal
state, but most do not construct a rendering engine. Existing upstream
rendering tests and installed live runs supply that additional coverage.
Wait-entry stress and the concurrent-stop schedule are bounded scheduling
tests, not proofs of every possible interleaving.

Separate bridge shutdown failures occurred, including aborts and an IMU bridge
segfault in corrected cycle 2. The newly observed segfault's cause was not
established. Detector context-invalid shutdown traces occurred in both baseline
and corrected cycles. The sweep's RViz and detector aborted; several other
children returned SIGINT status. Exploration's velocity/coverage overlays and
frontier node exited 1 during Python shutdown. These processes' results and
tracebacks are retained separately; this record does not describe every child
as clean. General recursive stop from a rendering
callback remains outside the correction's supported synchronization boundary.

The matched full-scenario Gazebo/Isaac C5 acceptance was outside this work.
