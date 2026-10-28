# RViz Image display teardown abort

Recorded dates: 2026-10-22
Tested revisions: `34b17d90f60b310111e9a9e04b69e35d9665ed8c`, `8345732ea38115fad6e8a3b57fe8e26a171ea3f0`, `feb01669f1297df2af755ce9cd2ed18083e7a8b2`
Provenance: complete

## Configuration and evidence boundary

Both root revisions above carried the uncommitted overlay change: RViz 14.1.23
(the third revision) with `patches/rviz_image_display_unsubscribe.patch`,
SHA-256 `b782976a615213f843e7c289a75e296a8f74d1fe01c53a6747403447e51d7f57`,
building only `rviz_default_plugins` and `rviz_visual_testing_framework`. The
standalone signal runs and the loader check used the first root revision; the
environment and sweep runs used the second, which adds the ros_gz overlay but
no RViz change. Host: RTX PRO 6000, CycloneDDS with the packaged configuration,
isolated domains. Evidence roots: stock runs in
`artifacts/benchmarks/shutdown_p7_red_20261022_ff72b294/`, patched runs in
`artifacts/benchmarks/shutdown_p7_20261022_34b17d90/`. The harness is the
2026-10-19 diagnosis's `rviz_signal_repro.py`.

## Defect and change

`ImageTransportDisplay::unsubscribe()` disconnected its
`message_filters::Connection` and reset the `SubscriberFilter` but kept the
connection, whose closure holds the raw address of the filter's `Signal1`.
A second unsubscribe (`onDisable()`, then the destructor) locked a mutex inside
the destroyed filter; the failing lock threw `std::system_error` out of a
destructor and RViz aborted. The patch disconnects only while the filter
exists and replaces the connection with an empty one before resetting the
filter. A unit test was not added: the stale call is undefined behaviour that
does not fail deterministically without a sanitizer build, so the signal runs
below are the proof.

## Results

| RViz, one SIGINT | benchmark.rviz | exploration.rviz |
|---|---|---|
| stock 14.1.23 | 6/20 aborted (system_error 4), 2 hung | 10/20 aborted (system_error 6) |
| patched | 0/20 aborted, 0 hung | 0/20 aborted, 0 hung |

With two SIGINTs 40 ms apart the patched RViz never aborted or hung, but 24 of
40 runs ended by signal 2: rclcpp handles the first SIGINT and lets the second
take the default action. That is rclcpp's design, not this defect; the sweep
no longer delivers a second signal. The patched process mapped
`install/rviz_default_plugins/lib/librviz_default_plugins.so` and kept
`rviz_common` from `/opt/ros/jazzy`; upstream `image_display_test` passed.

## A second, unpatched exit crash

Inside the live benchmark environment the patched RViz no longer aborts, but
it died with SIGSEGV in 9 of 10 Gazebo environment stops (stock RViz there:
-6 in 7 of 10, hung in 3). Under gdb, in all six stops tried, a CycloneDDS
receive thread (`recvMC`) faulted in `libddsc.so` while the main thread was in
`librmw_implementation`'s finalizer, unloading the RMW library at process exit:
a DDS participant -- which CycloneDDS deletes with the last node -- was still
alive after `main()` returned. The stock abort had been ending the process
before it reached this point. Upstream RViz has no fix as of this record; the
node that outlives `main()` is not yet identified.

Two-configuration mini sweeps with RViz on (the default) completed on both
simulators: no `std::system_error`, no -6, no -2, and no launch escalation;
every process but RViz exited 0, and RViz exited by SIGSEGV through the path
above.
