# ros_gz bridge ownership cycles and image_bridge teardown

Recorded dates: 2026-10-22
Tested revisions: `34b17d90f60b310111e9a9e04b69e35d9665ed8c`, `5912bfe7976a79d2cc9f373b2d40a8a3a3e33da2`
Provenance: complete

## Configuration and evidence boundary

The root revision and ros_gz 1.0.24 above were tested with the uncommitted dependency patch
`patches/ros_gz_bridge_shutdown.patch` applied to ros_gz 1.0.24, building only
`ros_gz_bridge` and `ros_gz_image` (every other ros_gz package resolved to the
installed 1.0.24 binaries). Host: RTX PRO 6000, CycloneDDS with the packaged
configuration, isolated domains 64/65 and one Gazebo partition per run, PTY
Ctrl-C (byte `0x03`). Evidence root:
`artifacts/benchmarks/shutdown_p6_20261022_34b17d90/`, with the stock and
intermediate populations in `first_955_only/` and `bridge_fixes_only/`.
Stock-bridge stops come from the same day's evidence roots
`shutdown_p3_20261022_e2394c52/`, `shutdown_p2_20261022_ff72b294/`, and
`shutdown_p4_20261022_de865203/`.

## Three defects, three upstream changes

1. `RosGzBridge -> handles_ -> BridgeHandle -> ros_node_ -> RosGzBridge`:
   every bridge handle owned its node (upstream issue #951). Fixed by
   [#955](https://github.com/gazebosim/ros_gz/pull/955), commit
   `97a4d64f7907ce1aea01cc1bcc525f4ef2b97952`, released in 1.0.25.
2. Every ROS-to-Gazebo subscription callback also captured the node, and
   `parameter_bridge` returned from `main()` without `rclcpp::shutdown()`, so
   node teardown raced logger finalization. Fixed by
   [#991](https://github.com/gazebosim/ros_gz/pull/991), commit
   `95e6fdea72fb57ef55a4136292dfa62370ab3fb9`, on `jazzy` but in no release.
   Its `test_node_lifetime` gtest failed for the ROS-to-Gazebo and
   bidirectional directions with #955 alone and passed with both
   (`test_node_lifetime_{red,green}.log`).
3. `image_bridge` subscribed with a raw `this`; Gazebo transport does not wait
   for an in-flight callback when a subscription or node is destroyed, so a
   frame arriving during teardown reached a destroyed handler (upstream issue
   #979). Upstream fixed it on `ros2` as #981, commit
   `43ff7ce1292d37dda2e79ff3c7906508334b0f29`, which does not apply to 1.0.24
   (the lazy-subscription code it edits does not exist there). The patch
   carries a hand backport: the shared callback guard, explicit handler and
   Gazebo-node teardown in `main()`, and the upstream teardown test unchanged.

## Results

| Population | Env-layer stops | Single-launch stops (mid-trial) |
|---|---|---|
| Stock 1.0.24 | 0/10 bridge crashes (plus 1/1 in a layered env stop) | 6/7 with a `parameter_bridge` crash |
| #955 only | 0/10 | 1/10 (`cmd_vel_bridge`, `corrupted double-linked list`) |
| #955 + #991 | 1/10: `image_bridge` -11 (defect 3) | 0/30 |
| #955 + #991 + #981 backport | 0/10 | 0/10 |

`test_image_bridge_teardown` (20 SIGINTs per run at about 300 Hz): stock
`image_bridge` was killed by signal 11 in trial 3 of the first run; the
backport crashed once in 580 trials (one of 29 runs, not reproduced by a
preloaded SIGSEGV backtrace shim in 200 trials). Upstream reports 31/100
stock and 0/100 with its fix at the same rate. Treat the residual as
unexplained, not as zero.

Every bridge process mapped `install/ros_gz_bridge/lib/libros_gz_bridge.so`
(90 of 90 mappings in each env population), the final population's
`image_bridge` processes ran the overlay executable, and the camera held
28.8-30.0 Hz. Library and executable hashes are in `overlay_hashes_final.txt`.
`colcon test` passed for `ros_gz_bridge` (391 tests) and `ros_gz_image` (14).

## Not caused by the bridge

In every single-launch stop `gzserver` aborted with `std::future_error: Broken
promise` (-6, once -11) after the benchmark runner's interrupted-scene rollback
reached it while it was stopping; that belongs to `ros_gz_sim` and remains
open. One `target_overlay_node` exited 1 because rclpy's default SIGINT
handler interrupted `take_message` -- the node does not use the workspace's
owned signal handling. RViz, launched by the environment layer, still died
in most env stops (-6/-15 on stock RViz, -11 later); that is RViz's own
teardown, tracked separately.

## Rollback boundary

Pins, patch, `BUILD_ONLY` entry, and ignore entries form one set; the
dependency runbook's ros_gz section owns the rollback procedure.
