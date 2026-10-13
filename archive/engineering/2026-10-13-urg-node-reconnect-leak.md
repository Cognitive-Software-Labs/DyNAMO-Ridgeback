# urg_node reconnect lockout: leaked socket on a failed open

Recorded dates: 2026-10-13
Tested revisions: `c4ba777493dc139234bbb2361e89c9bc4e781c80`, `0088d4e808bbfba8a683f8069d5fb56681f38017`
Provenance: partial

This was found by reading the sources and reproduced offline on Intel
(`cpr-r100-0160`) against a simulated Hokuyo on `127.0.0.1`, with the driver on
an isolated ROS domain. The robot's LiDARs were not touched. Sources:
`ros-drivers/urg_node` tag `1.1.2` (`1b3e550`), which matches the installed
`ros-jazzy-urg-node` 1.1.2, and `ros-drivers/urg_c` tag `1.0.4001` (`53136ab`).
The harness `tools/urg_reconnect_check` and
`patches/urg_node_close_failed_open.patch` were added after the tested
revision; the hash records the checkout they ran from. Raw evidence is on Intel
under
`/home/robot/DyNAMO-Ridgeback-rviz/artifacts/hardware/20261013T103430Z-urg-reconnect-offline/`.

## Mechanism

`urg_c`'s `urg_open` (`urg_sensor.c`) opens the TCP connection, then
handshakes (`QT`) and reads the sensor parameters (`PP`). If `PP` fails or
times out, it returns an error and leaves the socket open. The `urg_node`
Ethernet constructor of `URGCWrapper` (`urg_c_wrapper.cpp`) throws on that
error without calling `urg_close`; the serial constructor does close. A
destructor never runs for a constructor that throws, so the descriptor leaks.
A Hokuyo UST serves one client at a time, so the leaked connection takes its
only slot, and every later reconnect is refused.

The error-limit reconnect path (`urg_.reset()`) closes correctly. So does a
completely silent sensor: `urg_open` then returns 0 after a failed handshake,
and the later `initialize()` failure closes the socket. Calling `urg_close`
after any failed `urg_open` is safe. `tcpclient_open` marks the descriptor
invalid when it starts and again when it closes itself on failure, and
`tcpclient_close` skips an invalid descriptor.

## Offline reproduction

The simulated UST accepts one client, refuses others until that client closes,
and either never replies (`silent`) or answers only `QT` as a SCIP 2.0 sensor
(`qt-only`). Each run lasted 30 s.

| Mode | Stock 1.1.2 | Patched |
|---|---|---|
| `qt-only` | 1 accepted, 0 closed, held open; 61 "Could not open network Hokuyo" | 47 accepted, 47 closed |
| `silent` | 28 accepted, 28 closed | 29 accepted, 28 closed (the last attempt was still in progress when the run ended) |

The stock `qt-only` result shows the 2026-09-18 signature: one established
connection and a stream of refused reconnects. Locally a refusal returns at
once, so the retries come every 0.5 s. Against a real UST each attempt adds
`urg_c`'s 2 s connect timeout, which gives the 2.5 s cadence in that day's
journal.

## On-robot interruption

Both patches were deployed on `r100_0160` as an overlay
(`/home/robot/ros_overlays/urg_node_fix`, listed under
`system.ros2.workspaces` in `/etc/clearpath/robot.yaml`). The rear LiDAR's
session was then killed with `sudo ss -K dst 192.168.131.22 dport = 10940`, with
the robot stationary and e-stopped. Rear scan arrivals and the socket table
were recorded once a second. The evidence is in
`artifacts/hardware/20261013T111331Z-urg-interruption/` and
`…/20261013T123558Z-urg-interruption-2/`.

| Deployed | Result |
|---|---|
| Leak patch only (11:14 UTC) | "Could not grab single echo scan", then the driver exited with −13 (SIGPIPE). Nothing respawned it, and the rear scans stopped for about 110 s, until a manual `clearpath-sensors` restart |
| Both patches (12:36 UTC) | 5 failed grabs, then "Error count exceeded limit, reconnecting". The same process connected again and streamed after a 2.25 s scan gap, then ran at 40 Hz |

The first result exposed the second defect. `urg_c`'s `tcpclient_write` calls
`send(…, 0)` without `MSG_NOSIGNAL`, so after the streaming read consumed the
reset, the next write raised SIGPIPE, whose default action ends the process.
`urg_node_ignore_sigpipe.patch` ignores SIGPIPE in the node constructor. The
offline `reset` mode resets during the handshake, where the reset surfaces as
a plain error, so it cannot reproduce this case; the robot test is the
evidence.

## Limits

This test does not show that the 2026-09-18 sensors failed exactly at `PP`;
the journal does not record the first failed reopen. Offline, the simulated sensor never streams,
so resuming scans is shown only on the robot, after a reset session, not after a
half-answered open. The
patches are not in `dependencies/core.repos`.
