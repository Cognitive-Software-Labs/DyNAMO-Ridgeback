# urg_node reconnect lockout: leaked socket on a failed open

Recorded dates: 2026-10-13
Tested revisions: `c4ba777493dc139234bbb2361e89c9bc4e781c80`
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

## Limits

This test does not show that the 2026-09-18 sensors failed exactly at `PP`;
the journal does not record the first failed reopen. It also does not show the
patched driver resuming scans, because the simulated sensor never streams. The
patch has not been wired into `dependencies/core.repos` or deployed to the
`clearpath-sensors` service, and the backlog's on-robot interruption test has
not been run.
