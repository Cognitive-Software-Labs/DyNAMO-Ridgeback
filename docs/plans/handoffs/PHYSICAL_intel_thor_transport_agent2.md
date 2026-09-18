# PHYSICAL — Intel–Thor transport handoff (agent 2)

**Author:** agent 2 (Claude Code session on the robot host `cpr-r100-0160`)
**Written:** 2026-09-18, 18:50 UTC · **Repository HEAD:** `b939584` on
`feat/real-hardware-exploration`. This is agent 2's work merged with the
package split from origin.
**Governing plan:** [PHYSICAL — Intel–Thor deployment and transport qualification](../PHYSICAL_intel_thor_deployment.md),
section 2 "Transport, time, and readiness"
**Related:** [Intel–Thor DDS transport](../../physical/intel_thor_transport.md),
[D455 camera runs handoff (agent 1)](PHYSICAL_d455_camera_runs_agent1.md),
and the [Hokuyo recovery backlog item](../../BACKLOG.md#hokuyo-driver-recovery-after-read-timeouts)

The evidence folders below are under `artifacts/hardware/` on the robot host
and are not tracked by git.

## 1. Robot state at handoff (18:46 UTC)

| Item | State |
|---|---|
| Intel robot services | **CycloneDDS**, switched at 17:39 with `tools/intel_thor/intel_services_rmw apply`. `/etc/clearpath/robot.yaml` sets `system.ros2.middleware.implementation: rmw_cyclonedds_cpp` and `system.bash.env.CYCLONEDDS_URI`. `/etc/clearpath/cyclonedds.xml` binds `br0` and `wlp3s0`. Ten MyBotShop units carry `60-dynamo-cyclonedds.conf` drop-ins. |
| LiDARs | **Locked out** since 18:30:40: `urg_node` is in its reconnect loop, with about 50 connection errors a minute and no scans. Recovery: `sudo systemctl restart clearpath-sensors` |
| Camera | `realsense-camera.service` is active with the vendor configuration (agent 1 §1) |
| Receive buffers | `net.core.rmem_max` = 16 MiB, persisted in `/etc/sysctl.d/60-dynamo-dds-buffers.conf` on Intel and Thor |
| Thor | `~/DyNAMO-Ridgeback` is at `950aacb`, behind this HEAD. `ros-jazzy-rmw-cyclonedds-cpp` is installed. No perception environment exists yet. |
| Remotes | `origin` (GitHub) and `thor` (`robot@192.168.131.51:DyNAMO-Ridgeback`) |
| Agent 2 processes | None running |
| Driving | **Do not drive.** The LiDARs feed the collision monitor. |

## 2. Decisions made by the user in this session

- **CycloneDDS on both hosts** (D4 in the split plan). The benchmark was a tie
  within the link's capacity. The user chose Cyclone to match the simulator
  default.
- **Intel robot services keep the lab Wi-Fi as well as the robot Ethernet**,
  so laptops can still see the robot graph. Thor, and Intel readers of Thor
  output, are Ethernet-only.
- The branch `feat/real-hardware-exploration` keeps its name and is pushed to
  `origin` (D7).

## 3. Results

- **Link lag root cause.** The Ethernet is healthy: 0.4–0.6 ms ping and no
  loss. The lag came from DDS sending every sample over Wi-Fi as well, and
  from 212,992-byte receive buffers. See the transport diagnosis under
  Archived evidence.
- **Middleware benchmark.** Fast DDS, CycloneDDS, and a mixed pair tied
  within the link's capacity. Raw 1280×720 colour plus depth (about
  1.1 Gbit/s) does not fit the link. The mixed pair queued frames for 11 s
  under overload. See the benchmark under Archived evidence.
- **Reader-side rule.** A reader that advertises Wi-Fi makes Thor send to it
  over Wi-Fi (466 Mbit/s measured). Intel readers of Thor output therefore
  use `cyclonedds_intel.xml` through `dds_env.sh`.
- **`check_link`** passed with 0 failures after the switch, including the
  cross-host traffic test. The test uses the Ethernet-only configurations on
  both ends, so it does not exercise the services configuration (see §4).
- **Committed:** `948b02e`..`950aacb` (configurations, probe, `check_link`,
  `rmw_benchmark`, `intel_services_rmw`, docs, and archive records) and
  `0eabf98` (Hokuyo lockout documentation).

## 4. Open problem: the CycloneDDS services configuration floods the robot Ethernet

This is the most likely cause of the LiDAR lockouts, and it is not fixed.

- **Mechanism, measured by agent 1** (`20260918T183015-p1-cyclone/notes/wire_test.txt`).
  Starting one extra *local* camera subscriber makes `br0`, `eno1` (the
  LiDARs and Thor), and `enp2s0` (**the MCU, a 100 Mb/s port**) each transmit
  about 92 Mbit/s. Camera data meant for a reader on the same host leaves
  through the bridge. It is probably multicast: Cyclone's default
  `AllowMulticast` permits data multicast once a writer has several readers.
  Camera delivery to that subscriber also collapses (agent 1 §4).
- **LiDAR lockouts:** 17:43:53, 18:17:06, and 18:30:40. Each came 1–2 s after
  a new local subscriber started under the services configuration. In the
  18:30:40 case the subscriber was camera-only. There were none in 2.5 h on
  Fast DDS. Once a driver times out, the
  [`urg_node` reconnect bug](../../troubleshooting.md#hokuyo-drivers-stuck-reconnecting)
  makes the lockout permanent until `clearpath-sensors` restarts.
- **Why `check_link` missed it.** The check runs both probe ends on the
  Ethernet-only configurations, in an isolated domain, with one reader per
  topic. It never starts a second local reader of the robot's camera under
  `/etc/clearpath/cyclonedds.xml`. The idle wire measurement at 17:47 (0 Mbit/s
  on every port) was taken with only the vendor readers present.
- **Unexplained:** why the 92 Mbit/s wire load stalls the Hokuyo TCP sessions.
  Nothing was measured on the Hokuyo side.

## 5. Next steps, in order

1. **Stop the flood before anything else subscribes to the camera.** Choose
   one:
   - **Fix (preferred, unverified):** add
     `<General><AllowMulticast>spdp</AllowMulticast></General>` to
     `cyclonedds_intel_services.xml`, so discovery uses multicast and data is
     unicast. Consider the same setting for `cyclonedds_intel.xml` and
     `cyclonedds_thor.xml`. Install it with
     `sudo install -m 0644 src/ridgeback_autonomy_hardware/config/intel_thor/cyclonedds_intel_services.xml /etc/clearpath/cyclonedds.xml`,
     then restart the running robot services, with the robot stationary.
   - **Roll back to Fast DDS** until a fix is verified. The current
     `intel_services_rmw rollback` **rejects this robot's backup**: the backup
     was written by the older script, so `/etc/clearpath/dynamo-rmw-backup/`
     holds only `robot.yaml`. The script treats it as legacy and requires
     manual recovery. The pre-switch state is known exactly: `robot.yaml`
     without the `system.bash` and `system.ros2.middleware` keys (the backup
     copy), no `/etc/clearpath/cyclonedds.xml`, and none of the ten
     `60-dynamo-cyclonedds.conf` drop-ins. Manual rollback, with the robot
     stationary:

     ```bash
     sudo cp -p /etc/clearpath/dynamo-rmw-backup/robot.yaml /etc/clearpath/robot.yaml
     sudo rm -f /etc/clearpath/cyclonedds.xml /etc/systemd/system/*.service.d/60-dynamo-cyclonedds.conf
     sudo systemctl daemon-reload
     sudo systemctl restart clearpath-robot   # regenerates setup.bash with Fast DDS
     sudo systemctl restart clearpath-platform clearpath-sensors battery-sysfs-bridge clearpath-scan-merger clearpath-joy-x-combiner realsense-camera depth-to-mono8 ridgeback-camera-mjpeg mbs-webserver
     sudo rm -rf /etc/clearpath/dynamo-rmw-backup
     ```

   **Verify either path.** Start one extra local camera subscriber under
   `/etc/clearpath/setup.bash` and confirm that `eno1` and `enp2s0` stay near
   0 Mbit/s. Then run a 60 s camera capture, then watch
   `journalctl -u clearpath-sensors -f` for 30 minutes with the robot's normal
   readers active.
2. **Restart the LiDARs** (`sudo systemctl restart clearpath-sensors`) once
   step 1 is in place.
3. **Extend `check_link`** so it catches this failure. Measure wire load on
   `eno1`/`enp2s0` while a second local reader under the installed services
   configuration subscribes to the camera. Fail if the Hokuyo drivers have
   logged errors since they started.
4. **Close the Hokuyo backlog item.** Fix `urg_node` with a patch under
   `patches/` that closes the old session before reconnecting, or add a
   sensor-only supervisor. Then run the multi-hour soak it asks for.
5. **Update Thor** with `git push thor feat/real-hardware-exploration`, then
   check out that branch on Thor so both hosts match, and rerun `check_link`.
6. Continue the governing plan. Harden time sync (offsets of 0.6–4.2 ms on
   internet NTP), then install the Thor perception environment against the
   package split's localization install (`tools/check_localization_install`).
7. Update the [transport reference](../../physical/intel_thor_transport.md)
   (rules, known limits, the `check_link` table) and the archive incident entry
   with the flood mechanism and the third lockout, once step 1 is verified.

## 6. What not to do

- Do not start camera or scan subscribers under `/etc/clearpath/setup.bash`
  until step 1 is done. Each one can lock out both LiDARs.
- Do not run `sudo tools/intel_thor/intel_services_rmw rollback` or `apply`
  on this robot without first handling the legacy backup described in §5.
- Do not treat a passing `check_link` as proof that the robot services are
  safe. It does not test the services configuration.
- Do not drive while the LiDARs are down or while the flood is unresolved.

## Archived evidence

- [Intel–Thor DDS transport diagnosis](../../../archive/engineering/2026-09-18-intel-thor-dds-transport.md)
- [Intel–Thor middleware benchmark](../../../archive/engineering/2026-09-18-intel-thor-rmw-benchmark.md)
- [Hokuyo reconnect lockout](../../../archive/engineering/operational_incidents.md#hokuyo-reconnect-lockout--2026-09-18)
