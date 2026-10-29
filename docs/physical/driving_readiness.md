# Ridgeback driving command path and qualification

Driving readiness of `r100_0160` requires both a compatible software command
path and observed physical stopping. Graph discovery, stationary odometry, DDS
throughput and camera readiness do not establish the latter. This page routes to
the references that own each mechanism and lists what is installed now. The
[qualification plan](../plans/PHYSICAL_remaining_qualification.md#driving-and-feedback)
holds the status of each D-lane, and the
[project backlog](../BACKLOG.md#physical-command-chain-validation) holds the open
gates.

## Deployed command path

Nav2 (velocity smoother, collision monitor), the interactive marker, the
joystick combiner and RC input enter one `twist_mux` at priorities 1, 8, 10 and
12. The mux output `platform/cmd_vel` passes through the commissioning guard to
`platform/cmd_vel_limited` and the mecanum drive controller; every hop carries
`TwistStamped`. The guard also relays the firmware e-stop to the mux's
priority-255 lock on a reliable topic. Manual inputs bypass the collision
monitor. The [command path reference](driving_command_path.md) owns the
topology figure and the stop, stamp and speed semantics of each layer.

## Installed state

Last verified by a read-only stationary health batch with the physical stop
pressed on 2026-10-22; the receipts below live on the robot host under
`artifacts/hardware/`.

| Component | Installed now | Verification receipt | Owner |
|---|---|---|---|
| Guard speed mode | `--original-manual-speeds`: each planar axis 1.2 m/s, yaw 1.5 rad/s | `original-manual-install-20261021T121340Z`; restored after each temporary test lock | [guard](commissioning_guard.md#speed-modes) |
| Guard routing | Controller subscribes only to `platform/cmd_vel_limited`; one guard publisher | Every install and restore probe | [guard](commissioning_guard.md#installation-and-rollback) |
| E-stop relay | Reliable `platform/emergency_stop_mux`, mux-only remap | `20261009-stop-qos-install`, retained by later installs | [stop mechanisms](estop_stop_semantics.md#e-stop-delivery-to-the-mux) |
| Test-permit interlock | Not installed; installed temporarily for each moving trial, then restored | First install `20261009-test-interlock-install` | [guard](commissioning_guard.md#test-permit-interlock) |
| Default commissioning caps | Not installed; the fresh-SSD baseline | First install `20261006-commissioning-caps-install` | [guard](commissioning_guard.md#speed-modes) |
| Joystick combiner | Vendor `joy_x_combiner.py`, unchanged, SHA-256 `d3a4431a…40f7b` | Tracked recovery copy | [controller input](controller_input.md#mapping-and-scales) |
| Controller driver | `ds4drv` 0.5.5 virtual `/dev/input/ps4`; `joy_linux_node` repeats at 20 Hz | — | [controller input](controller_input.md#input-chain) |
| Controller autoconnect | `controller_autoconnect.py` with an ExecStart-only drop-in | Stationary setup receipt | [controller input](controller_input.md#autoconnect-helper) |
| Mux and controller | twist mux 4.5.0; mecanum controller 4.40.1, `reference_timeout` 0.1 s, 20 Hz | Live parameter reads | [command path](driving_command_path.md#layer-semantics) |
| Wheel feedback | Per-wheel field-age driver and feedback plugin | D5 acceptance | [wheel feedback](drive_feedback_diagnostics.md) |
| Autonomous motion | Disabled: `autonomous_motion_enabled:=false` (hardware default) | — | [backlog](../BACKLOG.md#physical-command-chain-validation) |

## E-stop delivery to the mux

The guard relays the best-effort firmware stop as reliable `platform/emergency_stop_mux`; see [stop mechanisms](estop_stop_semantics.md#e-stop-delivery-to-the-mux).

## Commissioning caps

The [commissioning guard](commissioning_guard.md#speed-modes) is the one speed limit on every source.

## Temporary commissioning test interlock

The test-permit mode is a moving-trial backstop; see the [test-permit contract](commissioning_guard.md#test-permit-interlock).

## Controller input

The [controller input reference](controller_input.md) owns the device chain,
the combiner's axis and button mapping, neutral-return semantics, disconnection
behaviour, the PS+Triangle binding, the autoconnect helper and recovery from the
connected-but-no-input failure.

## Check and recorder tools

The [check and recorder tools reference](driving_check_tools.md) owns what each
auditor, isolated check, observer mode, pulse helper and controller tool
measures, its flags and its limits. The root
[README](../../README.md#stationary-driving-readiness-checks) owns their
operator commands.

## Remaining physical verification

The [qualification procedure](driving_qualification_procedure.md) owns the
supervised physical procedure, operator sequencing, stop-action classification,
the D-lane verdict rules and the D8 stopping-distance method and speed ladder.
Which lanes remain open is held by the
[qualification plan](../plans/PHYSICAL_remaining_qualification.md#driving-and-feedback)
and the [backlog](../BACKLOG.md#physical-command-chain-validation), not here.

## Archived evidence

- [Commissioning guard installation and first low-speed trials](../../archive/engineering/2026-10-06-driving-guard-and-deadman-trials.md)
- [E-stop QoS repair and stationary disconnect trials](../../archive/engineering/2026-10-09-estop-qos-and-disconnect-trials.md)
- [Test-permit interlock, bounded-permit and moving-disconnect trials](../../archive/engineering/2026-10-12-interlock-and-moving-disconnect-trials.md)
- [Controller input loss, recovery and reconnect cycles](../../archive/engineering/2026-10-12-controller-input-loss-and-reconnect-cycles.md)
- [Nav2 command chain observed live](../../archive/engineering/2026-10-13-r100-0160-nav2-command-chain.md)
- [Bounded direction trials and D2 deadman and command-loss runs](../../archive/engineering/2026-10-14-direction-and-deadman-trials.md)
- [D2 physical-stop capture attempts](../../archive/engineering/2026-10-16-physical-stop-captures.md)
- [Original-manual guard profile and D4 handover trials](../../archive/engineering/2026-10-21-original-manual-and-d4-trials.md)
- [D-lane baseline, D2 preparation and the reconnected deadman run](../../archive/engineering/2026-10-22-d2-preparation-and-run.md)
