# Puma motor-driver and wheel-feedback runbook

This runbook holds the operator commands for the three root-owned layers that
replace parts of the apt Clearpath 2.9.4 platform on `r100_0160`, and for the
stationary feedback-interruption checks. The references own the mechanisms:

- [Wheel feedback overview](drive_feedback_diagnostics.md#installed-state): the
  installed layers, their receipts and rollback targets.
- [Per-wheel field-age driver](wheel_feedback_field_age.md): field eligibility,
  bounds, build chain and live acceptance criteria.
- [Wheel-feedback consumer](wheel_feedback_consumer.md): batch validation,
  inhibition, latching and lifecycle.
- [Motor-link fault injection](motor_link_fault_injection.md): the interruption
  checks' fault paths and the raised-wheel D6 and D7 trials.

## Before you start

- Work from `/home/robot/DyNAMO-Ridgeback`. Every builder, stager and checker
  writes to a fresh directory below `artifacts/hardware/`, written here as
  `<run-dir>` or `<candidate-dir>`.
- Apply and restore need sudo, an operator beside the stationary robot, the
  physical stop pressed and the controls centred. Each installer restarts only
  the platform (the repair driver also stops and restarts the guard) and verifies that
  camera, sensor and desktop services are unchanged.
- Rebuild every layer from the pinned official archive and the maintained
  patches. An artifact copy is evidence, not configuration authority.
- The installers check the guard mode. The repair driver needs the default
  caps or the test interlock; the consumer plugin needs the test interlock; the
  field-age driver accepts any verified mode and its restore requires the same
  mode it was staged with. From the original-manual profile, install the test
  interlock first with the
  [guard installer](driving_operator_runbook.md#guard-installation-and-rollback).
- Install in this order: repair driver, then the
  [auxiliary repair](stationary_repairs_runbook.md#auxiliary-repairs), then the
  consumer plugin, then the field-age driver. Remove in the reverse order:
  restore the field-age driver before the plugin.

## Repair driver

Download the official Clearpath 2.9.4 archive first; the builder verifies its
SHA-256 `49d0ef5bcc6421856d6a4644aa4bd0dd117b1052d569d5b3a3238385184206df`.
Build and stage into fresh directories without touching the robot:

```bash
python3 tools/hardware/build_puma_repair.py \
  --archive <downloaded-clearpath-2.9.4.tar.gz> \
  --output "$PWD/artifacts/hardware/<candidate-dir>"
python3 tools/hardware/install_puma_repair.py \
  --candidate "$PWD/artifacts/hardware/<candidate-dir>" \
  --run-dir "$PWD/artifacts/hardware/<run-dir>"
```

Inspect the applied-patch provenance, sanitizer results, full-package compiler
output and staged receipt. The installer rejects a candidate without verified
patch-application provenance. Apply the frozen stage:

```bash
sudo python3 tools/hardware/install_puma_repair.py \
  --apply --operator-confirmed \
  --run-dir "$PWD/artifacts/hardware/<run-dir>"
```

Apply copies the patched package into a root-controlled
`/opt/dynamo-puma/<package-hash>` prefix and adds three owned files: DDS
interface readiness and a scoped platform package overlay. It preserves the
exact caps or test-interlock files. A failure after platform shutdown leaves
the platform stopped. Restore uses the same receipt and conditions and returns
the previous capped platform; the pinned prefix stays on disk:

```bash
sudo python3 tools/hardware/install_puma_repair.py \
  --restore --operator-confirmed \
  --run-dir "$PWD/artifacts/hardware/<run-dir>"
```

## Feedback-consumer plugin

### Offline checks

None of these touches ROS on the robot, CAN or the installed driver. Each needs
a fresh output directory.

Reproduce the stock plugin's behaviour from the matching 2.9.4 source:

```bash
python3 tools/hardware/verify_puma_feedback_consumer.py \
  --source <matching-clearpath-2.9.4>/clearpath_hardware_interfaces/src/puma/hardware.cpp \
  --output "$PWD/artifacts/hardware/<run-dir>"
```

Build the candidate from the pinned archive, then check it with fake wheels on
loopback ROS domains 214 and 215:

```bash
python3 tools/hardware/build_puma_feedback_candidate.py \
  --source-archive <official-clearpath-robot-2.9.4-tar.gz> \
  --output "$PWD/artifacts/hardware/<candidate-dir>"
source /opt/ros/jazzy/setup.bash
python3 tools/hardware/check_puma_feedback_ros.py \
  --candidate "$PWD/artifacts/hardware/<candidate-dir>" \
  --output "$PWD/artifacts/hardware/<run-dir>"
python3 tools/hardware/check_puma_feedback_manager.py \
  --candidate "$PWD/artifacts/hardware/<candidate-dir>" \
  --output "$PWD/artifacts/hardware/<run-dir>"
```

The manager check's recovery service calls apply only to its fake component;
they are not a robot recovery procedure. Software checks do not qualify
physical braking. The [verification reference](wheel_feedback_consumer.md#verification)
owns what each check covers.

### Plugin installation and rollback

The [installer](../../tools/hardware/install_puma_feedback_plugin.py) stages a
root-owned `/opt/dynamo-puma-feedback/<manifest-hash>` prefix and the platform
drop-in `95-dynamo-puma-feedback.conf` from a verified candidate. It preserves
the guard, the Puma motor driver, the auxiliary launch and the other hardware
plugin descriptors.

```bash
python3 tools/hardware/install_puma_feedback_plugin.py --stage \
  --candidate "$PWD/artifacts/hardware/<candidate-dir>" \
  --run-dir "$PWD/artifacts/hardware/<run-dir>"
sudo python3 tools/hardware/install_puma_feedback_plugin.py --apply \
  --operator-confirmed \
  --run-dir "$PWD/artifacts/hardware/<run-dir>"
```

Apply first runs the platform clock gate and refuses, with nothing changed, when
the clock reference is too old; obtain a fresh NTP sample and retry. Success
checks the library loaded by the real controller manager, the stop state,
stationary odometry, capped routing and unchanged independent services. A failed
apply attempts to restore the original platform; read its `result.json`. If
apply is interrupted, or on request, restore from the same receipt:

```bash
sudo python3 tools/hardware/install_puma_feedback_plugin.py --restore \
  --operator-confirmed \
  --run-dir "$PWD/artifacts/hardware/<run-dir>"
```

The pinned package stays on disk as evidence after rollback. This verifies
startup while stopped, not braking, missing CAN feedback under movement or
long runs.

## Field-age driver

Build the candidate offline from the pinned archive; the
[build reference](wheel_feedback_field_age.md#build) lists the patch chain and
its separate checks:

```bash
python3 tools/hardware/stage_puma_field_candidate.py \
  --archive <downloaded-clearpath-2.9.4.tar.gz> \
  --output "$PWD/artifacts/hardware/<candidate-dir>"
```

Stage, apply and, on request, restore it with the
[field-driver installer](../../tools/hardware/install_puma_field_driver.py):

```bash
python3 tools/hardware/install_puma_field_driver.py --stage \
  --candidate "$PWD/artifacts/hardware/<candidate-dir>" \
  --run-dir "$PWD/artifacts/hardware/<run-dir>"
sudo python3 tools/hardware/install_puma_field_driver.py --apply \
  --operator-confirmed --run-dir "$PWD/artifacts/hardware/<run-dir>"
sudo python3 tools/hardware/install_puma_field_driver.py --restore \
  --operator-confirmed --run-dir "$PWD/artifacts/hardware/<run-dir>"
```

The running driver must be the pinned repair named by the plugin drop-in. Apply
refuses when the plugin drop-in, robot identity, guard mode, dependency versions
or running driver changed since staging; stage again in that case. It restarts
only `clearpath-platform.service`. A failed apply restores the original driver.
To restore the installed driver, use the receipt listed in the
[installed state](drive_feedback_diagnostics.md#installed-state); the
[installation reference](wheel_feedback_field_age.md#installation-and-rollback)
owns what each step verifies.

With the stop pressed, run the live acceptance for 20–300 seconds:

```bash
python3 tools/hardware/check_puma_field_driver_live.py \
  --duration 60 --run-dir "$PWD/artifacts/hardware/<live-check-run-dir>"
```

The live check refuses an existing directory; give it a fresh one, not the
install receipt.

If it fails, or the driver is not kept, restore it. The pass criteria are under
[live acceptance](wheel_feedback_field_age.md#live-acceptance).

## Offline completeness-layer checks

The completeness layer is the third patch of the installed field-age chain.
These tools check it without ROS, CAN or the installed driver. The package is a
`puma_motor_driver` source tree that already carries the fault-flag and
cache-safety patches:

```bash
python3 tools/hardware/verify_puma_freshness_patch.py \
  --package <verified-two-patch-source-package> \
  --output "$PWD/artifacts/hardware/<run-dir>"
python3 tools/hardware/replay_puma_freshness.py \
  --capture <saved-passive-candump-log> \
  --output "$PWD/artifacts/hardware/<run-dir>/freshness-flag-replay.json"
```

Both outputs must be fresh. The replay is a flag and scheduling model, not a
live driver test; the [measurement tools](wheel_feedback_field_age.md#measurement-tools)
list the newer capture and replay checks.

## Post-reboot check

After an installed plugin or clock-profile change, prepare a one-reboot check
while stopped and reboot only with the stop pressed and an operator present.
Verify after the desktop reconnects:

```bash
python3 tools/hardware/verify_feedback_boot.py --prepare \
  --run-dir "$PWD/artifacts/hardware/<run-dir>"
sudo systemctl reboot
# After reconnecting, with the physical e-stop still pressed:
cd /home/robot/DyNAMO-Ridgeback
sudo python3 tools/hardware/verify_feedback_boot.py --verify \
  --run-dir "$PWD/artifacts/hardware/<run-dir>"
```

The verifier requires a changed boot ID, a successful platform clock gate,
stationary stop and service checks, and the exact pinned plugin mapped by the
new controller manager. Its result covers that boot only.

## Stationary feedback interruption checks

Both checks run with the physical stop pressed, the red lights flashing, the
controller centred and an operator beside the robot. Neither sends movement or
a permit. Use a fresh directory for every run and keep the stop pressed while
reviewing `result.json`.

Pause only the Puma driver for 0.8 seconds; an independent one-second timer
resumes it:

```bash
sudo python3 tools/hardware/check_live_feedback_outage.py --operator-confirmed \
  --run-dir "$PWD/artifacts/hardware/<run-dir>"
```

Drop only incoming UDP from the motor MCU on port 11412 for 0.8 seconds; an
independent three-second firewall rollback is armed before the rule:

```bash
sudo python3 tools/hardware/check_motor_can_ingress.py --operator-confirmed \
  --run-dir "$PWD/artifacts/hardware/<run-dir>"
```

If the controller faults as intended, save the evidence, then recover with the
stop still pressed:

```bash
sudo systemctl restart clearpath-platform.service
sudo python3 tools/hardware/check_live_feedback_outage.py --verify-recovery \
  --operator-confirmed --run-dir "$PWD/artifacts/hardware/<recovery-run-dir>"
```

Use a fresh `<recovery-run-dir>`, separate from the outage run; the tool refuses
an existing directory. The recovery check waits for platform startup and requires fresh feedback, zero
wheel commands, stationary odometry and an active controller. None of this
proves braking while moving or the motors' response to an electrical CAN
failure; the
[interruption reference](motor_link_fault_injection.md#stationary-interruption-checks)
owns those limits.
