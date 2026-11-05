# Stationary host repairs runbook

This runbook holds the operator commands for the stationary host repairs on the
`r100_0160` Intel PC: the desktop and controller setup repair, the auxiliary
IMU, battery and clock repair, the fast stationary health batch, and the Intel
clock-source and poll-bound changes. The references own the mechanisms and
the installed state:

- [Stationary auxiliary checks](stationary_auxiliary_checks.md): IMU and
  battery semantics, the platform clock gate, clock-source and poll-bound
  rationale, and recorded results.
- [Recovery runbook](ridgeback_recovery.md): host reconstruction, the desktop
  service ownership and the installed auxiliary repair.
- [Controller input](controller_input.md#autoconnect-helper): the autoconnect
  helper.

## Before you start

- Work from `/home/robot/DyNAMO-Ridgeback`. Each installer and recorder writes
  to a fresh directory below `artifacts/hardware/`, written here as `<run-dir>`.
  Installers refuse a reused directory and changed inputs or service state
  since staging.
- Apply, restore and verify need sudo, the robot stationary, the physical stop
  pressed, the controller centred and an operator present. Sudo authentication
  belongs to the operator's terminal.
- None of these commands sends motion, resets the base or releases the stop.

## Desktop and controller setup repair

[`install_stationary_setup.py`](../../tools/hardware/install_stationary_setup.py)
stages a six-file repair: one VNC owner, independent 9005 and 9100 proxies, a
foreground software-rendered desktop with its own session bus, and the
root-pinned controller autoconnect helper. The helper connects only configured
paired and trusted devices and never restarts ROS or base services; Bluetooth
connection alone does not prove fresh joystick input.

Stage without changing services:

```bash
python3 tools/hardware/install_stationary_setup.py --stage \
  --run-dir "$PWD/artifacts/hardware/<run-dir>"
```

Apply detached, so the installer survives the planned desktop reconnect:

```bash
sudo python3 tools/hardware/install_stationary_setup.py --apply --detach \
  --operator-confirmed \
  --run-dir "$PWD/artifacts/hardware/<run-dir>"
```

Read `result.json` after reconnecting. It verifies listener ownership,
stationary state and unchanged hardware service activity. Preflight drift
prevents changes, and a failed installation restores the original files and
prior service activity. Only the desktop, proxy and autoconnect units are
stopped or started. This does not qualify cold boot, joystick reconnection or
physical stopping. Never use the legacy combined vendor-camera repair here.

Receipt-based rollback, under the same conditions:

```bash
sudo python3 tools/hardware/install_stationary_setup.py --restore --detach \
  --operator-confirmed \
  --run-dir "$PWD/artifacts/hardware/<run-dir>"
```

Rollback refuses changed managed files. Restarting the desktop closes remote
access temporarily.

## Auxiliary repairs

[`install_auxiliary_repairs.py`](../../tools/hardware/install_auxiliary_repairs.py)
corrects the IMU filter's YAML selector, marks unmeasured individual battery
cell voltages unknown and adds a read-only platform clock gate before startup.
It preserves the commissioning guard and the pinned Puma driver, and needs both
installed first. The [repair reference](stationary_auxiliary_checks.md#source-diagnosis-and-installed-repairs)
owns what each change does and does not claim.

```bash
python3 tools/hardware/install_auxiliary_repairs.py --stage \
  --run-dir "$PWD/artifacts/hardware/<run-dir>"
sudo python3 tools/hardware/install_auxiliary_repairs.py --apply \
  --operator-confirmed \
  --run-dir "$PWD/artifacts/hardware/<run-dir>"
```

Only the platform is restarted deliberately; its guard follows the normal
service dependency, and camera, sensor, merger and desktop units are checked
unchanged. A failed installation restores its prior platform integration.
Success still needs live IMU and battery data verification before the receipt
counts as verified. Rollback refuses an unknown or changed managed file:

```bash
sudo python3 tools/hardware/install_auxiliary_repairs.py --restore \
  --operator-confirmed \
  --run-dir "$PWD/artifacts/hardware/<run-dir>"
```

If an interrupted run leaves the receipt at `applying` while every file and
service is already installed, do not apply again into the same directory.
Finalize by verification only:

```bash
sudo python3 tools/hardware/install_auxiliary_repairs.py --verify \
  --operator-confirmed \
  --run-dir "$PWD/artifacts/hardware/<run-dir>"
```

Verification checks the root-owned files, independent services, stationary
routing, clock readiness and live IMU and battery fields without restarting
anything. It refuses a partial or mismatched installation; inspect it and use
the restore path rather than overwriting.

**Clock readiness.** The installed gate waits up to 900 seconds for an external
chrony source with normal leap status, at most 5 ms remaining correction,
10 ppm skew and a 180-second reference age. A failure holds the platform start; it never resets
or releases the stop. Inspect the same conditions without sudo or clock changes:

```bash
python3 tools/hardware/check_clock_ready.py --timeout 30
```

While the gate waits, the platform can stay `activating`; read its journal
rather than bypassing the gate. The
[gate reference](stationary_auxiliary_checks.md#intel-clock-and-boot-policy)
explains the bounds.

**Stationary IMU characterization**, with the stop pressed:

```bash
source /opt/ros/jazzy/setup.bash
python3 tools/hardware/characterize_stationary_imu.py --duration 60 \
  --run-dir "$PWD/artifacts/hardware/<run-dir>"
```

It records raw IMU, stop, raw odometry, MCU power and temperatures, aborts on a
released or stale stop, wheel motion or too little data, and applies no
calibration. One pose cannot separate accelerometer bias from tilt; see
[IMU characterization](stationary_auxiliary_checks.md#imu-characterization-and-actual-battery-hardware-boundary).

**Reboot verification.** Before the reboot, save the pre-reboot boot ID from
`/proc/sys/kernel/random/boot_id` as the `boot_id` field of
`boot-baseline.json` in the installation receipt directory. Reboot only with an
operator, the stop pressed and the controller neutral, then verify:

```bash
source /opt/ros/jazzy/setup.bash
python3 tools/hardware/verify_auxiliary_boot.py \
  --receipt "$PWD/artifacts/hardware/<auxiliary-install-run-dir>" \
  --output "$PWD/artifacts/hardware/<run-dir>"
```

The read-only verifier checks the changed boot ID, repaired file contents, the
clock gate before platform startup, service state, desktop listeners, stationary
routing and IMU and battery fields. Passing one boot does not qualify prolonged
time-source loss or full cold power cycles.

## Fast stationary health batch

One command covers the installed stationary checks. Keep the physical stop
pressed; no controller action is needed:

```bash
source /opt/ros/jazzy/setup.bash
python3 tools/hardware/fast_stationary_check.py
```

It observes IMU, battery, three scans, stop and relay, and raw odometry for 16
seconds (`--duration`), then checks guarded routing, service health and local
clock readiness. It writes a unique `artifacts/hardware/stationary-batch-*`
directory and prints one report naming only failed checks and the remaining
gates. It publishes no commands, subscribes to no camera images, performs no
Bluetooth transitions, restarts no services and requests no reboot. A healthy
result is not calibration, physical stopping, long-run recovery or a camera or
perception pass.

If only clock readiness fails, its `failure_details` names the condition and
measurements. Keep the passed checks as evidence and retry only
`check_clock_ready.py`. A small remaining correction can coexist with large
frequency uncertainty; never relabel a noisy source a pass by raising the
thresholds.

**Camera and LiDAR screens.** Run these as separate commands so a failed camera
screen cannot hide the health report. Use fresh output names and keep failed
runs:

```bash
python3 tools/camera_contract_check --backend hardware --guard \
  --warmup 3 --duration 12 \
  --output "artifacts/hardware/camera-screen-$(date -u +%Y%m%dT%H%M%SZ).json"
python3 tools/hardware/hokuyo_recovery_watch.py --dry-run --duration 35 \
  --report "artifacts/hardware/hokuyo-passive-$(date -u +%Y%m%dT%H%M%SZ).json"
```

The camera command creates local image subscribers; its scan guard ends the
reader trial on an individual scan gap above 250 ms, a merged gap above 500 ms,
driver errors or service changes. The Hokuyo command creates no fault and
restarts nothing; unavailable journal access is a failed observation, not
evidence of healthy reconnect handling. Do not bundle a service restart into
these passive commands. LiDAR interruption, desktop restart, reboot and
time-source loss need their own rollback and evidence plans. Do not interrupt
the MCU or CAN link during a LiDAR check, or disable networking to test time
sources: that also removes the operator's desktop path.

## Intel clock sources

Both changes edit only the Intel chrony profile, never restart chrony and never
step the clock. Thor is untouched. Run them with an operator beside the
stationary robot and the stop pressed, and check `check_clock_ready.py`
afterwards. An installed result still needs clock qualification.

**Source expansion** replaces the single upstream server with a four-source
pool:

```bash
sudo python3 tools/hardware/expand_clock_sources.py --apply --operator-confirmed \
  --run-dir "$PWD/artifacts/hardware/clock-sources-$(date -u +%Y%m%dT%H%M%SZ)"
python3 tools/hardware/check_clock_ready.py --timeout 30
```

Undo it from the original run directory; restore refuses a profile edited
afterwards and removes only that run's added sources:

```bash
sudo python3 tools/hardware/expand_clock_sources.py --restore --operator-confirmed \
  --run-dir "$PWD/artifacts/hardware/<original-run-dir>"
```

An interrupted apply can be restored from its saved intent. Applying again
needs a fresh directory and the original profile. Source scope and
reconstruction are under
[clock source expansion](stationary_auxiliary_checks.md#intel-clock-and-boot-policy).

**Poll bound.** If the 180-second clock gate fails only because chrony's
selected source is older than 180 seconds while still reachable, cap the
polling interval at 128 seconds:

```bash
python3 tools/hardware/bound_clock_poll.py --stage \
  --run-dir "$PWD/artifacts/hardware/<run-dir>"
sudo python3 tools/hardware/bound_clock_poll.py --apply --operator-confirmed \
  --run-dir "$PWD/artifacts/hardware/<run-dir>"
```

After the next source measurement, check `chronyc -n sources` and
`check_clock_ready.py --timeout 30`. Restore from the same receipt:

```bash
sudo python3 tools/hardware/bound_clock_poll.py --restore --operator-confirmed \
  --run-dir "$PWD/artifacts/hardware/<run-dir>"
```

The [poll-bound reference](stationary_auxiliary_checks.md#intel-clock-and-boot-policy)
explains the gate and the evidence.
