#!/usr/bin/env python3
"""Crash-safe power recorder ("black box") for the Ridgeback.

Writes one CSV row per MCU power message (~10 Hz) carrying the latest motor-driver, stop,
e-stop and BMS values. Every row is flushed and fsync'd, so the file survives a sudden power
cut to within about one row. One file per start, in /home/robot/power_blackbox/.

Read it back with:  power_report.py          (last minute before the previous power loss)
                    power_report.py --live   (live readout for multimeter / wiggle tests)
"""
import csv
import datetime
import glob
import math
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from clearpath_motor_msgs.msg import PumaMultiFeedback, PumaMultiStatus
from clearpath_platform_msgs.msg import Power, StopStatus
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String

NS = "/r100_0160/platform"
OUT_DIR = os.environ.get("POWER_BLACKBOX_DIR", "/home/robot/power_blackbox")
KEEP_FILES = 60
GAP_WARN_S = 2.0  # no power message for this long -> write a row saying so

# Order of Power.measured_voltages on the R100 (see clearpath_platform_msgs/msg/Power)
RAILS = ["batt_V", "v5_V", "v12_V", "inverter_V", "front_axle_V", "rear_axle_V", "light_V"]
WHEELS = ["front_left", "front_right", "rear_left", "rear_right"]
COLUMNS = (
    ["wall_time", "mono_s"] + RAILS
    + ["total_A", "battery_connected", "charger_connected", "charging_complete", "soc_pct"]
    + [f"bus_{w}_V" for w in WHEELS] + [f"fault_{w}" for w in WHEELS]
    + ["driver_temp_max_C"] + [f"motor_{w}_A" for w in WHEELS]
    + ["estop", "stop_power_status", "external_stop_present", "needs_reset", "event"]
)
DATA_COLUMNS = COLUMNS[2:-1]
LIVE_ONLY = set(RAILS) | {"total_A"}  # blanked in rows written while the MCU is silent


def fmt(x):
    if x is None:
        return ""
    if isinstance(x, float):
        return f"{x:.3f}" if math.isfinite(x) else ""
    return str(x)


def wheel_of(device_name):
    for w in WHEELS:
        if device_name.startswith(w):
            return w
    return None


class BlackBox(Node):
    def __init__(self):
        super().__init__("power_blackbox")
        os.makedirs(OUT_DIR, exist_ok=True)
        self.prune()
        stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        self.path = os.path.join(OUT_DIR, f"{stamp}.csv")
        self.f = open(self.path, "w", newline="")
        self.csv = csv.writer(self.f)
        self.csv.writerow(COLUMNS)
        self.sync()

        self.s = {c: None for c in DATA_COLUMNS}  # latest known value of every column
        self.events = ["recorder started"]
        self.last_power = None

        q = qos_profile_sensor_data  # every publisher here is VOLATILE; best effort matches all
        self.create_subscription(Power, f"{NS}/mcu/status/power", self.on_power, q)
        self.create_subscription(PumaMultiStatus, f"{NS}/motors/status", self.on_motor_status, q)
        self.create_subscription(PumaMultiFeedback, f"{NS}/motors/feedback", self.on_motor_feedback, q)
        self.create_subscription(StopStatus, f"{NS}/mcu/status/stop", self.on_stop, q)
        self.create_subscription(Bool, f"{NS}/emergency_stop", self.on_estop, q)
        self.create_subscription(BatteryState, f"{NS}/bms/state", self.on_bms, q)
        self.create_subscription(String, f"{NS}/mcu/status/alerts", self.on_alert, q)
        self.create_timer(1.0, self.check_gap)
        self.get_logger().info(f"recording to {self.path}")

    def prune(self):
        files = sorted(glob.glob(os.path.join(OUT_DIR, "*.csv")))
        for old in files[:max(0, len(files) - (KEEP_FILES - 1))]:
            os.remove(old)

    def sync(self):
        self.f.flush()
        os.fsync(self.f.fileno())

    def set(self, key, value):
        """Store a value, noting it as an event when it changes."""
        old = self.s[key]
        self.s[key] = value
        if old is not None and value != old:
            self.events.append(f"{key} {old}->{value}")

    def write_row(self, stale=False):
        row = [datetime.datetime.now().isoformat(timespec="milliseconds"), f"{time.monotonic():.3f}"]
        row += ["" if stale and c in LIVE_ONLY else fmt(self.s[c]) for c in DATA_COLUMNS]
        row.append("; ".join(self.events))
        self.events = []
        self.csv.writerow(row)
        self.sync()

    def on_power(self, m):
        now = time.monotonic()
        if self.last_power is not None and now - self.last_power > GAP_WARN_S:
            self.events.append(f"power msgs resumed after {now - self.last_power:.1f}s")
        self.last_power = now
        v = list(m.measured_voltages)
        for i, name in enumerate(RAILS):
            self.s[name] = v[i] if i < len(v) else None
        self.s["total_A"] = m.measured_currents[0] if len(m.measured_currents) else None
        self.set("battery_connected", m.battery_connected)
        self.set("charger_connected", m.charger_connected)
        self.set("charging_complete", m.charging_complete)
        self.write_row()

    def on_motor_status(self, m):
        temps = []
        for d in m.drivers:
            w = wheel_of(d.device_name)
            if w is None:
                continue
            self.s[f"bus_{w}_V"] = d.bus_voltage
            self.set(f"fault_{w}", d.fault)
            temps.append(d.temperature)
        if temps:
            self.s["driver_temp_max_C"] = max(temps)

    def on_motor_feedback(self, m):
        for d in m.drivers_feedback:
            w = wheel_of(d.device_name)
            if w is not None:
                self.s[f"motor_{w}_A"] = d.current

    def on_stop(self, m):
        self.set("stop_power_status", int(m.stop_power_status))
        self.set("external_stop_present", int(m.external_stop_present))
        self.set("needs_reset", int(m.needs_reset))

    def on_estop(self, m):
        self.set("estop", int(m.data))

    def on_bms(self, m):
        if math.isfinite(m.percentage):
            self.s["soc_pct"] = m.percentage * 100.0

    def on_alert(self, m):
        if m.data:
            self.events.append(f"alert: {m.data}")

    def check_gap(self):
        # The PC is alive but the MCU has gone quiet -- worth knowing if that precedes a cut.
        if self.last_power is None:
            return
        gap = time.monotonic() - self.last_power
        if gap > GAP_WARN_S:
            self.events.append(f"no power msg for {gap:.1f}s")
            self.write_row(stale=True)

    def close(self):
        # Only reached on a clean stop (shutdown/reboot/systemctl stop), never on a power cut.
        self.events.append("recorder stopped cleanly")
        self.write_row(stale=True)
        self.f.close()


def main():
    rclpy.init()
    node = BlackBox()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
