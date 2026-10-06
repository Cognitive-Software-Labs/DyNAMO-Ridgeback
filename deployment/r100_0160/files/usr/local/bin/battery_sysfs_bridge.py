#!/usr/bin/env python3
"""Battery display bridge, v3: a smooth "laptop-style" battery gauge for the Ridgeback.

Installed as /usr/local/bin/battery_sysfs_bridge.py (run by battery-sysfs-bridge.service).
Shows the robot battery on the desktop (btop / i3 / upower) through the test_power kernel
module (/sys/class/power_supply/test_battery), the same way a laptop battery is shown.

THE BATTERIES
  2 x DATOUBOSS 12.8 V / 100 Ah LiFePO4 in series: 25.6 V nominal, 100 Ah, 2.56 kWh. Each pack has
  its own internal BMS and no data interface, so the robot cannot read a true state of charge.

WHY NOT JUST SHOW THE VOLTAGE-BASED % (that is what v2 does)?
  LiFePO4 has an almost flat discharge curve: ~26.6 V down to ~25.6 V covers most of the capacity,
  and Clearpath's table (after lifepo4_lut.sh) still only spans 24.0-27.2 V. So the voltage-based %
  is worth +/-20-30 % in the middle, dips whenever the load rises, and bounces back when it falls.
  (Before lifepo4_lut.sh is applied it is worse: the lead-acid table reads 100 % until the BMS cuts.)

HOW THIS GAUGE WORKS  (every rate and threshold is a constant in the CONFIG section)
  1. Discharging -> count amp-hours. Each MCU power message (10 Hz) subtracts
     current x time / CAPACITY_AH. At the ~2 A idle draw that is ~2 % per hour.
  2. Charging -> estimated, not measured. The MCU's current sensor measures the robot's
     LOAD, not the current flowing into the battery, so the gauge rises at an ASSUMED rate
     (CHARGE_PCT_PER_HOUR), holds at 99 %, and shows 100 % only after the charger has
     reported "charging complete" continuously for FLOAT_CONFIRM_S.
  3. Low-voltage guard -> if the voltage-based % (smoothed over a minute) is below
     LOW_GUARD_BELOW_PCT and below the gauge, the gauge is pulled DOWN toward it within a few
     minutes. A collapsing battery still shows up; the smoothing does not hide it.
  4. Slow re-anchoring -> once the robot has been off the charger for REANCHOR_AFTER_S (so the
     post-charge voltage rise has faded) and the load is light, the gauge drifts very slowly toward
     the voltage-based % -- but ONLY near the bottom or top of the curve (REANCHOR_ONLY_OUTSIDE),
     never in LiFePO4's flat middle where the voltage estimate would drag a good count off course.
  5. Memory -> the gauge is saved to STATE_FILE every SAVE_EVERY_S and on plug/unplug, so it
     survives reboots and power cuts. If the robot was switched off with the charger
     connected, the off time is credited as charging time (same assumed rate, capped at 99 %).

KNOWN LIMITS
  - CAPACITY_AH is the nominal 100 Ah. If a pack is tired it holds less, so the count will run slow
    and the low-voltage guard will catch it near the bottom. Lower CAPACITY_AH if needed.
  - This gauge does NOT protect the battery. battery-guard.service does that (warning + clean
    shutdown before a BMS trip); it runs separately so switching the display never disables it.
  - Charging is estimated (see 2). A short top-up adds only a little, which is realistic.
  - Assumes Power.measured_currents[0] (R100_TOTAL_CURRENT) is the total battery draw.

CHECK IT     cat /var/lib/battery-gauge/state.json
             journalctl -u battery-sysfs-bridge -f      (one summary line every 5 min)
ROLL BACK    sudo bash ~/Downloads/power_debug/switch_display.sh v2        (voltage-based %)
             sudo bash ~/Downloads/power_debug/switch_display.sh original  (as before any changes)
"""
import datetime
import json
import math
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from clearpath_platform_msgs.msg import Power
from sensor_msgs.msg import BatteryState

# ---------------------------------------------------------------------------- CONFIG
CAPACITY_AH = 100.0          # 2 x 12.8 V/100 Ah LiFePO4 in series = 25.6 V, 100 Ah
CHARGE_PCT_PER_HOUR = 10.0   # assumed charging rate; calibrate against a real charge session
FLOAT_CONFIRM_S = 2 * 3600   # "charging complete" must hold this long before showing 100 %
LOW_GUARD_BELOW_PCT = 30.0   # below this voltage-based %, the gauge follows the voltage down
LOW_GUARD_TAU_S = 120.0      # ...with this time constant (seconds)
# LiFePO4's middle is too flat for voltage to mean anything (+/-20-30 %), so only re-anchor to it
# outside this band -- i.e. near the bottom (< 30 %) or near the top (> 90 %), where the curve moves.
REANCHOR_ONLY_OUTSIDE = (30.0, 90.0)
REANCHOR_AFTER_S = 3 * 3600  # off the charger this long before trusting the voltage again
REANCHOR_TAU_S = 3600.0      # ...then drift toward it with this (slow) time constant
REANCHOR_MAX_A = 5.0         # ...and only while the load is light (heavy load sags voltage)
VSOC_SMOOTH_TAU_S = 60.0     # extra smoothing of the voltage-based % (filters load spikes)
MAX_GAP_S = 60.0             # longer gaps between power messages are not integrated blind
MAX_STATE_AGE_S = 12 * 3600  # older saved state, with no charger connected -> start over
SAVE_EVERY_S = 30.0
SUMMARY_EVERY_S = 300.0
STALE_AFTER_S = 60.0

NS = "/r100_0160/platform"
PARAMS = "/sys/module/test_power/parameters"
STATE_DIR = os.environ.get("GAUGE_STATE_DIR", "/var/lib/battery-gauge")
STATE_FILE = os.path.join(STATE_DIR, "state.json")


# ---------------------------------------------------------------------------- gauge logic
def clamp(x, lo=0.0, hi=100.0):
    return max(lo, min(hi, x))


def approach(value, target, dt, tau):
    """Move value toward target like a first-order filter with time constant tau."""
    return value + (target - value) * (1.0 - math.exp(-dt / tau))


class Gauge:
    """Battery state-of-charge estimate, in %. Pure logic, no ROS, so it can be tested offline.

    saved:   dict from a previous to_state(), or None on the very first start
    v_soc:   Clearpath's voltage-based estimate, 0-100 %
    """

    def __init__(self, saved, now_wall, charger_connected, v_soc):
        self.charger = charger_connected
        self.v_soc_smooth = v_soc
        self.unplugged_wall = now_wall  # unknown on a fresh start: assume just unplugged
        self.float_since = None         # when "charging complete" became continuously true
        self.last_mono = None
        self.used_ah = 0.0              # drawn since the last unplug (for the log)

        age = now_wall - saved["saved_wall"] if saved else None
        if saved is None:
            self.soc = clamp(v_soc)
            self.origin = f"no saved state: starting from the voltage-based {v_soc:.0f} %"
        elif charger_connected:
            credit = CHARGE_PCT_PER_HOUR * max(0.0, age) / 3600.0
            self.soc = saved["soc"] if saved["soc"] >= 100.0 else min(saved["soc"] + credit, 99.0)
            self.origin = (f"resumed {saved['soc']:.1f} % saved {age / 60:.0f} min ago, "
                           f"charger connected -> credited +{credit:.1f} %")
        elif age <= MAX_STATE_AGE_S:
            self.soc = saved["soc"]
            self.unplugged_wall = saved.get("unplugged_wall", now_wall)
            self.used_ah = saved.get("used_ah_since_unplug", 0.0)
            self.origin = f"resumed {saved['soc']:.1f} % saved {age / 60:.0f} min ago"
        else:
            self.soc = clamp(v_soc)
            self.origin = (f"saved state is {age / 3600:.0f} h old: starting from the "
                           f"voltage-based {v_soc:.0f} %")

    def update(self, now_mono, now_wall, current_a, charger_connected, charging_complete, v_soc):
        """Advance the gauge to now; returns the new estimate in %."""
        dt = 0.0 if self.last_mono is None else now_mono - self.last_mono
        self.last_mono = now_mono
        if dt > MAX_GAP_S:
            dt = 0.0  # the MCU was silent for a while; don't guess what happened

        if charger_connected != self.charger:
            self.float_since = None
            if not charger_connected:  # just unplugged
                self.unplugged_wall = now_wall
                self.used_ah = 0.0
            self.charger = charger_connected

        if v_soc is not None and dt > 0:
            self.v_soc_smooth = approach(self.v_soc_smooth, v_soc, dt, VSOC_SMOOTH_TAU_S)

        if charger_connected:
            # rise at the assumed rate; never past 99 % until the float is confirmed
            self.soc = min(self.soc + CHARGE_PCT_PER_HOUR * dt / 3600.0, max(self.soc, 99.0))
            if charging_complete:
                if self.float_since is None:
                    self.float_since = now_mono
                if now_mono - self.float_since >= FLOAT_CONFIRM_S:
                    self.soc = 100.0
            else:
                self.float_since = None
        else:
            ah = max(0.0, current_a) * dt / 3600.0
            self.used_ah += ah
            self.soc -= 100.0 * ah / CAPACITY_AH
            vs = self.v_soc_smooth
            if vs < LOW_GUARD_BELOW_PCT and vs < self.soc:
                self.soc = approach(self.soc, vs, dt, LOW_GUARD_TAU_S)
            elif (now_wall - self.unplugged_wall >= REANCHOR_AFTER_S
                  and current_a <= REANCHOR_MAX_A
                  and not (REANCHOR_ONLY_OUTSIDE[0] <= vs <= REANCHOR_ONLY_OUTSIDE[1])):
                self.soc = approach(self.soc, vs, dt, REANCHOR_TAU_S)

        self.soc = clamp(self.soc)
        return self.soc

    def to_state(self, now_wall):
        return {
            "soc": round(self.soc, 3),
            "saved_wall": now_wall,
            "unplugged_wall": self.unplugged_wall,
            "used_ah_since_unplug": round(self.used_ah, 3),
        }


# ---------------------------------------------------------------------------- persistence
def load_state(logger=None):
    try:
        with open(STATE_FILE) as f:
            s = json.load(f)
        if isinstance(s.get("soc"), (int, float)) and isinstance(s.get("saved_wall"), (int, float)):
            return s
    except FileNotFoundError:
        return None
    except (OSError, ValueError) as e:
        if logger:
            logger.warning(f"ignoring unreadable {STATE_FILE}: {e}")
        return None
    return None


def save_state(state):
    """Atomic write: a power cut mid-save leaves the previous file intact."""
    os.makedirs(STATE_DIR, exist_ok=True)
    tmp = STATE_FILE + ".tmp"
    with open(tmp, "w") as f:
        json.dump(state, f, indent=2)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, STATE_FILE)


# ---------------------------------------------------------------------------- ROS node
class Bridge(Node):
    def __init__(self):
        super().__init__("battery_sysfs_bridge")
        self.saved = load_state(self.get_logger())
        self.gauge = None
        self.v_soc = None           # latest voltage-based %, 0-100
        self.current = 0.0          # latest total current, A
        self.volts = None           # latest battery voltage, V
        self.last_msg = None        # monotonic time of the latest power message
        self.stale = False
        self.last_key = None
        self.warned = set()
        self.w("battery_present", 1)
        q = qos_profile_sensor_data
        self.create_subscription(BatteryState, f"{NS}/bms/state", self.on_bms, q)
        self.create_subscription(Power, f"{NS}/mcu/status/power", self.on_power, q)
        self.create_timer(SAVE_EVERY_S, self.save)
        self.create_timer(SUMMARY_EVERY_S, self.summary)
        self.create_timer(30.0, self.check_stale)
        self.get_logger().info("v3 gauge: counting amp-hours -> /sys/class/power_supply/test_battery")

    # test_power matches these tokens case-insensitively and silently ignores anything else
    def w(self, name, val):
        try:
            with open(f"{PARAMS}/{name}", "w") as f:
                f.write(str(val))
        except OSError as e:
            if name not in self.warned:
                self.warned.add(name)
                self.get_logger().warning(f"cannot write {name}={val}: {e}")

    def on_bms(self, m):
        if math.isfinite(m.percentage) and 0.0 <= m.percentage <= 1.0:
            self.v_soc = m.percentage * 100.0

    def on_power(self, m):
        now_mono, now_wall = time.monotonic(), time.time()
        self.last_msg = now_mono
        if self.stale:
            self.stale = False
            self.last_key = None
            self.get_logger().info("battery data is live again")
        if len(m.measured_currents) and math.isfinite(m.measured_currents[0]):
            self.current = m.measured_currents[0]
        if len(m.measured_voltages) and math.isfinite(m.measured_voltages[0]):
            self.volts = m.measured_voltages[0]
        charger = m.charger_connected == 1

        if self.gauge is None:
            if self.v_soc is None:
                return  # wait for the first voltage-based estimate to start from
            self.gauge = Gauge(self.saved, now_wall, charger, self.v_soc)
            self.get_logger().info(f"gauge start: {self.gauge.origin}")
            self.save()

        was_charging = self.gauge.charger
        soc = self.gauge.update(now_mono, now_wall, self.current, charger,
                                m.charging_complete == 1, self.v_soc)
        if charger != was_charging:
            self.get_logger().info(f"charger {'connected' if charger else 'disconnected'} at {soc:.1f} %")
            self.save()
        self.publish(soc, charger)

    def publish(self, soc, charger):
        cap = int(round(soc))
        status = ("full" if cap >= 100 else "charging") if charger else "discharging"
        ac = "on" if charger else "off"
        volt_uv = int(self.volts * 1e6) if self.volts is not None else None
        cur_ua = int(abs(self.current) * 1e6)
        key = (cap, status, ac, None if volt_uv is None else volt_uv // 100000, cur_ua // 500000)
        if key == self.last_key:
            return
        self.last_key = key
        self.w("battery_capacity", cap)
        self.w("battery_status", status)
        self.w("ac_online", ac)
        if volt_uv is not None:
            self.w("battery_voltage", volt_uv)
        self.w("battery_current", cur_ua)

    def save(self):
        if self.gauge is None:
            return
        now = time.time()
        state = self.gauge.to_state(now)
        state.update({  # for humans reading the file
            "saved_at": datetime.datetime.fromtimestamp(now).isoformat(timespec="seconds"),
            "voltage_based_pct": None if self.v_soc is None else round(self.v_soc, 1),
            "battery_V": None if self.volts is None else round(self.volts, 3),
            "current_A": round(self.current, 3),
            "charger_connected": self.gauge.charger,
            "version": 3,
        })
        try:
            save_state(state)
        except OSError as e:
            if "state" not in self.warned:
                self.warned.add("state")
                self.get_logger().warning(f"cannot save {STATE_FILE}: {e}")

    def summary(self):
        if self.gauge is None:
            return
        g = self.gauge
        vs = "--" if self.v_soc is None else f"{self.v_soc:.0f} %"
        vv = "--" if self.volts is None else f"{self.volts:.2f} V"
        self.get_logger().info(
            f"gauge {g.soc:.1f} % | voltage-based {vs} | {vv} | {self.current:.1f} A | "
            f"charger {'on' if g.charger else 'off'} | used {g.used_ah:.1f} Ah since unplug")

    def check_stale(self):
        if self.last_msg is None or self.stale:
            return
        age = time.monotonic() - self.last_msg
        if age > STALE_AFTER_S:
            self.stale = True
            self.get_logger().warning(f"no power message for {age:.0f}s; marking not-charging")
            self.w("battery_status", "not-charging")


def main():
    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # normal systemd stop/restart -- don't spam the journal with a traceback
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
