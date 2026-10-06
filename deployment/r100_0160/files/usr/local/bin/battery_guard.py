#!/usr/bin/env python3
"""battery-guard: low-battery warning and clean shutdown for the Ridgeback's LiFePO4 packs.

WHY IT EXISTS
  The packs are 2 x DATOUBOSS 12.8 V / 100 Ah LiFePO4 in series, each with its own internal BMS and no
  data interface. When a BMS opens, the robot loses ALL power instantly: no warning, no clean shutdown,
  and it only powers on again with the charger connected. Clearpath's stack has nothing to configure for
  this -- battery_state_estimator exposes no parameters, and "Low Battery" is hardcoded in a compiled
  library -- so this service adds it.

WHAT IT DOES
  Watches the MCU's battery voltage and current (10 Hz), compensates for load
  (V_rest = V + I * R_INT), smooths over SMOOTH_TAU_S, and then:
    - WARNS    when V_rest stays below WARN_V for PERSIST_S
    - SHUTS DOWN cleanly when V_rest stays below SHUTDOWN_V for PERSIST_S
      ...but only when ACTION=shutdown. The default ACTION=warn never powers anything off; it logs
      "would power off now" instead, so the thresholds can be watched before being armed.
  Warnings go to the journal, a desktop pop-up and `wall`. Everything is inhibited while the charger
  is connected. Powering off stops the PC only; the MCU keeps drawing a little.

WHY THESE THRESHOLDS (measured on this robot, not guessed)
  The black-box recorder caught 9 cuts on battery in six days: six at 24.93-25.30 V, one at 23.60 V,
  two at 26.06 / 26.63 V. Every cut was instantaneous. That tight cluster is the signature of one weak
  cell opening its pack's BMS while the pack total still looks healthy. The defaults sit ABOVE the
  cluster so a clean shutdown happens first. R_INT = 0.062 ohm is the median dV/dI measured over 42,088
  current steps in those recordings. (A 24.4 V threshold would have been below 8 of the 9 cuts.)
  NOTE: a cut at 26.0-26.6 V cannot be predicted from pack voltage at all -- that needs the weak cell
  found and fixed with a multimeter.

CONFIG   /etc/battery-guard.conf     (key=value; restart the service after editing)
CHECK    journalctl -u battery-guard -f
ARM IT   set ACTION=shutdown in the config, then: sudo systemctl restart battery-guard
"""
import ast
import math
import os
import subprocess
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from clearpath_platform_msgs.msg import Power

NS = "/r100_0160/platform"
CONF_FILE = os.environ.get("GUARD_CONF", "/etc/battery-guard.conf")
SOC_TABLE = ("/opt/ros/jazzy/lib/python3.12/site-packages/"
             "clearpath_hardware_interfaces/battery_state/battery.py")
SUMMARY_EVERY_S = 300.0

DEFAULTS = {
    "WARN_V": 26.0,        # warn below this load-compensated voltage
    "SHUTDOWN_V": 25.7,    # shut down below this one
    "PERSIST_S": 30.0,     # must stay below for this long (ignores transients)
    "R_INT": 0.062,        # ohm, pack + wiring, measured from the recordings
    "SMOOTH_TAU_S": 60.0,  # smoothing of the compensated voltage
    "WARN_REPEAT_S": 300.0,
    "ACTION": "warn",      # "warn" (log + pop-up only) or "shutdown" (also power off)
}


def read_config(path=CONF_FILE):
    """key=value file; unknown keys ignored, bad values fall back to the default."""
    cfg = dict(DEFAULTS)
    try:
        for line in open(path):
            line = line.split("#", 1)[0].strip()
            if "=" not in line:
                continue
            k, v = (x.strip() for x in line.split("=", 1))
            if k not in DEFAULTS:
                continue
            cfg[k] = v if isinstance(DEFAULTS[k], str) else float(v)
    except (OSError, ValueError):
        pass
    return cfg


def read_soc_table(path=SOC_TABLE):
    """Return the DTM8A31 voltage->SOC table from the installed Clearpath file, or None."""
    try:
        tree = ast.parse(open(path).read())
    except (OSError, SyntaxError):
        return None
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == "DTM8A31":
            for item in node.body:
                if (isinstance(item, ast.Assign) and getattr(item.targets[0], "id", "") == "LUT"):
                    try:
                        return ast.literal_eval(item.value)
                    except ValueError:
                        return None
    return None


class Guard:
    """Threshold logic. Pure (no ROS), so it can be tested offline."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.v = None            # smoothed, load-compensated voltage
        self.below_warn = None   # time it first went below WARN_V
        self.below_shut = None
        self.last_warn = None
        self.fired = False       # shutdown already reported
        self.last_t = None

    def update(self, now, volts, amps, charger):
        """Returns None, "warn" or "shutdown" (the latter at most once)."""
        if volts is None or not math.isfinite(volts):
            return None
        amps = amps if (amps is not None and math.isfinite(amps)) else 0.0
        rest = volts + amps * self.cfg["R_INT"]
        dt = 0.0 if self.last_t is None else max(0.0, now - self.last_t)
        self.last_t = now
        if self.v is None:
            self.v = rest
        elif dt > 0:
            self.v += (rest - self.v) * (1.0 - math.exp(-dt / self.cfg["SMOOTH_TAU_S"]))

        if charger:  # charging or plugged in: nothing to warn about
            self.below_warn = self.below_shut = None
            self.fired = False
            return None

        if self.v < self.cfg["SHUTDOWN_V"]:
            self.below_shut = self.below_shut if self.below_shut is not None else now
            if now - self.below_shut >= self.cfg["PERSIST_S"] and not self.fired:
                self.fired = True
                return "shutdown"
        else:
            self.below_shut = None

        if self.v < self.cfg["WARN_V"]:
            self.below_warn = self.below_warn if self.below_warn is not None else now
            if now - self.below_warn >= self.cfg["PERSIST_S"] and (
                    self.last_warn is None or now - self.last_warn >= self.cfg["WARN_REPEAT_S"]):
                self.last_warn = now
                return "warn"
        else:
            self.below_warn = None
        return None


def desktop_env():
    """DISPLAY / DBUS address of the logged-in desktop session (both vary per session)."""
    for name in ("xfce4-power-manager", "xfce4-session", "xfce4-panel"):
        try:
            pids = subprocess.run(["pgrep", "-u", "robot", "-f", name], capture_output=True,
                                  text=True, timeout=5).stdout.split()
        except (OSError, subprocess.SubprocessError):
            continue
        for pid in pids:
            try:
                env = dict(
                    kv.split("=", 1) for kv in
                    open(f"/proc/{pid}/environ").read().split("\0") if "=" in kv)
            except OSError:
                continue
            if "DISPLAY" in env and "DBUS_SESSION_BUS_ADDRESS" in env:
                return {k: env[k] for k in ("DISPLAY", "DBUS_SESSION_BUS_ADDRESS", "XAUTHORITY")
                        if k in env}
    return None


def notify(title, body, logger):
    """Desktop pop-up plus wall. Never fatal -- the journal is the reliable channel."""
    env = desktop_env()
    if env:
        try:
            subprocess.run(["runuser", "-u", "robot", "--", "notify-send", "-u", "critical",
                            "-i", "battery-caution", title, body],
                           env={**os.environ, **env}, timeout=10, check=False)
        except (OSError, subprocess.SubprocessError) as e:
            logger.warning(f"desktop notification failed: {e}")
    else:
        logger.info("no desktop session found for the pop-up")
    try:
        subprocess.run(["wall", f"{title}: {body}"], timeout=10, check=False)
    except (OSError, subprocess.SubprocessError):
        pass


class GuardNode(Node):
    def __init__(self):
        super().__init__("battery_guard")
        self.cfg = read_config()
        self.guard = Guard(self.cfg)
        self.volts = self.amps = None
        self.charger = False
        self.create_subscription(Power, f"{NS}/mcu/status/power", self.on_power, qos_profile_sensor_data)
        self.create_timer(SUMMARY_EVERY_S, self.summary)
        self.get_logger().info(
            f"guard active: warn<{self.cfg['WARN_V']} V, shutdown<{self.cfg['SHUTDOWN_V']} V "
            f"(load-compensated, R={self.cfg['R_INT']} ohm), hold {self.cfg['PERSIST_S']:.0f}s, "
            f"ACTION={self.cfg['ACTION']}")
        self.check_soc_table()

    def check_soc_table(self):
        """Catch an apt upgrade having restored the lead-acid table."""
        lut = read_soc_table()
        if not lut:
            self.get_logger().warning(f"could not read the SOC table from {SOC_TABLE}")
        elif max(v for v, _ in lut) < 26.0:
            self.get_logger().warning(
                "SOC TABLE IS THE LEAD-ACID ONE (top entry %.1f V): the battery percentage will read "
                "100%% until the BMS cuts. Fix: sudo bash ~/Downloads/power_debug/lifepo4_lut.sh apply"
                % max(v for v, _ in lut))
        else:
            self.get_logger().info(f"SOC table looks like LiFePO4 (full at {max(v for v, _ in lut)} V)")

    def on_power(self, m):
        if len(m.measured_voltages):
            self.volts = m.measured_voltages[0]
        if len(m.measured_currents):
            self.amps = m.measured_currents[0]
        self.charger = m.charger_connected == 1
        level = self.guard.update(time.monotonic(), self.volts, self.amps, self.charger)
        if level == "warn":
            msg = (f"Battery low: {self.guard.v:.2f} V (compensated), below "
                   f"{self.cfg['WARN_V']} V. Put the robot on the charger.")
            self.get_logger().warning(msg)
            notify("Robot battery low", msg, self.get_logger())
        elif level == "shutdown":
            msg = (f"Battery critical: {self.guard.v:.2f} V (compensated), below "
                   f"{self.cfg['SHUTDOWN_V']} V.")
            if self.cfg["ACTION"] == "shutdown":
                self.get_logger().error(msg + " Shutting down cleanly now.")
                notify("Robot battery critical", msg + " Shutting down now.", self.get_logger())
                subprocess.run(["systemctl", "poweroff"], check=False)
            else:
                self.get_logger().error(msg + " WOULD POWER OFF NOW (ACTION=warn, so not doing it). "
                                              "Set ACTION=shutdown in /etc/battery-guard.conf to arm.")
                notify("Robot battery critical", msg + " (guard is warn-only)", self.get_logger())

    def summary(self):
        if self.volts is None:
            self.get_logger().info("waiting for MCU power messages")
            return
        self.get_logger().info(
            f"battery {self.volts:.2f} V | {self.amps or 0:.1f} A | compensated {self.guard.v:.2f} V | "
            f"charger {'on' if self.charger else 'off'} | warn<{self.cfg['WARN_V']} "
            f"shutdown<{self.cfg['SHUTDOWN_V']} | ACTION={self.cfg['ACTION']}")


def main():
    rclpy.init()
    node = GuardNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
