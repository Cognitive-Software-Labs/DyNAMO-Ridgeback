#!/usr/bin/env python3
"""Read the power black box recordings.

  power_report.py              the last minute before the PREVIOUS boot ended
  power_report.py FILE         the same, for a specific recording
  power_report.py --list       every recording: start, end, final voltage, clean stop or cut
  power_report.py --live       live readout at 5 Hz (multimeter comparison, wiggle test)
"""
import argparse
import csv
import glob
import io
import os
import sys
import time

OUT_DIR = os.environ.get("POWER_BLACKBOX_DIR", "/home/robot/power_blackbox")
NS = "/r100_0160/platform"
RAILS = ["batt_V", "v5_V", "v12_V", "inverter_V", "front_axle_V", "rear_axle_V", "light_V"]
WHEELS = ["front_left", "front_right", "rear_left", "rear_right"]
BUSES = [f"bus_{w}_V" for w in WHEELS]
CLEAN = "recorder stopped cleanly"


def num(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def boot_time():
    with open("/proc/stat") as f:
        for line in f:
            if line.startswith("btime"):
                return float(line.split()[1])
    return time.time()


def recordings():
    return sorted(glob.glob(os.path.join(OUT_DIR, "*.csv")))


def load(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    # The last line may be half-written if the power died mid-write; drop anything unparseable.
    return [r for r in rows if num(r.get("mono_s")) is not None]


def t(r):
    return num(r["mono_s"])


def last_with(rows, col, before=None):
    for r in reversed(rows):
        if before is not None and t(r) > before:
            continue
        if num(r.get(col)) is not None:
            return r
    return None


def dur(s):
    s = int(s)
    return f"{s // 3600}h{s % 3600 // 60:02d}m{s % 60:02d}s"


def v(r, col, w=6, d=2):
    x = num(r.get(col)) if r else None
    return f"{x:{w}.{d}f}" if x is not None else " " * (w - 2) + "--"


def report(path, window):
    rows = load(path)
    if not rows:
        print(f"{path}: no data rows")
        return
    first, last = rows[0], rows[-1]
    t_end = t(last)
    clean = any(CLEAN in (r.get("event") or "") for r in rows[-3:])
    print(f"Recording : {path}")
    print(f"Started   : {first['wall_time']}")
    print(f"Ended     : {last['wall_time']}   (ran {dur(t_end - t(first))})")
    print(f"How       : {'CLEAN STOP (shutdown/reboot/service stop)' if clean else 'RECORDING JUST ENDS -> power was cut'}")

    print("\nBattery voltage leading up to the end:")
    for label, back in [("-30 min", 1800), ("-10 min", 600), ("-5 min", 300), ("-1 min", 60),
                        ("-10 s", 10), ("-1 s", 1), ("last", 0)]:
        if t_end - back < t(first):
            continue
        r = last_with(rows, "batt_V", before=t_end - back)
        if r:
            print(f"  {label:>8}  {r['wall_time'][11:23]}  batt {v(r, 'batt_V')} V  "
                  f"axles {v(r, 'front_axle_V')}/{v(r, 'rear_axle_V')} V  current {v(r, 'total_A')} A")

    win = [r for r in rows if t(r) >= t_end - window]
    print(f"\nMin / max over the last {window:.0f} s:")
    for col in RAILS + ["total_A"] + BUSES:
        vals = [num(r.get(col)) for r in win if num(r.get(col)) is not None]
        if vals:
            print(f"  {col:18} {min(vals):7.2f} .. {max(vals):7.2f}")

    ev = [r for r in rows if r.get("event") and t(r) >= t_end - 600]
    print("\nEvents in the last 10 min:" if ev else "\nEvents in the last 10 min: none")
    for r in ev[-40:]:
        print(f"  {r['wall_time'][11:23]}  {r['event']}")

    print("\nLast 30 rows:")
    print("  time          batt    inv  axleF  axleR    12V     5V   amps  minbus estop rst  event")
    for r in rows[-30:]:
        buses = [num(r.get(b)) for b in BUSES if num(r.get(b)) is not None]
        mb = f"{min(buses):6.2f}" if buses else "    --"
        print(f"  {r['wall_time'][11:23]} {v(r, 'batt_V')} {v(r, 'inverter_V')} {v(r, 'front_axle_V')} "
              f"{v(r, 'rear_axle_V')} {v(r, 'v12_V')} {v(r, 'v5_V')} {v(r, 'total_A')} {mb} "
              f"{(r.get('estop') or '-'):>5} {(r.get('needs_reset') or '-'):>3}  {(r.get('event') or '')[:60]}")

    print("\nFirst-pass reading (check it against the numbers above):")
    for line in hints(rows, clean):
        print(f"  - {line}")


def hints(rows, clean):
    if clean:
        return ["This recording ended with a clean stop, so it is not a power cut. Pick an earlier one with --list."]
    out = []
    t_end = t(rows[-1])
    end = last_with(rows, "batt_V")
    if end is None:
        return ["No battery readings at all -- was the Clearpath stack running?"]
    ve = num(end["batt_V"])
    r10s, r10m = last_with(rows, "batt_V", t_end - 10), last_with(rows, "batt_V", t_end - 600)
    v10s = num(r10s["batt_V"]) if r10s else None
    v10m = num(r10m["batt_V"]) if r10m else None
    if v10s is not None and v10s - ve > 1.0:
        out.append(f"Battery fell {v10s - ve:.1f} V in the last 10 s ({v10s:.2f} -> {ve:.2f} V): a collapse right "
                   "before the cut. Suspect a high-resistance joint, a failing battery, or a load spike.")
    elif v10m is not None and v10m - ve > 1.0:
        out.append(f"Battery slid {v10m - ve:.1f} V over the last 10 min ({v10m:.2f} -> {ve:.2f} V): looks like "
                   "real depletion or a weak battery.")
    elif ve >= 23.5:
        out.append(f"Battery was healthy ({ve:.2f} V) right up to the last sample: the power vanished instantly. "
                   "That is not an empty battery -- suspect an open circuit (connector, fuse holder, switch, "
                   "contactor) or a power-board cutoff.")
    else:
        out.append(f"Battery was low ({ve:.2f} V) at the end without a sharp drop: check the batteries (D3/D7).")
    tail = " ".join(r.get("event") or "" for r in rows if t(r) >= t_end - 30)
    if "no power msg" in tail:
        out.append("The MCU stopped reporting before the PC died: the MCU/power board went down first.")
    if any(k in tail for k in ("needs_reset", "stop_power_status", "estop")):
        out.append("Stop/e-stop state changed in the last 30 s: look at the e-stop loop and contactor wiring.")
    if any(k in tail for k in ("battery_connected", "charger_connected")):
        out.append("battery_connected/charger_connected flipped in the last 30 s: look at the battery and "
                   "charge-port connections.")
    return out


def tail_row(path, header):
    with open(path, "rb") as f:
        f.seek(0, os.SEEK_END)
        f.seek(max(0, f.tell() - 8192))
        lines = f.read().decode(errors="replace").splitlines()[1:]
    good = [r for r in csv.DictReader(io.StringIO("\n".join([header] + lines)))
            if num(r.get("mono_s")) is not None]
    return good


def list_recordings():
    files = recordings()
    if not files:
        print(f"no recordings in {OUT_DIR}")
        return
    bt = boot_time()
    print(f"{'recording':24} {'started':19} {'ended':19} {'ran':>10} {'batt@end':>8}  how")
    for p in files:
        with open(p, newline="") as f:
            header = f.readline().strip()
            first = next(csv.DictReader(io.StringIO(header + "\n" + f.readline())), None)
        tail = tail_row(p, header)
        if not first or not tail:
            print(f"{os.path.basename(p):24} (empty)")
            continue
        last = tail[-1]
        vb = last_with(tail, "batt_V")
        clean = any(CLEAN in (r.get("event") or "") for r in tail[-3:])
        if clean:
            how = "clean stop"
        elif os.path.getmtime(p) >= bt:
            how = "current boot (still recording)"
        else:
            how = "*** POWER CUT ***"
        print(f"{os.path.basename(p):24} {first['wall_time'][:19]} {last['wall_time'][:19]} "
              f"{dur(t(last) - t(first)):>10} {v(vb, 'batt_V', 8):>8}  {how}")


def live():
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data as q
    from clearpath_motor_msgs.msg import PumaMultiStatus
    from clearpath_platform_msgs.msg import Power, StopStatus
    from std_msgs.msg import Bool

    s = {"prev": None}

    def on_power(m):
        s["v"] = list(m.measured_voltages)
        s["a"] = m.measured_currents[0] if len(m.measured_currents) else None
        s["chg"] = m.charger_connected

    def on_status(m):
        s["bus"] = [d.bus_voltage for d in m.drivers]

    def show():
        vs = s.get("v") or []
        g = lambda i: f"{vs[i]:6.2f}" if i < len(vs) else "    --"
        bus = " ".join(f"{b:5.2f}" for b in s.get("bus") or []) or "--"
        a = s.get("a")
        line = (f"{time.strftime('%H:%M:%S')}  batt {g(0)} | inv {g(3)} | axle F {g(4)} R {g(5)} | "
                f"12V {g(2)} | drivers {bus} | {a if a is None else f'{a:5.2f}'} A | "
                f"estop {s.get('estop', '-')} | reset {s.get('rst', '-')} | charger {s.get('chg', '-')}")
        if vs and s["prev"] is not None and s["prev"] - vs[0] > 0.3:
            line += f"   <<< DIP {s['prev'] - vs[0]:.2f} V"
        s["prev"] = vs[0] if vs else None
        print(line, flush=True)

    rclpy.init()
    n = Node("power_report_live")
    n.create_subscription(Power, f"{NS}/mcu/status/power", on_power, q)
    n.create_subscription(PumaMultiStatus, f"{NS}/motors/status", on_status, q)
    n.create_subscription(Bool, f"{NS}/emergency_stop", lambda m: s.update(estop=int(m.data)), q)
    n.create_subscription(StopStatus, f"{NS}/mcu/status/stop", lambda m: s.update(rst=int(m.needs_reset)), q)
    n.create_timer(0.2, show)
    print("live power readout -- Ctrl+C to stop. '<<< DIP' marks a battery drop > 0.3 V between lines.")
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file", nargs="?", help="recording to analyse (default: the previous boot's)")
    ap.add_argument("--list", action="store_true", help="list all recordings")
    ap.add_argument("--live", action="store_true", help="live readout")
    ap.add_argument("--window", type=float, default=60.0, help="seconds before the end to summarise (default 60)")
    a = ap.parse_args()
    if a.live:
        return live()
    if a.list:
        return list_recordings()
    path = a.file
    if path is None:
        bt = boot_time()
        older = [p for p in recordings() if os.path.getmtime(p) < bt]
        if not older:
            sys.exit(f"no recording from a previous boot in {OUT_DIR} yet (see --list)")
        path = older[-1]
    report(path, a.window)


if __name__ == "__main__":
    main()
