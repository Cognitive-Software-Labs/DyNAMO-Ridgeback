#!/usr/bin/env python3
"""Validate Gazebo chassis contact and recovery against a known wall.

Launch the ``target_distance_calibration`` Gazebo backend and bridge the model
pose before running this tool.  The validator drives the production mecanum
controller, resets the robot through Gazebo's ``set_pose`` service, and uses
Gazebo ground truth rather than wheel odometry for every measurement.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import statistics
import subprocess
import time

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


WALL_FACE_X = 11.9
START_CLEARANCE = 0.20
APPROACH_TIMEOUT_S = 6.0
SETTLE_S = 1.0
HOLD_S = 2.0
ZERO_S = 0.75
REVERSE_S = 1.0
FINAL_SETTLE_S = 0.75
REVERSE_SPEED = 0.10

STOP_ERROR_LIMIT = 0.010
PENETRATION_LIMIT = 0.005
HOLD_OSCILLATION_LIMIT = 0.005
TANGENT_DRIFT_LIMIT = 0.005
YAW_DRIFT_LIMIT = math.radians(0.5)
HOLD_SPEED_LIMIT = 0.02
RECOVERY_END_LIMIT_S = 0.25
RECOVERY_CLEARANCE = 0.080


@dataclass(frozen=True)
class CaseSpec:
    orientation: str
    yaw: float
    speed: float

    @property
    def name(self) -> str:
        return f"gazebo_{self.orientation}_{self.speed:.2f}"


@dataclass
class CaseResult:
    name: str
    orientation: str
    yaw_deg: float
    speed: float
    expected_support: float
    expected_stop_x: float
    settled_stop_x: float
    settled_gap: float
    max_penetration: float
    hold_oscillation: float
    tangent_drift: float
    yaw_drift_deg: float
    max_hold_speed: float
    recovery_end_s: float | None
    recovery_clearance: float
    checks: dict[str, bool]
    passed: bool


def parse_speed_list(value: str) -> list[float]:
    values = [float(part.strip()) for part in value.split(",") if part.strip()]
    if not values or any(value <= 0 for value in values):
        raise argparse.ArgumentTypeError("speeds must be positive comma-separated values")
    return values


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--geometry-json", type=Path, required=True,
                        help="measurements.json from compare_collision_envelopes.py")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--world", default="target_distance_calibration")
    parser.add_argument("--robot", default="r100_0001/robot")
    parser.add_argument("--namespace", default="r100_0001")
    parser.add_argument("--speeds", type=parse_speed_list,
                        default=parse_speed_list("0.05,0.10,0.20"))
    return parser.parse_args()


def rotate_xy(x: float, y: float, yaw: float) -> tuple[float, float]:
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return cosine * x - sine * y, sine * x + cosine * y


def support_x(points: list[list[float]], yaw: float) -> float:
    return max(rotate_xy(float(x), float(y), yaw)[0] for x, y in points)


def angle_error(left: float, right: float) -> float:
    return math.atan2(math.sin(left - right), math.cos(left - right))


class GroundTruthDriver(Node):
    def __init__(self, robot: str, namespace: str) -> None:
        super().__init__("gazebo_chassis_contact_validator")
        self.robot = robot
        self.pose: dict | None = None
        self.pose_sequence = 0
        self.publisher = self.create_publisher(
            TwistStamped, f"/{namespace}/platform/cmd_vel", 10
        )
        self.subscription = self.create_subscription(
            TFMessage, f"/model/{robot}/pose", self._pose_callback, 10
        )

    def _pose_callback(self, message: TFMessage) -> None:
        for transform in message.transforms:
            if transform.child_frame_id != self.robot:
                continue
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            yaw = math.atan2(
                2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
            )
            stamp = transform.header.stamp
            self.pose = {
                "sim_time": stamp.sec + stamp.nanosec * 1e-9,
                "x": translation.x,
                "y": translation.y,
                "z": translation.z,
                "yaw": yaw,
            }
            self.pose_sequence += 1
            return

    def command_world(self, vx: float, vy: float, yaw: float) -> None:
        body_x, body_y = rotate_xy(vx, vy, -yaw)
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.twist.linear.x = body_x
        message.twist.linear.y = body_y
        self.publisher.publish(message)

    def wait_for_pose(self, timeout: float = 5.0) -> dict:
        deadline = time.monotonic() + timeout
        initial = self.pose_sequence
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pose is not None and self.pose_sequence > initial:
                return dict(self.pose)
        raise RuntimeError("timed out waiting for Gazebo ground-truth pose")


def set_pose(world: str, robot: str, x: float, yaw: float) -> None:
    request = (
        f'name: "{robot}", position: {{x: {x:.12g}, y: 0.0, z: 0.3}}, '
        f'orientation: {{x: 0.0, y: 0.0, z: {math.sin(yaw / 2):.12g}, '
        f'w: {math.cos(yaw / 2):.12g}}}'
    )
    completed = subprocess.run(
        ["gz", "service", "-s", f"/world/{world}/set_pose",
         "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
         "--timeout", "2000", "--req", request],
        capture_output=True, text=True, timeout=10, check=False,
    )
    if completed.returncode or "data: true" not in completed.stdout:
        raise RuntimeError(
            f"Gazebo set_pose failed ({completed.returncode}): "
            f"{completed.stdout}{completed.stderr}"
        )


def sample_phase(node: GroundTruthDriver, spec: CaseSpec, phase: str,
                 duration: float, vx_world: float, support: float,
                 rows: list[dict]) -> None:
    started = time.monotonic()
    previous = rows[-1] if rows else None
    while time.monotonic() - started < duration:
        node.command_world(vx_world, 0.0, spec.yaw)
        pose = node.wait_for_pose(timeout=1.0)
        phase_time = time.monotonic() - started
        vx = vy = wz = 0.0
        if previous is not None:
            delta = pose["sim_time"] - previous["sim_time"]
            if delta > 1e-6:
                vx = (pose["x"] - previous["x"]) / delta
                vy = (pose["y"] - previous["y"]) / delta
                wz = angle_error(pose["yaw"], previous["yaw"]) / delta
        row = {
            "case": spec.name,
            "phase": phase,
            "phase_time": phase_time,
            **pose,
            "vx": vx,
            "vy": vy,
            "wz": wz,
            "gap": WALL_FACE_X - (pose["x"] + support),
        }
        rows.append(row)
        previous = row


def sample_approach(node: GroundTruthDriver, spec: CaseSpec, support: float,
                    rows: list[dict]) -> None:
    started = time.monotonic()
    while time.monotonic() - started < APPROACH_TIMEOUT_S:
        sample_phase(node, spec, "approach", 0.05, spec.speed, support, rows)
        if rows[-1]["gap"] <= 0.008:
            return
    raise RuntimeError(f"{spec.name}: did not reach wall within approach timeout")


def evaluate(spec: CaseSpec, support: float, rows: list[dict]) -> CaseResult:
    hold = [row for row in rows if row["phase"] == "hold"]
    cutoff = hold[-1]["phase_time"] - 1.0 if hold else math.inf
    settled = [row for row in hold if row["phase_time"] >= cutoff]
    reverse = [row for row in rows if row["phase"] == "reverse"]
    median = lambda values: statistics.median(values) if values else math.nan
    settled_gap = median([row["gap"] for row in settled])
    settled_x = median([row["x"] for row in settled])
    penetration = max(0.0, -min((row["gap"] for row in rows), default=math.nan))
    oscillation = (max(row["gap"] for row in settled)
                   - min(row["gap"] for row in settled)) if settled else math.nan
    tangent = (max(row["y"] for row in settled)
               - min(row["y"] for row in settled)) if settled else math.nan
    yaw_drift = (max(angle_error(row["yaw"], spec.yaw) for row in settled)
                 - min(angle_error(row["yaw"], spec.yaw) for row in settled)) \
        if settled else math.nan
    max_speed = max((math.hypot(row["vx"], row["vy"]) for row in settled),
                    default=math.nan)
    recovery_end = None
    for row in reverse:
        if row["gap"] >= settled_gap + 0.005:
            recovery_end = row["phase_time"]
            break
    recovery_clearance = (reverse[-1]["gap"] - settled_gap) if reverse else math.nan
    finite = [value for row in rows for value in
              (row["x"], row["y"], row["yaw"], row["gap"])]
    checks = {
        "samples_present": bool(settled and reverse),
        "stop_error": math.isfinite(settled_gap)
                      and abs(settled_gap) <= STOP_ERROR_LIMIT,
        "penetration": math.isfinite(penetration)
                       and penetration <= PENETRATION_LIMIT,
        "hold_oscillation": math.isfinite(oscillation)
                            and oscillation <= HOLD_OSCILLATION_LIMIT,
        "tangent_drift": math.isfinite(tangent)
                         and tangent <= TANGENT_DRIFT_LIMIT,
        "yaw_drift": math.isfinite(yaw_drift)
                     and yaw_drift <= YAW_DRIFT_LIMIT,
        "hold_speed": math.isfinite(max_speed) and max_speed <= HOLD_SPEED_LIMIT,
        "recovery_end": recovery_end is not None
                        and recovery_end <= RECOVERY_END_LIMIT_S,
        "recovery_clearance": math.isfinite(recovery_clearance)
                              and recovery_clearance >= RECOVERY_CLEARANCE,
        "finite": bool(finite) and all(math.isfinite(value) for value in finite),
    }
    return CaseResult(
        name=spec.name, orientation=spec.orientation,
        yaw_deg=math.degrees(spec.yaw), speed=spec.speed,
        expected_support=support, expected_stop_x=WALL_FACE_X - support,
        settled_stop_x=settled_x, settled_gap=settled_gap,
        max_penetration=penetration, hold_oscillation=oscillation,
        tangent_drift=tangent, yaw_drift_deg=math.degrees(yaw_drift),
        max_hold_speed=max_speed, recovery_end_s=recovery_end,
        recovery_clearance=recovery_clearance, checks=checks,
        passed=all(checks.values()),
    )


def write_trace_svg(path: Path, rows: list[dict]) -> None:
    width, height = 1050, 390
    pieces = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">',
              '<rect width="100%" height="100%" fill="#10151f"/>',
              '<style>text{font-family:system-ui,sans-serif;fill:#eaf0f7}</style>',
              '<text x="24" y="30" font-size="20">Gazebo chassis contact traces</text>']
    colors = {"front": "#58d6ff", "angle": "#f6c85f", "side": "#b88cff"}
    for panel, (field, label, low, high) in enumerate((
        ("gap", "signed separation (m)", -0.01, 0.03),
        ("y", "lateral position (m)", -0.03, 0.03),
        ("yaw", "yaw error (deg)", -2.0, 2.0),
    )):
        left, top, panel_width, panel_height = 20 + panel * 345, 50, 325, 310
        pieces.append(f'<rect x="{left}" y="{top}" width="{panel_width}" height="{panel_height}" rx="10" fill="#182231" stroke="#33445c"/>')
        pieces.append(f'<text x="{left + 12}" y="{top + 24}" font-size="14">{label}</text>')
        for orientation in colors:
            selected = [row for row in rows if row["case"] == f"gazebo_{orientation}_0.10"]
            if not selected:
                continue
            start = selected[0]["sim_time"]
            duration = max(selected[-1]["sim_time"] - start, 1e-6)
            points = []
            yaw0 = math.radians({"front": 0, "angle": 45, "side": 90}[orientation])
            for row in selected:
                value = row[field]
                if field == "y":
                    value -= selected[0]["y"]
                elif field == "yaw":
                    value = math.degrees(angle_error(value, yaw0))
                px = left + 12 + (row["sim_time"] - start) / duration * (panel_width - 24)
                py = top + 34 + (high - value) / (high - low) * (panel_height - 48)
                points.append(f"{px:.1f},{py:.1f}")
            pieces.append(f'<polyline points="{" ".join(points)}" fill="none" stroke="{colors[orientation]}" stroke-width="1.8"/>')
    pieces.extend(['<text x="24" y="382" font-size="13" fill="#58d6ff">front</text>',
                   '<text x="90" y="382" font-size="13" fill="#f6c85f">45°</text>',
                   '<text x="140" y="382" font-size="13" fill="#b88cff">side</text>', '</svg>'])
    path.write_text("\n".join(pieces) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    geometry = json.loads(args.geometry_json.read_text(encoding="utf-8"))
    hull = geometry.get("gazebo_collision_hull")
    if not hull:
        raise SystemExit("geometry JSON lacks gazebo_collision_hull; regenerate it")
    args.output_dir.mkdir(parents=True, exist_ok=False)
    specs = [
        CaseSpec(name, yaw, speed)
        for name, yaw in (("front", 0.0), ("angle", math.pi / 4),
                          ("side", math.pi / 2))
        for speed in args.speeds
    ]
    rclpy.init()
    node = GroundTruthDriver(args.robot, args.namespace)
    all_rows: list[dict] = []
    results: list[CaseResult] = []
    try:
        node.wait_for_pose()
        for spec in specs:
            support = support_x(hull, spec.yaw)
            start_x = WALL_FACE_X - support - START_CLEARANCE
            for _ in range(10):
                node.command_world(0.0, 0.0, spec.yaw)
                rclpy.spin_once(node, timeout_sec=0.05)
            set_pose(args.world, args.robot, start_x, spec.yaw)
            rows: list[dict] = []
            sample_phase(node, spec, "settle", SETTLE_S, 0.0, support, rows)
            sample_approach(node, spec, support, rows)
            sample_phase(node, spec, "hold", HOLD_S, spec.speed, support, rows)
            sample_phase(node, spec, "zero", ZERO_S, 0.0, support, rows)
            sample_phase(node, spec, "reverse", REVERSE_S, -REVERSE_SPEED,
                         support, rows)
            sample_phase(node, spec, "final_settle", FINAL_SETTLE_S, 0.0,
                         support, rows)
            result = evaluate(spec, support, rows)
            results.append(result)
            all_rows.extend(rows)
            status = "PASS" if result.passed else "FAIL"
            print(f"[{status}] {spec.name}: gap={result.settled_gap:+.4f} m "
                  f"penetration={result.max_penetration:.4f} m")
    finally:
        for _ in range(10):
            node.command_world(0.0, 0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        rclpy.shutdown()

    with (args.output_dir / "samples.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(all_rows[0]))
        writer.writeheader()
        writer.writerows(all_rows)
    summary = {
        "status": "PASS" if all(result.passed for result in results) else "FAIL",
        "passed": all(result.passed for result in results),
        "wall_face_x": WALL_FACE_X,
        "geometry_source": str(args.geometry_json.resolve()),
        "cases": [asdict(result) for result in results],
    }
    (args.output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    write_trace_svg(args.output_dir / "traces.svg", all_rows)
    print(f"GAZEBO CONTACT VALIDATION {summary['status']}")
    print(f"evidence: {args.output_dir.resolve()}")
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
