#!/usr/bin/env python3
"""Validate the Ridgeback chassis collider against a controlled wall.

The parent process starts one Isaac child process per requested boot, then
aggregates the evidence.  Each child composes the production robot USD into a
minimal stage and exercises three non-persistent collision configurations:

* ``hull_only`` -- the vendor chassis hull, with companion robot colliders off;
* ``full_robot`` -- every collider authored by the production robot asset;
* ``aabb_control`` -- the retired AABB, proving the harness distinguishes it.

No repository asset is modified.  All overrides live on the in-memory stage.

Run from a sourced workspace:

    OMNI_KIT_ACCEPT_EULA=YES isaac_venv/bin/python3 \
        tools/isaac/validate_chassis_contacts.py
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import subprocess
import sys
import traceback
from typing import Iterable, Sequence


REPO = Path(__file__).resolve().parents[2]
SIM_DIR = REPO / "src/ridgeback_autonomy_isaac/sim/isaac"
DEFAULT_ROBOT = (
    SIM_DIR / "usd/robots/ridgeback_r100/ridgeback_r100.usda"
)
DEFAULT_ARTIFACT_ROOT = REPO / "artifacts/isaac-collider-validation"
VALIDATION_SUBSTRATE = (
    "tools/isaac/import_ridgeback_urdf.py",
    "src/ridgeback_autonomy_isaac/sim/isaac/isaac_runner.py",
    "src/ridgeback_autonomy_isaac/sim/isaac/robot_rig.py",
    "src/ridgeback_autonomy_isaac/sim/isaac/usd/robots/ridgeback_r100",
)

WALL_FACE_X = 1.0
START_CLEARANCE = 0.20
APPROACH_TIMEOUT_S = 5.0
SETTLE_S = 0.5
HOLD_S = 2.0
ZERO_S = 1.0
REVERSE_S = 1.0
FINAL_SETTLE_S = 1.0
REVERSE_SPEED = 0.10

STOP_ERROR_LIMIT = 0.010
PENETRATION_LIMIT = 0.005
HOLD_OSCILLATION_LIMIT = 0.005
TANGENT_DRIFT_LIMIT = 0.005
YAW_DRIFT_LIMIT = math.radians(0.5)
HOLD_SPEED_LIMIT = 0.02
CONTACT_NORMAL_LIMIT = math.cos(math.radians(15.0))
RECOVERY_END_LIMIT_S = 0.25
RECOVERY_CLEARANCE = 0.080
BOOT_SPREAD_LIMIT = 0.003
AABB_DELTA_ERROR_LIMIT = 0.010


@dataclass(frozen=True)
class CaseSpec:
    configuration: str
    orientation: str
    yaw: float
    speed: float

    @property
    def name(self) -> str:
        return f"{self.configuration}_{self.orientation}_{self.speed:.2f}"


@dataclass
class ContactEvidence:
    sim_time: float
    paths: list[str]
    normal: tuple[float, float, float] | None
    impulse: float | None


@dataclass
class CaseResult:
    name: str
    configuration: str
    orientation: str
    yaw_deg: float
    speed: float
    expected_support: float
    expected_stop_x: float
    settled_stop_x: float
    settled_gap: float
    visual_gap: float | None
    max_penetration: float
    hold_oscillation: float
    tangent_drift: float
    yaw_drift_deg: float
    max_hold_speed: float
    contact_events: int
    contact_normal_alignment: float | None
    recovery_end_s: float | None
    recovery_clearance: float
    max_step_translation: float
    checks: dict[str, bool]
    passed: bool


def parse_speed_list(value: str) -> list[float]:
    values = [float(part.strip()) for part in value.split(",") if part.strip()]
    if not values or any(v <= 0.0 for v in values):
        raise argparse.ArgumentTypeError("speeds must be positive comma-separated values")
    return values


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--robot-usd", type=Path, default=DEFAULT_ROBOT)
    ap.add_argument("--output-dir", type=Path, default=None)
    ap.add_argument("--boots", type=int, default=3)
    ap.add_argument("--speeds", type=parse_speed_list,
                    default=parse_speed_list("0.05,0.10,0.20"))
    ap.add_argument("--physics-hz", type=float, default=120.0)
    ap.add_argument("--child", action="store_true", help=argparse.SUPPRESS)
    ap.add_argument("--boot-index", type=int, default=1, help=argparse.SUPPRESS)
    args = ap.parse_args()
    if args.boots < 1:
        ap.error("--boots must be at least 1")
    if args.physics_hz <= 0:
        ap.error("--physics-hz must be positive")
    args.robot_usd = args.robot_usd.resolve()
    if not args.robot_usd.exists():
        ap.error(f"robot USD does not exist: {args.robot_usd}")
    return args


def rotate_xy(point: Sequence[float], yaw: float) -> tuple[float, float]:
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * float(point[0]) - s * float(point[1]),
            s * float(point[0]) + c * float(point[1]))


def body_to_world(vx: float, vy: float, yaw: float) -> tuple[float, float]:
    return rotate_xy((vx, vy), yaw)


def world_to_body(vx: float, vy: float, yaw: float) -> tuple[float, float]:
    return rotate_xy((vx, vy), -yaw)


def support_xy(points: Sequence[Sequence[float]], yaw: float,
               normal: tuple[float, float] = (1.0, 0.0)) -> float:
    if not points:
        raise ValueError("support requires at least one point")
    return max(
        normal[0] * rotated[0] + normal[1] * rotated[1]
        for rotated in (rotate_xy(point, yaw) for point in points)
    )


def signed_separation(wall_plane: float, origin_xy: Sequence[float],
                      points: Sequence[Sequence[float]], yaw: float,
                      normal: tuple[float, float] = (1.0, 0.0)) -> float:
    surface = (normal[0] * float(origin_xy[0])
               + normal[1] * float(origin_xy[1])
               + support_xy(points, yaw, normal))
    return wall_plane - surface


def convex_hull_2d(points: Sequence[Sequence[float]]) -> list[tuple[float, float]]:
    unique = sorted({(float(p[0]), float(p[1])) for p in points})
    if len(unique) <= 1:
        return unique

    def cross(o, a, b):
        return ((a[0] - o[0]) * (b[1] - o[1])
                - (a[1] - o[1]) * (b[0] - o[0]))

    lower: list[tuple[float, float]] = []
    for point in unique:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)
    upper: list[tuple[float, float]] = []
    for point in reversed(unique):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)
    return lower[:-1] + upper[:-1]


def cylinder_local_points(radius: float, height: float, axis: str,
                          segments: int = 128) -> list[tuple[float, float, float]]:
    """Return a dense support polygon for a USD cylinder.

    At the Ridgeback wheel radius, 128 segments bound horizontal support error
    below 0.03 mm, far below the 10 mm contact gate.
    """
    if radius <= 0 or height <= 0 or segments < 8:
        raise ValueError("cylinder radius/height must be positive and segments >= 8")
    half = height / 2.0
    points = []
    for end in (-half, half):
        for index in range(segments):
            angle = 2.0 * math.pi * index / segments
            a, b = radius * math.cos(angle), radius * math.sin(angle)
            if axis == "X":
                points.append((end, a, b))
            elif axis == "Y":
                points.append((a, end, b))
            elif axis == "Z":
                points.append((a, b, end))
            else:
                raise ValueError(f"unsupported cylinder axis {axis!r}")
    return points


def _median(values: Iterable[float]) -> float:
    data = list(values)
    return statistics.median(data) if data else math.nan


def _angle_span(values: Sequence[float]) -> float:
    if not values:
        return math.nan
    unwrapped = [float(values[0])]
    for value in values[1:]:
        delta = math.atan2(math.sin(value - unwrapped[-1]),
                           math.cos(value - unwrapped[-1]))
        unwrapped.append(unwrapped[-1] + delta)
    return max(unwrapped) - min(unwrapped)


def evaluate_case(spec: CaseSpec, samples: Sequence[dict],
                  contacts: Sequence[ContactEvidence], expected_support: float,
                  visual_support: float | None) -> CaseResult:
    hold = [s for s in samples if s["phase"] == "hold"]
    if hold:
        cutoff = hold[-1]["phase_time"] - 1.0
        settled = [s for s in hold if s["phase_time"] >= cutoff]
    else:
        settled = []
    reverse = [s for s in samples if s["phase"] == "reverse"]

    settled_gap = _median(s["gap"] for s in settled)
    settled_x = _median(s["x"] for s in settled)
    max_penetration = max(0.0, -min((s["gap"] for s in samples), default=math.nan))
    hold_oscillation = ((max(s["gap"] for s in settled)
                         - min(s["gap"] for s in settled))
                        if settled else math.nan)
    tangent_drift = ((max(s["y"] for s in settled)
                      - min(s["y"] for s in settled))
                     if settled else math.nan)
    yaw_drift = _angle_span([s["yaw"] for s in settled])
    max_hold_speed = max((math.hypot(s["vx"], s["vy"]) for s in settled),
                         default=math.nan)

    alignment = None
    for event in contacts:
        if event.normal is None:
            continue
        magnitude = math.sqrt(sum(component * component for component in event.normal))
        if magnitude > 0:
            value = abs(event.normal[0]) / magnitude
            alignment = value if alignment is None else max(alignment, value)

    recovery_end = None
    target_gap = settled_gap + 0.005
    for sample in reverse:
        if sample["gap"] >= target_gap:
            recovery_end = sample["phase_time"]
            break
    recovery_clearance = ((reverse[-1]["gap"] - settled_gap)
                          if reverse else math.nan)

    max_step = 0.0
    for left, right in zip(samples, samples[1:]):
        if left["phase"] == right["phase"]:
            max_step = max(max_step, math.hypot(
                right["x"] - left["x"], right["y"] - left["y"]))

    visual_gap = None
    if visual_support is not None and math.isfinite(settled_x):
        visual_gap = WALL_FACE_X - (settled_x + visual_support)

    finite_values = [
        value for sample in samples
        for value in (sample["x"], sample["y"], sample["yaw"],
                      sample["vx"], sample["vy"], sample["wz"], sample["gap"])
    ]
    checks = {
        "samples_present": bool(settled and reverse),
        "stop_error": math.isfinite(settled_gap)
                      and abs(settled_gap) <= STOP_ERROR_LIMIT,
        "penetration": math.isfinite(max_penetration)
                       and max_penetration <= PENETRATION_LIMIT,
        "hold_oscillation": math.isfinite(hold_oscillation)
                            and hold_oscillation <= HOLD_OSCILLATION_LIMIT,
        "tangent_drift": math.isfinite(tangent_drift)
                         and tangent_drift <= TANGENT_DRIFT_LIMIT,
        "yaw_drift": math.isfinite(yaw_drift)
                     and yaw_drift <= YAW_DRIFT_LIMIT,
        "hold_speed": math.isfinite(max_hold_speed)
                      and max_hold_speed <= HOLD_SPEED_LIMIT,
        "contact_event": bool(contacts),
        "contact_normal": alignment is not None
                          and alignment >= CONTACT_NORMAL_LIMIT,
        "recovery_end": recovery_end is not None
                        and recovery_end <= RECOVERY_END_LIMIT_S,
        "recovery_clearance": math.isfinite(recovery_clearance)
                              and recovery_clearance >= RECOVERY_CLEARANCE,
        "finite": bool(finite_values) and all(math.isfinite(v) for v in finite_values),
        "no_constraint_jump": max_step <= 0.02,
    }
    if visual_gap is not None:
        checks["rendered_surface"] = abs(visual_gap) <= STOP_ERROR_LIMIT

    return CaseResult(
        name=spec.name,
        configuration=spec.configuration,
        orientation=spec.orientation,
        yaw_deg=math.degrees(spec.yaw),
        speed=spec.speed,
        expected_support=expected_support,
        expected_stop_x=WALL_FACE_X - expected_support,
        settled_stop_x=settled_x,
        settled_gap=settled_gap,
        visual_gap=visual_gap,
        max_penetration=max_penetration,
        hold_oscillation=hold_oscillation,
        tangent_drift=tangent_drift,
        yaw_drift_deg=math.degrees(yaw_drift),
        max_hold_speed=max_hold_speed,
        contact_events=len(contacts),
        contact_normal_alignment=alignment,
        recovery_end_s=recovery_end,
        recovery_clearance=recovery_clearance,
        max_step_translation=max_step,
        checks=checks,
        passed=all(checks.values()),
    )


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _git(args: Sequence[str]) -> str:
    result = subprocess.run(["git", *args], cwd=REPO, capture_output=True,
                            text=True, check=False)
    return result.stdout.strip()


def substrate_status() -> tuple[bool, list[str]]:
    output = _git(["status", "--porcelain", "--", *VALIDATION_SUBSTRATE])
    lines = [line for line in output.splitlines() if line.strip()]
    return not lines, lines


def case_specs(speeds: Sequence[float]) -> list[CaseSpec]:
    orientations = [
        ("front", 0.0),
        ("side", math.pi / 2.0),
        ("angle", math.pi / 4.0),
    ]
    specs = [
        CaseSpec("hull_only", name, yaw, speed)
        for name, yaw in orientations for speed in speeds
    ]
    specs.extend(
        CaseSpec("full_robot", name, yaw, 0.10)
        for name, yaw in orientations
    )
    specs.append(CaseSpec("aabb_control", "angle", math.pi / 4.0, 0.10))
    return specs


def _write_json(path: Path, value) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True,
                               allow_nan=False) + "\n", encoding="utf-8")


def _json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(v) for v in value]
    return value


def _write_samples(path: Path, samples: Sequence[dict]) -> None:
    if not samples:
        return
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(samples[0]))
        writer.writeheader()
        writer.writerows(samples)


def _svg_polyline(points: Sequence[tuple[float, float]], color: str,
                  width: float = 2.0, dashed: bool = False) -> str:
    coords = " ".join(f"{x:.1f},{y:.1f}" for x, y in points)
    dash = ' stroke-dasharray="6 4"' if dashed else ""
    return (f'<polyline points="{coords}" fill="none" stroke="{color}" '
            f'stroke-width="{width}"{dash}/>' )


def write_envelope_svg(path: Path, geometry: dict,
                       aggregate_cases: dict[str, dict]) -> None:
    width, height = 1050, 390
    panels = [("front", 0.0), ("angle", math.pi / 4), ("side", math.pi / 2)]
    colors = {"visual": "#58d6ff", "hull": "#f6c85f", "aabb": "#ff6b6b"}
    pieces = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#10151f"/>',
        '<style>text{font-family:system-ui,sans-serif;fill:#eaf0f7}</style>',
        '<text x="24" y="28" font-size="20" fill="#eaf0f7">'
        'Isaac chassis contact envelope</text>',
    ]
    scale = 180.0
    for index, (name, yaw) in enumerate(panels):
        left = 20 + index * 345
        top = 48
        panel_w, panel_h = 325, 320
        origin_x = left + 110
        origin_y = top + 160
        pieces.append(f'<rect x="{left}" y="{top}" width="{panel_w}" '
                      f'height="{panel_h}" rx="10" fill="#182231" stroke="#33445c"/>')
        pieces.append(f'<text x="{left + 14}" y="{top + 26}" font-size="16" '
                      'fill="#eaf0f7">'
                      f'{name} · {math.degrees(yaw):.0f}°</text>')

        key = f"hull_only_{name}_0.10"
        record = aggregate_cases.get(key)
        stop_x = record["median_stop_x"] if record else 0.0

        def project(raw):
            rotated = [rotate_xy(point, yaw) for point in raw]
            closed = rotated + rotated[:1]
            return [
                (origin_x + (stop_x + p[0] - WALL_FACE_X + 0.45) * scale,
                 origin_y - p[1] * scale)
                for p in closed
            ]

        wall_px = origin_x + 0.45 * scale
        pieces.append(f'<line x1="{wall_px:.1f}" y1="{top + 45}" '
                      f'x2="{wall_px:.1f}" y2="{top + panel_h - 20}" '
                      'stroke="#cfd8e3" stroke-width="4"/>')
        for geometry_key, color_key, dashed in (
            ("visual_outline", "visual", False),
            ("hull_outline", "hull", False),
            ("aabb_outline", "aabb", True),
        ):
            pieces.append(_svg_polyline(project(geometry[geometry_key]),
                                         colors[color_key], dashed=dashed))
    pieces.extend([
        '<text x="24" y="382" font-size="13" fill="#58d6ff">rendered chassis</text>',
        '<text x="175" y="382" font-size="13" fill="#f6c85f">convexHull</text>',
        '<text x="290" y="382" font-size="13" fill="#ff6b6b">retired AABB</text>',
        '</svg>',
    ])
    path.write_text("\n".join(pieces) + "\n", encoding="utf-8")


def write_trace_svg(path: Path, samples: Sequence[dict]) -> None:
    selected = [s for s in samples
                if s["case"] in {"hull_only_front_0.10",
                                  "hull_only_angle_0.10",
                                  "hull_only_side_0.10"}]
    width, height = 1050, 390
    pieces = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">',
        '<rect width="100%" height="100%" fill="#10151f"/>',
        '<style>text{font-family:system-ui,sans-serif;fill:#eaf0f7}</style>',
        '<text x="24" y="28" font-size="20" fill="#eaf0f7">'
        'Hull contact traces · boot 1</text>',
    ]
    colors = {"front": "#58d6ff", "angle": "#f6c85f", "side": "#b88cff"}
    for panel_index, (field, label, lo, hi) in enumerate((
        ("gap", "signed separation (m)", -0.01, 0.03),
        ("speed", "planar speed (m/s)", 0.0, 0.12),
        ("yaw_error", "yaw error (deg)", -1.0, 1.0),
    )):
        left = 20 + panel_index * 345
        top, panel_w, panel_h = 50, 325, 310
        pieces.append(f'<rect x="{left}" y="{top}" width="{panel_w}" '
                      f'height="{panel_h}" rx="10" fill="#182231" stroke="#33445c"/>')
        pieces.append(f'<text x="{left + 12}" y="{top + 24}" font-size="14" '
                      f'fill="#eaf0f7">{label}</text>')
        for orientation in ("front", "angle", "side"):
            rows = [s for s in selected if f"_{orientation}_" in s["case"]]
            if not rows:
                continue
            t0 = rows[0]["case_time"]
            duration = max(rows[-1]["case_time"] - t0, 1e-6)
            values = []
            yaw0 = rows[0]["yaw"]
            for row in rows:
                if field == "speed":
                    value = math.hypot(row["vx"], row["vy"])
                elif field == "yaw_error":
                    value = math.degrees(math.atan2(
                        math.sin(row["yaw"] - yaw0), math.cos(row["yaw"] - yaw0)))
                else:
                    value = row[field]
                x = left + 12 + (row["case_time"] - t0) / duration * (panel_w - 24)
                y = top + panel_h - 18 - (value - lo) / (hi - lo) * (panel_h - 52)
                values.append((x, max(top + 34, min(top + panel_h - 18, y))))
            pieces.append(_svg_polyline(values, colors[orientation], 1.5))
    pieces.extend([
        '<text x="24" y="382" font-size="13" fill="#58d6ff">front</text>',
        '<text x="82" y="382" font-size="13" fill="#f6c85f">45°</text>',
        '<text x="120" y="382" font-size="13" fill="#b88cff">side</text>',
        '</svg>',
    ])
    path.write_text("\n".join(pieces) + "\n", encoding="utf-8")


def aggregate_boots(boot_summaries: Sequence[dict]) -> tuple[dict, dict[str, dict]]:
    grouped: dict[str, list[dict]] = {}
    for summary in boot_summaries:
        for case in summary.get("cases", []):
            grouped.setdefault(case["name"], []).append(case)

    aggregate_cases: dict[str, dict] = {}
    repeat_checks: dict[str, bool] = {}
    for name, rows in sorted(grouped.items()):
        stops = [row["settled_stop_x"] for row in rows
                 if row["settled_stop_x"] is not None]
        spread = max(stops) - min(stops) if len(stops) > 1 else 0.0
        aggregate_cases[name] = {
            "boots": len(rows),
            "all_passed": len(rows) == len(boot_summaries)
                          and all(row["passed"] for row in rows),
            "median_stop_x": statistics.median(stops) if stops else None,
            "stop_spread": spread,
        }
        repeat_checks[name] = (len(rows) == len(boot_summaries)
                               and spread <= BOOT_SPREAD_LIMIT)

    control_checks: dict[str, bool] = {}
    for boot in boot_summaries:
        rows = {row["name"]: row for row in boot.get("cases", [])}
        hull = rows.get("hull_only_angle_0.10")
        aabb = rows.get("aabb_control_angle_0.10")
        if not hull or not aabb:
            control_checks[f"boot_{boot.get('boot_index', '?')}"] = False
            continue
        expected_delta = aabb["expected_stop_x"] - hull["expected_stop_x"]
        observed_delta = aabb["settled_stop_x"] - hull["settled_stop_x"]
        control_checks[f"boot_{boot['boot_index']}"] = (
            abs(observed_delta - expected_delta) <= AABB_DELTA_ERROR_LIMIT
        )

    checks = {
        "all_boots_completed": bool(boot_summaries),
        "all_cases_passed": bool(aggregate_cases)
                            and all(row["all_passed"]
                                    for row in aggregate_cases.values()),
        "cross_boot_repeatability": bool(repeat_checks)
                                    and all(repeat_checks.values()),
        "aabb_control": bool(control_checks) and all(control_checks.values()),
    }
    return {
        "checks": checks,
        "repeat_checks": repeat_checks,
        "aabb_control_checks": control_checks,
        "physics_passed": all(checks.values()),
    }, aggregate_cases


def run_parent(args: argparse.Namespace) -> int:
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output_dir = (args.output_dir or (DEFAULT_ARTIFACT_ROOT / timestamp)).resolve()
    output_dir.mkdir(parents=True, exist_ok=False)

    clean, dirty_lines = substrate_status()
    manifest = {
        "created_at": datetime.now(timezone.utc).isoformat(),
        "git_head": _git(["rev-parse", "HEAD"]),
        "git_describe": _git(["describe", "--always", "--dirty"]),
        "validation_substrate_clean": clean,
        "validation_substrate_status": dirty_lines,
        "robot_usd": str(args.robot_usd),
        "robot_usd_sha256": _sha256(args.robot_usd),
        "physics_hz": args.physics_hz,
        "boots": args.boots,
        "speeds": args.speeds,
        "command": " ".join(sys.argv),
    }
    _write_json(output_dir / "manifest.json", manifest)

    boot_summaries = []
    child_failures = []
    for boot_index in range(1, args.boots + 1):
        boot_dir = output_dir / f"boot-{boot_index}"
        boot_dir.mkdir()
        command = [
            sys.executable, str(Path(__file__).resolve()),
            "--child", "--boot-index", str(boot_index), "--boots", "1",
            "--robot-usd", str(args.robot_usd),
            "--output-dir", str(boot_dir),
            "--physics-hz", str(args.physics_hz),
            "--speeds", ",".join(str(v) for v in args.speeds),
        ]
        print(f"\n=== ISAAC COLLIDER BOOT {boot_index}/{args.boots} ===", flush=True)
        result = subprocess.run(command, cwd=REPO, env=os.environ.copy(),
                                capture_output=True, text=True, check=False)
        (boot_dir / "console.log").write_text(
            result.stdout + result.stderr, encoding="utf-8")
        print(result.stdout, end="", flush=True)
        if result.stderr:
            print(result.stderr, end="", file=sys.stderr, flush=True)
        summary_path = boot_dir / "summary.json"
        if summary_path.exists():
            boot_summaries.append(json.loads(summary_path.read_text(encoding="utf-8")))
        if result.returncode != 0:
            child_failures.append({"boot": boot_index, "returncode": result.returncode})

    aggregate, aggregate_cases = aggregate_boots(boot_summaries)
    aggregate["child_failures"] = child_failures
    aggregate["checks"]["child_processes"] = not child_failures
    aggregate["physics_passed"] = all(aggregate["checks"].values())
    aggregate["validation_substrate_clean"] = clean
    aggregate["conclusive"] = aggregate["physics_passed"] and clean
    aggregate["status"] = (
        "PASS" if aggregate["conclusive"]
        else "PROVISIONAL" if aggregate["physics_passed"]
        else "FAIL"
    )
    aggregate["cases"] = aggregate_cases
    _write_json(output_dir / "summary.json", _json_safe(aggregate))

    geometry_path = output_dir / "boot-1/geometry.json"
    if geometry_path.exists():
        geometry = json.loads(geometry_path.read_text(encoding="utf-8"))
        write_envelope_svg(output_dir / "envelopes.svg", geometry, aggregate_cases)
    samples_path = output_dir / "boot-1/samples.csv"
    if samples_path.exists():
        with samples_path.open(newline="", encoding="utf-8") as stream:
            rows = []
            for row in csv.DictReader(stream):
                converted = dict(row)
                for key in ("case_time", "phase_time", "x", "y", "yaw",
                            "vx", "vy", "wz", "gap"):
                    converted[key] = float(converted[key])
                rows.append(converted)
        write_trace_svg(output_dir / "traces.svg", rows)

    print(f"\nCOLLIDER VALIDATION {aggregate['status']}", flush=True)
    print(f"evidence: {output_dir}", flush=True)
    if not clean:
        print("result is provisional: validation substrate has uncommitted changes",
              flush=True)
        for line in dirty_lines:
            print(f"  {line}", flush=True)
    if aggregate["status"] == "PASS":
        return 0
    if aggregate["status"] == "PROVISIONAL":
        return 2
    return 1


def _points_for_prim(prim, UsdGeom, Gf, time_code) -> list[tuple[float, float]]:
    transform = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(time_code)
    local_points = []
    if prim.IsA(UsdGeom.Mesh):
        local_points = UsdGeom.Mesh(prim).GetPointsAttr().Get(time_code) or []
    elif prim.IsA(UsdGeom.Cube):
        half = float(UsdGeom.Cube(prim).GetSizeAttr().Get(time_code) or 1.0) / 2.0
        local_points = [Gf.Vec3d(x, y, z)
                        for x in (-half, half)
                        for y in (-half, half)
                        for z in (-half, half)]
    elif prim.IsA(UsdGeom.Cylinder):
        cylinder = UsdGeom.Cylinder(prim)
        local_points = cylinder_local_points(
            float(cylinder.GetRadiusAttr().Get(time_code)),
            float(cylinder.GetHeightAttr().Get(time_code)),
            str(cylinder.GetAxisAttr().Get(time_code)),
        )
    else:
        raise ValueError(f"unsupported collision geometry {prim.GetTypeName()} at {prim.GetPath()}")
    return [(float(world[0]), float(world[1]))
            for world in (transform.Transform(Gf.Vec3d(*point))
                          for point in local_points)]


def _world_bounds_z(prim, UsdGeom, time_code) -> tuple[float, float]:
    cache = UsdGeom.BBoxCache(
        time_code,
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.guide,
         UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
    )
    bounds = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    return float(bounds.GetMin()[2]), float(bounds.GetMax()[2])


def _set_collision_enabled(prim, UsdPhysics, enabled: bool) -> None:
    UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr().Set(enabled)


def run_child(args: argparse.Namespace) -> int:
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    sys.path.insert(0, str(SIM_DIR))

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    exit_code = 1
    try:
        exit_code = _run_isaac_boot(app, args, output_dir)
    except Exception:
        traceback.print_exc()
        print("COLLIDER BOOT FAILED", flush=True)
    app.close()
    return exit_code


def _run_isaac_boot(app, args: argparse.Namespace, output_dir: Path) -> int:
    import omni.timeline
    import omni.usd
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics

    from robot_rig import RidgebackRig
    from worlds import BASE_LINK_FLOOR_CLEARANCE

    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    floor = UsdGeom.Cube.Define(stage, "/World/floor")
    floor.CreateSizeAttr(1.0)
    UsdGeom.XformCommonAPI(floor).SetTranslate(Gf.Vec3d(0.0, 0.0, -0.05))
    UsdGeom.XformCommonAPI(floor).SetScale(Gf.Vec3f(8.0, 8.0, 0.1))
    UsdPhysics.CollisionAPI.Apply(floor.GetPrim())

    wall = UsdGeom.Cube.Define(stage, "/World/wall")
    wall.CreateSizeAttr(1.0)
    UsdGeom.XformCommonAPI(wall).SetTranslate(Gf.Vec3d(1.1, 0.0, 1.0))
    UsdGeom.XformCommonAPI(wall).SetScale(Gf.Vec3f(0.2, 4.0, 2.0))
    UsdPhysics.CollisionAPI.Apply(wall.GetPrim())

    scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
    PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim()).CreateTimeStepsPerSecondAttr(
        float(args.physics_hz))

    robot_path = "/World/ridgeback"
    robot = stage.DefinePrim(robot_path, "Xform")
    robot.GetReferences().AddReference(args.robot_usd.as_uri())
    UsdGeom.XformCommonAPI(robot).SetTranslate(
        Gf.Vec3d(0.0, 0.0, BASE_LINK_FLOOR_CLEARANCE))
    world_fix = stage.GetPrimAtPath(f"{robot_path}/drive_rig/world_fix")
    if not world_fix:
        raise RuntimeError("drive_rig/world_fix missing from robot USD")
    UsdPhysics.FixedJoint(world_fix).CreateLocalPos0Attr(
        Gf.Vec3f(0.0, 0.0, BASE_LINK_FLOOR_CLEARANCE))

    chassis_path = f"{robot_path}/Geometry/base_link/chassis_link"
    old_aabb_path = f"{chassis_path}/chassis_collision"
    old_aabb = stage.GetPrimAtPath(old_aabb_path)
    if not old_aabb or old_aabb.IsActive():
        raise RuntimeError("retired chassis AABB must exist and be inactive")
    old_aabb.SetActive(True)
    old_aabb = stage.GetPrimAtPath(old_aabb_path)

    collision_prims = [
        prim for prim in Usd.PrimRange(robot, Usd.TraverseInstanceProxies())
        if prim.HasAPI(UsdPhysics.CollisionAPI)
    ]
    original_enabled = {}
    for prim in collision_prims:
        attr = UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr()
        value = attr.Get() if attr else None
        original_enabled[str(prim.GetPath())] = bool(True if value is None else value)
    original_enabled[old_aabb_path] = False
    _set_collision_enabled(old_aabb, UsdPhysics, False)

    vendor_meshes = [
        prim for prim in collision_prims
        if "/vendor_chassis/" in str(prim.GetPath()) and prim.IsA(UsdGeom.Mesh)
    ]
    if len(vendor_meshes) != 2:
        raise RuntimeError(f"expected chassis hull + deck collision meshes, found {len(vendor_meshes)}")
    tc = Usd.TimeCode.Default()
    main_hull = max(vendor_meshes,
                    key=lambda prim: _world_bounds_z(prim, UsdGeom, tc)[1]
                    - _world_bounds_z(prim, UsdGeom, tc)[0])
    approximation = UsdPhysics.MeshCollisionAPI(main_hull).GetApproximationAttr().Get()
    if approximation != "convexHull":
        raise RuntimeError(f"main chassis collider is {approximation!r}, expected 'convexHull'")

    hull_points = _points_for_prim(main_hull, UsdGeom, Gf, tc)
    aabb_points = _points_for_prim(old_aabb, UsdGeom, Gf, tc)
    full_points = []
    for prim in collision_prims:
        if original_enabled.get(str(prim.GetPath()), False):
            full_points.extend(_points_for_prim(prim, UsdGeom, Gf, tc))

    visual_points = []
    for prim in Usd.PrimRange(robot, Usd.TraverseInstanceProxies()):
        path = str(prim.GetPath())
        if "/vendor_chassis/" not in path or "/visuals/" not in path:
            continue
        if not (prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Cube)):
            continue
        imageable = UsdGeom.Imageable(prim)
        if imageable.ComputeVisibility() == UsdGeom.Tokens.invisible:
            continue
        visual_points.extend(_points_for_prim(prim, UsdGeom, Gf, tc))
    if not visual_points:
        raise RuntimeError("no visible vendor chassis geometry found")

    geometry = {
        "main_hull_path": str(main_hull.GetPath()),
        "old_aabb_path": old_aabb_path,
        "active_collision_paths": sorted(
            path for path, enabled in original_enabled.items() if enabled),
        "hull_outline": convex_hull_2d(hull_points),
        "visual_outline": convex_hull_2d(visual_points),
        "aabb_outline": convex_hull_2d(aabb_points),
        "full_collision_outline": convex_hull_2d(full_points),
    }
    _write_json(output_dir / "geometry.json", geometry)

    roots = [prim.GetPath().pathString for prim in Usd.PrimRange(robot)
             if prim.HasAPI(UsdPhysics.ArticulationRootAPI)]
    rig_roots = [root for root in roots if "/drive_rig/" in root]
    if not roots:
        raise RuntimeError("no articulation root under robot")
    art_root = rig_roots[0] if rig_roots else roots[0]

    chassis = stage.GetPrimAtPath(chassis_path)
    PhysxSchema.PhysxContactReportAPI.Apply(chassis).CreateThresholdAttr().Set(0)
    PhysxSchema.PhysxRigidBodyAPI.Apply(chassis).CreateSleepThresholdAttr().Set(0)

    timeline = omni.timeline.get_timeline_interface()
    timeline.set_end_time(1.0e9)
    timeline.set_looping(False)
    from omni.kit.loop import _loop as omni_loop
    loop = omni_loop.acquire_loop_interface()
    dt = 1.0 / float(args.physics_hz)
    loop.set_manual_step_size(dt)
    loop.set_manual_mode(True, name="main")
    timeline.set_time_codes_per_second(float(args.physics_hz))

    current_case = [None]
    contact_records: dict[str, list[ContactEvidence]] = {}
    subscription = None

    def sdf_path(identifier) -> str:
        if not identifier:
            return ""
        try:
            from pxr import PhysicsSchemaTools
            return str(PhysicsSchemaTools.intToSdfPath(identifier))
        except Exception:
            return ""

    def on_contact(*callback_args):
        label = current_case[0]
        if label is None:
            return
        try:
            headers = callback_args[0]
            contact_data = callback_args[1] if len(callback_args) > 1 else []
            for header in headers:
                paths = []
                for field in ("actor0", "actor1", "collider0", "collider1"):
                    path = sdf_path(getattr(header, field, 0))
                    if path and path not in paths:
                        paths.append(path)
                if not any("/World/wall" in path for path in paths):
                    continue
                normal = None
                impulse = None
                try:
                    offset = header.contact_data_offset
                    count = header.num_contact_data
                    for datum in contact_data[offset:offset + count]:
                        vector = datum.impulse
                        magnitude = math.sqrt(vector.x**2 + vector.y**2 + vector.z**2)
                        if impulse is None or magnitude >= impulse:
                            impulse = magnitude
                            n = datum.normal
                            normal = (float(n.x), float(n.y), float(n.z))
                except Exception:
                    pass
                contact_records.setdefault(label, []).append(ContactEvidence(
                    sim_time=float(timeline.get_current_time()), paths=paths,
                    normal=normal, impulse=impulse))
        except Exception as exc:
            print(f"contact callback error: {exc}", flush=True)

    try:
        import omni.physx
        interface = omni.physx.get_physx_simulation_interface()
        for method_name in ("subscribe_physics_contact_report_events",
                            "subscribe_contact_report_events"):
            method = getattr(interface, method_name, None)
            if method:
                subscription = method(on_contact)
                print(f"contact reports: {method_name}", flush=True)
                break
    except Exception as exc:
        print(f"contact report subscription unavailable: {exc}", flush=True)
    if subscription is None:
        print("contact reports: unavailable (contact gates will fail)", flush=True)

    rig = RidgebackRig(art_root, odom_noise=0.0)
    timeline.play()
    for _ in range(240):
        app.update()
        if rig.ready():
            break
    else:
        raise RuntimeError("articulation tensor backend never initialized")
    rig.initialize()

    point_sets = {
        "hull_only": hull_points,
        "full_robot": full_points,
        "aabb_control": aabb_points,
    }

    def set_configuration(name: str):
        rig.set_planar_pose(-3.0, 0.0, 0.0)
        for prim in collision_prims:
            path = str(prim.GetPath())
            if name == "hull_only":
                enabled = path == str(main_hull.GetPath())
            elif name == "full_robot":
                enabled = original_enabled.get(path, False)
            elif name == "aabb_control":
                enabled = path == old_aabb_path
            else:
                raise ValueError(name)
            _set_collision_enabled(prim, UsdPhysics, enabled)
        for _ in range(20):
            now = float(timeline.get_current_time())
            rig.step(dt, now)
            app.update()

    all_samples: list[dict] = []
    results: list[CaseResult] = []
    active_configuration = None

    def run_steps(spec: CaseSpec, phase: str, count: int,
                  command: tuple[float, float, float] | None,
                  expected_points, case_start: float) -> list[dict]:
        rows = []
        phase_start = float(timeline.get_current_time())
        for _ in range(count):
            now = float(timeline.get_current_time())
            if command is not None:
                rig.set_cmd(*command, now=now)
            rig.step(dt, now)
            app.update()
            x, y, yaw = rig.ground_truth()
            vx, vy, wz = rig._rig_velocities()
            current_time = float(timeline.get_current_time())
            row = {
                "case": spec.name,
                "configuration": spec.configuration,
                "orientation": spec.orientation,
                "speed_command": spec.speed,
                "phase": phase,
                "case_time": current_time - case_start,
                "phase_time": current_time - phase_start,
                "sim_time": current_time,
                "x": x,
                "y": y,
                "yaw": yaw,
                "vx": vx,
                "vy": vy,
                "wz": wz,
                "gap": signed_separation(
                    WALL_FACE_X, (x, y), expected_points, yaw),
            }
            rows.append(row)
            all_samples.append(row)
        return rows

    for spec in case_specs(args.speeds):
        if spec.configuration != active_configuration:
            set_configuration(spec.configuration)
            active_configuration = spec.configuration
        expected_points = point_sets[spec.configuration]
        expected_support = support_xy(expected_points, spec.yaw)
        visual_support = (support_xy(visual_points, spec.yaw)
                          if spec.configuration == "hull_only" else None)
        start_x = WALL_FACE_X - expected_support - START_CLEARANCE
        rig.set_planar_pose(start_x, 0.0, spec.yaw)
        current_case[0] = spec.name
        contact_records[spec.name] = []
        case_start = float(timeline.get_current_time())

        run_steps(spec, "settle", round(SETTLE_S * args.physics_hz), None,
                  expected_points, case_start)
        command_xy = world_to_body(spec.speed, 0.0, spec.yaw)
        approach_command = (command_xy[0], command_xy[1], 0.0)
        stable_frames = 0
        max_approach = round(APPROACH_TIMEOUT_S * args.physics_hz)
        for _ in range(max_approach):
            rows = run_steps(spec, "approach", 1, approach_command,
                             expected_points, case_start)
            row = rows[-1]
            stopped_near_wall = (row["gap"] <= 0.015
                                 and math.hypot(row["vx"], row["vy"]) < 0.02)
            stable_frames = stable_frames + 1 if stopped_near_wall else 0
            if contact_records[spec.name] or stable_frames >= round(0.1 * args.physics_hz):
                break

        run_steps(spec, "hold", round(HOLD_S * args.physics_hz),
                  approach_command, expected_points, case_start)
        zero_command = (0.0, 0.0, 0.0)
        run_steps(spec, "zero", round(ZERO_S * args.physics_hz), zero_command,
                  expected_points, case_start)
        reverse_xy = world_to_body(-REVERSE_SPEED, 0.0, spec.yaw)
        run_steps(spec, "reverse", round(REVERSE_S * args.physics_hz),
                  (reverse_xy[0], reverse_xy[1], 0.0), expected_points,
                  case_start)
        run_steps(spec, "final", round(FINAL_SETTLE_S * args.physics_hz),
                  zero_command, expected_points, case_start)

        samples = [sample for sample in all_samples if sample["case"] == spec.name]
        result = evaluate_case(
            spec, samples, contact_records[spec.name], expected_support,
            visual_support)
        results.append(result)
        state = "PASS" if result.passed else "FAIL"
        failed = [name for name, passed in result.checks.items() if not passed]
        print(f"[{state}] {spec.name}: stop={result.settled_gap:+.4f} m "
              f"penetration={result.max_penetration:.4f} m "
              f"contacts={result.contact_events}"
              + (f" failed={','.join(failed)}" if failed else ""), flush=True)
        current_case[0] = None

    _write_samples(output_dir / "samples.csv", all_samples)
    boot_summary = {
        "boot_index": args.boot_index,
        "physics_hz": args.physics_hz,
        "robot_usd_sha256": _sha256(args.robot_usd),
        "contact_subscription": subscription is not None,
        "cases": [_json_safe(asdict(result)) for result in results],
        "passed": all(result.passed for result in results),
    }
    _write_json(output_dir / "summary.json", boot_summary)
    write_trace_svg(output_dir / "traces.svg", all_samples)
    verdict = boot_summary["passed"]
    print(f"COLLIDER BOOT {'PASS' if verdict else 'FAIL'} "
          f"({sum(result.passed for result in results)}/{len(results)})",
          flush=True)
    return 0 if verdict else 1


def main() -> int:
    args = parse_args()
    return run_child(args) if args.child else run_parent(args)


if __name__ == "__main__":
    raise SystemExit(main())
