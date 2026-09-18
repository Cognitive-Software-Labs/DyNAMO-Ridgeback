#!/usr/bin/env python3
"""Require the complete nine-boot matrix and produce a compact review figure."""

import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
from run import cases

CONDITIONS = [("physical", 120), ("physical", 240), ("baseline", 120)]


def aggregate(root):
    result = {}
    expected_cases = {case["name"] for case in cases()}
    for model, hz in CONDITIONS:
        boots = []
        for boot in range(1, 4):
            path = root / f"{model}-{hz}-boot{boot}" / "summary.json"
            summary = json.loads(path.read_text())
            if (
                not summary.get("complete_matrix")
                or set(summary["cases"]) != expected_cases
                or summary["model"] != model
                or summary["physics_hz"] != hz
            ):
                raise ValueError(f"incomplete or mismatched evidence: {path}")
            boots.append(summary)
        result[f"{model}-{hz}"] = {
            "all_required_pass": all(b["passed"] for b in boots),
            "rtf": [b["rtf"] for b in boots],
            "case_pass_counts": {
                name: sum(b["cases"][name]["passed"] for b in boots)
                for name in expected_cases
            },
            "failed_checks": {
                name: sorted(
                    {
                        check
                        for b in boots
                        for check, passed in b["cases"][name]["checks"].items()
                        if not passed
                    }
                )
                for name in expected_cases
            },
            "metrics": {
                name: {
                    key: [b["cases"][name]["metrics"][key] for b in boots]
                    for key in boots[0]["cases"][name]["metrics"]
                }
                for name in expected_cases
            },
        }
    return result


def figure(result, output):
    required = [case["name"] for case in cases() if case["kind"] != "diagnostic"]
    keys = [f"{model}-{hz}" for model, hz in CONDITIONS]
    data = np.array(
        [[result[key]["case_pass_counts"][name] for key in keys] for name in required]
    )
    data[0, 2] = -1
    fig, ax = plt.subplots(figsize=(10, 8))
    cmap = ListedColormap(["#e6e9ed", "#f8d7da", "#fce9bb", "#e4edc8", "#bce3d1"])
    ax.imshow(
        data,
        aspect="auto",
        cmap=cmap,
        norm=BoundaryNorm([-1.5, -0.5, 0.5, 1.5, 2.5, 3.5], 5),
    )
    ax.set_xticks(
        range(3), ["Physical · 120 Hz", "Physical · 240 Hz", "Production · 120 Hz"]
    )
    labels = [
        name.replace("_", " ")
        .replace("rotate -1", "rotate clockwise")
        .replace("rotate +1", "rotate counterclockwise")
        for name in required
    ]
    ax.set_yticks(range(len(required)), labels)
    for i in range(len(required)):
        for j in range(3):
            label = "N/A" if i == 0 and j == 2 else f"{data[i,j]}/3"
            ax.text(
                j,
                i,
                label,
                ha="center",
                va="center",
                fontweight="bold",
                color="#283644",
            )
    for spine in ax.spines.values():
        spine.set_visible(False)
    ax.set_title(
        "Required cases passed across three independent boots", pad=18, fontsize=15
    )
    fig.text(
        0.06,
        0.025,
        "N/A: the fixed-height production rig cannot qualify free gravity settling. Diagnostics excluded.\nA failed case means at least one acceptance check failed; green does not establish hardware fidelity.",
        fontsize=10,
    )
    fig.tight_layout(rect=(0, 0.09, 1, 1))
    fig.savefig(output, dpi=170)
    plt.close(fig)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("root", type=Path)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=True)
    result = aggregate(a.root)
    (a.output / "aggregate.json").write_text(
        json.dumps(result, indent=2, sort_keys=True)
    )
    figure(result, a.output / "qualification.png")
