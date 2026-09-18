#!/usr/bin/env python3
"""Run three independent boots per condition, serially for comparable timing."""

import argparse, json, os, subprocess, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    out = a.output.resolve()
    out.relative_to(ROOT / "artifacts")
    out.mkdir(parents=True, exist_ok=False)
    summaries = {}
    for model, hz in [("physical", 120), ("physical", 240), ("baseline", 120)]:
        for boot in range(1, 4):
            name = f"{model}-{hz}-boot{boot}"
            directory = out / name
            print("START", name, flush=True)
            with (out / (name + ".log")).open("w") as log:
                proc = subprocess.run(
                    [
                        sys.executable,
                        str(Path(__file__).with_name("run.py")),
                        "--output",
                        str(directory),
                        "--model",
                        model,
                        "--physics-hz",
                        str(hz),
                    ],
                    cwd=ROOT,
                    env=os.environ.copy(),
                    stdout=log,
                    stderr=subprocess.STDOUT,
                )
            path = directory / "summary.json"
            summary = (
                json.loads(path.read_text())
                if path.exists()
                else {
                    "status": "PROCESS_ERROR",
                    "returncode": proc.returncode,
                    "passed": False,
                }
            )
            summaries[name] = summary
            (out / "summary.json").write_text(json.dumps(summaries, indent=2))
            print("FINISH", name, summary["status"], flush=True)
            if summary["status"] not in ["PASS", "FAIL"] or not summary.get(
                "complete_matrix"
            ):
                raise RuntimeError(
                    f"Incomplete harness run: {name}; inspect its log before resuming"
                )
    return 0


if __name__ == "__main__":
    sys.exit(main())
