# Sampled capture and live-scan replay parity — 2026-11-03

Recorded dates: 2026-11-03
Tested revisions: `a6c255a2ac38c4fefa9063db249fd6f738ea6711`
Provenance: partial

## Tested state

Base revision `a6c255a`, with the sampled-window/monocular implementation and
live-scan recorder fix uncommitted during the probe. Its `frozen.json` records
experimental file hashes and the tracked source patch; verify those hashes
against the resulting implementation commit when reusing the evidence.
This is short readiness evidence, not the full qualification campaign.

Evidence roots under `artifacts/benchmarks/`:

- `sampled_capture_probe_20261103/`: failed Isaac trial-level polar parity.
- `sampled_capture_probe_20261103_r2/`: failed Isaac per-event polar parity.
  The failing batch used a scan 16.7 ms before the image live, while the recorder
  saved a closer scan 8.3 ms after it, unavailable at live selection time.
- `sampled_capture_probe_20261103_live_scan_fix/`: passing probe with the fix.
  Retain its report, campaign state, guards, frozen hashes, captures, sampled
  live references, mask caches, replay results and logs together.

## Change and verification

Sampled capture pins its scan to the selection named by the live `PolarBeams`
message, whether that message arrives before or after the raw batch. A later
closer scan cannot replace it. If the selected scan is unavailable, provisional
scan/transform evidence is cleared and the recorder waits within its existing
bound. It never substitutes another scan. The completion predicate and guard
require that selection wherever the live polar status proves a scan was used.
Legacy quota matching, live estimator behavior and parity tolerance are unchanged.

Both new regression cases failed before the fix. After it, the layered-replay
and qualification-helper suites passed 75 tests. A subsequently added guard
regression passed separately with `-k live_polar_scan`: 76 distinct focused
checks in total. Dependency checks passed, both changed packages rebuilt and
critical installed executables/assets matched source. Documentation links and
whitespace checks passed; Graphify was rebuilt. No new full-suite run is claimed.

The prepared probe was executed with:

```bash
tools/gpu-run /usr/bin/python3 tools/benchmark/qualification/campaign.py probe \
  artifacts/benchmarks/sampled_capture_probe_20261103_live_scan_fix
```

| Backend | Stored batches | Live-measured batches compared | Per-event parity, all three estimators | Largest compared coordinate delta |
|---|---:|---:|---|---:|
| Gazebo | 15 | 12 | Passed | 2.315e-7 m |
| Isaac | 15 | 15 | Passed | 4.691e-7 m |

Each backend covered box/stereo, box/monocular and real-SlimSAM
silhouette/monocular on `multi_04`. Every guard and repeated fixed-input numeric
check passed. Both simulator sessions exited with code zero, without outer
escalation or owned leftovers. Gazebo's three unmeasured box/monocular batches
remain explicit; that trial's aggregate parity is inconclusive.

Maximum measurement-window deviations: Gazebo 0.221 ms, Isaac 0.590 ms.
End-to-end check/replay stages took 78.801 s and 96.583 s; the overall probe phase
took 176.606 s. Final status: `probe-complete-not-a-campaign`, no errors.

## Remaining work

Collect the 16 live runs from a committed, clean tree using a fresh bundle,
charging measured prior probes to its budget. The later offline phase consumes
the same immutable inputs and only the remaining allocation. Preserve
inconclusive coverage, scientific margins, scene-audit and cancellation gaps;
passing this probe does not close them.
