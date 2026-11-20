# Monocular readiness gate and Isaac output repeatability

Recorded dates: 2026-11-05
Tested revisions: `cc86eef5a5c708be2a31aac64db29a91225f4023`
Provenance: partial

## Question and tested behavior

The preceding [cache-policy investigation](2026-11-05-isaac-cache-policy-and-residual-probes.md)
found missing first-scene monocular inputs during lazy model initialization.
The requirement for this experiment was approximately equal benchmark outputs
in two consecutive runs with the same configuration, rather than pixel identity.

The dirty-tree implementation warmed the selected monocular depth source on a
real matched camera frame, including an empty detection batch. It published
producer readiness after successful inference with the expected image grid.
The benchmark waited for that processing/readiness state before scene setup and
timed capture. Target detection, TF and successful distance estimates were not
prerequisites for warmup. Stereo and polar-only configurations skipped this gate.

## Conditions and provenance

Two fresh consecutive Isaac sessions (`gated_a`, `gated_b`) each ran
`interfere_control_01` and `chaos_03` with box-gated stereoscopic and monocular
projective, Euclidean and polar estimators. Both used the lab background with
props, OWLv2 at threshold 0.30 and a 10 FPS ceiling, 640×480, ideal depth,
PathTracing, deterministic mode, AA 0, RTF 1, a two-second settle and fixed
10-second sampled windows requesting 100 batches. GUI, overlays and recording
were enabled. The preceding result-cache/AA policy remained enabled.

Workstation-local evidence is under
`artifacts/benchmarks/isaac_repeatability_gate_20261105T192840Z/`:
source snapshots/hashes, dirty patch, sweeps, installed-executable hashes,
Isaac VERSION, session logs, typed captures, CSVs, `comparison.json` and process
results. The installed benchmark and mask executables were rebuilt and matched
their source hashes before either accepted run. Earlier `session_a`/`session_b`
attempts used stale installed executables and are explicitly excluded; only
`gated_a`/`gated_b` form this comparison. The wrapper required a logged gate
opening and saved readiness evidence before accepting a session.

The pair waited until shared GPU utilization was at most 30% for three successive
five-second observations; each launch also checked utilization and free memory.
Both sessions exited successfully in approximately 3.2 minutes. Runtime was
Isaac `6.1.0-rc.26+release.49347.2d230af4.gl`. Provenance is partial because the
complete binary/dependency environment was not locked. Ignored artifacts remain
local and are not carried by a documentation commit.

## Results

All 24 per-instance estimator outcome rows and their miss reasons matched.
Twenty scored distance pairs were comparable; the four remaining pairs were
matching depth-estimator gate misses. Maximum absolute distance differences:

| Input | Projective | Euclidean | Polar |
|---|---:|---:|---:|
| Stereoscopic | 0 mm | 0 mm | 4.732 mm |
| Monocular | 0.412 mm | 0.626 mm | 4.634 mm |

The largest relative difference over scored pairs was 0.243%. Both monocular
producer warmup timestamps preceded all their capture frames. Across 356
monocular batches, zero lacked depth or a live measurement; the first five
batches of each scene had both. No captured batch in either mode lacked RGB,
depth source, camera context, scan or live-record context.

| Session/input | Control raw batches | Chaos raw batches |
|---|---:|---:|
| A/stereoscopic | 73 | 92 |
| B/stereoscopic | 74 | 85 |
| A/monocular | 90 | 90 |
| B/monocular | 89 | 87 |

All windows remained below the requested 100 batches. The deadline was not
extended: detector FPS was a ceiling, not a guaranteed throughput. These count
differences are separate from the eliminated cold-start context gaps.

## Interpretation and limits

This pair supports practical output repeatability for the two tested scenes and
confirms that monocular cold initialization occurred outside capture. It does
not establish byte-identical rendering, equal wall-time sample counts, improved
absolute accuracy, a numerical acceptance tolerance or full-matrix qualification.
The persistent matching gate misses are not startup failures. Other estimator
settings and scene populations require their own evidence.
