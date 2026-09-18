# Benchmark semantics

What a scenario, an event, an observation, an instance row, and a miss reason
mean, and which runs may be compared with which. Start at
[running benchmarks](running_benchmarks.md) if you are choosing how to launch.

## Scenario and comparison discipline

The packaged full set is `config/benchmark_scenarios_full.yaml` (88 scenes,
20 multi-robot in the recorded 2026-08-24 transition; verify current source
before reporting counts). It includes occlusion, clutter, traps, and probe
scenes; the [gallery](benchmark_scenarios_v2_gallery.html) is a visual reference.

- Compare runs against the exact same scenario YAML and revision. Regenerating
  with a new seed expands coverage; it is not an A/B baseline.
- A scene can override repeats. `repeats:=1` still allows a probe with
  `repeats_override: 8` to collect its intended within-pose sample.
- Record current parameters and code provenance; an older scene/model/render
  configuration is not directly interchangeable with a current run.
- Never run `cleanup.sh` between persistent sweep configurations. A sweep owns
  one simulator/RViz/detector environment and restarts the per-config layer.

### Scenario schema

Scenario YAML is declarative. A file contains optional defaults and a non-empty
`scenes` list. Every scene has a unique id and at least one target; objects and
per-scene repeat overrides are optional:

```yaml
defaults:
  robot_yaw_rad: 3.141592653589793
scenes:
  - id: example
    repeats_override: 2
    robots:
      - {x: 3.0, y: 0.4, yaw: 3.14}
    objects:
      - {model: hospital_bed, x: 2.0, y: 0.3, yaw: 1.57}
```

Coordinates are world-frame planar poses: `x` is forward, `y` is lateral
(positive left), and `yaw` is radians about +Z. The runner pins spawned models
to the floor. `benchmarking/scenarios.py` is the authoritative parser and
validation contract; scenario-file comments record generation-specific design
and certification details.

## Event, observation, and instance semantics

One **event** is one aligned measurement frame keyed by frame id, exact stamp,
detection count, and bounding boxes. Pointcloud and mask messages with that key
merge. A multi-robot frame is still one event.

One **observation** is one detected box. Status histograms count observations,
so a two-box event contributes two statuses per estimator. A statusless
pointcloud value gets coarse OK/UNSET inferred from finiteness.

One **instance row** is a ground-truth target for one trial and estimator.
Association uses per-estimator planar positions, so every estimator scores on
its own usable events. Each final trial estimate is the median of that
estimator's associated values. Consequences:

- An estimator can miss while another scores; there is no common-event gate.
- A zero-detection trial records every instance as missed for every estimator.
- `detector_miss`, `gate_miss`, and `no_value` are scored outcomes. A skipped
  trial means infrastructure failure (for example spawn/stream failure), not a
  perception miss.
- No-detection frames and per-frame raw estimates are not themselves accuracy rows.

## Miss reasons and attribution boundary

Mask measurement messages carry a status per detection and estimator.
`compute_status_histogram` counts every box; a specific failure outranks UNSET
when selecting a dominant explanation. Trial CSV `miss_reason` is populated only
for `no_value`; the other miss categories are already named by `outcome`.

Attribution never borrows another estimator's position or assumes detection
order. In a multi-instance trial, a box status reaches an instance row only when
that same estimator's locator associates the box to that ground-truth instance.
Statuses on unmatched boxes remain observation-level evidence. The one exception
is an event containing exactly one target and one box, where identity is
unambiguous even without an estimator locator. When a `no_value` row has no safe
association, `miss_reason` is blank and the report renders `no_value (reason
unknown)` rather than copying the trial's dominant reason onto one or more
unproven identities.

`scored` rows need no reason; `gate_miss` and `detector_miss` are already explicit
outcomes. The collage remains a trial-level evidence view and may show a dominant
estimator reason; it is not an instance attribution source.

Accuracy/miss fields are per instance. `observations` and `reason_histogram` are
per detected box. Do not add those granularities together or interpret coverage
as accuracy. The authoritative enum is `ridgeback_common/miss_reason.py`; the pipeline
[glossary](../target_localization/target_localization_pipeline.md)
explains stages.

Historical scenario/status transitions are retained in the Archived evidence section below.

## Archived evidence

- [benchmark history](../../archive/engineering/benchmark_evolution.md)
