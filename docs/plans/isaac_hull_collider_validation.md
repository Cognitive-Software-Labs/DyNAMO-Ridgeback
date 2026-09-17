# Isaac hull-collider live validation

Status: implemented; provisional live evidence captured, clean-revision rerun
required for closure.

The active gap is owned by
[`BACKLOG.md`](../BACKLOG.md#isaac-hull-collider-validation). This plan turns
its completion criteria into a deterministic PhysX contact gate without
modifying the production robot asset.

## Harness

`tools/isaac/validate_chassis_contacts.py` composes the checked-out robot USD
into a minimal floor-and-wall stage and derives the expected stop plane from
the composed geometry. It exercises three stage-local configurations:

- `hull_only`: vendor chassis hull, with companion colliders disabled;
- `full_robot`: every production collider enabled as authored;
- `aabb_control`: retired AABB enabled in place of the hull.

The parent process starts three fresh Isaac boots. Each boot runs frontal,
lateral, and 45-degree hull contacts at 0.05, 0.10, and 0.20 m/s; whole-robot
contacts at 0.10 m/s; and one angled AABB control. Every case approaches,
holds inward for two seconds, stops, reverses, and settles.

## Gates

- settled collider-to-wall error at most 10 mm;
- penetration at most 5 mm;
- held-contact normal oscillation and tangential drift at most 5 mm;
- held-contact yaw drift at most 0.5 degrees and speed at most 0.02 m/s;
- a wall contact normal within 15 degrees of the expected normal;
- contact recovery within 0.25 s and at least 80 mm clearance after 1 s;
- at most 3 mm stop-position spread across boots;
- the observed hull-versus-AABB delta matches the geometric prediction within
  10 mm.

Raw evidence belongs under `artifacts/isaac-collider-validation/`. A run is
conclusive only when the importer, runner, drive rig, and robot USD are clean
at a committed revision. A dirty-substrate run is labelled provisional and
does not close the backlog.

## Provisional evidence — 2026-09-17

The first complete run exercised 39/39 cases successfully across three fresh
Isaac 6.1 boots on an NVIDIA RTX PRO 6000 Blackwell. All cases passed their
individual gates, all cross-boot stop-position spreads were 0 mm, and the
retired-AABB control differed from its geometric prediction by 0.323 mm.

Worst observed values remained inside the declared gates:

- collider-to-wall error: 0.000335 mm;
- penetration: 0.002951 mm;
- held-contact oscillation: 0.000436 mm;
- tangential drift: 2.762 mm;
- yaw drift: 0.262 degrees;
- held-contact speed: 0.00391 m/s;
- recovery start: 0.100 s;
- clearance after recovery: 95.210 mm;
- largest per-step translation: 1.667 mm.

The aggregate result and plots are in
`artifacts/isaac-collider-validation/20260917T213000Z_full/`. The run is
`PROVISIONAL`, not a backlog-closing pass, because the robot USD, importer, and
Isaac runner were already modified in the working tree. The manifest records
the exact substrate, including committed base
`ca215ccc53f123896154ba666228142d3486f872` and robot-USD SHA-256
`135338fd5246b7369a2e4858131101cbe113b8fe51435fa05e7b9cd323e4cdd1`.

Isaac also reported pre-existing fallback-inertia warnings for imported
collider-free child links and its usual TGS velocity-iteration warning. Neither
warning coincided with a non-finite state, contact discontinuity, or failed
gate. They remain recorded in the per-boot logs rather than being suppressed
by this validation work.

## Closure

On a conclusive pass, record the accepted tolerance in `isaac/robot-model.md`,
move the durable result to `history/`, remove this plan, and remove the backlog
item. On failure, retain the plan and backlog with the failing evidence; do not
tune the collider or thresholds inside the validation run.
