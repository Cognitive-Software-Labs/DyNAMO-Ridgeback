# Robot collision model and navigation footprint

The shared robot reference distinguishes three envelopes. They serve different
purposes and must be compared in the same frame before claiming agreement.

| Envelope | Owner | Meaning |
|---|---|---|
| Physical occupied volume | Measured robot, including attachments | The space the hardware occupies |
| Simulator collision geometry | Each backend's collision model | Shapes used by that simulator to resolve contact |
| Navigation footprint | Shared Nav2 configuration | The planar polygon supplied to navigation; it does not generate simulator colliders |

The [geometry reference](geometry.md) owns mounts and dimensional provenance.
A rendered mesh, a physics collider, and a navigation footprint may differ.
An image of one does not establish the others.

## Configured navigation footprint

![Configured Nav2 footprint in the base_link plane, with forward and left axes.](assets/navigation-footprint.png)

The figure plots the identical local/global costmap `footprint` entries from
[`nav2_params.yaml`](../../src/ridgeback_autonomy/config/nav2_params.yaml).
It shows the configured polygon before any runtime padding or costmap inflation;
no physical-body or simulator-collider outline is asserted by this plot.
The configuration comments describe an intended circumscribing body octagon.
That intent still needs the [envelope audit](../BACKLOG.md#collision-envelope-and-navigation-footprint-validation)
against the actual backend meshes and physical attachments.

Nav2's collision monitor consumes `local_costmap/published_footprint` for its
approach polygon. Inspect the effective parameters and published footprint when
qualifying a deployment; a source configuration alone does not prove the live
robot's clearance.

## Backend contact representations

| Backend | Current reference or limitation |
|---|---|
| Gazebo | Collision elements come through the generated robot description; agreement with the Isaac hull and physical robot has not been established by this documentation split. |
| Isaac | The importer grafts vendor chassis geometry, uses its convex hull, and authors additional parts. The [Isaac collider reference](../isaac/robot-model.md#colliders) owns those shapes and its contact-envelope comparison. |
| Hardware | The physical robot determines contact. Navigation geometry must account for the deployed attachments and required clearance; software polygons do not prove physical clearance. |

The existing yellow-hull/red-box contact figure compares Isaac representations.
It stays with that backend. Its contact tolerance and validation results cannot
be transferred to Gazebo or hardware without separate evidence.

## Qualification ownership

[Shared mast geometry](../BACKLOG.md#shared-camera-mast-and-bracket-geometry),
[dimensional provenance](../BACKLOG.md#robot-geometry-and-drawing-audit), and
[collision-envelope validation](../BACKLOG.md#collision-envelope-and-navigation-footprint-validation)
are separate completion gates. A geometry change that affects benchmark inputs
or measured behavior requires rerunning the affected benchmarks before quoting
results.
