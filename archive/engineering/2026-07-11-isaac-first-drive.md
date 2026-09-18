# Isaac 6.0 first-drive snapshot — July 11, 2026

Recorded dates: 2026-07-11

Tested revisions: unknown

Provenance: partial

## Preserved capture

The [self-contained first-drive presentation](assets/isaac-first-drive/index.html)
contains the original embedded images and reported measurements from a 36-second
scripted drive through `mock_hospital` on Isaac Sim 6.0.1. It is preserved
unchanged, including its original conclusions. It is a historical capture,
not a current setup guide or a qualified exploration baseline.

The presentation names motion change `659307ca` and sensor/EKF change
`75d0caab`, but does not establish the exact tested checkout, dirty-tree state,
or full environment. Those identifiers are not asserted as tested revisions.
No simulation was rerun during archival.

## Interpretation limits

Archival annotation, September 18, 2026: the later
[sensor-attachment investigation](port-history.md#lidars-detached-from-the-articulation-2026-09-11)
invalidated earlier exploration figures. This short scripted capture does not
establish autonomous exploration quality; its original “what this proves”
section must be read as a claim made at capture time.

The old regeneration and teleoperation instructions remain in the
[pre-archive documentation snapshot](https://github.com/Cognitive-Software-Labs/DyNAMO-Ridgeback/blob/cd0232682c781072316b3317f91ad0b2d1251334/docs/demos/isaac-first-drive/README.md).
That link identifies a documentation snapshot, not the tested implementation.
