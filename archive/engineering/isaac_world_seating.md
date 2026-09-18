# Isaac world seating and map regeneration

Recorded dates: 2026-09-17

Tested revisions: unknown

Provenance: partial

This is dated evidence. Missing revisions or preserved worktree inputs limit
reproducibility; no new measurements were made during archive curation.

On 2026-09-17 the Isaac runner stopped applying the raised
`mock_hospital` spawn height to every world. The September 17 change registered
world floor tops in `worlds.py`; the runner derived
`spawn_z = floor_z + 0.02617`, and the analytic map tools derived the lidar
plane from that same floor as `floor_z + 0.25257`.

## Height validation

`tools/isaac/diag_rig.py` checked the authored geometry and then ran each case
for 300 live frames in Isaac Sim 6.1:

| World | floor z | base_link z | wheel-cylinder bottom | front/rear lidar z | Result |
|---|---:|---:|---:|---:|---|
| `mock_hospital` | 0.05000 | 0.07617 | 0.05027 | 0.30257 | pass |
| `warehouse` | 0.00000 | 0.02617 | 0.00027 | 0.25257 | pass |

The 0.27 mm cylinder-bottom residual comes from the primitive's 0.0759 m
radius; the spawn clearance is the measured wheel-mesh bottom, 0.02617 m.
Both lidar planes matched exactly, and neither live run drifted over the final
60 frames. The acceptance tolerance was 0.5 mm.

## Regenerated stock maps

All stock-world map sets (`.pgm`, `.yaml`, `.png`, and `.npz`) were regenerated
from the Isaac 6.1 assets at z=0.25257 m. Dimensions and origins did not change.

| World | Size | Occupied | Free | Unknown | Cells changed from z=0.3024 |
|---|---:|---:|---:|---:|---:|
| `warehouse` | 440×680 | 2,980 | 236,039 | 60,181 | 606 |
| `warehouse_full` | 680×1160 | 14,463 | 650,260 | 124,077 | 2,158 |
| `office` | 760×2040 | 37,895 | 690,994 | 821,511 | 1,777 |
| `hospital` | 1560×880 | 47,212 | 538,085 | 787,503 | 3,237 |

The previews were inspected after generation. This work deliberately produced
no new exploration or SLAM-quality numbers; those measurements remain gated
on the separate lidar and exploration recertification backlog item.
