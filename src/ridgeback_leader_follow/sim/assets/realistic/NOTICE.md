# Realistic scene assets

Furniture, plants, trees, surface maps and the daylight HDRI are from
[Poly Haven](https://polyhaven.com), released under
[CC0 1.0](https://polyhaven.com/license). Powered by Poly Haven.
`sources.json` records publisher download URLs, byte counts and SHA-256 hashes.
Original wayfinding panels are drawn by `prepare_scene_signs.py` for this
project and are dedicated to CC0. Per-prop `conversion.json` files record the Blender version, geometry bounds
and derived geometry hashes. Original glTF files retain their material data. Large meshes use a
180,000-triangle simulation LOD; exported static poses are checked against the
vertex cache. Runtime tree collision uses an approximate trunk cylinder.

The walking Sam, Harper and Jordan models are from Innerscene, adapted from
MakeHuman CC0 assets and released under CC0:

- [Sam, relaxed walk](https://www.innerscene.com/tools/library/3d-parts/relaxed-walk-sam-3d-person-animated-732f1206).
- [Harper, relaxed walk](https://www.innerscene.com/tools/library/3d-parts/relaxed-walk-harper-3d-person-animated-b504cbac).
- [Jordan, relaxed walk](https://www.innerscene.com/tools/library/3d-parts/relaxed-walk-jordan-3d-person-animated-7ab9479d).

Their original GLB files embed the publisher's copyright dedication. The bake
uses Blender 4.2.9, 48 evaluated frames at 24 Hz, normalized to 1.72 m facing
+X with the lowest foot position on z=0. Per-instance uniform scaling adds
stature variation. Collada conversion receipts verify agreement with the Isaac
vertex cache. Maya is reused from the separately attributed `maya` directory.

The cast contains four source characters with repeated clothing/stature
variants, rather than nine independently modelled identities. Deliberate
look-alikes share their leader's model, scale and clothing settings.

Rebuild using `sim/fetch_realistic_assets.py` and
`sim/prepare_realistic_assets.py`; the maintained workflow is documented in
`docs/leader_following/environments.md`. Large model and animation files are
stored in Git LFS. Source licenses remain CC0 after conversion.
