# Maya F3 outfit variants

Derived from the CC0 Innerscene **Maya** pedestrian; the
[Maya notice](../maya/NOTICE.md) owns its source, dedication and original hash.
These derivatives are also dedicated to the public domain under
[CC0 1.0](https://creativecommons.org/publicdomain/zero/1.0/).
No other artwork source is used.

## Contents

- `outfits/<outfit>/top.png`, `trousers.png`: garment textures recoloured to
  the albedo listed in `ridgeback_leader_follow/crowd_appearance.py`.
- `hair/black/`, `hair/blonde/`: recoloured `hair_roots.png` and `ponytail.png`;
  brown hair uses the unmodified Maya textures.
- `walk.dae`: Gazebo Collada skin and 2-second walk. Its texture references are
  canonical names (`top.png`, `body.png`, ...); the F3 scene writes one skin per
  outfit at run time and links those names to the Maya or outfit textures.
- `outfits.json`: palette, method, source and output SHA-256 receipt.
- `collada.json`: Blender version, pose checks and the `walk.dae` SHA-256.

## Conversion

From the package directory `src/ridgeback_leader_follow/sim`:

```bash
python3 prepare_f3_outfits.py assets/maya assets/maya_f3
<blender-4.2.9>/blender -b --python prepare_f3_gazebo_human.py -- assets/maya assets/maya_f3
```

Recolouring works in linear light: each pixel keeps its luminance relative to
the source texture mean, multiplied by the outfit albedo; alpha is unchanged.

The Collada export applies the same +X rotation, 1.72 m height and floor seating
as `prepare_lab_human.py`, then makes three Gazebo-specific changes that leave
the world-space geometry unchanged:

- all skinned meshes are joined into one skin;
- the `foot_l`/`foot_r` root bones are parented under their calves and the walk
  is re-keyed from the recorded armature-space poses, because Gazebo does not
  animate extra skeleton roots;
- the eyebrow, eyelash and ponytail materials get `<transparent opaque="A_ONE">`,
  which gz-common reads as an alpha cutout (fixed 0.5 threshold; the source uses
  0.28 for the ponytail).

The export fails unless every vertex of the evaluated walk matches the Isaac
vertex cache `../maya/walk.npz` to within 1 mm; `collada.json` records the measured
deviation. The glTF garment UV scale is not carried into Collada; the garment
textures are nearly uniform, so the rendered colour is unaffected.
