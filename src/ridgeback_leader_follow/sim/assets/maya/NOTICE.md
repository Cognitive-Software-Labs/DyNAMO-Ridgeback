# Maya lab pedestrian

Source: [Relaxed walk — Maya, Innerscene](https://www.innerscene.com/tools/library/3d-parts/relaxed-walk-maya-3d-person-animated-2c6669a2).
The author dedicates the model to the public domain under
[CC0 1.0](https://creativecommons.org/publicdomain/zero/1.0/).
The source GLB also embeds this dedication in `asset.copyright`.
Innerscene describes the character as adapted from MakeHuman/MPFB CC0 body,
rig and skin assets, with an original authored walk rather than motion capture.

`source.glb` is the downloaded original (SHA-256
`3c22e60a599526d264f7cbb744e5bc1f6b1e93a79daade322942c49e7aac6ab3`).
`prepare_lab_human.py` converts it using Blender 4.2.9: 48 evaluated frames
at 24 Hz, rotated to face +X, normalized to 1.72 m, with the feet on z=0.
`conversion.json` records the derived file hashes and gait scale.
`materials.json` retains the source PBR parameters; base-color textures,
triangle material assignments, UV transforms and alpha masks are preserved.
The runtime uses simplified PreviewSurface materials, without the source
normal/sheen extensions. Alternating clothing tint is a runtime variation;
all pedestrians still share one body and rig.
