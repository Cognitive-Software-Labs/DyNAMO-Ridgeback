# Unitree G1 USD asset

`g1_29dof_with_dex1_base_fix1.usd` is vendored unmodified from Unitree
Robotics' official Isaac Lab simulation assets:

- Repo: https://github.com/unitreerobotics/unitree_sim_isaaclab
- Asset bundle: https://huggingface.co/datasets/unitreerobotics/unitree_sim_isaaclab_usds
  (`assets/robots/g1-29dof-dex1-base-fix-usd/`, bundle of 2025-07-23)
- License: Apache License 2.0, Copyright HangZhou YuShu TECHNOLOGY CO., LTD.
  ("Unitree Robotics")

Used here as a static visual/collision target for the G1 distance benchmark
(fixed-base articulation — it stands rigidly where spawned). `g1.usda` is a
thin repo-local wrapper that moves the origin from the pelvis to the ground
contact point to match the gz-era model convention.

The gz-era STL model remains at `sim/models/g1/` as the fallback source
should this vendored asset ever need replacing (STL→USD via Isaac's
asset converter).
