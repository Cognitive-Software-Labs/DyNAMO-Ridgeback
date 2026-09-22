# Plan handoffs

State snapshots written by an agent or person handing unfinished plan work to
the next owner. Each file names its author, date, tested revision, the live
robot state at handoff, results with evidence paths, and the open next steps.
A handoff is a dated record: the governing plan under `docs/plans/` stays the
source of truth for what the work is, and resolved findings move to their
technical reference.

Filenames follow the plan convention, `<SCOPE>_<topic>_<author>.md`.

- [ISAAC — Camera support and deck geometry discussion](ISAAC_camera_support_geometry_session.md): retain deck placement for now; discuss fitting the camera support while distinguishing geometry from camera TF.
