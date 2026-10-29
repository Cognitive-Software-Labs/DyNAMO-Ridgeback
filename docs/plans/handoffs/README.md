# Plan handoffs

State snapshots written by an agent or person handing unfinished plan work to
the next owner. Each file names its author, date, tested revision, the live
robot state at handoff, results with evidence paths, and the open next steps.
A handoff is a dated record: the governing plan under `docs/plans/` stays the
source of truth for what the work is, and resolved findings move to their
technical reference.

Filenames follow the plan convention, `<SCOPE>_<topic>_<author>.md`.

When the receiving owner has absorbed a handoff (its open steps are scheduled
in a plan or backlog, its results are in references), move it to
`archive/engineering/<date>-<topic>-handoff.md` with the standard archive
header and immutable links. A handoff never stays here as permanent history.
