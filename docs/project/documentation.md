# Documentation ownership

The documentation system has one rule: a durable fact has one canonical owner.
Other pages summarize it only when useful and link back to that owner.

## Writing checklist

<!-- writing-checklist:start -->
Before editing any Markdown page:

1. Find the fact's owner in the canonical-owners table of `docs/project/documentation.md` and edit that page, not the one you happen to have open.
2. Current pages state the current system. A run, trial or session narrative goes in a new dated archive record; the current page keeps only its result and links the record under `## Archived evidence`.
3. When plan or backlog work completes, write its result into the owning reference in the same commit and remove it from the active list. A plan's status table may summarize, never hold the only copy.
4. Documentation rides the code commit that changes the behavior it describes.
5. No dates in the headings of current pages.
6. Page near its budget: split out a self-contained section. Raise a `doc-budget` only for a page that cannot be split, with the reason.
7. Before committing a removal or condensation, run `tools/check_removed_facts.py` and `python3 tools/check_doc_links.py`.
<!-- writing-checklist:end -->

## Canonical owners

| Information | Canonical home |
|---|---|
| Installation, public commands, launch arguments, operator workflow | root `README.md` |
| Physical host reconstruction after reinstall or SSD failure | `docs/physical/ridgeback_recovery.md`, with small deployment inputs under `deployment/<robot>/` |
| Short repository orientation | `docs/project/PROJECT_CONTEXT.md` |
| Documentation policy and precedence | this page |
| Cross-cutting repository conventions | `docs/project/conventions.md` |
| Shared robot mounts, dimensions and physical/model provenance | `docs/robot/geometry.md` |
| Physical, simulator and navigation collision-envelope distinctions | `docs/robot/collision_model.md` |
| Backend-specific robot import, physics and sensors | backend topic reference, such as `docs/isaac/robot-model.md` |
| Implemented behavior and interfaces | topic reference under `docs/` |
| Current failures and diagnostic recipes | `docs/troubleshooting.md` |
| Robot, exploration, deployment and cross-cutting gaps | `docs/BACKLOG.md` |
| Target-distance benchmarking and controlled estimator-evaluation gaps | `docs/target_distance_benchmarking/BACKLOG.md` |
| Approved work not yet complete | `docs/plans/` |
| Unabsorbed handoff snapshots between owners of a plan | `docs/plans/handoffs/` |
| Rejected, deferred, or unselected approaches | `docs/do_not_try_again/` |
| Dated experiments, migrations, and validation evidence | `archive/engineering/` |

`AGENTS.md` and `CLAUDE.md` are thin discovery entrypoints. They route to
`docs/project/PROJECT_CONTEXT.md` and retain only tool-specific instructions
that must be visible before deeper documentation is read.

They route differently because the tools differ. `CLAUDE.md` uses Claude Code's
`@docs/project/PROJECT_CONTEXT.md` import, which inlines the file at session
start, and the project context in turn imports `conventions.md`, so both arrive
without the agent choosing to read them. An import path must stay outside
backticks and code blocks or it is ignored. `AGENTS.md` names the same two files
as prose because Codex reads that file verbatim and has no import syntax.

Only the project context and the conventions are imported; every other document
stays a link so it loads when a task needs it. Neither entrypoint restates a
rule it routes to — the graphify rules, in particular, are owned by
`docs/project/conventions.md` alone.

## Update rules

- Update the root README when setup, public commands, launch arguments, or an
  operator workflow changes.
- Update a technical reference when implemented behavior or an interface
  changes.
- Add troubleshooting only when a reader can act on the symptom today. Move a
  completed investigation's detailed evidence to the archive and leave a short fix
  pointer where useful.
- Put a gap in its owning backlog only when it remains active and has a
  completion criterion. Keep one entry; link cross-cutting prerequisites rather
  than copying them. Use a plan when the remaining work needs a detailed
  execution procedure; a checklist belongs in the backlog.
- When work completes, remove it from active lists, put the implemented contract
  in its technical reference, and preserve only durable evidence in the archive.
- Do not copy large implementation descriptions into project context. Add a
  one- or two-sentence summary and link to the topic owner.
- Prefer relative Markdown links within current documentation or within the
  archive. Historical implementation sources use immutable commit links.
- A plan that tracks status across items links each completed item's result in
  its reference. Retiring a plan, handoff or section first moves any fact found
  only there; `tools/check_removed_facts.py` lists candidates.

## Host setup and reconstruction

Host setup must be reconstructible by another agent from the repository, rather
than depend on a remembered terminal session or files that exist only on the
robot SSD. The root README routes normal installation to the detailed
[Ridgeback recovery runbook](../physical/ridgeback_recovery.md); that runbook
owns host-specific reconstruction. Store small necessary scripts, service units,
drop-ins and non-secret configs under `deployment/<robot>/`, or link to their
maintained repository source when one already owns them. Do not create a second
independent implementation: deployed copies are labelled snapshots with origin
and hashes; the maintained source stays authoritative.

When changing host setup, update its reconstruction instructions and inputs in
the same repository change. Cover the following as applicable:

- OS/package/source versions, account/device permissions, network topology and
  identity, dependency/build paths and prerequisite vendor source access.
- Exact files/commands, owner and mode, service enablement and dependencies,
  verification, rollback, and effects on camera, desktop or base ownership.
- Installed state separately from a template, proposed repair or historical
  claim. Record inspection date and content hashes; include known faults and
  clean-install/cold-boot verification limits.
- What is generated and how to regenerate it. Avoid using stale `build/`,
  `install/` or generated Clearpath files as the configuration authority.
- Fresh local entry/enrolment for passwords, private keys, Wi-Fi and device
  credentials. Keep credential-bearing configs out of Git or preserve a clearly
  labelled template with placeholders; never present a redacted hash as the
  original installed file's hash.

Make reconstruction actionable without requiring a backup service or disk-image
workflow. An off-robot backup is a separate task only when requested. Identify
external vendor sources that the repository cannot reconstruct, rather than
claiming completeness from an installer or an old handover. Verification logs
belong under `artifacts/`; durable setup instructions and necessary small inputs
belong in tracked repository files. Changes must reach the intended Git remote
through the normal review/publish workflow to survive loss of this checkout.

## Plan names and scope

Plan filenames start with an uppercase scope followed by a descriptive topic:
`<SCOPE>_<topic>.md`. Titles and navigation labels use the same scope prefix,
for example `BENCHMARKING — Execution-environment qualification`.

Choose the scope by the work's primary owner:

- `BENCHMARKING` — target-distance benchmark tooling, qualification, and experiments.
- `PHYSICAL` — physical-robot deployment, calibration, and hardware validation.
- `ISAAC` — Isaac-specific implementation or qualification.
- `GAZEBO` — Gazebo-specific implementation or qualification.
- `EXPLORATION` — exploration behavior, navigation, and exploration evaluation.

A benchmarking experiment comparing remote and physical-seat execution stays
`BENCHMARKING`; physical-seat access does not make it physical-robot validation.
Use a backend name in the topic when it narrows the work. Update filenames,
titles, and incoming links together; retain immutable historical links unchanged.

## Archived evidence

Current references own mechanisms, assumptions, parameter rationale, constraints,
and limitations as well as interfaces. A reader must not need an archived
experiment to understand how the implemented system works. Verify facts against
code before extracting them from an old record. Current operational instructions
belong in the README/runbook; actionable diagnoses in troubleshooting; unfinished
work in the backlog or an active plan.

Keep an archival record only if it explains a consequential decision, preserves
interpretable measurements, distinguishes incompatible result populations, or
prevents a repeated investigation. Remove routine test counts, implementation
inventories, session chatter and obsolete to-do lists. Preserve the method,
conditions, unfavorable results, uncertainty and limits needed to interpret the
retained conclusion. Scope historical behavior within the claim itself; avoid
unqualified words such as "today" or "currently".

Each record starts with one title and these fields before its first level-two
heading (the archive index is exempt):

```text
Recorded dates: 2026-09-01, 2026-09-02
Tested revisions: `<full 40-character commit hash>`
Provenance: partial
```

Dates use comma-separated ISO dates or `unknown`. Revisions use comma-separated
full hashes in backticks or `unknown`. Provenance is `complete`, `partial`, or
`unknown`: complete requires known dates/revisions and preserved inputs sufficient
to reconstruct the tested state; partial identifies missing pieces explicitly;
unknown means the tested state cannot be established. A documentation commit is
not a tested revision. For multiple runs, identify which revision belongs to
which result in the body. A new dirty-tree experiment must preserve its patch,
relevant untracked inputs, configuration and dependency pins before claiming
reproducibility. Never manufacture missing provenance for older records.

Archive links may point to other archived records/assets or immutable source
references. Links to this repository on GitHub must use `blob`, `tree`, or
`commit` with a full hash, never a branch name; working-tree links outside the
archive are prohibited. If an exact historical source is unavailable, retain
the necessary dated explanation and state the gap. A later documentation
snapshot must be labeled as such, never presented as the tested implementation.
Corrections identify their date and basis instead of silently modernizing an
old result.

The main documentation index links once to the archive index. Topic references
may link to individual records only under `## Archived evidence`; those links
support provenance, not the explanation of current behavior. Keep dated figures
in their records rather than embedding them into current references. Access and
search rules are owned by [conventions](conventions.md#current-knowledge-and-archive-access).

## Visual ownership and document scale

Visuals are part of the documentation contract, not disposable decoration.
Use a diagram, plot, annotated render, or comparison image when it makes a
spatial relationship, mechanism, state transition, or measured result easier
to verify than prose alone.

- Put each durable visual under the asset directory of the document that owns
  the claim, and embed it beside that claim. A current geometry figure belongs
  with its technical reference; a run-specific trace belongs with its archived
  record.
- Runtime output under `artifacts/` is evidence input, not durable
  documentation. Promote the interpretation-ready figures needed by future
  readers into tracked documentation assets.
- Give embedded visuals descriptive alt text and a short caption that explains
  what to notice. Preserve the command, revision, or evidence path needed to
  regenerate them when that provenance matters.

## Page size and splitting

Current pages have line budgets: 600 for references, the README, backlogs and
other current pages; 500 for plans; 60 for one backlog item. Archive records and
handoffs have none, since evidence stays whole and handoffs are temporary.

A page that needs more space is split, not stretched. Extract a section a reader
can understand on its own into `docs/<same family>/<topic>.md` and leave a short
summary and link where it was. A backlog item that outgrows 60 lines moves its
procedure into a plan and links it.

Only a page that cannot be split, such as one indivisible table or state
machine, may raise its own budget with one HTML comment near its top:
`<!-- doc-budget: 900 lines; reason: one indivisible launch-argument table -->`.
The reason must say why splitting does not work; raising a budget shows in the
diff and is reviewed like any other change. A page waiting for a scheduled split
may carry its current size with the reason `split pending`, so it can shrink
but not grow.

Aggregate documents must remain compact routers rather than grow into permanent
catch-alls. This applies especially to indexes, overview pages, archive indexes
or changelog pages, backlogs, and long-lived plans.

- Review an aggregate page whenever adding a substantial section. If a topic
  has its own narrative, evidence, assets, lifecycle, or reuse value, extract
  it into a dedicated Markdown file under the same ownership family.
- Leave a short summary and relative link at the aggregate owner after an
  extraction. Do not retain a second editable copy of the extracted prose.
- Prefer semantic extraction over a hard line-count limit: the trigger is that
  a reader can understand or maintain the chunk independently. Regularly split
  the largest self-contained chunks before the aggregate page stops being
  quickly scannable.

## Precedence

Current code and generated launch arguments are the final authority when a
checkable documentation claim disagrees with the repository. Among documents:

1. The topic's current technical reference owns implemented behavior.
2. The root README owns public operator workflow.
3. The backlog and active plans own unfinished work.
4. The archive describes the system at the recorded date and never overrides a
   current contract.

Resolve contradictions at the canonical owner and replace duplicate prose with
a link. Do not preserve two independently editable versions of the same fact.

## Offline documentation validation

Run `python3 tools/check_doc_links.py` from the workspace (or pass `--root`).
Install the test dependency `python3-markdown-it` through rosdep or apt first.
The same check runs in the autonomy package's `test_doc_links` pytest/ament gate.

It checks Markdown links and images in root Markdown files, `docs/`, and
`archive/engineering/` even though search ignores the archive, including
reference-style links, relative file targets, and Markdown heading fragments.
Code examples, external URLs, email links and absolute/home-local paths are
excluded from target-existence checks. Archive headers, working-tree link
boundaries, immutable repository source links and the placement of archive
references in current docs are checked separately. Non-Markdown fragments are
not interpreted. Diagnostics identify the
source block's first line and target; failures exit nonzero. The check needs
neither network access nor simulator/build output. Raw HTML links are outside
this Markdown-link gate; explicit HTML anchors in Markdown are recognized.
The same run enforces page shape on current documentation: the
[page budgets](#page-size-and-splitting) and `doc-budget` markers, no dated
headings in current pages, no duplicated level-one or level-two headings, a
`Status:` line in each plan and a link to it from `docs/index.md`, a
`**Completion criteria.**` paragraph in each backlog item, and flowchart node
IDs that are not reused for different nodes. Each failure message names the fix.

The checker validates structure and declared provenance, not the truth of prose
or the completeness of an experiment's preserved state. Those require review.
`tools/check_removed_facts.py` supports that review: it lists distinctive names,
quantities, dates, paths and hashes from deleted Markdown lines that no longer
appear anywhere in the tree. The tracked pre-commit hook, installed by
`tools/install_hooks`, runs the link and shape checker whenever Markdown is
staged. The `doc-steward` Claude agent reviews a documentation diff against
this page.
