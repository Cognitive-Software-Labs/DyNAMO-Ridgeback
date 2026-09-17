# Documentation ownership

The documentation system has one rule: a durable fact has one canonical owner.
Other pages summarize it only when useful and link back to that owner.

## Canonical owners

| Information | Canonical home |
|---|---|
| Installation, public commands, launch arguments, operator workflow | root `README.md` |
| Short repository orientation | `docs/project/PROJECT_CONTEXT.md` |
| Documentation policy and precedence | this page |
| Cross-cutting repository conventions | `docs/project/conventions.md` |
| Implemented behavior and interfaces | topic reference under `docs/` |
| Current failures and diagnostic recipes | `docs/troubleshooting.md` |
| Active evidenced engineering gaps | `docs/BACKLOG.md` |
| Approved work not yet complete | `docs/plans/` |
| Rejected, deferred, or unselected approaches | `docs/do_not_try_again/` |
| Dated experiments, migrations, and validation evidence | `docs/history/` |

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
  completed investigation's detailed evidence to history and leave a short fix
  pointer where useful.
- Put a gap in the backlog only when it remains active and has a completion
  criterion. Put an implementation document in plans only while work remains.
- When work completes, remove it from active lists, put the implemented contract
  in its technical reference, and preserve only durable evidence in history.
- Do not copy large implementation descriptions into project context. Add a
  one- or two-sentence summary and link to the topic owner.
- Prefer relative Markdown links so documentation remains valid in worktrees and
  on repository hosts.

## Visual ownership and document scale

Visuals are part of the documentation contract, not disposable decoration.
Use a diagram, plot, annotated render, or comparison image when it makes a
spatial relationship, mechanism, state transition, or measured result easier
to verify than prose alone.

- Put each durable visual under the asset directory of the document that owns
  the claim, and embed it beside that claim. A current geometry figure belongs
  with its technical reference; a run-specific trace belongs with its history
  record.
- Runtime output under `artifacts/` is evidence input, not durable
  documentation. Promote the interpretation-ready figures needed by future
  readers into tracked documentation assets.
- Give embedded visuals descriptive alt text and a short caption that explains
  what to notice. Preserve the command, revision, or evidence path needed to
  regenerate them when that provenance matters.

Aggregate documents must remain compact routers rather than grow into permanent
catch-alls. This applies especially to indexes, overview pages, broad history
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
4. History describes the system at the recorded date and never overrides a
   current contract.

Resolve contradictions at the canonical owner and replace duplicate prose with
a link. Do not preserve two independently editable versions of the same fact.

## Offline link validation

Run `python3 tools/check_doc_links.py` from the workspace (or pass `--root`).
Install the test dependency `python3-markdown-it` through rosdep or apt first.
The same check runs in the autonomy package's `test_doc_links` pytest/ament gate.

It checks Markdown links and images in root Markdown files and `docs/`, including
reference-style links, relative file targets, and Markdown heading fragments.
Code examples, external URLs, email links and absolute/home-local paths are
excluded. Non-Markdown fragments are not interpreted. Diagnostics identify the
source block's first line and target; failures exit nonzero. The check needs
neither network access nor simulator/build output. Raw HTML links are outside
this Markdown-link gate; explicit HTML anchors in Markdown are recognized.
