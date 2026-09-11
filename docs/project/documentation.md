# Documentation ownership

The documentation system has one rule: a durable fact has one canonical owner.
Other pages summarize it only when useful and link back to that owner.

## Canonical owners

| Information | Canonical home |
|---|---|
| Installation, public commands, launch arguments, operator workflow | root `README.md` |
| Short repository orientation | `docs/project/context.md` |
| Documentation policy and precedence | this page |
| Cross-cutting repository conventions | `docs/project/conventions.md` |
| Implemented behavior and interfaces | topic reference under `docs/` |
| Current failures and diagnostic recipes | `docs/ISSUES.md` |
| Active evidenced engineering gaps | `docs/BACKLOG.md` |
| Approved work not yet complete | `docs/plans/` |
| Rejected, deferred, or unselected approaches | `docs/do_not_try_again/` |
| Dated experiments, migrations, and validation evidence | `docs/history/` |

`AGENTS.md` and `CLAUDE.md` are thin discovery entrypoints. They point to the
project context and retain only tool-specific instructions that must be visible
before deeper documentation is read.

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
