# CLAUDE.md

## Shared guidance

@docs/project/PROJECT_CONTEXT.md

The imported project context is the repository orientation. Follow its links to:
- documentation ownership
- project and graphify conventions
- subsystem architecture and technical references

There are currently no Claude-specific overrides beyond that shared guidance.

## graphify

This project has a graphify knowledge graph at `graphify-out/`.

Rules:
- Before answering architecture or codebase questions, read `graphify-out/GRAPH_REPORT.md` for god nodes and community structure
- After modifying code files in this session, run `bash "$(git rev-parse --show-toplevel)/tools/rebuild_graphify"` to keep the repo-root graph current
- Do not use the old `python3 -c "from graphify.watch import _rebuild_code ..."` one-liner in this repo; `graphify` is not available on system `python3`, and the helper is the canonical rebuild path
