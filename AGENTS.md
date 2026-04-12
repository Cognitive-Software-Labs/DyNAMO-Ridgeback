## Shared Guidance

Read `AI_CONTEXT.md` for the canonical shared repository guidance before doing substantial work in this repo.

## graphify

This project has a graphify knowledge graph at `graphify-out/`.

Rules:
- Before answering architecture or codebase questions, read `graphify-out/GRAPH_REPORT.md` for god nodes and community structure
- If `graphify-out/wiki/index.md` exists, navigate it instead of reading raw files
- After modifying code files in this session, run `bash "$(git rev-parse --show-toplevel)/tools/rebuild_graphify"` to keep the repo-root graph current
- Do not use the old `python3 -c "from graphify.watch import _rebuild_code ..."` one-liner in this repo; `graphify` is not available on system `python3`, and the helper is the canonical rebuild path
