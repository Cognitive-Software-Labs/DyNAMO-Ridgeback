# Benchmark GUI execution decision — September 11, 2026

Recorded dates: 2026-09-11

Tested revisions: unknown

Provenance: partial

This is dated evidence. Missing revisions or preserved worktree inputs limit
reproducibility; no new measurements were made during archive curation.

## Decision

The implementation developed against base revision `18eacb41e7da8afd7e22bdfe7eaeaebb5906dca9` moved job writes
and offline execution into the server. Browser downloads did not expose their
destination to the page, while job-relative input paths were interpreted from
the downloaded job's parent. Inspection separately used the server's working
directory, so displayed input validity could disagree with execution.
The server-side job file used absolute paths anchored once on the workspace.

## Cancellation investigation

The wrapper shell died on SIGINT before a signal-ignoring child. Checking only
the wrapper PID therefore stopped escalation prematurely. The fix watched the
process group for cancellation while preserving command-line-verified PID
checks for reattachment. A regression child inherited `SIG_IGN` to require
escalation; a live GUI cancellation was not observed because the small replay
finished first.

## Rename evidence

The audit inspected 118 benchmark directories. Offline and trial output
provenance did not require the result's own location. Typed artifact lineage
used content IDs and hashes, although saved jobs still referred to paths.
Sweeps stored absolute output paths requiring a rewrite. Config subdirectory
names were part of resume/report addressing, and staging directories were
incomplete output. These findings motivated distinct rename rules rather than
uniform directory renaming.

The audit also found an existing moved sweep whose stored output path still
named the old host location. Rewriting all sweep entries avoided depending on
those already-stale old values. An empty-string separator test rejected all
Linux rename candidates until corrected.

## Bounded validation and exclusions

A real browser measurement job with relative inputs/outputs completed five
trials and 25 events, reached exit zero, and wrote a report. The run was renamed
through the results panel. This did not establish live-system supervision:
the cleanup catch-all matched the GUI's installed executable. Skipping cleanup
was rejected because duplicate Gazebo processes caused target-pose failures.

Concurrent runs were refused to avoid invalidating worker-timing comparisons.
No screenshot artifact or exact tested worktree snapshot was preserved by the
original record; `18eacb41e7da8afd7e22bdfe7eaeaebb5906dca9` identifies its base, not all implementation changes.
