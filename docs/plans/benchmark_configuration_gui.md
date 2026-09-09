# Plan: local benchmark configuration GUI

Status: **PROPOSED — REVIEW REQUIRED.** This plan defines a small local GUI for
choosing a benchmark level, configuring only valid inputs, validating the
result, and exporting a reproducible job. It depends on the shared profile and
job-spec contract in
[`layered_replay_profiles.md`](layered_replay_profiles.md). It does not yet run
or supervise benchmarks.

Prepared 2026-09-09 against `aafa4385`.

## Decision

Build a dependency-light local browser configurator:

```text
vanilla HTML/CSS/JS in local browser
                 |
                 v
loopback-only Python configuration service
                 |
                 v
shared profile registry + artifact/sweep/scenario validators
                 |
                 v
canonical job YAML/JSON + exact command preview
```

Use Python's standard HTTP server facilities and static assets installed with
`ridgeback_autonomy`. Bind only to `127.0.0.1`, use a random available port,
load no CDN assets, require no npm build, and provide `--no-open` for remote or
headless sessions.

The MVP is a **configurator**, not a process supervisor. It validates and
exports a job that the existing command-line tools execute. A run button would
introduce cancellation, signal forwarding, log streaming, process ownership,
stale ROS cleanup, resume, and browser-disconnect semantics; add that only under
a separate reviewed extension if command export proves insufficient.

## User outcome

An operator should be able to answer “what am I trying to benchmark?” before
seeing raw parameters:

- **Tune measurement** — use frozen detections/masks and rerun measurement.
- **Compare box with current SlimSAM** — use paired frozen mask caches and rerun
  measurement.
- **Try another mask model or mask settings** — use exact RGB/depth/detections,
  rerun mask production, then measurement.
- **Measure latency, throughput, or integration** — configure the existing live
  system benchmark and clearly show that offline replay is not valid evidence.

The question selects a recommended profile, but the profile remains visible and
can be changed deliberately. Every screen shows two summaries:

- **Held constant** — detector output, mask output, sensor evidence, or system
  components that will not rerun;
- **Rerun** — the stages and parameter axes that can affect this job.

## Layout

Use a compact responsive workspace rather than one long settings form:

```text
+---------------------------+  +----------------------------------+
| 1. Benchmark question     |  | Evidence boundary                |
| [measurement] [masks]     |  | Frozen: detector, RGB/depth ...  |
| [mask model] [live]       |  | Rerun: masks -> measurement      |
+---------------------------+  +----------------------------------+

+----------------------------------------------------------------+
| 2. Inputs                                                     |
| Dataset/cache/scenario  [validated path]   provenance summary  |
+----------------------------------------------------------------+

+----------------------------------------------------------------+
| 3. Variants                                                   |
| name        estimator       valid profile-owned parameters     |
| baseline    projective      [recipe] [band] [minimum pixels]   |
| candidate   euclidean       [recipe] [percentile] [...]        |
+----------------------------------------------------------------+

+---------------------------+  +----------------------------------+
| 4. Resource estimate      |  | 5. Review and export             |
| trials / batches / size   |  | errors, warnings, exact command  |
| expected vs hard timeout  |  | [Download job] [Copy command]    |
+---------------------------+  +----------------------------------+
```

Cards should have clear selected states, restrained gradients, useful hover
explanations, and visible focus rings. At narrow widths they stack without
horizontal scrolling; tables become labelled variant cards. Colour is never
the only indication of validity.

## Scope and non-goals

The MVP includes:

- benchmark-question and explicit-profile selection;
- discovery of known datasets/caches/runs under the repository's standard
  `artifacts/benchmarks` root;
- server-side validation of manually entered paths outside that root;
- artifact provenance, lineage, completeness, size, trial/event, and capability
  summary;
- scenario, repeat, evidence quota or live duration, sweep variants, worker
  limit, output path, and comparison baseline configuration;
- profile-aware controls and structured errors;
- estimated trial count, capture duration, storage, materialization work, and
  measurement work;
- canonical job export and exact argv-style command preview;
- import/edit of a previously exported job.

The MVP excludes:

- executing, cancelling, resuming, or monitoring jobs;
- arbitrary shell command entry;
- remote/multi-user access, authentication, or network binding;
- dataset uploads through the browser;
- artifact deletion or mutation;
- report charts or result analysis;
- an IDE-like raw YAML editor as the primary interface;
- detector-model replay until the core feature supports it.

## One contract, not two validators

The GUI must not encode benchmark legality in JavaScript. Its service imports
the same ROS-free profile registry, artifact loader, scenario loader, and sweep
validator as the command-line tools. The browser receives a JSON capability
description and structured validation response:

```json
{
  "field": "segmentation_model",
  "code": "upstream_stage_frozen",
  "message": "This profile freezes mask output.",
  "suggested_profile": "mask-model"
}
```

JavaScript renders fields from the capability description, manages local form
state, and displays backend decisions. It never maintains a second hard-coded
list of estimators, gates, recipes, parameter ranges, profile compatibility, or
defaults.

The exported job must pass the command-line validator byte-for-byte after
canonical serialization. Importing and immediately exporting an unchanged job
must be stable.

## Configuration model

Use the canonical job specification owned by the core feature. The GUI edits
these sections:

- `question` and `profile`;
- input artifacts and required parent mapping;
- live capture settings or offline evidence quota;
- scenario and repeats;
- mask materialization variants, when the profile owns the mask stage;
- measurement variants and named comparison baseline;
- CPU/model worker limits;
- output directory and reproducibility metadata.

The UI should distinguish quantities that users commonly conflate:

- **Offline evidence amount:** raw detector batches per trial. Empty batches
  count. A read-only estimate translates batches and detector FPS into expected
  seconds, while `capture_timeout_sec` remains visibly labelled as a stall
  bound, not the sample amount.
- **Live measurement window:** `capture_sec` per trial, used for system timing
  and integration work.
- **Drain bound:** maximum wait for exact RGB/depth/context/live-result evidence
  after the batch quota is reached.

Do not hide these semantics behind one generic “capture time” input.

## Local service and safety contract

Add an installed `target_benchmark_configurator` command with:

- `--bind 127.0.0.1` fixed for the MVP;
- optional `--port`, defaulting to an available ephemeral port;
- `--no-open` to print the URL without launching a browser;
- an unguessable per-process token in the URL and every mutating request;
- Ctrl-C shutdown and an optional idle timeout;
- no shell invocation and no job execution endpoints.

Proposed API surface:

- `GET /api/capabilities` — profile/axis/question descriptor;
- `GET /api/artifacts` — read-only scan of the standard artifact root;
- `POST /api/inspect` — validate and summarize one explicit path;
- `POST /api/validate` — return a resolved job plus structured issues;
- `POST /api/render` — return canonical YAML/JSON and an argv array;
- static application assets under the package share directory.

Reject non-loopback binding. Treat paths as data: expand and resolve them on the
server, report what they identify, and never interpolate them into a shell
string. The command preview may be formatted for humans, but the canonical
machine representation remains an argument list.

The service performs no writes. “Download job” is a browser download, and the
output path is validated as a future destination rather than created. This
keeps the first version reversible and prevents a configuration UI from
silently modifying benchmark artifacts.

## Resource estimates

Estimates must be labelled as estimates and show their assumptions:

- trials = scenario trials x repeats;
- expected offline capture duration = trials x (settle time + batches / observed
  or configured detector FPS + expected drain);
- hard upper bound = trials x relevant timeouts, shown separately;
- sensor-capture storage = events x RGB bytes plus depth bytes, adjusted by a
  measured compression factor when known;
- mask-cache storage = observed or estimated packed region pixels plus metadata;
- measurement work = trial/event/variant count and selected CPU workers;
- mask-model work = event/model-variant count and selected device workers.

Never promise wall time from a formula. If a compatible prior artifact or run
contains measured rates, show “based on <artifact>”; otherwise show a range or
“unknown” rather than a false precise duration.

## Implementation phases

### Phase 0: interaction contract

- Freeze the four user questions and their profile mappings.
- Sketch wide and narrow states for empty, valid, warning, invalid, and imported
  jobs.
- Define the JSON capability and structured-error shape with the core plan.
- Create representative fixtures for legacy measurement data, sensor capture,
  box/SlimSAM caches, corrupt lineage, and a live job.

Gate: a reviewer can configure each supported question on paper and can see why
every visible field belongs to that profile.

### Phase 1: pure configuration service

- Add importable handlers around the shared registry and existing validators.
- Implement artifact discovery/inspection without importing ROS or model
  libraries.
- Implement canonical job import, validation, rendering, and command argv
  generation.
- Start a loopback-only HTTP server and package static assets.

Gate: endpoint tests prove the service returns the same resolved values and
errors as the CLI for every fixture; no endpoint writes or executes anything.

### Phase 2: question, profile, and evidence UI

- Build the question cards and profile recommendation flow.
- Render the held-constant/rerun explanation directly from capabilities.
- Add artifact discovery, path inspection, provenance, lineage, and completeness
  cards.
- Show a specific upgrade path when evidence is too shallow, such as “this
  legacy dataset cannot rerun SlimSAM; capture `mask-model` evidence.”

Gate: selecting a question produces the correct profile and never offers an
axis owned by a frozen stage.

### Phase 3: variant editor and estimates

- Add scenario/repeat and capture controls with separate batch, expected-time,
  timeout, drain, and live-duration semantics.
- Build variants from capability metadata, including estimator-specific fields,
  defaults, ranges, names, duplicate detection, and baseline selection.
- Add copy/duplicate/remove controls with stable keyboard behavior.
- Render storage/work estimates and their assumptions.

Gate: all shipped sweep examples round-trip through the editor, and adding an
irrelevant knob is impossible through normal controls and rejected on imported
jobs.

### Phase 4: review and export

- Display a compact experiment summary: question, profile, frozen stages, rerun
  stages, inputs, variants, resources, outputs, warnings, and supported claims.
- Export canonical YAML/JSON and show the exact command plus working-directory
  assumptions.
- Support clipboard copy and browser download with clear success/failure state.
- Preserve form state across refresh in browser-local storage, excluding
  secrets; provide an explicit reset.

Gate: a downloaded job validates unchanged in the CLI and reproduces the
reviewed command. The GUI creates no benchmark output directory.

### Phase 5: visual and operational verification

- Test current Chromium/Firefox behavior with the local service.
- Capture wide and 360 px screenshots for each important state.
- Verify keyboard-only operation, labels, focus visibility, contrast, reduced
  motion, overflow, long paths, long variant names, and backend-unavailable
  behavior.
- Test loopback binding, token rejection, idle/Ctrl-C shutdown, headless
  `--no-open`, installed-package asset resolution, and concurrent-tab reads.
- Update README and the benchmark reference only after the workflow passes.

Gate: a new operator can produce a valid job for all four questions without
consulting launch source or learning which parameters are irrelevant.

## Verification matrix

Automated tests must cover:

- capability JSON and profile-question mapping;
- parity between GUI service and CLI validation;
- legacy/new artifact inspection and corrupt/missing lineage;
- scenario and sweep errors, unknown fields, duplicate variants and invalid
  baselines;
- batch-versus-seconds semantics and estimate assumptions;
- canonical import/export stability and shell-hostile path characters;
- loopback-only startup, request token, no-write/no-execute contract, shutdown
  and missing-browser behavior;
- installed static assets and route/content types.

Manual browser gates must cover the responsive and accessibility cases above.
Do not add a heavyweight browser-test dependency for this small surface unless
manual regressions demonstrate the need; backend behavior carries the
automated correctness burden.

## Stop condition

Stop when the GUI can inspect available evidence, configure every currently
supported offline profile and the existing live-system benchmark, prevent or
explain invalid axes, show honest capture/storage/work estimates, and export a
CLI-valid canonical job without writing artifacts or starting processes.

Do not add job execution, live logs, cancellation, report visualization,
artifact management, remote access, user accounts, or plugin architecture in
this deliverable. If operators consistently need a run button after using the
MVP, plan a narrow process-supervision extension against observed workflow
friction.

## Deliverables

- Installed loopback-only configurator service and static local UI.
- Four question-first flows backed by the shared profile registry.
- Artifact inspection, variant editing, estimates, validation, import/export,
  and exact command preview.
- Endpoint, contract, packaging, safety, and manual visual/accessibility proof.
- Wide/narrow reference screenshots and updated operator documentation.
