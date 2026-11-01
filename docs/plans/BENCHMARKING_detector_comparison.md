# BENCHMARKING — Humanoid detector comparison

Status: **Paused by the operator, 1 November 2026.** No further comparison
experiments are scheduled. The selected production operating point remains in
place; comparison-only tooling has been removed from the checkout after saving
the private snapshot below.

This plan owns only unfinished work and resumption instructions. The
[detection reference](../target_localization/detection.md) owns current behavior
and parameters. Experiment results, sampling details, limitations and the
operating-point decision belong to the archived record linked at the end;
they are intentionally not repeated here.

## Preserved work

Private tooling snapshot, outside the Git checkout:

```text
/home/stefi/ScenesForDetectionTesting/comparison-snapshots/detector-comparison-pause-20261101.tar.gz
SHA-256: 521dbf3ab1467db4fc5e228e56b19b730fba1890934a6af18e1df0b610d8d1b4
```

The adjacent `.sha256` file verifies the archive. Inside it, `manifest.json`
records the source commit and per-file hashes; `RESTORE.md` explains restoration.
`checkout/` contains the evaluator package, CLI, three test modules, frozen
configuration/specification files and the full pre-pause plan. It is a snapshot
of the tooling at pause, not proof of the exact historical inference state.

Images, annotations, weights and run outputs are **not** in the snapshot. Keep
them private in their existing locations, identified in the archived record.
This local archive is not an independent backup.

## Work remaining if resumed

- Resolve the outstanding annotation-scope decision before treating accuracy
  selection as final; consult the archived record and preserved scope policy.
- Freeze a fresh baseline and evaluation contract against the production
  settings at resumption. Preserve historical experiment identities rather than
  changing old configs or relabeling their predictions.
- Restore and verify the evaluator, dataset integrity, source-group separation,
  neutral inference inputs, deterministic matching and baseline parity.
- Complete independent development comparisons: generic OWL wording, explicit
  robot/person competition, crop verification and standalone alternatives.
  The snapshot retains the detailed candidate grids and scoring procedure.
- Measure complete-method workstation compute on an uncontended GPU. Cached
  threshold sweeps do not establish compute cost.
- Consider training only if the untrained candidates justify it and source
  permissions allow it. Keep training/validation groups separate.
- Freeze finalists before held-out evaluation; assess lab and photograph
  populations separately, plus robot type, visibility and mixed-box limits.
- Qualify any selected method on Thor and through the downstream measurement
  pipeline before integration. Existing source-group correlations and missing
  coverage must remain explicit.

No comparison result establishes recognition of every humanoid or universal
person rejection. Keep prompts generic; do not introduce G1/R1-specific hints.
Preserve robot recall against the newly frozen baseline before preferring fewer
human false detections, and inspect conservative mixed-box behavior alongside
ordinary matching.

## Resume procedure

1. Explicitly reopen this work and review the archived decision and limitations.
2. Verify the snapshot checksum, extract into a temporary directory and verify
   all paths against `manifest.json` before copying files into the checkout.
   Compare with current files first; retain this compact plan rather than
   overwriting it with the pre-pause plan.
3. Use the saved full plan for the detailed matching, candidate, compute and
   checkpoint contracts. Reconcile them with current production code and model
   versions before freezing a new run. Do not silently reuse an obsolete
   production-baseline identity.
4. Confirm the private inputs and evidence are still present. Preserve approved
   annotations and historical runs; write derivatives to a new private run.
5. Restore model-free checks, then verify baseline parity before expensive runs.
   Keep held-out inputs out of tuning and log configuration/model provenance.
6. Resume only the required checkpoint, updating this plan with remaining work;
   keep completed experimental findings in their dated archive record.

The [combined-model throughput plan](BENCHMARKING_model_throughput.md) owns the
separate segmentation/depth throughput experiment.

## Archived evidence

- [OWLv2 threshold selection and live exploration observation](../../archive/engineering/2026-11-01-owlv2-threshold-selection.md) — canonical record of the offline experiments, sampled frames, live operator observation, decision and limitations.
