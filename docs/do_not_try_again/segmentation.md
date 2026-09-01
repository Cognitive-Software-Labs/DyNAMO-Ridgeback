# Segmentation candidates

Research only. These are alternatives to the current
[box-prompted producer](../target_localization/segmentation.md), not additional supported
runtime producers or committed implementation tasks.

| Candidate | Evidence/status | What would justify further work |
|---|---|---|
| Florence-2 | Rejected in the recorded spike: leg-gap fidelity, latency, and overlapping-instance separation failed | New evidence addressing those failures, not an automatic retry |
| SAM3 concept segmentation | Passed the three offline gates; not adopted or integrated | Explicit detector-plus-segmenter replacement decision, robot GPU budget, integrated distance/coverage comparison |
| Sim-trained dedicated segmenter | Proposed, not spiked | Accept closed vocabulary and training maintenance; demonstrate dataset and sim-to-real feasibility |

Detailed measurements, checkpoint/version pins, the eight-frame dataset, and
artifact references live once in [Segmentation experiments](../history/segmentation_experiments.md).
The SAM3 spike's untested benchmark-parity gate must not be reported as passed.

## Evaluation criteria for a new experiment

1. State whether the candidate replaces segmentation alone or also detection;
   retain one index-aligned mask per accepted detection at the integration boundary.
2. Fix the frames, target classes, occlusion/crowding cases, baseline, and gates
   before evaluating. Reuse the recorded protocol as a starting point, not as
   evidence that its small simulated dataset represents hardware.
3. Measure silhouette/leg-gap fidelity, instance separation, warm synchronized
   latency (all instances), and resident/peak GPU memory. Distinguish measured
   results from estimated costs and from mask-score confidence.
4. An offline pass permits an adoption decision or opt-in integration experiment,
   not a default change. Compare distance errors, misses/coverage, and costs on
   fixed benchmark scenes before claiming pipeline parity.
5. Validate imagery and resource limits on the target robot before deployment;
   see [hardware validation](../plans/camera_hardware_validation.md).

## Untested proposal: a sim-trained dedicated segmenter

**How it works:** generate per-instance labels from a simulator with suitable
ground-truth output (the data-generation pipeline is not implemented here), fine-tune a small instance-segmentation network (YOLO-seg family) on
them, run it standalone — no detector, no prompting.

**Hypothesized benefits, not measured here:** low inference cost (real-robot CPU feasibility untested);
detector-independent by construction; multi-instance native; crowding handled
by putting clutter in the training scenes — which sim generates for free;
reduced manual annotation, with dataset generation and validation still required.

**Against:** closed vocabulary — the network knows the G1 and nothing else; a
new target class means regenerating data and retraining; sim-to-real transfer
on real D455 imagery is the classic failure mode and must be validated before
any real-hardware reliance; a training pipeline becomes repo infrastructure to
own and maintain.
