# Exact-stamp depth-delivery experiments

Rejected or bounded alternatives from the 2026-09-01 investigation. The selected
CycloneDDS remedy and full evidence are in
[exact-stamp depth availability](../history/exact_stamp_depth_availability.md).

## Do not widen the subscription queue as the fix

Changing the depth subscription history from 5 to 30 did not improve the
predicted internal counter. In isolated one-repeat runs, unresolved exact lookup
misses changed from 84/333 (25.2%) at depth 5 to 88/316 (27.8%) at depth 30.
Benchmark coverage happened to move from 68.3% to 72.6%, but total detections
differed and the mechanism counter worsened. The experimental parameter and its
tests were reverted.

Reconsider only if a future trace shows valid samples queued in DDS but not taken
before the configured history is overwritten. An aggregate coverage change by
itself is insufficient.

## Do not use the UDP-only Fast DDS profile for this symptom

Disabling Fast DDS shared memory made image delivery worse: the one-repeat run
recorded 178/309 internal misses (57.6%) and 39.4% benchmark coverage. The
repository's toggle, XML profile, forced manual-mapping environment, stale-file
cleanup, and special diagnostics were therefore removed.

Do not reintroduce them without a new transport trace that moves the exact
missing-stamp counter and a demonstrated requirement that CycloneDDS cannot meet.

## Do not add retries or change exact matching

None of 111 traced missing depth stamps arrived later. The misses were permanent
gaps at the node callback boundary, not a lookup race. Retrying, sleeping,
tolerance widening, or nearest-frame substitution cannot recover a message that
the subscriber never received and would weaken the timestamp contract.

## Do not redesign the worker or executor for this symptom

The trace recorded no pending detection replacements, sub-millisecond lock
timings, 0.266 ms mean dequeue age, and 2.972 ms mean worker time. The missing
stamps never reached the callback. A multithreaded executor, worker priority,
or broader concurrency redesign therefore targets the wrong counter.
