# Qualification campaign stopped by systemd-oomd

Recorded dates: 2026-11-03
Tested revisions: `4e22a20fdc5e15622f0980f81e0de5facca187bf`
Provenance: partial

## Incident and conclusion

The qualification campaign started at 01:24:08 CET on 2026-11-03 and was killed
by systemd-oomd at 02:25:17 CET. The benchmark did not consume most of the
workstation's physical RAM. Other applications were running, but the evidence
does not identify the workload that caused the pressure spike or establish
that another user's job caused it.

The affected service was
`ridgeback-qualification-qualification-20261103-4e22a20.service`, with evidence
under `artifacts/benchmarks/qualification_20261103_4e22a20/`. Its frozen input
record identifies the implementation and experimental inputs. Host history is
incomplete: no per-process memory snapshot at the kill time was found, and the
system journal containing oomd's detailed decision was inaccessible to stefi.

## Measured evidence

All times below are Europe/Berlin, CET. GiB values use binary units.

| Observation | Evidence and interpretation |
| --- | --- |
| Physical RAM | 91.96 GiB. |
| Benchmark service peak | 8.23 GiB, approximately 9% of physical RAM; recorded swap peak zero. The service had no finite `MemoryHigh` or `MemoryMax`. |
| stefi account slice peak | 36.30 GiB retained since the slice became active on 2026-10-17. This is a historical upper bound, not account usage measured at 02:25. |
| Another application | Claude's desktop scope under stefi was killed at 02:25:01, 16 seconds before the benchmark. Its recorded peak was 4.8 GiB. The two scopes' peaks need not have occurred simultaneously. |
| Before the incident | At 02:20:01, sysstat recorded 38.01 GiB available RAM and 99.98% occupancy of the machine's 4 GiB swap. |
| After the incident | At 02:30:03, sysstat recorded 52.56 GiB available RAM. |
| Pressure during the interval | Between the 02:20 and 02:30 samples, memory pressure averaged 16.37% for some stalls and 15.67% for full stalls. Ten-minute aggregates do not locate or attribute the short pressure spike. |

The retained service result is `oom-kill`; the user journal explicitly records
systemd-oomd killing 34 processes in the benchmark unit. This was not a normal
benchmark completion or an 8 GiB service memory cap.

## What remains unknown

The available RAM reading five minutes before the kill does not exclude a
subsequent transient spike. Almost-full swap alone does not prove that RAM was
exhausted at that instant. Neither observation identifies the allocation owner.

During inspection, `oomctl` reported a 50% memory-pressure threshold on
`user@1009.service` and a default duration of 20 seconds. This describes stalled
execution, not a 50%-of-RAM limit. The current configuration does not prove that
every setting was identical at the incident time.

systemd-oomd monitors pressure and selects eligible descendant cgroups to kill.
Being selected is not evidence that the selected cgroup created the pressure;
see the [systemd-oomd documentation](https://www.freedesktop.org/software/systemd/man/latest/systemd-oomd.service.html).
The exact triggering workload therefore remains unconfirmed. No evidence
supports attributing the incident to a particular other user.

## Effect on qualification

Two Gazebo repetition-1 conditions completed: box/stereo and silhouette/stereo,
each with 109 trials, zero skipped trials and 545 stored batches. Box/monocular
was interrupted. These results remain preserved in the original bundle; they
are not a completed 16-condition campaign or a completed replay qualification.

The killed owner could not record its final elapsed phase. The subsequent
restart accounting charged 3,668.369798 seconds from systemd's monotonic start
and exit timestamps to the same original time allocation. Restarts preserve
failed evidence and start a fresh full matrix rather than merging interrupted
sessions into new repetitions.

## Follow-up and limits

Raw memory accounting, user journal entries, sysstat samples and the inspected
oomd policy were saved under
`artifacts/benchmarks/oom_investigation_20261103/`. The files are
`benchmark_memory.txt`, `incident_user_journal.txt`,
`system_memory_history.txt`, `system_swap_history.txt`,
`system_pressure_history.txt`, `account_memory_bounds.txt` and
`oomd_policy.txt`.

A local recorder for host, account and campaign memory/pressure, coupled with
an early orderly-stop guard, is recommended follow-up. It was not implemented
by this investigation. Such a guard would improve diagnosis and reduce the
chance of another abrupt loss; it cannot guarantee protection from an abrupt
external pressure spike. Exclusive use by heavy workloads reduces contention
but does not establish the original cause.

No oomd setting, service exemption, swap configuration or benchmark measurement
setting was changed. The existing workstation-admission guard remains active;
this report does not qualify execution alongside other heavy jobs.
