# r100_0160 restarted Thor processes hidden from Intel by IGMP snooping

Recorded dates: 2026-10-13
Tested revisions: unknown
Provenance: partial

This record was extracted from the troubleshooting entry at the
[documentation snapshot](https://github.com/Cognitive-Software-Labs/DyNAMO-Ridgeback/blob/10dc4940c8663619de1a88c2aa21e1ab5715b2e7/docs/troubleshooting.md#intel-receives-nothing-from-a-restarted-thor-for-minutes)
and from the message of commit
[`f1281d42`](https://github.com/Cognitive-Software-Labs/DyNAMO-Ridgeback/commit/f1281d42fd7389babbf21f8e79434b1e5152d0a8),
which shipped the fix. That commit is the change that followed the
measurement, not a tested revision.

Gaps: the repository does not hold the raw captures, the number of Thor
restarts measured, the checkout or revision each host ran, or the robot
Ethernet switch model. The measurement method below is reconstructed from the
two sources above.

## Symptom

After Thor's localization restarted, Intel's `localization/status` stayed
`missing` and no health or measurements arrived. Delivery then resumed without
any action. Thor-local echoes looked healthy. Sometimes one early message got
through, and some publishers (for example the detector) appeared on Intel while
others never did.

## Measurement

- Intel heard Thor's own DDS discovery announcements stop within seconds of a
  restart. They resumed 313–381 s later, and delivery to Intel resumed at the
  same moment.
- Firewalls, ports and socket buffers were ruled out.
- Unicast DDS peers did not help Intel's consumers that were already running.
- With `br0` set as the IGMP querier, the first measurement reached Intel
  0.2 s after Thor's detector was ready.

## Conclusion

The robot Ethernet had no IGMP querier. IGMP snooping therefore stopped
forwarding the restarted processes' discovery multicast to Intel until the
membership entry aged out. netplan cannot set the querier. The fix commit
added a systemd-networkd drop-in for `br0` and a `check_link` test for it. A
later reboot kept the drop-in in effect; see the
[RViz observer record](2026-10-13-r100-0160-intel-rviz-observer.md#follow-up-on-2026-10-14).

The measurement covers restarts while both hosts stayed up, on one robot
segment. It did not test a switch replacement, Intel's own restart, or a
segment where another device is the querier.
