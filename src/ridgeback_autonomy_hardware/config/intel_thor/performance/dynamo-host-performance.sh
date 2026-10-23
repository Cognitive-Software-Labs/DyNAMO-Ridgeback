#!/bin/sh
# Installed by tools/intel_thor/host_performance as
# /usr/local/sbin/dynamo-host-performance; edit the repository copy.
#
# Every CPU frequency policy runs the performance governor. Where the driver
# also offers an energy preference (intel_pstate on Intel), it is performance
# too; intel_pstate already implies that under the performance governor and
# may refuse the write, which is harmless.
for policy in /sys/devices/system/cpu/cpufreq/policy*; do
    echo performance > "$policy/scaling_governor" || exit 1
    if [ -w "$policy/energy_performance_preference" ]; then
        { echo performance > "$policy/energy_performance_preference"; } 2>/dev/null || true
    fi
done
