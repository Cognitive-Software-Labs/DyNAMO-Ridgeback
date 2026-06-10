#!/usr/bin/env bash
# One-time host setup for ROS 2 / CycloneDDS large-message tuning.
#
# DDS drops data when large payloads (camera images, costmaps, point clouds) are
# split into UDP fragments and the kernel buffers are too small. This installs a
# persistent sysctl drop-in so the fix survives reboots -- run once, never again.
#
# Requires sudo. After this, start_exploration.sh stops warning about rmem_max.
# ref: https://www.stereolabs.com/docs/ros2/dds-and-network-tuning
set -euo pipefail

CONF=/etc/sysctl.d/60-ros-dds.conf

read -r -d '' BODY <<'EOF' || true
# ROS 2 / CycloneDDS large-message network tuning (DyNAMO-Ridgeback)
# Installed by tools/setup_dds.sh
net.core.rmem_max = 2147483647
net.ipv4.ipfrag_high_thresh = 134217728
net.ipv4.ipfrag_time = 3
EOF

echo "Installing $CONF (needs sudo)..."
echo "$BODY" | sudo tee "$CONF" >/dev/null
sudo sysctl --system >/dev/null

echo "Done. Active values:"
sysctl net.core.rmem_max net.ipv4.ipfrag_high_thresh net.ipv4.ipfrag_time
