#!/bin/bash
# Kill ALL ROS 2 / Gazebo processes from previous launches.
# Usage: bash cleanup.sh
#
# This is aggressive — it kills everything ROS/Gazebo related.
# Run before launching to ensure a clean slate.

set -e

echo "=== Killing all ROS/Gazebo processes ==="
CURRENT_USER="$(id -un)"
SELF_PID="$$"
PARENT_PID="$PPID"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Every executable this workspace installs lives under install/<pkg>/lib/, so
# one path match covers all of them -- present and future. The enumerated
# PATTERNS list below cannot do that and rotted twice during the Isaac port:
# scan_merger_node (added in 2ed1674a) was never listed, and neither was the
# leftover-verification regex, so a merger orphaned on 2026-09-10 survived
# every cleanup for 16 h and silently double-published the merged scan into a
# later benchmark run. Matching `/lib/` specifically (not `/install/`) keeps
# this from matching a shell whose own command line contains
# `source install/setup.bash` or a `share/...launch.py` path.
REPO_NODES_RE="${SCRIPT_DIR}/install/[^ ]*/lib/"

kill_matches() {
    local pattern="$1"
    pgrep -u "$CURRENT_USER" -f "$pattern" 2>/dev/null | while read -r pid; do
        if [ "$pid" != "$SELF_PID" ] && [ "$pid" != "$PARENT_PID" ]; then
            kill -9 "$pid" 2>/dev/null || true
        fi
    done
}

# Kill ros2 launch processes first (they may respawn children)
kill_matches "ros2.*launch"
sleep 0.5

# Kill Gazebo
kill_matches "gz sim"
kill_matches "ruby.*gz"
kill_matches "gz-sim"

# Kill Isaac Sim (user-scoped: shared box, leave co-tenants alone)
kill_matches "isaac_runner.py"
kill_matches "omni.kit"

# Kill this workspace's own nodes by install path (see REPO_NODES_RE above).
# Scoped to THIS checkout on purpose: the box is shared and worktrees run in
# parallel, so a bare node-name match would reach into a co-worker session.
kill_matches "$REPO_NODES_RE"

# Kill known ROS node executables that come from /opt/ros rather than this
# workspace (nav2, slam_toolbox, rviz, ...), plus the historical gz-era names.
PATTERNS=(
    parameter_bridge
    image_bridge
    async_slam_toolbox_node
    sync_slam_toolbox_node
    lifecycle_manager
    controller_server
    planner_server
    smoother_server
    behavior_server
    velocity_smoother
    collision_monitor
    bt_navigator
    explore
    ekf_node
    robot_state_publisher
    joy_linux_node
    teleop_node
    tf_relay
    spawner
    cmd_vel_bridge
    marker_server
    twist_server
    ros2-daemon
    rviz2
    camera_windows_node
    g1_detector_node
    g1_camera_measurement_node
    g1_lidar_measurement_node
    g1_overlay_node
    g1_distance_benchmark_runner
    g1_detection_node
    velocity_overlay_node
    coverage_overlay_node
    hud_node
    imu_filter_madgwick_node
    imu_filter_madgwick
    frontier_explorer_node
    isaac_runner
)

for pat in "${PATTERNS[@]}"; do
    kill_matches "$pat"
done

sleep 1

# One leftover regex, used by both checks below. It was duplicated verbatim and
# BOTH copies omitted this workspace's own install path, so an orphaned repo
# node was neither killed nor reported -- the failure that hid a stray
# scan_merger for 16 h. Keep REPO_NODES_RE first.
LEFTOVER_RE="${REPO_NODES_RE}|ros2|gz sim|parameter_bridge|slam_toolbox|nav2|explore|ekf_node|tf_relay|robot_state_pub|joy_linux|teleop|marker_server|image_bridge|rviz|overlay_node|hud_node"

leftovers() {
    ps -u "$CURRENT_USER" -o user=,pid=,pcpu=,pmem=,args= \
        | grep -E "$LEFTOVER_RE" \
        | grep -v grep | grep -v cleanup.sh | grep -v start_exploration.sh \
        | grep -v "bash -c" || true
}

# Verify nothing is left
REMAINING=$(leftovers)

if [ -n "$REMAINING" ]; then
    echo "WARNING: Some processes still running:"
    echo "$REMAINING"
    echo ""
    echo "Force-killing remaining PIDs..."
    echo "$REMAINING" | awk '{print $2}' | xargs kill -9 2>/dev/null || true
    sleep 0.5
fi

# Clean up FastRTPS/FastDDS shared memory files AND semaphore locks.
# POSIX semaphores live in /dev/shm/sem.* — the glob fastrtps_* misses them,
# leaving stale port-mutex locks that cause "Failed init_port … open_and_lock_file
# failed" on the next launch, which breaks TRANSIENT_LOCAL topic delivery.
rm -f /dev/shm/fastrtps_* 2>/dev/null || true
rm -f /dev/shm/sem.fastrtps_* 2>/dev/null || true

echo "=== Cleanup complete ==="
# Final check
STILL=$(leftovers)
if [ -n "$STILL" ]; then
    echo "WARNING: Could not kill:"
    echo "$STILL"
else
    echo "All ROS/Gazebo processes terminated."
fi
