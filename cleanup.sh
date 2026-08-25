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

kill_matches() {
    local pattern="$1"
    pgrep -u "$CURRENT_USER" -f "$pattern" 2>/dev/null | while read -r pid; do
        if [ "$pid" != "$SELF_PID" ] && [ "$pid" != "$PARENT_PID" ]; then
            kill -9 "$pid" 2>/dev/null || true
        fi
    done
}

# Stop a benchmark screen recorder FIRST, and gently. ffmpeg writes the mp4
# index when it exits, so the kill -9 below would leave an unplayable file.
if pgrep -u "$CURRENT_USER" -f "ffmpeg.*x11grab" >/dev/null 2>&1; then
    echo "Stopping screen recorder..."
    pkill -INT -u "$CURRENT_USER" -f "ffmpeg.*x11grab" 2>/dev/null || true
    for _ in 1 2 3 4 5 6 7 8 9 10; do
        pgrep -u "$CURRENT_USER" -f "ffmpeg.*x11grab" >/dev/null 2>&1 || break
        sleep 0.5
    done
    kill_matches "ffmpeg.*x11grab"
fi

# Kill ros2 launch processes first (they may respawn children)
kill_matches "ros2.*launch"
sleep 0.5

# Kill Gazebo
kill_matches "gz sim"
kill_matches "ruby.*gz"
kill_matches "gz-sim"

# Kill all known ROS node executables
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
    g1_mask_measurement_node
    g1_estimate_viz_node
    aligned_depth_node
    g1_distance_benchmark_runner
    g1_detection_node
    velocity_overlay_node
    coverage_overlay_node
    hud_node
    imu_filter_madgwick_node
    imu_filter_madgwick
    frontier_explorer_node
)

for pat in "${PATTERNS[@]}"; do
    kill_matches "$pat"
done

# Catch-all for this package's nodes. The named list above has repeatedly gone
# stale as nodes were added, leaving orphans alive for hours after a launch was
# killed -- and duplicates then fight over the measurement topics on the next
# run. Every node here is installed under lib/ridgeback_autonomy, so match that.
kill_matches "lib/ridgeback_autonomy/"

sleep 1

# Verify nothing is left
REMAINING=$(ps -u "$CURRENT_USER" -o user=,pid=,pcpu=,pmem=,args= | grep -E "ros2|gz sim|parameter_bridge|slam_toolbox|nav2|explore|ekf_node|tf_relay|robot_state_pub|joy_linux|teleop|marker_server|image_bridge|rviz" | grep -v grep | grep -v cleanup.sh | grep -v start_exploration.sh | grep -v "bash -c" || true)

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
STILL=$(ps -u "$CURRENT_USER" -o user=,pid=,pcpu=,pmem=,args= | grep -E "ros2|gz sim|parameter_bridge|slam_toolbox|nav2|explore|ekf_node|tf_relay|robot_state_pub|joy_linux|teleop|marker_server|image_bridge|rviz" | grep -v grep | grep -v cleanup.sh | grep -v start_exploration.sh | grep -v "bash -c" || true)
if [ -n "$STILL" ]; then
    echo "WARNING: Could not kill:"
    echo "$STILL"
else
    echo "All ROS/Gazebo processes terminated."
fi
