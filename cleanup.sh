#!/bin/bash
# Kill ALL ROS 2 / Gazebo processes from previous launches.
# Usage: bash cleanup.sh
#
# This is aggressive — it kills everything ROS/Gazebo related.
# Run before launching to ensure a clean slate.

set -e

echo "=== Killing all ROS/Gazebo processes ==="

# Kill ros2 launch processes first (they may respawn children)
pkill -9 -f "ros2.*launch" 2>/dev/null || true
sleep 0.5

# Kill Gazebo
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "ruby.*gz" 2>/dev/null || true
pkill -9 -f "gz-sim" 2>/dev/null || true

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
    g1_detection_node
)

for pat in "${PATTERNS[@]}"; do
    pkill -9 -f "$pat" 2>/dev/null || true
done

sleep 1

# Verify nothing is left
REMAINING=$(ps aux | grep -E "ros2|gz sim|parameter_bridge|slam_toolbox|nav2|explore|ekf_node|tf_relay|robot_state_pub|joy_linux|teleop|marker_server|image_bridge|rviz" | grep -v grep | grep -v cleanup.sh | grep -v "bash -c" || true)

if [ -n "$REMAINING" ]; then
    echo "WARNING: Some processes still running:"
    echo "$REMAINING"
    echo ""
    echo "Force-killing remaining PIDs..."
    echo "$REMAINING" | awk '{print $2}' | xargs kill -9 2>/dev/null || true
    sleep 0.5
fi

# Clean up FastRTPS shared memory files
rm -f /dev/shm/fastrtps_* 2>/dev/null || true

echo "=== Cleanup complete ==="
# Final check
STILL=$(ps aux | grep -E "ros2|gz sim|parameter_bridge|slam_toolbox|nav2|explore|ekf_node|tf_relay|robot_state_pub|joy_linux|teleop|marker_server|image_bridge|rviz" | grep -v grep | grep -v cleanup.sh | grep -v "bash -c" || true)
if [ -n "$STILL" ]; then
    echo "WARNING: Could not kill:"
    echo "$STILL"
else
    echo "All ROS/Gazebo processes terminated."
fi
