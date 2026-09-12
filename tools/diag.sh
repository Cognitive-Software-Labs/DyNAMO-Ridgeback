#!/bin/bash
# Full diagnostic for the Ridgeback SLAM exploration stack
# Usage: bash tools/diag.sh [logfile] [world]

set -o pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash 2>/dev/null
[ -f "$REPO_ROOT/install/setup.bash" ] && source "$REPO_ROOT/install/setup.bash" 2>/dev/null

LOGFILE="${1:-}"
WORLD_ARG="${2:-}"
NS="r100_0001"
TIMEOUT=5

section() { echo -e "\n===== $1 ====="; }

# 1. Processes
section "PROCESSES"
echo "-- Gazebo:"
pgrep -a "gz sim\|ruby.*gz" 2>/dev/null || echo "  NOT RUNNING"
echo "-- ROS launch:"
pgrep -a "ros2.*launch" 2>/dev/null || echo "  NOT RUNNING"
echo "-- Key nodes (ps):"
pgrep -af "slam_toolbox|nav2|explore|controller_server|ekf_node|parameter_bridge" 2>/dev/null | sed 's/^/  /' || echo "  NONE"

# 2. Gazebo sim state (via gz transport, independent of ROS)
section "GAZEBO SIM STATE"
if command -v gz &>/dev/null || source /opt/ros/jazzy/setup.bash 2>/dev/null; then
    WORLD_TOPIC=""
    if [ -n "$WORLD_ARG" ]; then
        WORLD_TOPIC="/world/${WORLD_ARG}/stats"
    else
        WORLD_TOPIC=$(timeout $TIMEOUT gz topic -l 2>/dev/null | grep '^/world/.*/stats$' | head -1)
    fi

    if [ -n "$WORLD_TOPIC" ]; then
        echo "  Topic: $WORLD_TOPIC"
        timeout $TIMEOUT gz topic -e -t "$WORLD_TOPIC" -n 1 2>/dev/null | head -12 || echo "  Cannot reach Gazebo transport"
    else
        echo "  No active /world/*/stats topic found"
    fi
else
    echo "  gz command not available"
fi

# 3. ROS Clock
section "ROS CLOCK (/clock)"
CLOCK=$(timeout $TIMEOUT ros2 topic echo /clock --once 2>/dev/null)
if [ -n "$CLOCK" ]; then
    echo "$CLOCK"
else
    echo "  NO DATA (timeout ${TIMEOUT}s)"
    echo "  Publisher count: $(timeout $TIMEOUT ros2 topic info /clock 2>/dev/null | grep 'Publisher count' || echo 'unknown')"
fi

# 4. TF (namespaced)
section "TF (/${NS}/tf)"
TF=$(timeout $TIMEOUT ros2 topic echo /${NS}/tf --once 2>/dev/null | head -15)
if [ -n "$TF" ]; then
    echo "$TF"
else
    echo "  NO DATA (timeout ${TIMEOUT}s)"
fi

# 5. Key topics - publish rates
section "TOPIC RATES (sampled 3s each)"
for topic in /${NS}/sensors/lidar2d_0/scan /${NS}/map /${NS}/cmd_vel; do
    echo -n "  $topic: "
    HZ=$(timeout 4 ros2 topic hz "$topic" --window 3 2>/dev/null | tail -1)
    if [ -n "$HZ" ]; then
        echo "$HZ"
    else
        echo "no data"
    fi
done

# 6. Node list (key nodes only)
section "ROS NODES (key)"
NODES=$(timeout $TIMEOUT ros2 node list 2>/dev/null | grep -v "SHM\|RTPS" | sort -u)
for pat in slam_toolbox controller_server planner_server bt_navigator explore_node ekf_node lifecycle_manager clock_bridge platform_velocity_controller; do
    match=$(echo "$NODES" | grep "$pat" | head -1)
    if [ -n "$match" ]; then
        echo "  [OK] $match"
    else
        echo "  [--] $pat NOT FOUND"
    fi
done

# 7. SLAM lifecycle state
section "SLAM LIFECYCLE"
STATE=$(timeout $TIMEOUT ros2 service call /${NS}/slam_toolbox/get_state lifecycle_msgs/srv/GetState {} 2>/dev/null | grep "label")
if [ -n "$STATE" ]; then
    echo "  $STATE"
else
    echo "  Cannot query (node missing or not a lifecycle node)"
fi

# 8. Nav2 lifecycle state
section "NAV2 LIFECYCLE"
for node in controller_server planner_server bt_navigator; do
    STATE=$(timeout $TIMEOUT ros2 service call /${NS}/${node}/get_state lifecycle_msgs/srv/GetState {} 2>/dev/null | grep "label")
    if [ -n "$STATE" ]; then
        echo "  $node: $STATE"
    else
        echo "  $node: unknown"
    fi
done

# 9. Key errors from log
section "RECENT ERRORS (from log)"
if [ -n "$LOGFILE" ] && [ -f "$LOGFILE" ]; then
    grep -E "\[ERROR\]|\[FATAL\]|process has died|RuntimeError|Traceback" "$LOGFILE" 2>/dev/null | tail -10
    echo "---"
    echo "  SLAM lines:"
    grep "slam_toolbox" "$LOGFILE" 2>/dev/null | tail -5
    echo "  Clock/EKF lines:"
    grep -E "ekf_node|clock" "$LOGFILE" 2>/dev/null | grep -iv "No clock received" | tail -5
else
    echo "  No logfile"
fi

section "DONE"
