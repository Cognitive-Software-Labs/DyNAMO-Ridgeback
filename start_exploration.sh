#!/bin/bash
# Clean up stale processes and launch the exploration stack.
#
# Usage:
#   bash start_exploration.sh                   # defaults to mock_hospital
#   bash start_exploration.sh office
#   bash start_exploration.sh mock_hospital

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD="${1:-mock_hospital}"
DEPTH_ANYTHING_ENABLED="${DEPTH_ANYTHING_ENABLED:-false}"
shift 2>/dev/null || true

# FastDDS shared-memory locks can get stale after Gazebo/ROS crashes and make
# nodes disappear from discovery. Use UDP-only FastDDS transport for local runs.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$SCRIPT_DIR/fastrtps_no_shm.xml}"
export RMW_FASTRTPS_USE_QOS_FROM_XML="${RMW_FASTRTPS_USE_QOS_FROM_XML:-1}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

set +u
source /opt/ros/jazzy/setup.bash
source "$SCRIPT_DIR/install/setup.bash"
set -u

bash "$SCRIPT_DIR/cleanup.sh"

exec ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py \
    world:="$WORLD" \
    depth_anything_enabled:="$DEPTH_ANYTHING_ENABLED" \
    "$@"
