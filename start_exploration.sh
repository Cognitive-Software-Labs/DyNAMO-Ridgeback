#!/bin/bash
# Clean up stale processes and launch the exploration stack.
#
# Usage:
#   bash start_exploration.sh                                # mock_hospital + explore_lite
#   bash start_exploration.sh office                         # office + explore_lite
#   bash start_exploration.sh mock_hospital custom           # mock_hospital + custom explorer
#   EXPLORER=custom bash start_exploration.sh office         # office + custom explorer
#   DEPTH_ANYTHING_ENABLED=true bash start_exploration.sh    # enable Depth-Anything
#   FASTRTPS_NO_SHM=true bash start_exploration.sh           # use the UDP-only FastDDS profile

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD="${1:-mock_hospital}"
if [[ $# -gt 0 ]]; then
    shift
fi

EXPLORER="${EXPLORER:-explore_lite}"
if [[ $# -gt 0 && "$1" != *":=" ]]; then
    EXPLORER="$1"
    shift
fi

if [[ "$EXPLORER" != "explore_lite" && "$EXPLORER" != "custom" ]]; then
    echo "Unknown explorer '$EXPLORER'. Expected 'explore_lite' or 'custom'." >&2
    exit 2
fi

DEPTH_ANYTHING_ENABLED="${DEPTH_ANYTHING_ENABLED:-false}"

# FastDDS shared-memory locks can get stale after Gazebo/ROS crashes and make
# nodes disappear from discovery. The UDP-only FastDDS profile sidesteps that.
# Default off (shared memory on); set FASTRTPS_NO_SHM=true to use the UDP-only profile.
FASTRTPS_NO_SHM="${FASTRTPS_NO_SHM:-false}"
if [[ "$FASTRTPS_NO_SHM" == "true" ]]; then
    export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
    export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$SCRIPT_DIR/fastrtps_no_shm.xml}"
    export RMW_FASTRTPS_USE_QOS_FROM_XML="${RMW_FASTRTPS_USE_QOS_FROM_XML:-1}"
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

set +u
source /opt/ros/jazzy/setup.bash
source "$SCRIPT_DIR/install/setup.bash"
set -u

bash "$SCRIPT_DIR/cleanup.sh"

# Timestamped, non-clobbering log: one file per launch so a stalled session
# can be reviewed after the fact instead of being overwritten by the next run.
LOG_DIR="${LOG_DIR:-$SCRIPT_DIR/logs}"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/ridgeback_$(date +%Y-%m-%d_%H-%M-%S)_${WORLD}_${EXPLORER}.log"
echo "Logging to $LOG_FILE"
exec > >(tee "$LOG_FILE") 2>&1

exec ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py \
    world:="$WORLD" \
    explorer:="$EXPLORER" \
    depth_anything_enabled:="$DEPTH_ANYTHING_ENABLED" \
    "$@"
