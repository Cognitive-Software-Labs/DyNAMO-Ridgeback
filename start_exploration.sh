#!/bin/bash
# Clean up stale processes and launch the exploration stack.
#
# Usage:
#   bash start_exploration.sh                                # mock_hospital + explore_lite
#   bash start_exploration.sh office                         # office + explore_lite
#   bash start_exploration.sh mock_hospital custom           # mock_hospital + custom explorer
#   EXPLORER=custom bash start_exploration.sh office         # office + custom explorer
#   RMW_IMPLEMENTATION=rmw_fastrtps_cpp bash start_exploration.sh  # explicit RMW override

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

# CycloneDDS is the measured default for image/depth delivery. An explicitly
# selected RMW remains authoritative.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

set +u
source /opt/ros/jazzy/setup.bash
source "$SCRIPT_DIR/install/setup.bash"
set -u

bash "$SCRIPT_DIR/cleanup.sh"

# One non-clobbering directory per launch, with console and ROS logs together.
# LOG_DIR overrides the exploration output root; ROS_LOG_DIR remains independent.
LOG_DIR="${LOG_DIR:-$SCRIPT_DIR/artifacts/exploration}"
mkdir -p "$LOG_DIR"
RUN_LOG_DIR="$(mktemp -d "$LOG_DIR/ridgeback_$(date +%Y-%m-%d_%H-%M-%S)_${WORLD}_${EXPLORER}.XXXXXX")"
LOG_FILE="$RUN_LOG_DIR/console.log"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$RUN_LOG_DIR/ros}"
mkdir -p "$ROS_LOG_DIR"
echo "Logging to $LOG_FILE"
exec > >(tee "$LOG_FILE") 2>&1

exec ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py \
    world:="$WORLD" \
    explorer:="$EXPLORER" \
    "$@"
