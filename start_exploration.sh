#!/bin/bash
# Clean up stale processes and launch the exploration stack.
#
# Usage:
#   bash start_exploration.sh                                # mock_hospital
#   bash start_exploration.sh office                         # office world
#   bash start_exploration.sh warehouse key:=value ...       # extra launch args
#   bash start_exploration.sh headless_rendering:=true       # default world, EGL rendering
#   bash start_exploration.sh --ros-args --log-level info     # pass options through
#   RMW_IMPLEMENTATION=rmw_fastrtps_cpp bash start_exploration.sh  # explicit RMW override
#
# The positional world is optional: everything from the first launch argument
# or option onward is passed through without requiring a world placeholder.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Everything bound for ros2 launch rather than for this script: ``name:=value``
# arguments and ``-``-prefixed options. Neither can name a world, so neither may
# be consumed as the optional positional.
is_passthrough() {
    [[ "$1" == *":="* || "$1" == -* ]]
}

WORLD=mock_hospital
if [[ $# -gt 0 ]] && ! is_passthrough "$1"; then
    WORLD="$1"
    shift
fi

# Resolve the provider before setting process policy. ``backend`` is canonical;
# ``sim`` remains a compatibility alias and is only consulted when backend was
# not supplied.
BACKEND=
for arg in "$@"; do
    [[ "$arg" == backend:=* ]] && BACKEND="${arg#backend:=}"
done
if [ -z "$BACKEND" ]; then
    BACKEND=gz
    for arg in "$@"; do
        [[ "$arg" == sim:=* ]] && BACKEND="${arg#sim:=}"
    done
fi

# CycloneDDS is the measured default for image/depth delivery. An explicitly
# selected RMW remains authoritative. Public launches supply the package-owned
# CycloneDDS participant-index config when CYCLONEDDS_URI is unset.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
if [ "$BACKEND" != hardware ]; then
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
fi

set +u
source /opt/ros/jazzy/setup.bash
source "$SCRIPT_DIR/install/setup.bash"
set -u

if [ "$BACKEND" = hardware ]; then
    echo "Hardware backend: preserving existing ROS and Clearpath processes"
else
    bash "$SCRIPT_DIR/cleanup.sh"
fi

# One non-clobbering directory per launch, with console and ROS logs together.
# LOG_DIR overrides the exploration output root; ROS_LOG_DIR remains independent.
LOG_DIR="${LOG_DIR:-$SCRIPT_DIR/artifacts/exploration}"
mkdir -p "$LOG_DIR"
RUN_LOG_DIR="$(mktemp -d "$LOG_DIR/ridgeback_$(date +%Y-%m-%d_%H-%M-%S)_${BACKEND}_${WORLD}.XXXXXX")"
LOG_FILE="$RUN_LOG_DIR/console.log"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$RUN_LOG_DIR/ros}"
mkdir -p "$ROS_LOG_DIR"
echo "Logging to $LOG_FILE"
exec > >(tee "$LOG_FILE") 2>&1

exec ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py \
    world:="$WORLD" \
    "$@"
