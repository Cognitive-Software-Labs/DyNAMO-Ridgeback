#!/bin/bash
# One owned simulation launch; Ctrl-C shuts down its process tree.
set -e
PARKING_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PARKING_BACKEND="${1:-}"
case "$PARKING_BACKEND" in
    gz|isaac) shift ;;
    *) echo "Usage: $0 gz|isaac [--plan-only|--planning-only] [name:=value ...]" >&2; exit 2 ;;
esac
PARKING_PLAN_ONLY=false
PARKING_EXPLORE=true
PARKING_MOCK=false
PARKING_DRIVE=true
if [ "${1:-}" = --plan-only ]; then
    shift
    PARKING_PLAN_ONLY=true
    PARKING_EXPLORE=false
    PARKING_MOCK=true
    PARKING_DRIVE=false
elif [ "${1:-}" = --planning-only ]; then
    shift
    # Existing exploration (including G1 mapping/observation return) may move;
    # the pre-parking/final/wait/return-to-start execution remains disabled.
    PARKING_DRIVE=false
fi

# No broad cleanup: refuse another simulator rather than touching its owner.
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/ridgeback-parking-${UID}.lock"
flock -n 9 || { echo 'Another parking demo owns this workstation lock.' >&2; exit 1; }
if pgrep -f '(^|/)(isaac_runner\.py|gzserver)( |$)|[/ ]gz sim( |$)' >/dev/null; then
    echo 'A simulator is already running. Stop it in its owning terminal before this demo.' >&2
    exit 1
fi
source "$PARKING_ROOT/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI="file://$PARKING_ROOT/install/ridgeback_autonomy/share/ridgeback_autonomy/config/parking_demo_cyclonedds.xml"
export GZ_PARTITION="ridgeback-parking-${UID}-${ROS_DOMAIN_ID}" GZ_IP=127.0.0.1
# Environments can live outside this checkout; never guess a physical service.
export ISAAC_PYTHON="${ISAAC_PYTHON:-$PARKING_ROOT/isaac_venv/bin/python3}"
if [ -z "${RIDGEBACK_PERCEPTION_VENV:-}" ] && [ -d "$PARKING_ROOT/perception_venv" ]; then
    export RIDGEBACK_PERCEPTION_VENV="$PARKING_ROOT/perception_venv"
fi
if [ -z "${RIDGEBACK_PERCEPTION_VENV:-}" ] || [ ! -x "$RIDGEBACK_PERCEPTION_VENV/bin/python3" ]; then
    echo 'Set RIDGEBACK_PERCEPTION_VENV to the existing perception_venv before launching.' >&2
    exit 1
fi
cd "$PARKING_ROOT"
export RIDGEBACK_WORKSPACE="$PARKING_ROOT"
exec ros2 launch ridgeback_autonomy ridgeback_mission.launch.py \
    backend:="$PARKING_BACKEND" demo_scene_enabled:=true \
    mission_management_enabled:=true exploration_enabled:="$PARKING_EXPLORE" \
    stationary_mock_handoff_enabled:="$PARKING_MOCK" \
    parking_navigation_enabled:="$PARKING_DRIVE" demo_plan_only:="$PARKING_PLAN_ONLY" \
    headless:=false "$@"
