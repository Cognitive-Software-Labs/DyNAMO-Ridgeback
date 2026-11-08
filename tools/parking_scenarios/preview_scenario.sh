#!/bin/bash
# Scene views of one parking scenario: preview_scenario.sh gz|isaac SCENARIO OUT_DIR.
# Starts start_parking_demo.sh --plan-only (no exploration, no driving, so the robot
# stays at its start) in its own session, waits until startup has spawned and
# verified the scene, captures the documentation views with capture_scene_views.py,
# and stops only that session with SIGTERM. Waits for a free workstation first.
B=$1; SC=$2; D=$(realpath -m "$3"); P=$(cd "$(dirname "$0")" && pwd); ROOT=$(cd "$P/../.." && pwd)
: "${RIDGEBACK_PERCEPTION_VENV:?set RIDGEBACK_PERCEPTION_VENV}"
mkdir -p "$D"; cd "$ROOT"; LOG="$D/${B}_${SC}_console.log"
while pgrep -f '(^|/)(isaac_runner\.py|gzserver)( |$)|[/ ]gz sim( |$)' >/dev/null; do timeout 5 tail -n0 -f /dev/null; done
# ...and for the demo's workstation lock, which a stopping demo holds until it exits.
LOCK="${XDG_RUNTIME_DIR:-/tmp}/ridgeback-parking-${UID}.lock"
until flock -n "$LOCK" true 2>/dev/null; do timeout 2 tail -n0 -f /dev/null; done
if [ "$B" = isaac ]; then
  python3 "$P/scene_views.py" isaac "$SC" "$D/isaac_${SC}_views.json" || exit 1
  export RIDGEBACK_ISAAC_VIEW_CAMERAS="$D/isaac_${SC}_views.json"
fi
git rev-parse HEAD > "$D/${B}_${SC}_commit.txt"; git status --short >> "$D/${B}_${SC}_commit.txt"
setsid bash start_parking_demo.sh "$B" --plan-only demo_scenario:="$SC" > "$LOG" 2>&1 &
LPID=$!
result=timeout
for _ in $(seq 300); do
  grep -q 'Created and verified demo scenario' "$LOG" 2>/dev/null && { result=ready; break; }
  grep -qE 'STARTUP_FAILED' "$LOG" 2>/dev/null && { result=startup_failed; break; }
  kill -0 $LPID 2>/dev/null || { result=launch_exited; break; }
  timeout 2 tail -n0 -f /dev/null
done
if [ $result = ready ]; then
  timeout 10 tail -n0 -f /dev/null    # let the views' first frames render
  ( source install/setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1 CYCLONEDDS_URI="file://$PWD/install/ridgeback_autonomy/share/ridgeback_autonomy/config/parking_demo_cyclonedds.xml"
    timeout 300 python3 "$P/capture_scene_views.py" "$B" "$SC" "$D" ) >> "$D/capture.log" 2>&1
  result=captured
fi
kill -TERM -- -$LPID 2>/dev/null
for _ in $(seq 60); do ps -eo sid= | awk -v g=$LPID '$1==g' | grep -q . || break; timeout 1 tail -n0 -f /dev/null; done
ps -eo sid= | awk -v g=$LPID '$1==g' | grep -q . && kill -KILL -- -$LPID 2>/dev/null
echo "$B $SC $result" | tee -a "$D/previews.txt"
