#!/bin/bash
# Camera views of G1 from chosen robot poses: probe_g1_views.sh SCENARIO OUT_DIR NAME:X:Y:YAW_DEG...
# Gazebo only. Starts start_parking_demo.sh gz --plan-only in its own session like
# preview_scenario.sh, runs capture_g1_views.py (which moves the robot), and stops only
# that session with SIGTERM. Waits for a free workstation first.
SC=$1; D=$(realpath -m "$2"); shift 2; P=$(cd "$(dirname "$0")" && pwd); ROOT=$(cd "$P/../.." && pwd)
: "${RIDGEBACK_PERCEPTION_VENV:?set RIDGEBACK_PERCEPTION_VENV}"
mkdir -p "$D"; cd "$ROOT"; LOG="$D/${SC}_console.log"
while pgrep -f '(^|/)(isaac_runner\.py|gzserver)( |$)|[/ ]gz sim( |$)' >/dev/null; do timeout 5 tail -n0 -f /dev/null; done
LOCK="${XDG_RUNTIME_DIR:-/tmp}/ridgeback-parking-${UID}.lock"
until flock -n "$LOCK" true 2>/dev/null; do timeout 2 tail -n0 -f /dev/null; done
git rev-parse HEAD > "$D/${SC}_commit.txt"; git status --short >> "$D/${SC}_commit.txt"
setsid bash start_parking_demo.sh gz --plan-only demo_scenario:="$SC" > "$LOG" 2>&1 &
LPID=$!
result=timeout
for _ in $(seq 300); do
  grep -q 'Created and verified demo scenario' "$LOG" 2>/dev/null && { result=ready; break; }
  grep -qE 'STARTUP_FAILED' "$LOG" 2>/dev/null && { result=startup_failed; break; }
  kill -0 $LPID 2>/dev/null || { result=launch_exited; break; }
  timeout 2 tail -n0 -f /dev/null
done
if [ $result = ready ]; then
  timeout 10 tail -n0 -f /dev/null
  ( source install/setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1 CYCLONEDDS_URI="file://$PWD/install/ridgeback_autonomy/share/ridgeback_autonomy/config/parking_demo_cyclonedds.xml"
    timeout 600 python3 "$P/capture_g1_views.py" "$D/$SC" "$@" ) >> "$D/probe.log" 2>&1
  result=captured
fi
kill -TERM -- -$LPID 2>/dev/null
for _ in $(seq 60); do ps -eo sid= | awk -v g=$LPID '$1==g' | grep -q . || break; timeout 1 tail -n0 -f /dev/null; done
ps -eo sid= | awk -v g=$LPID '$1==g' | grep -q . && kill -KILL -- -$LPID 2>/dev/null
echo "$SC $result" | tee -a "$D/probes.txt"
