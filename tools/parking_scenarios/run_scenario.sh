#!/bin/bash
# One parking scenario run: run_scenario.sh gz|isaac SCENARIO OUT_DIR.
# Waits for a free workstation, starts start_parking_demo.sh in its own session, records a small
# bag (no scans), samples Gazebo ground truth, takes RViz/Gazebo screenshots at key states,
# and stops only that session with SIGTERM. LIMIT (s) bounds the run.
B=$1; SC=$2; D=$(realpath -m "$3"); P=$(cd "$(dirname "$0")" && pwd); ROOT=$(cd "$P/../.." && pwd); LIMIT=${LIMIT:-900}
: "${RIDGEBACK_PERCEPTION_VENV:?set RIDGEBACK_PERCEPTION_VENV}"; export DISPLAY="${DISPLAY:?a display is needed for screenshots}"
mkdir -p "$D/shots"; cd "$ROOT"
while pgrep -f '(^|/)(isaac_runner\.py|gzserver)( |$)|[/ ]gz sim( |$)' >/dev/null; do timeout 5 tail -n0 -f /dev/null; done
git rev-parse HEAD > "$D/commit.txt"; git status --short >> "$D/commit.txt"; cp src/ridgeback_autonomy/config/parking_scenarios.yaml "$D/scenarios.yaml"
setsid bash start_parking_demo.sh $B demo_scenario:=$SC > "$D/console.log" 2>&1 &
LPID=$!
( until [ -s "$D/console.log" ]; do timeout 1 tail -n0 -f /dev/null; done; source install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1 CYCLONEDDS_URI="file://$PWD/install/ridgeback_autonomy/share/ridgeback_autonomy/config/parking_demo_cyclonedds.xml"
  exec ros2 bag record -s mcap -o "$D/bag" /r100_0001/parking_mission/state /r100_0001/parking/visualization /r100_0001/plan /r100_0001/map /r100_0001/detections/target/raw /r100_0001/measurements/target/mask > "$D/bag_record.log" 2>&1 ) & RPID=$!
[ "$B" = gz ] && { python3 "$P/truth_track.py" "$D" & TPID=$!; }
shot() {  # RViz and Gazebo windows of this session only
  for name in rviz2 ruby python kit; do
    for pid in $(ps -eo pid=,sid=,comm= | awk -v s=$LPID -v n=$name '$2==s && index($3,n)==1 {print $1}'); do
      for w in $(xdotool search --onlyvisible --pid $pid 2>/dev/null); do
        import -window $w "$D/shots/$1_${name}_$w.png" 2>/dev/null; done; done; done; }
declare -A done; end=$((SECONDS + LIMIT)); result=timeout
while [ $SECONDS -lt $end ]; do
  for st in G1_LOCALIZED NAVIGATING_TO_PREPARKING PARKED; do
    [ -z "${done[$st]}" ] && grep -q -- "-> $st;" "$D/console.log" 2>/dev/null && { done[$st]=1; [ $st = G1_LOCALIZED ] && [ $B = gz ] && python3 "$P/gz_view.py" "$SC" >> "$D/view.log" 2>&1; timeout 3 tail -n0 -f /dev/null; shot "$st"; }
  done
  r=$(grep -oE -- "-> (MISSION_COMPLETE|MISSION_FAILED|PARKING_PLANNING_FAILED|PARKING_EXECUTION_FAILED|MOTION_SETTLEMENT_FAILED)" "$D/console.log" 2>/dev/null | tail -1)
  [ -n "$r" ] && { result=${r#-> }; break; }
  kill -0 $LPID 2>/dev/null || { result=launch_exited; break; }
  timeout 2 tail -n0 -f /dev/null
done
timeout 3 tail -n0 -f /dev/null; shot "END_$result"
kill -TERM $RPID 2>/dev/null; pkill -TERM -P $RPID 2>/dev/null; timeout 30 tail --pid=$RPID -f /dev/null
[ -n "$TPID" ] && kill $TPID 2>/dev/null
kill -TERM -- -$LPID 2>/dev/null
for k in $(seq 60); do ps -eo sid= | awk -v g=$LPID '$1==g' | grep -q . || break; timeout 1 tail -n0 -f /dev/null; done
ps -eo sid= | awk -v g=$LPID '$1==g' | grep -q . && kill -KILL -- -$LPID 2>/dev/null
echo "$SC $B $result $(ls -td artifacts/exploration/parking_*_${B}.* | head -1)" | tee "$D/result.txt"
