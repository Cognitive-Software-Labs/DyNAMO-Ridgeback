#!/bin/bash
# usage: run_follow.sh NAME SCENARIO DOMAIN [extra crowd_runner args]
name=$1; scenario=$2; domain=$3; shift 3
cd /home/pratham/Documents/ChatGPT/DyNAMO-Ridgeback
source /opt/ros/jazzy/setup.bash; source install/setup.bash
export ROS_DOMAIN_ID=$domain RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOG_DIR=$PWD/artifacts/ros
out=artifacts/leader-following/f4-dev/$name; mkdir -p $(dirname $out)
$( [ "${BACKEND:-isaac}" = isaac ] && echo "env OMNI_KIT_ACCEPT_EULA=YES isaac_venv/bin/python" || echo python3) -m ridgeback_leader_follow.crowd_runner --backend ${BACKEND:-isaac} \
  ${PROFILE:---scene-profile f3} --scenario $scenario --drive --real-time-factor 0.5 --ros-domain $domain \
  --output-dir $out "$@" > $out.sim.log 2>&1 &
sim=$!
setsid ros2 launch ridgeback_leader_follow follow_leader.launch.py odom_topic:=${ODOM:-platform/odom} $FOLLOW_ARGS > $out.follow.log 2>&1 &
follow=$!
( REC_ODOM=${ODOM:-platform/odom} timeout 1800 python3 /tmp/claude-1001/-home-pratham--claude-spares-spare-breXZk/5be7fa59-5207-403c-9e0b-2dee1164bec7/scratchpad/follow/recorder.py $out.events.jsonl > /dev/null 2>&1 & echo $! > $out.echo.pid )
wait $sim; code=$?
kill -INT -- -$follow 2>/dev/null; sleep 5; kill -TERM -- -$follow 2>/dev/null; kill -INT $(cat $out.echo.pid) 2>/dev/null
echo "sim exit=$code"
