#!/bin/bash
# usage: run_follow.sh NAME SCENARIO DOMAIN [extra crowd_runner args]
name=$1; scenario=$2; domain=$3; shift 3
cd /home/pratham/Documents/ChatGPT/DyNAMO-Ridgeback
source /opt/ros/jazzy/setup.bash; source install/setup.bash
export ROS_DOMAIN_ID=$domain RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOG_DIR=$PWD/artifacts/ros
out=artifacts/leader-following/f4-dev/$name; mkdir -p $(dirname $out)
$( [ "${BACKEND:-isaac}" = isaac ] && echo "env OMNI_KIT_ACCEPT_EULA=YES isaac_venv/bin/python" || echo python3) -m ridgeback_leader_follow.crowd_runner --backend ${BACKEND:-isaac} \
  --scene-profile f3 --scenario $scenario --drive --real-time-factor 0.5 --ros-domain $domain \
  --output-dir $out "$@" > $out.sim.log 2>&1 &
sim=$!
setsid ros2 launch ridgeback_leader_follow follow_leader.launch.py odom_topic:=${ODOM:-platform/odom} $FOLLOW_ARGS > $out.follow.log 2>&1 &
follow=$!
( timeout 900 ros2 topic echo --csv /r100_0001/leader_follow/follow_status > $out.status.csv 2>/dev/null & echo $! > $out.echo.pid )
wait $sim; code=$?
kill -INT -- -$follow 2>/dev/null; sleep 5; kill -TERM -- -$follow 2>/dev/null; kill $(cat $out.echo.pid) 2>/dev/null
echo "sim exit=$code"
