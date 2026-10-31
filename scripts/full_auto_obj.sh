#!/bin/bash
# Usage: bash ~/full_auto_obj.sh <object_name>   e.g. bottle
# One autonomous run: home -> T1 -> grasp -> T2 -> place. Times each stage into ~/trials.csv.
# Start: robot at home, run-stop solid, can on Table 1.
# Also records the run as a demo for VLA training (~/demos/<date>_<object>/). RECORD=0 turns that off.
source ~/ros_env.sh
OBJ=${1:?usage: bash ~/full_auto_obj.sh <object_name>}
ros2 service call /switch_to_navigation_mode std_srvs/srv/Trigger > /dev/null
LOG=~/object_trials.csv
[ -f $LOG ] || echo "date,object,total_s,nav1_s,align1_s,grasp_s,nav2_s,align2_s,place_s,result,failed_at" > $LOG
REC=${RECORD:-1}; EP=~/demos/$(date +%F_%H-%M-%S)_$OBJ
if [ "$REC" = 1 ]; then
  mkdir -p "$EP"; echo start > ~/demo_stage
  /usr/bin/python3 ~/demo_logger.py "$EP" > "$EP/logger.log" 2>&1 & LOGPID=$!
  trap 'kill -TERM $LOGPID 2>/dev/null; wait $LOGPID 2>/dev/null; echo "demo saved: $EP"' EXIT
fi
meta() { [ "$REC" = 1 ] && printf '{"object": "%s", "instruction": "put the %s on table 2", "result": "%s", "failed_at": "%s", "source": "scripted"}\n' \
  "$OBJ" "${OBJ//_/ }" "$1" "$2" > "$EP/meta.json"; }
T=(); START=$(date +%s)
row() { while [ ${#T[@]} -lt 6 ]; do T+=(""); done
  echo "$(date +%F_%T),$OBJ,$(( $(date +%s)-START )),$(IFS=,; echo "${T[*]}"),$1,$2" >> $LOG; meta "$1" "$2"; }
step() { local name=$1; shift; local t0=$(date +%s); echo "=== $name ==="; echo "$name" > ~/demo_stage
  "$@"; local rc=$?; T+=($(( $(date +%s)-t0 )))
  if [ $rc -ne 0 ]; then row FAIL "$name"; echo "FAILED at $name (logged)"; exit 1; fi; }
nav() { bash "$1" 2>&1 | tee /tmp/nav_out.log; ! grep -qiE "rejected|aborted|canceled|no /amcl_pose" /tmp/nav_out.log; }
step nav1   nav ~/t1.sh
T+=("")   # align1 skipped: keep CSV columns aligned
step grasp  /usr/bin/python3 ~/grasp_auto.py t1 go
step nav2   nav ~/t2.sh
step align2 /usr/bin/python3 ~/aruco_align.py align t2
step place  /usr/bin/python3 ~/pick_place.py place
row OK ""
echo "SUCCESS in $(( $(date +%s)-START )) s (logged)"
