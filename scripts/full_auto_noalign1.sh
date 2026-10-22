#!/bin/bash
# One autonomous run: home -> T1 -> grasp -> T2 -> place. Times each stage into ~/trials.csv.
# Start: robot at home, run-stop solid, can on Table 1.
source ~/ros_env.sh
ros2 service call /switch_to_navigation_mode std_srvs/srv/Trigger > /dev/null
LOG=~/trials.csv
[ -f $LOG ] || echo "date,total_s,nav1_s,align1_s,grasp_s,nav2_s,align2_s,place_s,result,failed_at" > $LOG
T=(); START=$(date +%s)
row() { while [ ${#T[@]} -lt 6 ]; do T+=(""); done
  echo "$(date +%F_%T),$(( $(date +%s)-START )),$(IFS=,; echo "${T[*]}"),$1,$2" >> $LOG; }
step() { local name=$1; shift; local t0=$(date +%s); echo "=== $name ==="
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
