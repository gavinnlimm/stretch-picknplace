#!/bin/bash
# Record ONE teleoperated demo for imitation-learning training (ACT / Diffusion Policy).
# usage: bash ~/record_demo.sh <object> [task]          task = grasp (default) | drop | sort
#   e.g.  bash ~/record_demo.sh soup_can grasp      (pick up the object at Table 1)
#         bash ~/record_demo.sh soup_can drop       (drop a held object into the yellow container at Table 2)
# The stage label written to ~/demo_stage is teleop_<task>, so the converter can pick pick and drop frames apart.
#   Drive the robot in another tab:  ros2 run stretch_core keyboard_teleop
#   Optional wrist camera:           WRIST_TOPIC=/wrist_camera/image_raw bash ~/record_demo.sh soup_can
# Saves ~/demos/<date>_teleop_<task>_<object>/ (frames/, steps.jsonl, meta.json) - same format as scripted runs.
source ~/ros_env.sh
OBJ=${1:?usage: bash ~/record_demo.sh <object> [task]}; TASK=${2:-grasp}
case $TASK in
  grasp) INSTR="pick up the ${OBJ//_/ }" ;;
  drop)  INSTR="drop the ${OBJ//_/ } in the yellow container" ;;
  sort)  INSTR="put the ${OBJ//_/ } in the bin" ;;
  *)     INSTR="$TASK the ${OBJ//_/ }" ;;
esac
EP=~/demos/$(date +%F_%H-%M-%S)_teleop_${TASK}_$OBJ
mkdir -p "$EP"; echo "teleop_$TASK" > ~/demo_stage
/usr/bin/python3 ~/demo_logger.py "$EP" ${WRIST_TOPIC:+--wrist-topic $WRIST_TOPIC} > "$EP/logger.log" 2>&1 & PID=$!
trap 'kill -TERM $PID 2>/dev/null' EXIT
sleep 1
echo "RECORDING -> $EP"
echo "Task: \"$INSTR\". Drive the robot now; press Enter when the task is finished."
read -r
kill -TERM $PID; wait $PID 2>/dev/null; trap - EXIT
read -r -p "Did it succeed? (y/n): " OK
[ "$OK" = y ] && RES=OK || RES=FAIL
read -r -p "Did it include a correction (recovered from a mistake)? (y/n): " COR
# meta.json: stage = the label stored on every frame, result = OK (success) / FAIL, correction = recovered from a mistake
printf '{"object": "%s", "task": "%s", "stage": "teleop_%s", "instruction": "%s", "result": "%s", "source": "teleop", "correction": %s}\n' \
  "$OBJ" "$TASK" "$TASK" "$INSTR" "$RES" "$([ "$COR" = y ] && echo true || echo false)" > "$EP/meta.json"
N=$(wc -l < "$EP/steps.jsonl" 2>/dev/null || echo 0)
echo "saved $N frames ($((N/10)) s) -> $EP  [$RES]"
[ "$N" -lt 10 ] && echo "WARNING: almost no frames - is the camera tab running?  (see $EP/logger.log)"
