#!/bin/bash
# One teleop PICK demo that starts from exactly the pose the planner's pick starts from.
# usage (robot, driver running in either mode, camera running): bash ~/record_pick_demo.sh
#   1. position mode + pick_reset.py: arm in, lift 0.75, gripper +0.13 (wide), head pan -1.29 tilt -0.75
#   2. back to gamepad mode, then record_demo.sh soup_can grasp (drive with the gamepad, Enter when done)
# Put the robot on its start tape and the can in place BEFORE running it (the reset does not move the base).
source ~/ros_env.sh
switch() {   # the driver sometimes drops a reply: try 3 times, 10 s each
  for i in 1 2 3; do timeout 10 ros2 service call /switch_to_$1_mode std_srvs/srv/Trigger > /dev/null && return 0; echo "no reply from the driver, retrying"; done
  echo "could not switch to $1 mode (is the driver running?)"; exit 1
}
switch position
yes '' | /usr/bin/python3 ~/pick_reset.py --grip 0.13 || { echo "pick_reset failed"; exit 1; }
switch gamepad
echo "Start pose set: lift 0.75, arm in, gripper +0.13, head -1.29/-0.75. Gamepad is active."
read -r -p "Hands on the gamepad? Recording starts the moment you press Enter (do not move before that): "
bash ~/record_demo.sh soup_can grasp
