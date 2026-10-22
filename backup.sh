#!/bin/bash
# Pull latest files from the robot into this organized folder. Run: bash backup.sh
R=hello-robot@10.49.243.94
D="$(cd "$(dirname "$0")" && pwd)"
scp -O "$R:~/{t1.sh,t2.sh,go_t1_stage.sh,go_t2_stage.sh,go_home.sh,go_t1.sh,go_t2.sh,auto_fix.py,get_pose.py}" "$D/scripts/navigation/"
scp -O "$R:~/pick_place.py" "$D/scripts/manipulation/"
scp -O "$R:~/{aruco_align.py,aruco_check.py,aruco_cal.json}" "$D/scripts/camera/"
scp -O "$R:~/nav.rviz" "$D/config/"
scp -O "$R:~/stretch_user/maps/lab_map_v3.*" "$D/maps/"
echo "Backup done -> $D"
