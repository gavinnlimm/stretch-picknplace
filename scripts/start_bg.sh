#!/bin/bash
# Starts nav, camera, VNC, RViz and the home pose in the background. Survives closed Mac tabs.
mkdir -p ~/logs
pkill -f "Xvfb|x11vnc|rviz2|/opt/ros/humble|ament_ws"; sleep 3
bg() { setsid nohup bash -ic "source ~/ros_env.sh; $2" > ~/logs/$1.log 2>&1 < /dev/null & }
bg nav  "ros2 launch stretch_nav2 navigation.launch.py use_rviz:=false map:=\$HELLO_FLEET_PATH/maps/lab_map_v3.yaml"
bg vnc  "Xvfb :1 -screen 0 1600x900x24 & sleep 2; x11vnc -display :1 -forever -localhost -rfbauth ~/.vnc/passwd"
bg cam  "ros2 launch stretch_core d435i_high_resolution.launch.py"
bg rviz "sleep 8; export LIBGL_ALWAYS_SOFTWARE=1; DISPLAY=:1 rviz2 -d ~/nav.rviz"
bg pose "sleep 30; ros2 topic pub -t 3 -r 1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}'; for i in 1 2 3 4 5 6 7 8 9 10; do timeout 5 ros2 service call /request_nomotion_update std_srvs/srv/Empty; sleep 0.5; done; echo POSE SET"
echo "started (pose set after ~30 s). check: tail -2 ~/logs/pose.log"
