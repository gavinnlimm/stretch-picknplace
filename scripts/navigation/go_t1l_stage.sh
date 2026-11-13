# Nav2 to the staging spot for t1l (learned-pick parking): 0.60 m ahead of it along its heading, like go_t1_stage.sh is for t1
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: -1.003, y: 0.358}, orientation: {z: -0.0366, w: 0.9993}}}}"
