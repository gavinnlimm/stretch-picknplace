# Stretch RE1 – Table 1 → Table 2 can pick & place (HRC² Lab)

Backup of the scripts that live in `~/` on the robot (hello-robot@10.49.243.94).
On the robot all files sit directly in `~/`; this folder just groups them.

## Folders
| Folder | What's in it |
|---|---|
| `scripts/navigation/` | `t1.sh` / `t2.sh` (one-command: Nav2 to 60 cm staging spot + `auto_fix.py`), staging goals, `go_home.sh`, `get_pose.py` (read AMCL pose) |
| `scripts/manipulation/` | `pick_place.py` – `pick`, `place`, `show`, `stow`, `full` |
| `scripts/camera/` | `aruco_align.py` (`calib` / `align`), `aruco_check.py` (list visible tags), `aruco_cal.json` (calibration) |
| `config/` | `nav.rviz` (RViz layout) |
| `maps/` | `lab_map_v3` = current map (teleop-built, home = 0,0). Older maps in `maps/old/` |

## Key values
- Table 1 goal (−1.146, 0.345, −4.2°), Table 2 goal (−0.660, 1.682, −178.9°) on `lab_map_v3`
- ArUco tags (DICT_6X6_250, 8 cm): **ID 8** = Table 1, **ID 7** = Table 2 place target
- Grip −0.01 (full can), reach 0.336, wrist −0.065, carry lift 0.86

## Run the full task
1. Startup on robot (each tab: `conda deactivate` ×2, source ROS):
   - homing: `stretch_free_robot_process.py; stretch_system_check.py; stretch_robot_home.py`
   - robot on home tape → Nav2: `ros2 launch stretch_nav2 navigation.launch.py use_rviz:=false map:=${HELLO_FLEET_PATH}/maps/lab_map_v3.yaml`
   - initial pose at home (`/initialpose` x 0, y 0, w 1)
   - camera: `ros2 launch stretch_core d435i_high_resolution.launch.py`
   - optional RViz on Mac: Xvfb :1 + x11vnc (pw `stretch`) + tunnel `ssh -N -L 5901:localhost:5900` → `vnc://localhost:5901`
2. Can on Table 1 spot, tag 7 clear, charger unplugged, run-stop solid.
3. `python3 ~/pick_place.py full`

## Back up from robot
`bash backup.sh` (from this folder, robot on) – pulls the latest files into the right subfolders.
