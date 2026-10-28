# Stretch RE1: autonomous pick and place (HRC² Lab)

One command takes an object from Table 1 and places it at tag 7 on Table 2:

    bash ~/full_auto_obj.sh <object_name>      # e.g. soup_can, carton, soda_bottle, paper_cup, coke_can

The scripts live in `~/` on the robot (hello-robot@stretch-re1-1050); this repo groups them in folders.
The robot's IP changes when it restarts or changes networks (check with `hostname -I` on the robot). Keep it in one place with a `Host stretch` entry in `~/.ssh/config`, then `ssh stretch`.

## Pipeline (what one run does)
| Stage | What happens | Files |
|---|---|---|
| nav1 | Nav2 to the Table 1 staging spot, then `auto_fix` (AMCL refresh, turn-drive-turn onto the table pose) | `t1.sh`, `go_t1_stage.sh`, `auto_fix.py` |
| grasp | Depth only (no Table 1 tag): scan with the head, find the object and measure its height and width, drive, re-measure, side-step if more than 46 cm from the table, lower + extend to a size-based height, close to the measured width, lift + retract | `grasp_auto.py`, `can_detect.py`, `aruco_cal.json`, `gripper_cal.json` |
| nav2 | Nav2 to the Table 2 staging spot, then `auto_fix` | `t2.sh`, `go_t2_stage.sh`, `auto_fix.py` |
| align2 | Find tag 7 with the head camera while the object is held low (`ALIGN_LIFT`, default 0.66) | `aruco_align.py` |
| place | Reach by the measured distance, lower to the same height offset the object was grasped at (`~/grasp_last.json`), release, lift clear, retract | `pick_place.py place` |

Each run logs stage times to `~/object_trials.csv` (`date,object,total_s,nav1_s,align1_s,grasp_s,nav2_s,align2_s,place_s,result,failed_at`).
`full_auto_noalign1.sh` is the same run without the object name (logs to `trials.csv`). `full_auto.sh` is the original, with Table 1 tag alignment.

## Key values
- Map `lab_map_v3`, home = (0, 0). Table 1 pose (-1.146, 0.345, -4.2°), Table 2 pose (-0.660, 1.682, -178.9°).
- Tag 7 (DICT_6X6_250, 8 cm) = Table 2 place target. Tag 8 is only stored in `aruco_cal.json` (search region and table height); it is no longer detected.
- Adaptive grasp (`ADAPT=0` restores the old fixed values):
  - Grasp lift = 0.7447 + half the object's height above 11 cm (max +8 cm). Carry lift 0.86 (at least 6 cm above the grasp). Retract 0.02.
  - Gripper opens to 0.13 and closes to the measured width minus 1.5 cm, using `gripper_cal.json` (position to fingertip gap). Without the file it closes to -0.01.
  - Object center uses the measured width instead of a fixed can radius.
- Gripper calibration (fingertip gap): 0.13 = 12 cm, 0.10 = 11, 0.07 = 9.5, 0.04 = 7.5, 0.02 = 6.5, 0.00 = 5 cm. Redo with `python3 ~/gripper_cal.py` (run-stop released).
- Detection window (`can_detect.py`): points 2–30 cm above the table, object top 8–25 cm, width under 11 cm.

## Starting up
1. Robot on the home tape, area clear, run-stop solid, charger unplugged.
2. Log in (each tab): `ssh stretch` (or `ssh hello-robot@<robot ip>`), then `source ~/ros_env.sh`. Without it, conda's Python runs and `rclpy` fails to import.
3. Home: `stretch_free_robot_process.py; stretch_system_check.py; stretch_robot_home.py`
4. Nav tab: `ros2 launch stretch_nav2 navigation.launch.py use_rviz:=false map:=${HELLO_FLEET_PATH}/maps/lab_map_v3.yaml`
5. Camera tab: `ros2 launch stretch_core d435i_high_resolution.launch.py`
6. Set the start position: publish `/initialpose` (x 0, y 0, w 1), then call `/request_nomotion_update` about 10 times.
   `bash ~/start_bg.sh` does 4–6 in the background if you don't want tabs.
7. Object on the Table 1 spot, tag 7 clear, then run the command at the top.

Options: `ALIGN_LIFT=<m>` sets the carry height during the tag search; `NO_OVERLAP=1` makes the arm lower/extend and lift/retract one after the other instead of together; `ADAPT=0` uses the old fixed grasp.

## Results (Oct 2026)
| Object | Result |
|---|---|
| Soup can | 6/6, avg ~91 s |
| Carton | 3/3, avg 115 s (including one run rotated 90°) |
| 7UP bottle | detection window and tag-search height had to be changed; 2/2 afterwards (110 s, 112 s) |
| Paper cup | 1/1 (99 s) |

Adaptive grasp, Oct 28 (one run each, all successful):

| Object | Measured height, width | Grasp height | Close (gap) | Total |
|---|---|---|---|---|
| 7UP bottle | 21.4 cm, 6.7 cm | +5.3 cm | 0.002 (~5 cm) | 88 s |
| Soup can | 10.9 cm, 8.0 cm | +0.0 cm | 0.020 (~6.5 cm) | 101 s |
| Coke can | 12.3 cm, 6.2 cm | +0.7 cm | 0.000 (~5 cm) | 103 s |
| Paper cup | 11.3 cm, 8.8 cm | +0.1 cm | 0.036 (~7.2 cm) | 105 s |
| Carton | 14.2 cm, 5.9 cm | +1.6 cm | 0.000 (~5 cm) | 109 s (incl. side-step) |

## Known limits
- Detection still only accepts objects 8–25 cm tall and under 11 cm wide, in a search strip around the stored tag position. The 7 cm reach offset is fixed.
- Narrow objects (target gap under 5 cm) close only to 0.00, the narrowest calibrated point, so they get less squeeze than the old -0.01.
- Objects usually sit 16–21 cm along the table from where the robot stops, so it drives along the table before every grasp.
- Table 2 needs tag 7 to be visible; a tall carried object can block it unless it is held low.
- The Table 1 target often leaves the robot about 10–17 cm too far from the table, which costs a side-step (~9 s).
- `aruco_align.py` does not restore navigation mode if it exits early.

## Back up from the robot
`bash backup.sh` (robot on; uses the `stretch` ssh alias). Or set the address with `ROBOT=hello-robot@<ip> bash backup.sh`.
